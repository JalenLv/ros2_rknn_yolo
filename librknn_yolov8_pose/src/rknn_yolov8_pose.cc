#include <librknn_yolov8_pose/rknn_yolov8_pose.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace rknn_yolo {

namespace {

constexpr char kLetterboxPaddingValue = 114;

} // namespace

YoloV8Pose::YoloV8Pose(const std::string& model_path, const std::string& label_path, rclcpp::Logger logger)
    : logger_(std::move(logger)) {
    init_model(model_path, label_path);
}

YoloV8Pose::~YoloV8Pose() {
    release_model();
}

void YoloV8Pose::init_model(const std::string& model_path, const std::string& label_path) {
    int ret = init_post_process(label_path.c_str());
    if (ret != 0) {
        RCLCPP_ERROR(logger_, "Failed to load labels from %s", label_path.c_str());
        throw std::runtime_error("init_post_process failed");
    }

    ret = rknn_init(&rknn_ctx_, static_cast<void*>(const_cast<char*>(model_path.c_str())), 0, 0, nullptr);
    if (ret < 0) {
        RCLCPP_ERROR(logger_, "rknn_init failed for %s: %d", model_path.c_str(), ret);
        deinit_post_process();
        throw std::runtime_error("rknn_init failed");
    }

    ret = rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != RKNN_SUCC) {
        RCLCPP_ERROR(logger_, "rknn_query(RKNN_QUERY_IN_OUT_NUM) failed: %d", ret);
        release_model();
        throw std::runtime_error("rknn_query failed");
    }

    input_attrs_.resize(io_num_.n_input);
    for (uint32_t i = 0; i < io_num_.n_input; ++i) {
        auto& attr = input_attrs_[i];
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;
        ret = rknn_query(rknn_ctx_, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
        if (ret != RKNN_SUCC) {
            RCLCPP_ERROR(logger_, "rknn_query(RKNN_QUERY_INPUT_ATTR, %u) failed: %d", i, ret);
            release_model();
            throw std::runtime_error("rknn input attr query failed");
        }
    }

    output_attrs_.resize(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        auto& attr = output_attrs_[i];
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;
        ret = rknn_query(rknn_ctx_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
        if (ret != RKNN_SUCC) {
            RCLCPP_ERROR(logger_, "rknn_query(RKNN_QUERY_OUTPUT_ATTR, %u) failed: %d", i, ret);
            release_model();
            throw std::runtime_error("rknn output attr query failed");
        }
    }

    is_quant_ = output_attrs_[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
                output_attrs_[0].type != RKNN_TENSOR_FLOAT16;

    if (input_attrs_[0].fmt == RKNN_TENSOR_NCHW) {
        model_channel_ = input_attrs_[0].dims[1];
        model_height_ = input_attrs_[0].dims[2];
        model_width_ = input_attrs_[0].dims[3];
    } else {
        model_height_ = input_attrs_[0].dims[1];
        model_width_ = input_attrs_[0].dims[2];
        model_channel_ = input_attrs_[0].dims[3];
    }

    dst_image_.width = model_width_;
    dst_image_.height = model_height_;
    dst_image_.width_stride = model_width_;
    dst_image_.height_stride = model_height_;
    dst_image_.format = IMAGE_FORMAT_RGB888;
    dst_image_.size = get_image_size(&dst_image_);
    dst_image_.fd = -1;
    dst_image_.virt_addr = static_cast<unsigned char*>(std::malloc(dst_image_.size));
    if (dst_image_.virt_addr == nullptr) {
        RCLCPP_ERROR(logger_, "Failed to allocate %d bytes for input buffer", dst_image_.size);
        release_model();
        throw std::bad_alloc();
    }

    RCLCPP_INFO(logger_, "model input: %dx%dx%d", model_width_, model_height_, model_channel_);
}

void YoloV8Pose::release_model() {
    if (rknn_ctx_ != 0) {
        rknn_destroy(rknn_ctx_);
        rknn_ctx_ = 0;
    }

    if (dst_image_.virt_addr != nullptr) {
        std::free(dst_image_.virt_addr);
        dst_image_.virt_addr = nullptr;
    }

    input_attrs_.clear();
    output_attrs_.clear();
    deinit_post_process();
}

int YoloV8Pose::infer(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    yolo_msgs::msg::Detections::SharedPtr detections) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(logger_, "cv_bridge conversion failed: %s", e.what());
        return -1;
    }

    cv::Mat& mat = cv_ptr->image;

    // Point directly at the cv_bridge-owned buffer to avoid an extra memcpy.
    src_image_.width = mat.cols;
    src_image_.height = mat.rows;
    src_image_.width_stride = mat.step[0] / static_cast<int>(mat.elemSize());
    src_image_.height_stride = mat.rows;
    src_image_.format = IMAGE_FORMAT_RGB888;
    src_image_.virt_addr = mat.data;
    src_image_.size = static_cast<int>(mat.total() * mat.elemSize());
    src_image_.fd = -1;

    dst_image_.width = model_width_;
    dst_image_.height = model_height_;
    dst_image_.width_stride = model_width_;
    dst_image_.height_stride = model_height_;
    dst_image_.format = IMAGE_FORMAT_RGB888;
    dst_image_.fd = -1;

    letterbox_t letter_box{};
    if (convert_image_with_letterbox(&src_image_, &dst_image_, &letter_box, kLetterboxPaddingValue) != 0) {
        RCLCPP_ERROR(logger_, "convert_image_with_letterbox failed");
        return -1;
    }

    std::vector<rknn_input> inputs(io_num_.n_input);
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = model_width_ * model_height_ * model_channel_;
    inputs[0].buf = dst_image_.virt_addr;

    int ret = rknn_inputs_set(rknn_ctx_, io_num_.n_input, inputs.data());
    if (ret < 0) {
        RCLCPP_ERROR(logger_, "rknn_inputs_set failed: %d", ret);
        return -1;
    }

    ret = rknn_run(rknn_ctx_, nullptr);
    if (ret < 0) {
        RCLCPP_ERROR(logger_, "rknn_run failed: %d", ret);
        return -1;
    }

    std::vector<rknn_output> outputs(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        outputs[i].index = i;
        outputs[i].want_float = !is_quant_;
    }

    ret = rknn_outputs_get(rknn_ctx_, io_num_.n_output, outputs.data(), nullptr);
    if (ret < 0) {
        RCLCPP_ERROR(logger_, "rknn_outputs_get failed: %d", ret);
        return -1;
    }

    // Build a lightweight compatibility view for post_process().
    rknn_app_context_t app_ctx{};
    app_ctx.rknn_ctx = rknn_ctx_;
    app_ctx.io_num = io_num_;
    app_ctx.input_attrs = input_attrs_.data();
    app_ctx.output_attrs = output_attrs_.data();
    app_ctx.model_width = model_width_;
    app_ctx.model_height = model_height_;
    app_ctx.model_channel = model_channel_;
    app_ctx.is_quant = is_quant_;

    ret = post_process(&app_ctx, outputs.data(), &letter_box, BOX_THRESH, NMS_THRESH, &od_results_);
    rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs.data());
    if (ret != 0) {
        RCLCPP_ERROR(logger_, "post_process failed: %d", ret);
        return -1;
    }

    populate_results(mat, img->header, detections);
    return 0;
}

void YoloV8Pose::populate_results(
    const cv::Mat& mat,
    const std_msgs::msg::Header& header,
    yolo_msgs::msg::Detections::SharedPtr detections) {
    detections->image_meta.header = header;
    detections->image_meta.width = static_cast<uint16_t>(mat.cols);
    detections->image_meta.height = static_cast<uint16_t>(mat.rows);

    detections->detections.clear();
    detections->detections.reserve(od_results_.count);
    for (int i = 0; i < od_results_.count; ++i) {
        const object_detect_result& det_result = od_results_.results[i];
        yolo_msgs::msg::Detection detection;
        detection.class_id = static_cast<uint16_t>(det_result.cls_id);
        detection.class_name = std::string(coco_cls_to_name(det_result.cls_id));
        detection.confidence = det_result.prop;
        detection.id = yolo_msgs::msg::Detection::UNCHECKED;
        detection.bbox.xmin = static_cast<uint16_t>(det_result.box.left);
        detection.bbox.ymin = static_cast<uint16_t>(det_result.box.top);
        detection.bbox.xmax = static_cast<uint16_t>(det_result.box.right);
        detection.bbox.ymax = static_cast<uint16_t>(det_result.box.bottom);

        detection.keypoints.reserve(NUM_KEYPOINTS);
        for (int j = 0; j < NUM_KEYPOINTS; ++j) {
            yolo_msgs::msg::Keypoint keypoint;
            keypoint.id = static_cast<uint8_t>(j);
            keypoint.x = det_result.keypoints[j][0];
            keypoint.y = det_result.keypoints[j][1];
            keypoint.confidence = det_result.keypoints[j][2];
            detection.keypoints.push_back(keypoint);
        }

        detections->detections.push_back(std::move(detection));
    }
}

} // namespace rknn_yolo
