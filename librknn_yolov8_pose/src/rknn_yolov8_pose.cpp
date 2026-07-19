#include <librknn_yolov8_pose/rknn_yolov8_pose.hpp>

#include "detection_postprocessor.hpp"
#include "letterbox_preprocessor.hpp"

#include <rknn_api.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <yolo_msgs/msg/keypoint.hpp>

namespace rknn_yolo {
namespace {

constexpr int kDetectionOutputCount = 3;
constexpr int kExpectedInputCount = 1;
constexpr int kExpectedOutputCount = 4;
constexpr int kDflChannels = 64;
constexpr int kKeypointCount = 17;
constexpr int kKeypointValues = 3;

std::optional<PixelFormat>
pixel_format_for_encoding(const std::string &encoding) {
    using namespace sensor_msgs::image_encodings;

    if (encoding == RGB8) {
        return PixelFormat::RGB888;
    }
    if (encoding == BGR8) {
        return PixelFormat::BGR888;
    }
    if (encoding == RGBA8) {
        return PixelFormat::RGBA8888;
    }
    if (encoding == BGRA8) {
        return PixelFormat::BGRA8888;
    }
    if (encoding == YUV422_YUY2) {
        return PixelFormat::YUYV422;
    }
    if (encoding == YUV422) {
        return PixelFormat::UYVY422;
    }
    return std::nullopt;
}

bool image_storage_is_valid(const sensor_msgs::msg::Image &image) {
    if (image.width == 0U || image.height == 0U || image.step == 0U) {
        return false;
    }

    const std::size_t height = static_cast<std::size_t>(image.height);
    const std::size_t step = static_cast<std::size_t>(image.step);
    return step <= std::numeric_limits<std::size_t>::max() / height &&
           image.data.size() >= step * height;
}

} // namespace

struct YoloV8Pose::Impl {
    // Constructor
    Impl(const std::string &model_path, const std::string &label_path,
         rclcpp::Logger logger)
        : logger_(std::move(logger)) {
        try {
            // Loads class names for postprocessing
            load_labels(label_path);
            // Init RKNN context and model
            // Validates the complete inference pipeline
            initialize_model(model_path);
        } catch (...) {
            destroy_context();
            throw;
        }
    }

    // Destructor
    ~Impl() { destroy_context(); }

    /**
     * @brief Runs inference on one frame
     * @param image Input image message
     * @param detections Output detections message
     * @return 0 on success, negative on failure
     */
    int infer(const sensor_msgs::msg::Image::ConstSharedPtr &image,
              yolo_msgs::msg::Detections::SharedPtr detections) {
        if (image == nullptr || detections == nullptr) {
            RCLCPP_ERROR(logger_,
                         "infer received a null image or detections message");
            return -1;
        }

        SrcView source;
        cv_bridge::CvImageConstPtr converted_image;
        const std::optional<PixelFormat> direct_format =
            pixel_format_for_encoding(image->encoding);

        if (direct_format.has_value()) {
            if (!image_storage_is_valid(*image) ||
                image->width > static_cast<std::uint32_t>(
                                   std::numeric_limits<int>::max()) ||
                image->height > static_cast<std::uint32_t>(
                                    std::numeric_limits<int>::max())) {
                RCLCPP_ERROR(logger_, "invalid %s image buffer",
                             image->encoding.c_str());
                return -1;
            }

            source.data = image->data.data();
            source.width = static_cast<int>(image->width);
            source.height = static_cast<int>(image->height);
            source.step_bytes = static_cast<std::size_t>(image->step);
            source.format = *direct_format;
        } else {
            try {
                converted_image = cv_bridge::toCvShare(
                    image, sensor_msgs::image_encodings::RGB8);
            } catch (const cv_bridge::Exception &exception) {
                RCLCPP_ERROR(logger_, "cv_bridge conversion failed: %s",
                             exception.what());
                return -1;
            }

            const cv::Mat &converted = converted_image->image;
            if (converted.empty() || converted.data == nullptr) {
                RCLCPP_ERROR(logger_, "cv_bridge produced an empty image");
                return -1;
            }
            source.data = converted.data;
            source.width = converted.cols;
            source.height = converted.rows;
            source.step_bytes = converted.step[0];
            source.format = PixelFormat::RGB888;
        }

        Letterbox letterbox;
        if (preprocessor_.process(source, letterbox) != 0) {
            RCLCPP_ERROR(logger_, "letterbox preprocessing failed");
            return -1;
        }

        int ret = rknn_inputs_set(rknn_ctx_, io_num_.n_input, inputs_.data());
        if (ret < 0) {
            RCLCPP_ERROR(logger_, "rknn_inputs_set failed: %d", ret);
            return -1;
        }

        ret = rknn_run(rknn_ctx_, nullptr);
        if (ret < 0) {
            RCLCPP_ERROR(logger_, "rknn_run failed: %d", ret);
            return -1;
        }

        for (std::size_t i = 0; i < outputs_.size(); ++i) {
            outputs_[i] = {};
            outputs_[i].index = static_cast<std::uint32_t>(i);
            outputs_[i].want_float = static_cast<std::uint8_t>(!is_quantized_);
        }

        ret = rknn_outputs_get(rknn_ctx_, io_num_.n_output, outputs_.data(),
                               nullptr);
        if (ret < 0) {
            RCLCPP_ERROR(logger_, "rknn_outputs_get failed: %d", ret);
            return -1;
        }

        const std::vector<Detection> *decoded = nullptr;
        try {
            decoded =
                &postprocessor_->decode(outputs_, output_attrs_, letterbox);
        } catch (const std::exception &exception) {
            rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs_.data());
            RCLCPP_ERROR(logger_, "detection postprocessing failed: %s",
                         exception.what());
            return -1;
        } catch (...) {
            rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs_.data());
            RCLCPP_ERROR(logger_, "detection postprocessing failed");
            return -1;
        }

        rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs_.data());
        populate_results(*decoded, *image, detections);
        return 0;
    }

  private:
    [[noreturn]] void fail(const std::string &message) {
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        throw std::runtime_error(message);
    }

    void load_labels(const std::string &label_path) {
        std::ifstream stream(label_path);
        if (!stream.is_open()) {
            fail("failed to open label file: " + label_path);
        }

        std::string label;
        while (std::getline(stream, label)) {
            if (!label.empty() && label.back() == '\r') {
                label.pop_back();
            }
            labels_.push_back(label);
        }
        if (stream.bad()) {
            fail("failed while reading label file: " + label_path);
        }
        if (labels_.empty()) {
            fail("label file is empty: " + label_path);
        }
    }

    void initialize_model(const std::string &model_path) {
        // Creates the RKNN context
        // The resulting hanndle is stored in `rknn_ctx_`
        int ret = rknn_init(
            &rknn_ctx_,
            static_cast<void *>(const_cast<char *>(model_path.c_str())), 0, 0,
            nullptr);
        if (ret < 0) {
            rknn_ctx_ = 0;
            fail("rknn_init failed for " + model_path + ": " +
                 std::to_string(ret));
        }
        context_initialized_ = true;

        // Queries IO tensor counts
        // Requires 1 input and 4 output tensors
        ret = rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_,
                         sizeof(io_num_));
        if (ret != RKNN_SUCC) {
            fail("rknn_query(RKNN_QUERY_IN_OUT_NUM) failed: " +
                 std::to_string(ret));
        }
        if (io_num_.n_input != kExpectedInputCount ||
            io_num_.n_output != kExpectedOutputCount) {
            fail("expected " + std::to_string(kExpectedInputCount) +
                 " input(s) and " + std::to_string(kExpectedOutputCount) +
                 " output(s), got " + std::to_string(io_num_.n_input) +
                 " input(s) and " + std::to_string(io_num_.n_output) +
                 " output(s)");
        }

        // Queries IO tensor attributes
        // Later validated in `validate_model()`
        // See detailed attributes in `rknn_api.h` for `rknn_tensor_attr`
        input_attrs_.resize(io_num_.n_input);
        for (std::uint32_t i = 0; i < io_num_.n_input; ++i) {
            rknn_tensor_attr &attr = input_attrs_[i];
            std::memset(&attr, 0, sizeof(attr));
            attr.index = i;
            ret = rknn_query(rknn_ctx_, RKNN_QUERY_INPUT_ATTR, &attr,
                             sizeof(attr));
            if (ret != RKNN_SUCC) {
                fail("rknn_query(RKNN_QUERY_INPUT_ATTR, " + std::to_string(i) +
                     ") failed: " + std::to_string(ret));
            }
        }

        output_attrs_.resize(io_num_.n_output);
        for (std::uint32_t i = 0; i < io_num_.n_output; ++i) {
            rknn_tensor_attr &attr = output_attrs_[i];
            std::memset(&attr, 0, sizeof(attr));
            attr.index = i;
            ret = rknn_query(rknn_ctx_, RKNN_QUERY_OUTPUT_ATTR, &attr,
                             sizeof(attr));
            if (ret != RKNN_SUCC) {
                fail("rknn_query(RKNN_QUERY_OUTPUT_ATTR, " + std::to_string(i) +
                     ") failed: " + std::to_string(ret));
            }
        }

        // Validates the model's input and output tensor attributes and
        // determine the inference logic depends on the metadata
        validate_model();

        // Allocates a persistent input buffer
        const std::uint64_t input_size =
            static_cast<std::uint64_t>(model_width_) *
            static_cast<std::uint64_t>(model_height_) *
            static_cast<std::uint64_t>(model_channels_);
        if (input_size > std::numeric_limits<std::size_t>::max() ||
            input_size > std::numeric_limits<std::uint32_t>::max()) {
            fail("model input tensor is too large");
        }
        input_buffer_.resize(static_cast<std::size_t>(input_size));

        // Init preprocessing
        if (preprocessor_.init(input_buffer_.data(), model_width_,
                               model_height_) != 0) {
            fail("failed to initialize letterbox preprocessor");
        }

        // Prepares the input descriptor
        inputs_.resize(io_num_.n_input);
        inputs_[0] = {};
        inputs_[0].index = 0;
        inputs_[0].type = RKNN_TENSOR_UINT8;
        inputs_[0].fmt = RKNN_TENSOR_NHWC;
        inputs_[0].size = input_size;
        inputs_[0].buf = input_buffer_.data();

        // Prepares the output and postprocessing state
        outputs_.resize(io_num_.n_output);
        postprocessor_ = std::make_unique<DetectionPostprocessor>(
            model_width_, model_height_, output_attrs_);

        if (postprocessor_->num_classes() != num_classes_) {
            fail("postprocessor class count does not match the model");
        }

        RCLCPP_INFO(logger_, "model input: %dx%dx%d, classes: %d", model_width_,
                    model_height_, model_channels_, num_classes_);
    }

    /**
     * @brief Validates the model's input and output tensor attributes and
     * determine the inference logic depends on the metadata
     *
     * Input: a 640x640 image of shape [1 (batch size), 3 (channel), 640, 640]
     * (NCHW) or [1, 640, 640, 3] (NHWC)
     *
     * Output:
     *   - output 0: small-obj detection head of shape [1, 64 + 1 (num_classes),
     * 80, 80]
     *   - output 1: medium-obj detection head of shape [1, 64 + 1
     * (num_classes), 40, 40]
     *   - output 2: large-obj detection head of shape [1, 64 + 1 (num_classes),
     * 20, 20]
     *   - output 3: keypoints for all three heads of shape [17 (num_kpts), 3 (x
     * + y + conf), 8400 (80*80 + 40*40 + 20*20)]
     */
    void validate_model() {
        /* Validates the input tensor */
        // 4 dimensions, batch size 1
        const rknn_tensor_attr &input = input_attrs_.front();
        if (input.n_dims != 4U || input.dims[0] != 1U) {
            fail("model input must be a four-dimensional, batch-one tensor");
        }

        // NCHW or NHWC layout
        std::uint32_t width = 0U;
        std::uint32_t height = 0U;
        std::uint32_t channels = 0U;
        if (input.fmt == RKNN_TENSOR_NCHW) {
            channels = input.dims[1];
            height = input.dims[2];
            width = input.dims[3];
        } else if (input.fmt == RKNN_TENSOR_NHWC) {
            height = input.dims[1];
            width = input.dims[2];
            channels = input.dims[3];
        } else {
            fail("model input must use NCHW or NHWC layout");
        }

        // Nonzero and representable dimensions
        if (width == 0U || height == 0U ||
            width >
                static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
            height >
                static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
            fail("model input dimensions are invalid");
        }
        // 3 channels
        if (channels != 3U) {
            fail("model input must have exactly three channels");
        }

        // Populates members for later use
        model_width_ = static_cast<int>(width);
        model_height_ = static_cast<int>(height);
        model_channels_ = static_cast<int>(channels);

        /* Validates the output tensors */
        // 4 dimensions, batch size 1, (64 + num_classes) channels
        const rknn_tensor_attr &first_output = output_attrs_.front();
        if (first_output.n_dims != 4U || first_output.dims[0] != 1U ||
            first_output.dims[1] <= static_cast<std::uint32_t>(kDflChannels)) {
            fail("model output must be a four-dimensional, "
                 "batch-one tensor with more than " +
                 std::to_string(kDflChannels) + " channels");
        }

        // Determines # of classes
        const std::uint32_t class_count =
            first_output.dims[1] - static_cast<std::uint32_t>(kDflChannels);
        if (class_count >
                static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
            class_count > static_cast<std::uint32_t>(
                              std::numeric_limits<std::uint16_t>::max())) {
            fail("model class count is too large");
        }
        num_classes_ = static_cast<int>(class_count);

        // Determines whether the model is quantized
        // We decode affine INT8 values using scale and zero point,
        // and request and decode floating-point values.
        is_quantized_ =
            first_output.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
            first_output.type != RKNN_TENSOR_FLOAT16;

        // Validates the three detection heads
        std::uint64_t total_anchors = 0U;
        for (int i = 0; i < kDetectionOutputCount; ++i) {
            const rknn_tensor_attr &output = output_attrs_[i];
            // 4 dimensions, batch size 1, NCHW layout, same channel count as
            // first output, nonzero representable width and height
            if (output.n_dims != 4U || output.dims[0] != 1U ||
                output.fmt != RKNN_TENSOR_NCHW ||
                output.dims[1] != first_output.dims[1] ||
                output.dims[2] == 0U || output.dims[3] == 0U ||
                output.dims[2] > static_cast<std::uint32_t>(
                                     std::numeric_limits<int>::max()) ||
                output.dims[3] > static_cast<std::uint32_t>(
                                     std::numeric_limits<int>::max())) {
                fail("detection output " + std::to_string(i) +
                     " has an invalid shape or layout");
            }
            // Contains exactly the stated # elements
            // An integral and equal horizontal/vertical stride
            const std::uint64_t expected_elements =
                static_cast<std::uint64_t>(output.dims[1]) *
                static_cast<std::uint64_t>(output.dims[2]) *
                static_cast<std::uint64_t>(output.dims[3]);
            if (output.n_elems != expected_elements ||
                model_height_ % static_cast<int>(output.dims[2]) != 0 ||
                model_width_ % static_cast<int>(output.dims[3]) != 0 ||
                model_height_ / static_cast<int>(output.dims[2]) !=
                    model_width_ / static_cast<int>(output.dims[3])) {
                fail("detection output " + std::to_string(i) +
                     " has an invalid grid stride");
            }
            // Quantized detection outputs must be affine INT8 tensors
            if (is_quantized_ &&
                (output.type != RKNN_TENSOR_INT8 ||
                 output.qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC ||
                 output.scale == 0.0F)) {
                fail("quantized detection outputs must be affine INT8 tensors");
            }

            // Sums the grid cells for later keypoint validation
            total_anchors += static_cast<std::uint64_t>(output.dims[2]) *
                             static_cast<std::uint64_t>(output.dims[3]);
        }

        // Validates the keypoint head
        const rknn_tensor_attr &keypoints = output_attrs_[3];
        // Quantized keypoint outputs must be FLOAT16 tensors
        if (is_quantized_ && keypoints.type != RKNN_TENSOR_FLOAT16) {
            fail("quantized model keypoint output must be FLOAT16");
        }
        // Contains exactly the stated # elements
        const std::uint64_t required_keypoint_values =
            total_anchors * static_cast<std::uint64_t>(kKeypointCount) *
            static_cast<std::uint64_t>(kKeypointValues);
        if (keypoints.n_elems != required_keypoint_values) {
            fail("keypoint output does not match the decoded anchor layout");
        }

        // Validates the label count matches the model's class count
        if (labels_.size() != static_cast<std::size_t>(num_classes_)) {
            fail("label count (" + std::to_string(labels_.size()) +
                 ") does not match model class count (" +
                 std::to_string(num_classes_) + ")");
        }
    }

    void populate_results(
        const std::vector<Detection> &decoded,
        const sensor_msgs::msg::Image &image,
        const yolo_msgs::msg::Detections::SharedPtr &detections) const {
        static_assert(
            kKeypointCount == yolo_msgs::msg::Keypoint::NUM_KEYPOINTS,
            "YOLOv8 pose and yolo_msgs must use the same keypoint count");

        detections->image_meta.header = image.header;
        detections->image_meta.width = static_cast<std::uint16_t>(image.width);
        detections->image_meta.height =
            static_cast<std::uint16_t>(image.height);

        detections->detections.clear();
        detections->detections.reserve(decoded.size());
        for (const Detection &result : decoded) {
            yolo_msgs::msg::Detection detection;
            detection.class_id = static_cast<std::uint16_t>(result.cls_id);
            detection.class_name =
                labels_.at(static_cast<std::size_t>(result.cls_id));
            detection.confidence = result.score;
            detection.id = yolo_msgs::msg::Detection::UNCHECKED;
            detection.bbox.xmin = static_cast<std::uint16_t>(result.box.left);
            detection.bbox.ymin = static_cast<std::uint16_t>(result.box.top);
            detection.bbox.xmax = static_cast<std::uint16_t>(result.box.right);
            detection.bbox.ymax = static_cast<std::uint16_t>(result.box.bottom);

            detection.keypoints.reserve(kKeypointCount);
            for (int i = 0; i < kKeypointCount; ++i) {
                yolo_msgs::msg::Keypoint keypoint;
                keypoint.id = static_cast<std::uint8_t>(i);
                keypoint.x = result.keypoints[i][0];
                keypoint.y = result.keypoints[i][1];
                keypoint.confidence = result.keypoints[i][2];
                detection.keypoints.push_back(keypoint);
            }
            detections->detections.push_back(std::move(detection));
        }
    }

    void destroy_context() noexcept {
        if (context_initialized_) {
            rknn_destroy(rknn_ctx_);
            rknn_ctx_ = 0;
            context_initialized_ = false;
        }
    }

    rclcpp::Logger logger_;
    rknn_context rknn_ctx_{0};
    bool context_initialized_{false};
    rknn_input_output_num io_num_{};
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;
    int model_width_{0};
    int model_height_{0};
    int model_channels_{0};
    int num_classes_{0};
    bool is_quantized_{false};

    std::vector<std::string> labels_;
    std::vector<std::uint8_t> input_buffer_;
    LetterboxPreprocessor preprocessor_;
    std::unique_ptr<DetectionPostprocessor> postprocessor_;
    std::vector<rknn_input> inputs_;
    std::vector<rknn_output> outputs_;
};

YoloV8Pose::YoloV8Pose(const std::string &model_path,
                       const std::string &label_path, rclcpp::Logger logger)
    : impl_(std::make_unique<Impl>(model_path, label_path, std::move(logger))) {
}

YoloV8Pose::~YoloV8Pose() = default;

int YoloV8Pose::infer(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                      yolo_msgs::msg::Detections::SharedPtr detections) {
    return impl_->infer(image, std::move(detections));
}

} // namespace rknn_yolo
