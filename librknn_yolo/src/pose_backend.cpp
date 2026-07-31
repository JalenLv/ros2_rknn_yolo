#include "pose_backend.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yolo_msgs/msg/keypoint.hpp>

namespace rknn_yolo {
namespace {

constexpr int kDetectionOutputCount = 3;
constexpr int kExpectedOutputCount  = 4;
constexpr int kDflChannels          = 64;
constexpr int kKeypointCount        = 17;
constexpr int kKeypointValues       = 3;

} // namespace

PoseBackend::PoseBackend() = default;

PoseBackend::~PoseBackend() = default;

void PoseBackend::init(rclcpp::Node &node) {
    params_ = declare_common_params(node);

    // Creates the core engine, validates the outputs, and finalizes the IO
    engine_ = std::make_unique<CoreEngine>(
        params_.model_path, params_.label_path, node.get_logger());
    validate_outputs();
    engine_->finalize_io(params_.use_rga, is_quantized_);

    // Prepares the postprocessing state
    postprocessor_ = std::make_unique<PosePostprocessor>(
        engine_->model_width(), engine_->model_height(),
        engine_->output_attrs());

    if (postprocessor_->num_classes() != num_classes_) {
        engine_->fail("postprocessor class count does not match the model");
    }

    RCLCPP_INFO(engine_->logger(), "model input: %dx%dx%d, classes: %d",
                engine_->model_width(), engine_->model_height(),
                engine_->model_channels(), num_classes_);
}

void PoseBackend::validate_outputs() {
    const std::vector<rknn_tensor_attr> &output_attrs = engine_->output_attrs();
    if (output_attrs.size() != static_cast<std::size_t>(kExpectedOutputCount)) {
        engine_->fail("pose backend expects " +
                      std::to_string(kExpectedOutputCount) + " outputs, got " +
                      std::to_string(output_attrs.size()));
    }

    // Validates the output tensors
    // 4 dimensions, batch size 1, (64 + num_classes) channels
    const rknn_tensor_attr &first_output = output_attrs.front();
    if (first_output.n_dims != 4U || first_output.dims[0] != 1U ||
        first_output.dims[1] <= static_cast<std::uint32_t>(kDflChannels)) {
        engine_->fail(
            "model output must be a four-dimensional, batch-one tensor "
            "with more than " +
            std::to_string(kDflChannels) + " channels");
    }

    // Determines # of classes
    const std::uint32_t class_count =
        first_output.dims[1] - static_cast<std::uint32_t>(kDflChannels);
    if (class_count >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        class_count > static_cast<std::uint32_t>(
                          std::numeric_limits<std::uint16_t>::max())) {
        engine_->fail("model class count is too large");
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
        const rknn_tensor_attr &output =
            output_attrs[static_cast<std::size_t>(i)];
        // 4 dimensions, batch size 1, NCHW layout, same channel count as
        // first output, nonzero representable width and height
        if (output.n_dims != 4U || output.dims[0] != 1U ||
            output.fmt != RKNN_TENSOR_NCHW ||
            output.dims[1] != first_output.dims[1] || output.dims[2] == 0U ||
            output.dims[3] == 0U ||
            output.dims[2] >
                static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
            output.dims[3] >
                static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
            engine_->fail("detection output " + std::to_string(i) +
                          " has an invalid shape or layout");
        }

        // Contains exactly the stated # elements
        // An integral and equal horizontal/vertical stride
        const std::uint64_t expected_elements =
            static_cast<std::uint64_t>(output.dims[1]) *
            static_cast<std::uint64_t>(output.dims[2]) *
            static_cast<std::uint64_t>(output.dims[3]);
        if (output.n_elems != expected_elements ||
            engine_->model_height() % static_cast<int>(output.dims[2]) != 0 ||
            engine_->model_width() % static_cast<int>(output.dims[3]) != 0 ||
            engine_->model_height() / static_cast<int>(output.dims[2]) !=
                engine_->model_width() / static_cast<int>(output.dims[3])) {
            engine_->fail("detection output " + std::to_string(i) +
                          " has an invalid grid stride");
        }

        // Quantized detection outputs must be affine INT8 tensors
        if (is_quantized_ &&
            (output.type != RKNN_TENSOR_INT8 ||
             output.qnt_type != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC ||
             output.scale == 0.0F)) {
            engine_->fail(
                "quantized detection outputs must be affine INT8 tensors");
        }

        // Sums the grid cells for later keypoint validation
        total_anchors += static_cast<std::uint64_t>(output.dims[2]) *
                         static_cast<std::uint64_t>(output.dims[3]);
    }

    // Validates the keypoint head
    const rknn_tensor_attr &keypoints = output_attrs[3];
    // Quantized keypoint outputs must be FLOAT16 tensors
    if (is_quantized_ && keypoints.type != RKNN_TENSOR_FLOAT16) {
        engine_->fail("quantized model keypoint output must be FLOAT16");
    }
    // Contains exactly the stated # elements
    const std::uint64_t required_keypoint_values =
        total_anchors * static_cast<std::uint64_t>(kKeypointCount) *
        static_cast<std::uint64_t>(kKeypointValues);
    if (keypoints.n_elems != required_keypoint_values) {
        engine_->fail(
            "keypoint output does not match the decoded anchor layout");
    }

    // Validates the label count matches the model's class count
    if (engine_->labels().size() != static_cast<std::size_t>(num_classes_)) {
        engine_->fail("label count (" +
                      std::to_string(engine_->labels().size()) +
                      ") does not match model class count (" +
                      std::to_string(num_classes_) + ")");
    }
}

/**
 * @brief Runs inference on one frame
 * @param image Input image message
 * @param detections Output detections message
 * @return 0 on success, negative on failure
 */
int PoseBackend::infer(sensor_msgs::msg::Image::ConstSharedPtr image,
                       yolo_msgs::msg::Detections::SharedPtr   detections) {
    if (!engine_) {
        RCLCPP_ERROR(rclcpp::get_logger("librknn_yolo"),
                     "pose backend init() not called");
        return -1;
    }
    // Checks for null pointers
    if (image == nullptr || detections == nullptr) {
        RCLCPP_ERROR(engine_->logger(),
                     "infer received a null image or detections message");
        return -1;
    }

    Letterbox letterbox;
    if (engine_->run_frame(image, letterbox) != 0) {
        return -1;
    }

    // Runs the postprocessor to decode the RKNN outputs into detections
    const std::vector<Detection> *decoded = nullptr;
    try {
        decoded = &postprocessor_->decode(engine_->outputs(),
                                          engine_->output_attrs(), letterbox);
    } catch (const std::exception &exception) {
        engine_->release_outputs();
        RCLCPP_ERROR(engine_->logger(), "detection postprocessing failed: %s",
                     exception.what());
        return -1;
    } catch (...) {
        engine_->release_outputs();
        RCLCPP_ERROR(engine_->logger(), "detection postprocessing failed");
        return -1;
    }

    // Cleans up the RKNN output buffers and populates the detections message
    engine_->release_outputs();
    populate_results(*decoded, *image, detections);
    return 0;
}

void PoseBackend::populate_results(
    const std::vector<Detection> &decoded, const sensor_msgs::msg::Image &image,
    const yolo_msgs::msg::Detections::SharedPtr &detections) const {
    static_assert(kKeypointCount == yolo_msgs::msg::Keypoint::NUM_KEYPOINTS,
                  "YOLOv8 pose and yolo_msgs must use the same keypoint count");

    detections->image_meta.header = image.header;
    detections->image_meta.width  = static_cast<std::uint16_t>(image.width);
    detections->image_meta.height = static_cast<std::uint16_t>(image.height);

    detections->detections.clear();
    detections->detections.reserve(decoded.size());
    for (const Detection &result : decoded) {
        yolo_msgs::msg::Detection detection;
        detection.class_id = static_cast<std::uint16_t>(result.cls_id);
        detection.class_name =
            engine_->labels().at(static_cast<std::size_t>(result.cls_id));
        detection.confidence = result.score;
        detection.id         = yolo_msgs::msg::Detection::UNCHECKED;
        detection.bbox.xmin  = static_cast<std::uint16_t>(result.box.left);
        detection.bbox.ymin  = static_cast<std::uint16_t>(result.box.top);
        detection.bbox.xmax  = static_cast<std::uint16_t>(result.box.right);
        detection.bbox.ymax  = static_cast<std::uint16_t>(result.box.bottom);

        detection.keypoints.reserve(kKeypointCount);
        for (int i = 0; i < kKeypointCount; ++i) {
            yolo_msgs::msg::Keypoint keypoint;
            keypoint.id         = static_cast<std::uint8_t>(i);
            keypoint.x          = result.keypoints[i][0];
            keypoint.y          = result.keypoints[i][1];
            keypoint.confidence = result.keypoints[i][2];
            detection.keypoints.push_back(keypoint);
        }
        detections->detections.push_back(std::move(detection));
    }
}

} // namespace rknn_yolo
