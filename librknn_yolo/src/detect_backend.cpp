#include "detect_backend.hpp"

#include "core_engine.hpp"
#include "detect_postprocessor.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace rknn_yolo {
namespace {

constexpr int kNumDetectionHeads = 3;

}  // namespace

DetectBackend::DetectBackend() = default;

DetectBackend::~DetectBackend() = default;

void DetectBackend::init(rclcpp::Node& node) {
    params_ = declare_common_params(node);
    engine_ = std::make_unique<CoreEngine>(
        params_.model_path, params_.label_path, node.get_logger());

    validate_outputs();
    engine_->finalize_io(params_.use_rga, is_quantized_);
    postprocessor_ = std::make_unique<DetectPostprocessor>(
        engine_->model_width(),
        engine_->model_height(),
        engine_->output_attrs());

    if (postprocessor_->num_classes() != num_classes_) {
        engine_->fail("postprocessor class count does not match the model");
    }

    RCLCPP_INFO(
        engine_->logger(),
        "model input: %dx%dx%d, classes: %d",
        engine_->model_width(),
        engine_->model_height(),
        engine_->model_channels(),
        num_classes_);
}

void DetectBackend::validate_outputs() {
    const std::vector<rknn_tensor_attr>& attrs = engine_->output_attrs();
    if (attrs.size() != 6U && attrs.size() != 9U) {
        std::string message =
            "detect backend expects 6 or 9 outputs, got " +
            std::to_string(attrs.size());
        if (attrs.size() == 4U) {
            message += "; looks like a pose model; use backend: pose";
        }
        engine_->fail(message);
    }

    outputs_per_branch_ =
        static_cast<int>(attrs.size() / kNumDetectionHeads);
    is_quantized_ =
        attrs.front().qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
        attrs.front().type != RKNN_TENSOR_FLOAT16;

    for (int branch = 0; branch < kNumDetectionHeads; ++branch) {
        const std::size_t box_index =
            static_cast<std::size_t>(branch * outputs_per_branch_);
        const std::size_t score_index = box_index + 1U;
        const rknn_tensor_attr& box = attrs[box_index];
        const rknn_tensor_attr& score = attrs[score_index];

        if (box.n_dims != 4U || box.dims[0] != 1U ||
            box.fmt != RKNN_TENSOR_NCHW || box.dims[1] == 0U ||
            box.dims[1] % 4U != 0U || box.dims[2] == 0U ||
            box.dims[3] == 0U ||
            box.dims[1] > static_cast<std::uint32_t>(
                              std::numeric_limits<int>::max()) ||
            box.dims[2] > static_cast<std::uint32_t>(
                              std::numeric_limits<int>::max()) ||
            box.dims[3] > static_cast<std::uint32_t>(
                              std::numeric_limits<int>::max())) {
            engine_->fail(
                "detect box output " + std::to_string(box_index) +
                " has an invalid shape or layout");
        }

        const int branch_dfl = static_cast<int>(box.dims[1] / 4U);
        if (branch == 0) {
            dfl_length_ = branch_dfl;
        } else if (branch_dfl != dfl_length_) {
            engine_->fail(
                "detect box outputs use inconsistent DFL lengths");
        }

        const std::uint64_t expected_box_elements =
            static_cast<std::uint64_t>(box.dims[1]) *
            static_cast<std::uint64_t>(box.dims[2]) *
            static_cast<std::uint64_t>(box.dims[3]);
        if (box.n_elems != expected_box_elements) {
            engine_->fail(
                "detect box output " + std::to_string(box_index) +
                " has an invalid element count");
        }

        if (score.n_dims != 4U || score.dims[0] != 1U ||
            score.fmt != RKNN_TENSOR_NCHW || score.dims[1] == 0U ||
            score.dims[2] != box.dims[2] ||
            score.dims[3] != box.dims[3] ||
            score.dims[1] > static_cast<std::uint32_t>(
                                std::numeric_limits<int>::max()) ||
            score.dims[1] > static_cast<std::uint32_t>(
                                std::numeric_limits<std::uint16_t>::max())) {
            engine_->fail(
                "detect score output " + std::to_string(score_index) +
                " has an invalid shape or layout");
        }

        const int branch_classes = static_cast<int>(score.dims[1]);
        if (branch == 0) {
            num_classes_ = branch_classes;
        } else if (branch_classes != num_classes_) {
            engine_->fail(
                "detect score outputs use inconsistent class counts");
        }

        const std::uint64_t expected_score_elements =
            static_cast<std::uint64_t>(score.dims[1]) *
            static_cast<std::uint64_t>(score.dims[2]) *
            static_cast<std::uint64_t>(score.dims[3]);
        if (score.n_elems != expected_score_elements) {
            engine_->fail(
                "detect score output " + std::to_string(score_index) +
                " has an invalid element count");
        }

        if (engine_->model_height() %
                    static_cast<int>(box.dims[2]) !=
                0 ||
            engine_->model_width() %
                    static_cast<int>(box.dims[3]) !=
                0 ||
            engine_->model_height() /
                    static_cast<int>(box.dims[2]) !=
                engine_->model_width() /
                    static_cast<int>(box.dims[3])) {
            engine_->fail(
                "detect branch " + std::to_string(branch) +
                " has an invalid grid stride");
        }

        if (outputs_per_branch_ == 3) {
            const std::size_t score_sum_index = box_index + 2U;
            const rknn_tensor_attr& score_sum = attrs[score_sum_index];
            if (score_sum.n_dims != 4U ||
                score_sum.dims[0] != 1U ||
                score_sum.fmt != RKNN_TENSOR_NCHW ||
                score_sum.dims[1] != 1U ||
                score_sum.dims[2] != box.dims[2] ||
                score_sum.dims[3] != box.dims[3] ||
                score_sum.n_elems !=
                    static_cast<std::uint64_t>(box.dims[2]) *
                        static_cast<std::uint64_t>(box.dims[3])) {
                engine_->fail(
                    "detect score-sum output " +
                    std::to_string(score_sum_index) +
                    " has an invalid shape or layout");
            }
        }

        if (is_quantized_) {
            for (int output = 0; output < outputs_per_branch_; ++output) {
                const std::size_t index = box_index +
                    static_cast<std::size_t>(output);
                const rknn_tensor_attr& attr = attrs[index];
                if (attr.type != RKNN_TENSOR_INT8 ||
                    attr.qnt_type !=
                        RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC ||
                    attr.scale == 0.0F) {
                    engine_->fail(
                        "quantized detect output " +
                        std::to_string(index) +
                        " must be an affine INT8 tensor with nonzero scale");
                }
            }
        }
    }

    if (engine_->labels().size() !=
        static_cast<std::size_t>(num_classes_)) {
        engine_->fail(
            "label count (" +
            std::to_string(engine_->labels().size()) +
            ") does not match model class count (" +
            std::to_string(num_classes_) + ")");
    }
}

int DetectBackend::infer(
    sensor_msgs::msg::Image::ConstSharedPtr image,
    yolo_msgs::msg::Detections::SharedPtr detections) {
    if (!engine_) {
        RCLCPP_ERROR(
            rclcpp::get_logger("librknn_yolo"),
            "detect backend init() not called");
        return -1;
    }
    if (image == nullptr || detections == nullptr) {
        RCLCPP_ERROR(
            engine_->logger(),
            "infer received a null image or detections message");
        return -1;
    }

    Letterbox letterbox;
    if (engine_->run_frame(image, letterbox) != 0) {
        return -1;
    }

    const std::vector<DetectedBox>* decoded = nullptr;
    try {
        decoded = &postprocessor_->decode(
            engine_->outputs(), engine_->output_attrs(), letterbox);
    } catch (const std::exception& exception) {
        engine_->release_outputs();
        RCLCPP_ERROR(
            engine_->logger(),
            "detection postprocessing failed: %s",
            exception.what());
        return -1;
    } catch (...) {
        engine_->release_outputs();
        RCLCPP_ERROR(
            engine_->logger(), "detection postprocessing failed");
        return -1;
    }

    engine_->release_outputs();
    populate_results(*decoded, *image, detections);
    return 0;
}

void DetectBackend::populate_results(
    const std::vector<DetectedBox>& decoded,
    const sensor_msgs::msg::Image& image,
    const yolo_msgs::msg::Detections::SharedPtr& detections) const {
    detections->image_meta.header = image.header;
    detections->image_meta.width =
        static_cast<std::uint16_t>(image.width);
    detections->image_meta.height =
        static_cast<std::uint16_t>(image.height);

    detections->detections.clear();
    detections->detections.reserve(decoded.size());
    for (const DetectedBox& result : decoded) {
        yolo_msgs::msg::Detection detection;
        detection.class_id =
            static_cast<std::uint16_t>(result.cls_id);
        detection.class_name = engine_->labels().at(
            static_cast<std::size_t>(result.cls_id));
        detection.confidence = result.score;
        detection.id = yolo_msgs::msg::Detection::UNCHECKED;
        detection.bbox.xmin =
            static_cast<std::uint16_t>(result.box.left);
        detection.bbox.ymin =
            static_cast<std::uint16_t>(result.box.top);
        detection.bbox.xmax =
            static_cast<std::uint16_t>(result.box.right);
        detection.bbox.ymax =
            static_cast<std::uint16_t>(result.box.bottom);
        detections->detections.push_back(std::move(detection));
    }
}

}  // namespace rknn_yolo
