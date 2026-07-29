#ifndef LIBRKNN_YOLO__POSE_BACKEND_HPP_
#define LIBRKNN_YOLO__POSE_BACKEND_HPP_

#include "backend_params.hpp"
#include "core_engine.hpp"
#include "pose_postprocessor.hpp"

#include <librknn_yolo/yolo.hpp>

#include <memory>
#include <vector>

namespace rknn_yolo {

/**
 * @brief YOLOv8 pose backend: three merged detection heads plus a keypoint
 * head, decoded into detections carrying 17 COCO keypoints each.
 */
class PoseBackend final : public Yolo {
  public:
    /**
     * @brief Trivial constructor; all setup happens in `init()`.
     */
    PoseBackend();
    ~PoseBackend() override;

    /**
     * @brief Declares the common parameters, creates the core engine,
     * validates the pose output layout, and prepares the postprocessor.
     * @throws std::runtime_error on missing parameters, file io errors, empty
     * label file, RKNN api errors, or invalid model IO.
     */
    void init(rclcpp::Node &node) override;

    /**
     * @brief Runs pose inference on one frame and fills `detections` with
     * boxes and keypoints.
     */
    int infer(sensor_msgs::msg::Image::ConstSharedPtr image,
              yolo_msgs::msg::Detections::SharedPtr   detections) override;

  private:
    /**
     * @brief Validates the model's output tensor attributes and determine the
     * inference logic depends on the metadata.
     *
     * - output 0: small-obj detection head of shape [1, 64 + 1 (num_classes),
     * 80, 80]
     * - output 1: medium-obj detection head of shape [1, 64 + 1 (num_classes),
     * 40, 40]
     * - output 2: large-obj detection head of shape [1, 64 + 1 (num_classes),
     * 20, 20]
     * - output 3: keypoints for all three heads of shape [17 (num_kpts), 3 (x +
     * y + conf), 8400 (80*80 + 40*40 + 20*20)]
     */
    void validate_outputs();
    /**
     * @brief Converts decoded detections into the output message, attaching
     * the image meta and the 17 keypoints per detection.
     */
    void populate_results(
        const std::vector<Detection>                &decoded,
        const sensor_msgs::msg::Image               &image,
        const yolo_msgs::msg::Detections::SharedPtr &detections) const;

    CommonParams                       params_;
    std::unique_ptr<CoreEngine>        engine_;
    std::unique_ptr<PosePostprocessor> postprocessor_;

    int  num_classes_{0};
    bool is_quantized_{false};
};

} // namespace rknn_yolo

#endif // LIBRKNN_YOLO__POSE_BACKEND_HPP_
