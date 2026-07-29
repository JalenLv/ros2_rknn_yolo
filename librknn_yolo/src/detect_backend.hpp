#ifndef LIBRKNN_YOLO__DETECT_BACKEND_HPP_
#define LIBRKNN_YOLO__DETECT_BACKEND_HPP_

#include "backend_params.hpp"

#include <librknn_yolo/yolo.hpp>

#include <memory>
#include <vector>

namespace rknn_yolo {

class CoreEngine;
class DetectPostprocessor;
struct DetectedBox;

/**
 * @brief YOLOv8/YOLO11 detect backend for models exported with split
 * per-branch outputs and sigmoid folded into the graph.
 */
class DetectBackend final : public Yolo {
  public:
    /**
     * @brief Trivial constructor; all setup happens in `init()`.
     */
    DetectBackend();
    /**
     * @brief Default destructor; members tear down the engine and
     * postprocessor.
     */
    ~DetectBackend() override;

    /**
     * @brief Declares the common parameters, creates the core engine,
     * validates the detect output layout, and prepares the postprocessor.
     * @throws std::runtime_error on missing parameters or a model mismatch.
     */
    void init(rclcpp::Node &node) override;
    /**
     * @brief Runs object detection on one frame and fills `detections` with
     * boxes; keypoints are left empty.
     * @return 0 on success, negative on failure.
     */
    int  infer(sensor_msgs::msg::Image::ConstSharedPtr image,
               yolo_msgs::msg::Detections::SharedPtr   detections) override;

  private:
    /**
     * @brief Validates the model's output tensor attributes and determine the
     * inference logic depends on the metadata.
     *
     * Expects 6 or 9 outputs, one group per detection head (80x80, 40x40,
     * 20x20 grids at 640x640): a box branch of shape [1, 4 * dfl_len, H, W],
     * a score branch of shape [1, num_classes, H, W], and, on nine-output
     * models, a score-sum branch of shape [1, 1, H, W] used for early
     * filtering.
     */
    void validate_outputs();
    /**
     * @brief Converts decoded boxes into the output message, attaching the
     * image meta; keypoints are left empty.
     */
    void populate_results(
        const std::vector<DetectedBox>              &decoded,
        const sensor_msgs::msg::Image               &image,
        const yolo_msgs::msg::Detections::SharedPtr &detections) const;

    CommonParams                         params_;
    std::unique_ptr<CoreEngine>          engine_;
    std::unique_ptr<DetectPostprocessor> postprocessor_;

    int  num_classes_{0};
    int  dfl_length_{0};
    int  outputs_per_branch_{0};
    bool is_quantized_{false};
};

} // namespace rknn_yolo

#endif // LIBRKNN_YOLO__DETECT_BACKEND_HPP_
