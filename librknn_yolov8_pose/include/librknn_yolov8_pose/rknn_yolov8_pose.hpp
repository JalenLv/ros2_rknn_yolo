#ifndef LIBRKNN_YOLOV8_POSE__RKNN_YOLOV8_POSE_HPP_
#define LIBRKNN_YOLOV8_POSE__RKNN_YOLOV8_POSE_HPP_

#include <memory>
#include <string>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detections.hpp>

namespace rknn_yolo {

class YoloV8Pose {
  public:
    YoloV8Pose(const std::string &model_path, const std::string &label_path,
               rclcpp::Logger logger);
    ~YoloV8Pose();

    YoloV8Pose(const YoloV8Pose &) = delete;
    YoloV8Pose &operator=(const YoloV8Pose &) = delete;

    /**
     * Run pose inference for one image.
     *
     * \return 0 on success, or a negative value when preprocessing or RKNN
     * execution fails.
     */
    int infer(const sensor_msgs::msg::Image::ConstSharedPtr &image,
              yolo_msgs::msg::Detections::SharedPtr detections);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rknn_yolo

#endif // LIBRKNN_YOLOV8_POSE__RKNN_YOLOV8_POSE_HPP_
