#ifndef LIBRKNN_YOLO__YOLO_HPP_
#define LIBRKNN_YOLO__YOLO_HPP_

#include <memory>

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detections.hpp>

namespace rknn_yolo {

/**
 * @brief Abstract class for YOLO backends.
 */
class Yolo {
  public:
    virtual ~Yolo() = default;

    /**
     * @brief Initialize the YOLO backend.
     * @param node The ROS 2 node used for parameter declaration and logging.
     * @throws std::runtime_error and logs the corresponding error.
     */
    virtual void init(rclcpp::Node &node) = 0;

    /**
     * @brief Run inference for one image.
     * @param image The input image for inference.
     * @param detections The output detections.
     * @return 0 on success, negative on failure.
     */
    virtual int infer(sensor_msgs::msg::Image::ConstSharedPtr image,
                      yolo_msgs::msg::Detections::SharedPtr   detections) = 0;
};

/**
 * @brief Factory function to create a YOLO backend
 * @param node used for parameter declaration and backend selection
 * @return A unique pointer to the created YOLO backend
 * @throws std::invalid_argument if the specified backend is not supported;
 *         std::runtime_error if required parameters are missing or invalid.
 */
std::unique_ptr<Yolo> make_yolo(rclcpp::Node &node);

} // namespace rknn_yolo

#endif // LIBRKNN_YOLO__YOLO_HPP_
