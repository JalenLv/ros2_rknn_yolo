#ifndef RKNN_YOLO_YOLO_NODE_HPP
#define RKNN_YOLO_YOLO_NODE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <bboxes_kpoints_msgs/msg/bounding_boxes_keypoints.hpp>
#include <librknn_yolov8_pose/rknn_yolov8_pose.h>

/**
 * @brief ROS 2 node wrapper for RKNN YOLO pose inference.
 *
 * This node subscribes to images, runs inference on the latest cached frame,
 * and publishes bounding boxes with keypoints.
 */
class YoloNode : public rclcpp::Node {
public:
    YoloNode(const std::string &node_name = "yolo_node");
    ~YoloNode();

private:
    void timer_callback();

    message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber;
    std::shared_ptr<message_filters::Cache<sensor_msgs::msg::Image>> image_cache;
    rclcpp::Publisher<bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints>::SharedPtr bbox_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<rknn_yolo::YoloV8Pose> yolo;
};

#endif // RKNN_YOLO_YOLO_NODE_HPP
