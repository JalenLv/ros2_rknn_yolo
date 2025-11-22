#ifndef RKNN_YOLO_YOLO_NODE_HPP
#define RKNN_YOLO_YOLO_NODE_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <bboxes_kpoints_msgs/msg/bounding_boxes_keypoints.hpp>
#include <bboxes_kpoints_msgs/msg/bounding_box_keypoints.hpp>
#include <librknn_yolov8_pose/rknn_yolov8_pose.h>

/**
 * @brief YoloNode class for ROS2 integration
 * 
 * This class subscribes to image topics, performs YOLO inference,
 * and publishes bounding box results.
 * 
 * Maximum FPS: 30, which is controlled via a timer.
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

    rknn_yolo::YoloV8Pose yolo;
};

#endif // RKNN_YOLO_YOLO_NODE_HPP