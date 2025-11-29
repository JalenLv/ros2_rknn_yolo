#include "rknn_yolo/yolo_node.hpp"

#include <rclcpp/qos.hpp>

YoloNode::YoloNode(const std::string &node_name) : Node(node_name), yolo() {
    RCLCPP_INFO(this->get_logger(), "Initializing YoloNode...");

    // Subscribe to image topic using message_filters
    this->declare_parameter("image_topic", "/image_raw");
    std::string image_topic = this->get_parameter("image_topic").as_string();
    image_subscriber.subscribe(this, image_topic, rmw_qos_profile_sensor_data);
    
    // Create a cache with a history size of 10
    image_cache = std::make_shared<message_filters::Cache<sensor_msgs::msg::Image>>(image_subscriber, 10);

    // Publisher for bounding boxes with keypoints
    this->declare_parameter("bbox_kpoints_topic", "/bounding_boxes_keypoints");
    std::string bbox_kpoints_topic = this->get_parameter("bbox_kpoints_topic").as_string();
    bbox_publisher = this->create_publisher<bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints>(
        bbox_kpoints_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
    );

    // Timer for fps control
    this->declare_parameter("fps", 15);
    int fps = this->get_parameter("fps").as_int();
    timer = this->create_wall_timer(
        std::chrono::milliseconds(1000 / fps),
        std::bind(&YoloNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "YoloNode initialized.");
}

YoloNode::~YoloNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down YoloNode...");
    RCLCPP_INFO(this->get_logger(), "YoloNode shut down.");
}

void YoloNode::timer_callback() {
    // Get the latest image from the cache before the current time
    auto msg = image_cache->getElemBeforeTime(this->now());

    if (!msg) {
        // No image available yet
        return;
    }

    // Create output message
    auto bboxes_msg = std::make_shared<bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints>();
    bboxes_msg->header = msg->header;

    // Run inference
    if (yolo.infer(msg, bboxes_msg) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Inference failed");
        return;
    }

    // Publish results
    bbox_publisher->publish(*bboxes_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloNode>());
    rclcpp::shutdown();
    return 0;
}
