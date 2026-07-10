#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/message_traits.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <yolo_msgs/msg/detections.hpp>
#include <yolo_msgs/msg/keypoint.hpp>

namespace message_filters {
namespace message_traits {

template<>
struct TimeStamp<yolo_msgs::msg::Detections> {
    static rclcpp::Time value(const yolo_msgs::msg::Detections& msg) {
        return rclcpp::Time(msg.image_meta.header.stamp, RCL_ROS_TIME);
    }
};

} // namespace message_traits
} // namespace message_filters

namespace {

using Keypoint = yolo_msgs::msg::Keypoint;

constexpr std::size_t kKeypointCount =
    static_cast<std::size_t>(Keypoint::NUM_KEYPOINTS);

constexpr std::array<std::pair<uint8_t, uint8_t>, 19> kSkeleton = {{
    {Keypoint::LEFT_ANKLE, Keypoint::LEFT_KNEE},
    {Keypoint::LEFT_KNEE, Keypoint::LEFT_HIP},
    {Keypoint::RIGHT_ANKLE, Keypoint::RIGHT_KNEE},
    {Keypoint::RIGHT_KNEE, Keypoint::RIGHT_HIP},
    {Keypoint::LEFT_HIP, Keypoint::RIGHT_HIP},
    {Keypoint::LEFT_SHOULDER, Keypoint::LEFT_HIP},
    {Keypoint::RIGHT_SHOULDER, Keypoint::RIGHT_HIP},
    {Keypoint::LEFT_SHOULDER, Keypoint::RIGHT_SHOULDER},
    {Keypoint::LEFT_SHOULDER, Keypoint::LEFT_ELBOW},
    {Keypoint::RIGHT_SHOULDER, Keypoint::RIGHT_ELBOW},
    {Keypoint::LEFT_ELBOW, Keypoint::LEFT_WRIST},
    {Keypoint::RIGHT_ELBOW, Keypoint::RIGHT_WRIST},
    {Keypoint::LEFT_EYE, Keypoint::RIGHT_EYE},
    {Keypoint::NOSE, Keypoint::LEFT_EYE},
    {Keypoint::NOSE, Keypoint::RIGHT_EYE},
    {Keypoint::LEFT_EYE, Keypoint::LEFT_EAR},
    {Keypoint::RIGHT_EYE, Keypoint::RIGHT_EAR},
    {Keypoint::LEFT_EAR, Keypoint::LEFT_SHOULDER},
    {Keypoint::RIGHT_EAR, Keypoint::RIGHT_SHOULDER},
}};

} // namespace

class YoloVisualizerPose : public rclcpp::Node {
public:
    YoloVisualizerPose();
    ~YoloVisualizerPose() {};

private:
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<yolo_msgs::msg::Detections> detections_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image,
        yolo_msgs::msg::Detections>> sync_;

    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
        const yolo_msgs::msg::Detections::ConstSharedPtr detections_msg
    );
};

YoloVisualizerPose::YoloVisualizerPose() : Node("yolo_visualizer_pose") {
    // Subscribe to topics
    // Use rmw_qos_profile_sensor_data for image to match best effort if needed, 
    // but TimeSynchronizer requires exact matching policies usually. 
    // However, message_filters subscribers take a QoS.
    this->declare_parameter("image_topic", "/image_raw");
    image_sub_.subscribe(this, this->get_parameter("image_topic").as_string(), rmw_qos_profile_sensor_data);
    this->declare_parameter("bbox_kpoints_topic", "/bounding_boxes_keypoints");
    detections_sub_.subscribe(this, this->get_parameter("bbox_kpoints_topic").as_string(), rmw_qos_profile_default);

    // Synchronize topics
    // Queue size 10
    sync_ = std::make_shared<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image,
        yolo_msgs::msg::Detections>>(image_sub_, detections_sub_, 10);
    sync_->registerCallback(std::bind(&YoloVisualizerPose::callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "YoloVisualizerPose initialized.");
}

void YoloVisualizerPose::callback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
    const yolo_msgs::msg::Detections::ConstSharedPtr detections_msg
) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;

    // Draw bounding boxes and skeletons
    for (const auto& detection : detections_msg->detections)
    {
        const auto& bbox = detection.bbox;

        // Draw bounding box (Blue)
        cv::rectangle(img,
            cv::Point(bbox.xmin, bbox.ymin), cv::Point(bbox.xmax, bbox.ymax),
            cv::Scalar(255, 0, 0), 3
        );

        // Draw label (Red)
        std::string label = detection.class_name + " " +
            std::to_string(detection.confidence * 100).substr(0, 4) + "%";
        cv::putText(img,
            label, cv::Point(bbox.xmin, bbox.ymin + 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0, 0, 255), 2
        );

        if (detection.id != yolo_msgs::msg::Detection::UNCHECKED) {
            std::string id_text = "ID: " + std::to_string(detection.id);
            cv::putText(img,
                id_text, cv::Point(bbox.xmin, bbox.ymax - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 2
            );
        }

        std::array<const Keypoint*, kKeypointCount> keypoints_by_id{};
        for (const auto& keypoint : detection.keypoints) {
            const std::size_t id = static_cast<std::size_t>(keypoint.id);
            if (id < keypoints_by_id.size()) {
                keypoints_by_id[id] = &keypoint;
            }
        }

        // Draw skeleton (Orange)
        for (const auto& [first_id, second_id] : kSkeleton) {
            const Keypoint* first = keypoints_by_id[first_id];
            const Keypoint* second = keypoints_by_id[second_id];
            if (first == nullptr || second == nullptr) {
                continue;
            }

            cv::line(img,
                cv::Point(first->x, first->y),
                cv::Point(second->x, second->y),
                cv::Scalar(0, 165, 255), 3
            );
        }

        // Draw keypoints (Yellow)
        for (const auto& kp : detection.keypoints) {
            cv::circle(img,
                cv::Point(kp.x, kp.y), 3,
                cv::Scalar(0, 255, 255), cv::FILLED
            );
        }
    }

    cv::imshow("YOLOv8 Pose", img);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloVisualizerPose>());
    rclcpp::shutdown();
    return 0;
}
