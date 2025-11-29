#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <bboxes_kpoints_msgs/msg/bounding_boxes_keypoints.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class YoloVisualizerPose : public rclcpp::Node {
public:
    YoloVisualizerPose();
    ~YoloVisualizerPose() {};

private:
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints> bboxes_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints>> sync_;

    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
        const bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints::ConstSharedPtr bboxes_msg
    );
};

YoloVisualizerPose::YoloVisualizerPose() : Node("yolo_visualizer_pose") {
    // Subscribe to topics
    // Use rmw_qos_profile_sensor_data for image to match best effort if needed, 
    // but TimeSynchronizer requires exact matching policies usually. 
    // However, message_filters subscribers take a QoS.
    image_sub_.subscribe(this, "/camera/color/image_raw", rmw_qos_profile_sensor_data);
    bboxes_sub_.subscribe(this, "/bounding_boxes_keypoints", rmw_qos_profile_default);

    // Synchronize topics
    // Queue size 10
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints>>(image_sub_, bboxes_sub_, 10);
    sync_->registerCallback(std::bind(&YoloVisualizerPose::callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "YoloVisualizerPose initialized.");
}

void YoloVisualizerPose::callback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
    const bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints::ConstSharedPtr bboxes_msg
) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;

    // Skeleton indices from main.cc (1-based)
    const int skeleton[38] = {16, 14, 14, 12, 17, 15, 15, 13, 12, 13, 6, 12, 7, 13, 6, 7, 6, 8, 
                7, 9, 8, 10, 9, 11, 2, 3, 1, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7}; 

    // Draw bounding boxes and skeletons
    for (const auto& bbox : bboxes_msg->bounding_boxes)
    {
        // Draw bounding box (Blue)
        cv::rectangle(img,
            cv::Point(bbox.xmin, bbox.ymin), cv::Point(bbox.xmax, bbox.ymax),
            cv::Scalar(255, 0, 0), 3
        );

        // Draw label (Red)
        std::string label = bbox.class_id + " " + std::to_string(bbox.probability * 100).substr(0, 4) + "%";
        cv::putText(img,
            label, cv::Point(bbox.xmin, bbox.ymin + 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0, 0, 255), 2
        );

        // Draw skeleton (Orange)
        for (int i = 0; i < 38; i += 2) {
            int idx1 = skeleton[i] - 1;
            int idx2 = skeleton[i+1] - 1;

            if (idx1 < (int)bbox.keypoints.size() && idx2 < (int)bbox.keypoints.size()) {
                const auto &kp1 = bbox.keypoints[idx1];
                const auto &kp2 = bbox.keypoints[idx2];

                // Draw line
                cv::line(img,
                    cv::Point(kp1.x, kp1.y), cv::Point(kp2.x, kp2.y),
                    cv::Scalar(0, 165, 255), 3
                );
            }
        }

        // Draw keypoints (Yellow)
        for (const auto& kp : bbox.keypoints) {
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
