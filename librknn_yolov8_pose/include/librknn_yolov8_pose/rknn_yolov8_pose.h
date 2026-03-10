#ifndef RKNN_YOLOV8_POSE_H
#define RKNN_YOLOV8_POSE_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <bboxes_kpoints_msgs/msg/bounding_boxes_keypoints.hpp>
#include <bboxes_kpoints_msgs/msg/bounding_box_keypoints.hpp>

#include "rknn_api.h"
#include "librknn_yolov8_pose/postprocess.h"

namespace rknn_yolo {

static constexpr int NUM_KEYPOINTS = 17;

class YoloV8Pose {
public:
    YoloV8Pose(const std::string& model_path, const std::string& label_path, rclcpp::Logger logger);
    ~YoloV8Pose();

    YoloV8Pose(const YoloV8Pose&) = delete;
    YoloV8Pose& operator=(const YoloV8Pose&) = delete;

    /**
     * @brief Infer bounding boxes with keypoints given an image message
     * @param img Input image message
     * @param bboxes Output bounding boxes with keypoints
     * @return 0 on success, negative on error
     */
    int infer(
        const sensor_msgs::msg::Image::ConstSharedPtr& img,
        bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints::SharedPtr bboxes);

private:
    rclcpp::Logger logger_;

    rknn_context rknn_ctx_ = 0;
    rknn_input_output_num io_num_{};
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;
    int model_width_ = 0;
    int model_height_ = 0;
    int model_channel_ = 0;
    bool is_quant_ = false;

    image_buffer_t src_image_{};
    image_buffer_t dst_image_{};
    object_detect_result_list od_results_{};

    void init_model(const std::string& model_path, const std::string& label_path);
    void release_model();
    void populate_results(
        const cv::Mat& mat,
        const std_msgs::msg::Header& header,
        bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints::SharedPtr bboxes);
};

} // namespace rknn_yolo

#endif // RKNN_YOLOV8_POSE_H
