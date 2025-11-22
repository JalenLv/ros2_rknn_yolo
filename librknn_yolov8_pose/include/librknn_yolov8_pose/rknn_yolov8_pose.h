#ifndef RKNN_YOLOV8_POSE_H
#define RKNN_YOLOV8_POSE_H

#include "yolov8-pose.h"
#include "postprocess.h"

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bboxes_kpoints_msgs/msg/bounding_boxes_keypoints.hpp>
#include <bboxes_kpoints_msgs/msg/bounding_box_keypoints.hpp>

#ifndef MODEL_PATH
#define MODEL_PATH "/home/orangepi/ros2_ws/src/rknn_yolo/librknn_yolov8_pose/model/yolov8_pose.rknn"
#endif

namespace rknn_yolo {

class YoloV8Pose {
public:
    YoloV8Pose();
    ~YoloV8Pose();

    /**
     * @brief Infer bounding boxes with keypoints given an image message
     * @param img Input image message
     * @param bboxes Output bounding boxes with keypoints
     * @return 0 on success, negative on error
     */
    int infer(
        const sensor_msgs::msg::Image::ConstPtr img,
        const bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints::SharedPtr bboxes
    );

private:
    const int skeleton[38] = {
        16, 14, 14, 12, 17, 15, 15, 13, 12, 13, 6, 12, 7, 13, 6, 7, 6,
        8, 7, 9, 8, 10, 9, 11, 2, 3, 1, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7
    };

    rknn_app_context_t rknn_app_ctx;
    image_buffer_t src_image;
    object_detect_result_list od_results;

    /**
     * @brief Convert cv::Mat to image_buffer_t
     * @param mat Input OpenCV Mat
     * @return 0 on success, negative on error
     */
    int cvmat_to_image_buffer(const cv::Mat &mat);
};

} // namespace rknn_yolo

#endif // RKNN_YOLOV8_POSE_H