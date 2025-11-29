#include <librknn_yolov8_pose/rknn_yolov8_pose.h>

#include <string>
#include <cv_bridge/cv_bridge.h>

rknn_yolo::YoloV8Pose::YoloV8Pose() {
    rknn_app_ctx = {0};
    src_image = {0};

    init_post_process();

    if (init_yolov8_pose_model(MODEL_PATH, &rknn_app_ctx)) {
        printf("Failed to initialize YOLOv8 Pose model.\n");
        exit(1);
    }
}

rknn_yolo::YoloV8Pose::~YoloV8Pose() {
    deinit_post_process();

    if (release_yolov8_pose_model(&rknn_app_ctx)) {
        printf("Failed to release YOLOv8 Pose model.\n");
        exit(1);
    }

    if (src_image.virt_addr != NULL)
        free(src_image.virt_addr);
}

int rknn_yolo::YoloV8Pose::infer(
    const sensor_msgs::msg::Image::ConstSharedPtr img,
    const bboxes_kpoints_msgs::msg::BoundingBoxesKeypoints::SharedPtr bboxes
) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception &e) {
        printf("cv_bridge exception: %s\n", e.what());
        return -1;
    }
    cv::Mat mat = cv_ptr->image;

    // Convert cv::Mat to image_buffer_t
    if (cvmat_to_image_buffer(mat) != 0) {
        printf("Failed to convert cv::Mat to image_buffer_t.\n");
        return -1;
    }

    // Run inference
    if (inference_yolov8_pose_model(&rknn_app_ctx, &src_image, &od_results)) {
        printf("Inference failed.\n");
        return -1;
    }

    // Populate bboxes with results
    bboxes->header = img->header;
    bboxes->image_header = img->header;
    bboxes->bounding_boxes.clear();
    for (int i = 0; i < od_results.count; i++) {
        object_detect_result* det_result = &(od_results.results[i]);
        bboxes_kpoints_msgs::msg::BoundingBoxKeypoints bbox;

        bbox.probability = det_result->prop;
        bbox.xmin = det_result->box.left;
        bbox.ymin = det_result->box.top;
        bbox.xmax = det_result->box.right;
        bbox.ymax = det_result->box.bottom;
        bbox.id = 0;
        bbox.img_width = mat.cols;
        bbox.img_height = mat.rows;
        bbox.class_id_int = det_result->cls_id;
        bbox.class_id = std::string(coco_cls_to_name(bbox.class_id_int));

        for (int j = 0; j < 17; ++j) {
            bboxes_kpoints_msgs::msg::Keypoint keypoint;
            keypoint.x = det_result->keypoints[j][0];
            keypoint.y = det_result->keypoints[j][1];
            keypoint.score = det_result->keypoints[j][2];
            bbox.keypoints.push_back(keypoint);
        }

        bboxes->bounding_boxes.push_back(bbox);
    }

    return 0;
}

int rknn_yolo::YoloV8Pose::cvmat_to_image_buffer(const cv::Mat &mat) {
    src_image.width = mat.cols;
    src_image.height = mat.rows;
    src_image.width_stride = mat.cols;
    src_image.height_stride = mat.rows;

    // Determine format based on OpenCV Mat type
    // Since we requested RGB8 from cv_bridge, we expect 3 channels
    if (mat.channels() != 3) {
        return -1; // Unsupported format
    }
    src_image.format = IMAGE_FORMAT_RGB888;

    size_t required_size = mat.total() * mat.elemSize();
    // Only realloc if buffer is NULL or too small
    if (src_image.virt_addr != NULL || src_image.size < required_size) {
        if (src_image.virt_addr != NULL) {
            free(src_image.virt_addr);
        }
        src_image.virt_addr = (unsigned char *)malloc(required_size);
        if (src_image.virt_addr == nullptr) {
           return -1; // Memory allocation failed
        }
        src_image.size = required_size;
    }

    // If OpenCV Mat is BGR and we need RGB, convert it
    // Since we requested RGB8 from cv_bridge, the mat is already in RGB format.
    // We can just copy the data.
    memcpy(src_image.virt_addr, mat.data, required_size);

    // No file descriptor for non-DMA memory
    src_image.fd = -1;

    return 0;
}
