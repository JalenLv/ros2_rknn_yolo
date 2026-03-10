#ifndef RKNN_YOLOV8_POSE_POSTPROCESS_H
#define RKNN_YOLOV8_POSE_POSTPROCESS_H

#include <stdint.h>
#include <stdbool.h>

#include "rknn_api.h"
#include "librknn_yolov8_pose/common.h"
#include "librknn_yolov8_pose/image_utils.h"

#define OBJ_CLASS_NUM 1
#define OBJ_NUMB_MAX_SIZE 128
#define NMS_THRESH 0.4
#define BOX_THRESH 0.5

typedef struct {
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs;
    rknn_tensor_attr* output_attrs;
    int model_channel;
    int model_width;
    int model_height;
    bool is_quant;
} rknn_app_context_t;

typedef struct {
    image_rect_t box;
    float keypoints[17][3];//keypoints x,y,conf
    float prop;
    int cls_id;
} object_detect_result;

typedef struct {
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

#ifdef __cplusplus
extern "C" {
#endif

int init_post_process(const char* label_path);
void deinit_post_process();
char* coco_cls_to_name(int cls_id);
int post_process(
    rknn_app_context_t* app_ctx,
    void* outputs,
    letterbox_t* letter_box,
    float conf_threshold,
    float nms_threshold,
    object_detect_result_list* od_results);

#ifdef __cplusplus
}
#endif

#endif // RKNN_YOLOV8_POSE_POSTPROCESS_H
