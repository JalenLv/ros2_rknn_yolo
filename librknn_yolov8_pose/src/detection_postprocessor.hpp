// Copyright (c) 2024 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIBRKNN_YOLOV8_POSE__DETECTION_POSTPROCESSOR_HPP_
#define LIBRKNN_YOLOV8_POSE__DETECTION_POSTPROCESSOR_HPP_

#include <vector>

#include "rknn_api.h"

#include "letterbox_preprocessor.hpp"

namespace rknn_yolo {

struct Rect {
    int left;
    int top;
    int right;
    int bottom;
};

struct Detection {
    Rect box;
    float keypoints[17][3];
    float score;
    int cls_id;
};

class DetectionPostprocessor {
public:
    DetectionPostprocessor(
        int model_width,
        int model_height,
        const std::vector<rknn_tensor_attr>& output_attrs);

    const std::vector<Detection>& decode(
        const std::vector<rknn_output>& outputs,
        const std::vector<rknn_tensor_attr>& output_attrs,
        const Letterbox& letterbox);

    int num_classes() const noexcept;

private:
    int model_width_;
    int model_height_;
    int num_classes_;
    bool is_quantized_;

    std::vector<float> candidate_boxes_;
    std::vector<float> candidate_scores_;
    std::vector<int> candidate_class_ids_;
    std::vector<int> sort_indices_;
    std::vector<int> merge_indices_;
    std::vector<Detection> results_;
};

}  // namespace rknn_yolo

#endif  // LIBRKNN_YOLOV8_POSE__DETECTION_POSTPROCESSOR_HPP_
