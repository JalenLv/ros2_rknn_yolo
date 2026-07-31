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

#ifndef LIBRKNN_YOLO__POSE_POSTPROCESSOR_HPP_
#define LIBRKNN_YOLO__POSE_POSTPROCESSOR_HPP_

#include <vector>

#include "rknn_api.h"

#include "letterbox_preprocessor.hpp"
#include "postprocess_common.hpp"

namespace rknn_yolo {

/**
 * @brief One decoded pose detection in original-image pixel coordinates.
 */
struct Detection {
    Rect box;
    // 17 COCO keypoints as (x, y, confidence).
    float keypoints[17][3];
    float score;
    int cls_id;
};

/**
 * @brief Decodes raw pose-model outputs (three merged DFL box/score heads
 * plus the keypoint tensor) into detections, applying score thresholding and
 * per-class NMS.
 */
class PosePostprocessor {
public:
    /**
     * @brief Caches the model geometry and quantization info from the output
     * attributes and preallocates the decode buffers.
     */
    PosePostprocessor(
        int model_width,
        int model_height,
        const std::vector<rknn_tensor_attr>& output_attrs);

    /**
     * @brief Decodes one frame's outputs into detections mapped back to
     * original-image coordinates through the letterbox.
     * @return the decoded detections; the reference stays valid until the
     * next call.
     */
    const std::vector<Detection>& decode(
        const std::vector<rknn_output>& outputs,
        const std::vector<rknn_tensor_attr>& output_attrs,
        const Letterbox& letterbox);

    /** @return the class count implied by the model's output shape. */
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

#endif  // LIBRKNN_YOLO__POSE_POSTPROCESSOR_HPP_
