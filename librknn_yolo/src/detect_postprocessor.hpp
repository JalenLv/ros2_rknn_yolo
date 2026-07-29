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

#ifndef LIBRKNN_YOLO__DETECT_POSTPROCESSOR_HPP_
#define LIBRKNN_YOLO__DETECT_POSTPROCESSOR_HPP_

#include "letterbox_preprocessor.hpp"
#include "postprocess_common.hpp"

#include <rknn_api.h>

#include <vector>

namespace rknn_yolo {

/**
 * @brief One decoded box detection in original-image pixel coordinates.
 */
struct DetectedBox {
    Rect box;
    float score;
    int cls_id;
};

/**
 * @brief Decodes raw detect-model outputs (per-branch DFL box, class score,
 * and optional score-sum tensors) into boxes, applying score thresholding
 * and per-class NMS.
 */
class DetectPostprocessor {
public:
    /**
     * @brief Caches the model geometry and quantization info from the output
     * attributes and preallocates the decode buffers.
     */
    DetectPostprocessor(
        int model_width,
        int model_height,
        const std::vector<rknn_tensor_attr>& output_attrs);

    /**
     * @brief Decodes one frame's outputs into boxes mapped back to
     * original-image coordinates through the letterbox.
     * @return the decoded boxes; the reference stays valid until the
     * next call.
     */
    const std::vector<DetectedBox>& decode(
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
    int dfl_length_;
    int outputs_per_branch_;

    std::vector<float> candidate_boxes_;
    std::vector<float> candidate_scores_;
    std::vector<int> candidate_class_ids_;
    std::vector<int> sort_indices_;
    std::vector<int> merge_indices_;
    std::vector<float> dfl_scratch_;
    std::vector<DetectedBox> results_;
};

}  // namespace rknn_yolo

#endif  // LIBRKNN_YOLO__DETECT_POSTPROCESSOR_HPP_
