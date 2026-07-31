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

#include "detect_postprocessor.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rknn_yolo {
namespace {

constexpr int kNumDetectionHeads = 3;
constexpr std::size_t kMaxDetections = 128;
constexpr float kBoxThreshold = 0.25F;
constexpr float kNmsThreshold = 0.45F;

template<typename T, typename BoxDequantizer, typename ScoreDequantizer>
int decode_branch(
    const T* box_tensor,
    const T* score_tensor,
    const T* score_sum_tensor,
    int grid_height,
    int grid_width,
    int stride,
    int dfl_length,
    int num_classes,
    T score_threshold,
    T score_sum_threshold,
    const BoxDequantizer& dequantize_box,
    const ScoreDequantizer& dequantize_score,
    std::vector<float>& dfl_scratch,
    std::vector<float>& boxes,
    std::vector<float>& scores,
    std::vector<int>& class_ids) {
    int valid_count = 0;
    const int grid_size = grid_height * grid_width;

    for (int row = 0; row < grid_height; ++row) {
        for (int column = 0; column < grid_width; ++column) {
            const int cell = row * grid_width + column;
            if (score_sum_tensor != nullptr &&
                score_sum_tensor[cell] < score_sum_threshold) {
                continue;
            }

            int max_class_id = -1;
            T max_score = score_threshold;
            for (int class_id = 0; class_id < num_classes; ++class_id) {
                const T score =
                    score_tensor[class_id * grid_size + cell];
                if (score > max_score) {
                    max_score = score;
                    max_class_id = class_id;
                }
            }

            if (max_class_id == -1) {
                continue;
            }

            for (int channel = 0; channel < 4 * dfl_length; ++channel) {
                dfl_scratch[static_cast<std::size_t>(channel)] =
                    dequantize_box(
                        box_tensor[channel * grid_size + cell]);
            }

            float box[4];
            compute_dfl(dfl_scratch.data(), dfl_length, box);

            const float x1 =
                (-box[0] + static_cast<float>(column) + 0.5F) *
                static_cast<float>(stride);
            const float y1 =
                (-box[1] + static_cast<float>(row) + 0.5F) *
                static_cast<float>(stride);
            const float x2 =
                (box[2] + static_cast<float>(column) + 0.5F) *
                static_cast<float>(stride);
            const float y2 =
                (box[3] + static_cast<float>(row) + 0.5F) *
                static_cast<float>(stride);

            boxes.push_back(x1);
            boxes.push_back(y1);
            boxes.push_back(x2 - x1);
            boxes.push_back(y2 - y1);
            boxes.push_back(0.0F);
            scores.push_back(dequantize_score(max_score));
            class_ids.push_back(max_class_id);
            ++valid_count;
        }
    }

    return valid_count;
}

std::size_t candidate_capacity(
    const std::vector<rknn_tensor_attr>& output_attrs,
    int outputs_per_branch) {
    std::size_t capacity = 0U;
    for (int branch = 0; branch < kNumDetectionHeads; ++branch) {
        const rknn_tensor_attr& box = output_attrs[
            static_cast<std::size_t>(branch * outputs_per_branch)];
        capacity += static_cast<std::size_t>(box.dims[2]) *
                    static_cast<std::size_t>(box.dims[3]);
    }
    return capacity;
}

}  // namespace

DetectPostprocessor::DetectPostprocessor(
    int model_width,
    int model_height,
    const std::vector<rknn_tensor_attr>& output_attrs)
    : model_width_(model_width),
      model_height_(model_height),
      num_classes_(0),
      is_quantized_(false),
      dfl_length_(0),
      outputs_per_branch_(0) {
    if (output_attrs.size() != 6U && output_attrs.size() != 9U) {
        throw std::invalid_argument(
            "YOLO detect requires six or nine outputs");
    }

    outputs_per_branch_ =
        static_cast<int>(output_attrs.size() / kNumDetectionHeads);
    const rknn_tensor_attr& first_box = output_attrs[0];
    const rknn_tensor_attr& first_score = output_attrs[1];
    dfl_length_ = static_cast<int>(first_box.dims[1]) / 4;
    num_classes_ = static_cast<int>(first_score.dims[1]);
    is_quantized_ =
        first_box.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
        first_box.type != RKNN_TENSOR_FLOAT16;

    const std::size_t capacity =
        candidate_capacity(output_attrs, outputs_per_branch_);
    if (capacity >
        static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument(
            "YOLO detect anchor count exceeds supported indexing");
    }

    candidate_boxes_.reserve(capacity * 5U);
    candidate_scores_.reserve(capacity);
    candidate_class_ids_.reserve(capacity);
    sort_indices_.reserve(capacity);
    merge_indices_.resize(capacity);
    dfl_scratch_.resize(static_cast<std::size_t>(4 * dfl_length_));
    results_.reserve(kMaxDetections);
}

const std::vector<DetectedBox>& DetectPostprocessor::decode(
    const std::vector<rknn_output>& outputs,
    const std::vector<rknn_tensor_attr>& output_attrs,
    const Letterbox& letterbox) {
    candidate_boxes_.clear();
    candidate_scores_.clear();
    candidate_class_ids_.clear();
    sort_indices_.clear();
    results_.clear();

    if (outputs.size() != output_attrs.size() ||
        outputs.size() !=
            static_cast<std::size_t>(outputs_per_branch_ *
                                     kNumDetectionHeads)) {
        return results_;
    }

    for (int branch = 0; branch < kNumDetectionHeads; ++branch) {
        const std::size_t box_index =
            static_cast<std::size_t>(branch * outputs_per_branch_);
        const std::size_t score_index = box_index + 1U;
        const bool has_score_sum = outputs_per_branch_ == 3;
        const std::size_t score_sum_index = box_index + 2U;

        const rknn_tensor_attr& box_attr = output_attrs[box_index];
        const rknn_tensor_attr& score_attr = output_attrs[score_index];
        const int grid_height = static_cast<int>(box_attr.dims[2]);
        const int grid_width = static_cast<int>(box_attr.dims[3]);
        const int stride = model_height_ / grid_height;

        if (is_quantized_) {
            const rknn_tensor_attr* score_sum_attr =
                has_score_sum ? &output_attrs[score_sum_index] : nullptr;
            const int8_t* score_sum = has_score_sum
                ? static_cast<const int8_t*>(
                      outputs[score_sum_index].buf)
                : nullptr;
            const int8_t score_sum_threshold = has_score_sum
                ? quantize_affine(
                      kBoxThreshold,
                      score_sum_attr->zp,
                      score_sum_attr->scale)
                : int8_t{};

            decode_branch(
                static_cast<const int8_t*>(outputs[box_index].buf),
                static_cast<const int8_t*>(outputs[score_index].buf),
                score_sum,
                grid_height,
                grid_width,
                stride,
                dfl_length_,
                num_classes_,
                quantize_affine(
                    kBoxThreshold, score_attr.zp, score_attr.scale),
                score_sum_threshold,
                AffineDequantizer{box_attr.zp, box_attr.scale},
                AffineDequantizer{score_attr.zp, score_attr.scale},
                dfl_scratch_,
                candidate_boxes_,
                candidate_scores_,
                candidate_class_ids_);
        } else {
            const float* score_sum = has_score_sum
                ? static_cast<const float*>(outputs[score_sum_index].buf)
                : nullptr;
            decode_branch(
                static_cast<const float*>(outputs[box_index].buf),
                static_cast<const float*>(outputs[score_index].buf),
                score_sum,
                grid_height,
                grid_width,
                stride,
                dfl_length_,
                num_classes_,
                kBoxThreshold,
                kBoxThreshold,
                IdentityDequantizer{},
                IdentityDequantizer{},
                dfl_scratch_,
                candidate_boxes_,
                candidate_scores_,
                candidate_class_ids_);
        }
    }

    if (candidate_scores_.empty()) {
        return results_;
    }

    for (std::size_t i = 0; i < candidate_scores_.size(); ++i) {
        sort_indices_.push_back(static_cast<int>(i));
    }
    stable_sort_indices(
        candidate_scores_, sort_indices_, merge_indices_);

    for (int class_id = 0; class_id < num_classes_; ++class_id) {
        suppress_class(
            candidate_boxes_,
            candidate_class_ids_,
            sort_indices_,
            class_id,
            kNmsThreshold);
    }

    for (const int index : sort_indices_) {
        if (index == -1) {
            continue;
        }
        if (results_.size() >= kMaxDetections) {
            break;
        }

        const std::size_t box_offset =
            static_cast<std::size_t>(index) * 5U;
        const float x1 = candidate_boxes_[box_offset] -
                         static_cast<float>(letterbox.x_pad);
        const float y1 = candidate_boxes_[box_offset + 1U] -
                         static_cast<float>(letterbox.y_pad);
        const float width = candidate_boxes_[box_offset + 2U];
        const float height = candidate_boxes_[box_offset + 3U];

        results_.emplace_back();
        DetectedBox& detection = results_.back();
        detection.box.left = clamp_to_int(
            x1 / letterbox.scale, 0, letterbox.src_width - 1);
        detection.box.top = clamp_to_int(
            y1 / letterbox.scale, 0, letterbox.src_height - 1);
        detection.box.right = clamp_to_int(
            (x1 + width) / letterbox.scale, 0, letterbox.src_width - 1);
        detection.box.bottom = clamp_to_int(
            (y1 + height) / letterbox.scale, 0, letterbox.src_height - 1);
        detection.score =
            candidate_scores_[static_cast<std::size_t>(index)];
        detection.cls_id =
            candidate_class_ids_[static_cast<std::size_t>(index)];
    }

    return results_;
}

int DetectPostprocessor::num_classes() const noexcept {
    return num_classes_;
}

}  // namespace rknn_yolo
