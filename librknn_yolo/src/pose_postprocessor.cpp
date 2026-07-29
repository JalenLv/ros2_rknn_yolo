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

#include "pose_postprocessor.hpp"

#include <Float16.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace rknn_yolo {
namespace {

constexpr int kDflBins = 16;
constexpr int kBoxChannels = 4 * kDflBins;
constexpr int kNumDetectionHeads = 3;
constexpr int kKeypointOutputIndex = 3;
constexpr int kNumKeypoints = 17;
constexpr int kKeypointValues = 3;
constexpr std::size_t kMaxDetections = 128;
constexpr float kBoxThreshold = 0.5F;
constexpr float kNmsThreshold = 0.4F;

template<typename T, typename Dequantizer>
int decode_branch(
    const T* input,
    int grid_height,
    int grid_width,
    int stride,
    int num_classes,
    T raw_threshold,
    const Dequantizer& dequantize,
    int anchor_offset,
    std::vector<float>& boxes,
    std::vector<float>& scores,
    std::vector<int>& class_ids) {
    int valid_count = 0;
    const int grid_size = grid_width * grid_height;

    for (int h = 0; h < grid_height; ++h) {
        for (int w = 0; w < grid_width; ++w) {
            const int anchor_index = h * grid_width + w;
            for (int class_id = 0; class_id < num_classes; ++class_id) {
                const T raw_score = input[(kBoxChannels + class_id) * grid_size + anchor_index];
                if (!(raw_score >= raw_threshold)) {
                    continue;
                }

                const float score = sigmoid(dequantize(raw_score));
                std::array<float, kBoxChannels> locations;
                for (int i = 0; i < kBoxChannels; ++i) {
                    locations[static_cast<std::size_t>(i)] =
                        dequantize(input[i * grid_size + anchor_index]);
                }

                for (int i = 0; i < kBoxChannels / kDflBins; ++i) {
                    softmax(locations.data() + i * kDflBins, kDflBins);
                }

                std::array<float, 4> xyxy{0.0F, 0.0F, 0.0F, 0.0F};
                for (int bin = 0; bin < kDflBins; ++bin) {
                    const float bin_value = static_cast<float>(bin);
                    xyxy[0] += locations[bin] * bin_value;
                    xyxy[1] += locations[kDflBins + bin] * bin_value;
                    xyxy[2] += locations[2 * kDflBins + bin] * bin_value;
                    xyxy[3] += locations[3 * kDflBins + bin] * bin_value;
                }

                xyxy[0] = (static_cast<float>(w) + 0.5F) - xyxy[0];
                xyxy[1] = (static_cast<float>(h) + 0.5F) - xyxy[1];
                xyxy[2] = (static_cast<float>(w) + 0.5F) + xyxy[2];
                xyxy[3] = (static_cast<float>(h) + 0.5F) + xyxy[3];

                std::array<float, 4> xywh;
                xywh[0] = ((xyxy[0] + xyxy[2]) / 2.0F) * static_cast<float>(stride);
                xywh[1] = ((xyxy[1] + xyxy[3]) / 2.0F) * static_cast<float>(stride);
                xywh[2] = (xyxy[2] - xyxy[0]) * static_cast<float>(stride);
                xywh[3] = (xyxy[3] - xyxy[1]) * static_cast<float>(stride);
                xywh[0] -= xywh[2] / 2.0F;
                xywh[1] -= xywh[3] / 2.0F;

                boxes.push_back(xywh[0]);
                boxes.push_back(xywh[1]);
                boxes.push_back(xywh[2]);
                boxes.push_back(xywh[3]);
                boxes.push_back(static_cast<float>(anchor_offset + anchor_index));
                scores.push_back(score);
                class_ids.push_back(class_id);
                ++valid_count;
            }
        }
    }

    return valid_count;
}

template<typename T>
void copy_keypoints(
    const T* keypoint_output,
    int total_anchors,
    int keypoint_index,
    const Letterbox& letterbox,
    Detection& detection) {
    for (int keypoint = 0; keypoint < kNumKeypoints; ++keypoint) {
        const int keypoint_offset = keypoint * kKeypointValues * total_anchors;
        detection.keypoints[keypoint][0] =
            (static_cast<float>(keypoint_output[keypoint_offset + keypoint_index]) -
             static_cast<float>(letterbox.x_pad)) /
            letterbox.scale;
        detection.keypoints[keypoint][1] =
            (static_cast<float>(keypoint_output[keypoint_offset + total_anchors + keypoint_index]) -
             static_cast<float>(letterbox.y_pad)) /
            letterbox.scale;
        detection.keypoints[keypoint][2] =
            static_cast<float>(keypoint_output[keypoint_offset + 2 * total_anchors + keypoint_index]);
    }
}

std::size_t candidate_capacity(
    const std::vector<rknn_tensor_attr>& output_attrs,
    int num_classes) {
    std::size_t capacity = 0U;
    for (int i = 0; i < kNumDetectionHeads; ++i) {
        const auto& attr = output_attrs[static_cast<std::size_t>(i)];
        capacity += static_cast<std::size_t>(attr.dims[2]) *
                    static_cast<std::size_t>(attr.dims[3]) *
                    static_cast<std::size_t>(num_classes);
    }
    return capacity;
}

}  // namespace

PosePostprocessor::PosePostprocessor(
    int model_width,
    int model_height,
    const std::vector<rknn_tensor_attr>& output_attrs)
    : model_width_(model_width),
      model_height_(model_height),
      num_classes_(0),
      is_quantized_(false) {
    if (output_attrs.size() < static_cast<std::size_t>(kNumDetectionHeads + 1)) {
        throw std::invalid_argument("YOLOv8 pose requires three detection outputs and one keypoint output");
    }
    if (output_attrs[0].dims[1] <= static_cast<uint32_t>(kBoxChannels)) {
        throw std::invalid_argument("YOLOv8 pose detection output has no class channels");
    }

    num_classes_ = static_cast<int>(output_attrs[0].dims[1]) - kBoxChannels;
    is_quantized_ =
        output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
        output_attrs[0].type != RKNN_TENSOR_FLOAT16;

    const std::size_t capacity = candidate_capacity(output_attrs, num_classes_);
    if (capacity > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("YOLOv8 pose anchor count exceeds supported indexing");
    }
    candidate_boxes_.reserve(capacity * 5U);
    candidate_scores_.reserve(capacity);
    candidate_class_ids_.reserve(capacity);
    sort_indices_.reserve(capacity);
    merge_indices_.resize(capacity);
    results_.reserve(kMaxDetections);
}

const std::vector<Detection>& PosePostprocessor::decode(
    const std::vector<rknn_output>& outputs,
    const std::vector<rknn_tensor_attr>& output_attrs,
    const Letterbox& letterbox) {
    candidate_boxes_.clear();
    candidate_scores_.clear();
    candidate_class_ids_.clear();
    sort_indices_.clear();
    results_.clear();

    if (outputs.size() < static_cast<std::size_t>(kNumDetectionHeads + 1) ||
        output_attrs.size() < static_cast<std::size_t>(kNumDetectionHeads + 1)) {
        return results_;
    }

    int anchor_offset = 0;
    for (int i = 0; i < kNumDetectionHeads; ++i) {
        const auto& attr = output_attrs[static_cast<std::size_t>(i)];
        const int grid_height = static_cast<int>(attr.dims[2]);
        const int grid_width = static_cast<int>(attr.dims[3]);
        const int stride = model_height_ / grid_height;

        if (is_quantized_) {
            const AffineDequantizer dequantize{attr.zp, attr.scale};
            const int8_t threshold = quantize_affine(unsigmoid(kBoxThreshold), attr.zp, attr.scale);
            decode_branch(
                static_cast<const int8_t*>(outputs[static_cast<std::size_t>(i)].buf),
                grid_height,
                grid_width,
                stride,
                num_classes_,
                threshold,
                dequantize,
                anchor_offset,
                candidate_boxes_,
                candidate_scores_,
                candidate_class_ids_);
        } else {
            decode_branch(
                static_cast<const float*>(outputs[static_cast<std::size_t>(i)].buf),
                grid_height,
                grid_width,
                stride,
                num_classes_,
                unsigmoid(kBoxThreshold),
                IdentityDequantizer{},
                anchor_offset,
                candidate_boxes_,
                candidate_scores_,
                candidate_class_ids_);
        }

        anchor_offset += grid_height * grid_width;
    }

    if (candidate_scores_.empty()) {
        return results_;
    }

    for (std::size_t i = 0; i < candidate_scores_.size(); ++i) {
        sort_indices_.push_back(static_cast<int>(i));
    }
    stable_sort_indices(candidate_scores_, sort_indices_, merge_indices_);

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

        const std::size_t box_offset = static_cast<std::size_t>(index) * 5U;
        const float x1 = candidate_boxes_[box_offset] - static_cast<float>(letterbox.x_pad);
        const float y1 = candidate_boxes_[box_offset + 1U] - static_cast<float>(letterbox.y_pad);
        const float width = candidate_boxes_[box_offset + 2U];
        const float height = candidate_boxes_[box_offset + 3U];
        const int keypoint_index = static_cast<int>(candidate_boxes_[box_offset + 4U]);

        results_.emplace_back();
        Detection& detection = results_.back();
        if (is_quantized_) {
            copy_keypoints(
                static_cast<const rknpu2::float16*>(outputs[kKeypointOutputIndex].buf),
                anchor_offset,
                keypoint_index,
                letterbox,
                detection);
        } else {
            copy_keypoints(
                static_cast<const float*>(outputs[kKeypointOutputIndex].buf),
                anchor_offset,
                keypoint_index,
                letterbox,
                detection);
        }

        detection.box.left =
            static_cast<int>(
                static_cast<float>(clamp_to_int(x1, 0, model_width_)) / letterbox.scale);
        detection.box.top =
            static_cast<int>(
                static_cast<float>(clamp_to_int(y1, 0, model_height_)) / letterbox.scale);
        detection.box.right =
            static_cast<int>(
                static_cast<float>(clamp_to_int(x1 + width, 0, model_width_)) / letterbox.scale);
        detection.box.bottom =
            static_cast<int>(
                static_cast<float>(clamp_to_int(y1 + height, 0, model_height_)) / letterbox.scale);
        detection.score = candidate_scores_[static_cast<std::size_t>(index)];
        detection.cls_id = candidate_class_ids_[static_cast<std::size_t>(index)];
    }

    return results_;
}

int PosePostprocessor::num_classes() const noexcept {
    return num_classes_;
}

}  // namespace rknn_yolo
