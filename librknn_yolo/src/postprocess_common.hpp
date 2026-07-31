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

#ifndef LIBRKNN_YOLO__POSTPROCESS_COMMON_HPP_
#define LIBRKNN_YOLO__POSTPROCESS_COMMON_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rknn_yolo {

/**
 * @brief Axis-aligned bounding box in pixel coordinates.
 */
struct Rect {
    int left;
    int top;
    int right;
    int bottom;
};

/**
 * @brief Logistic sigmoid; maps a logit to a probability.
 */
inline float sigmoid(float value) {
    return 1.0F / (1.0F + std::exp(-value));
}

/**
 * @brief Inverse sigmoid; maps a probability threshold into logit domain.
 */
inline float unsigmoid(float value) {
    return -1.0F * std::log((1.0F / value) - 1.0F);
}

/**
 * @brief Clamps `value` to [minimum, maximum] and truncates to int32.
 */
inline int32_t clip_to_int(float value, float minimum, float maximum) {
    const float clipped =
        value <= minimum ? minimum : (value >= maximum ? maximum : value);
    return static_cast<int32_t>(clipped);
}

/**
 * @brief Maps a float threshold into the affine INT8 domain so comparisons
 * can happen on raw quantized values.
 */
inline int8_t quantize_affine(
    float value, int32_t zero_point, float scale) {
    const float quantized = (value / scale) + static_cast<float>(zero_point);
    return static_cast<int8_t>(
        clip_to_int(quantized, -128.0F, 127.0F));
}

/**
 * @brief No-op dequantizer for floating-point outputs.
 */
struct IdentityDequantizer {
    float operator()(float value) const noexcept {
        return value;
    }
};

/**
 * @brief Dequantizes affine INT8 values using zero point and scale.
 */
struct AffineDequantizer {
    int32_t zero_point;
    float scale;

    float operator()(int8_t value) const noexcept {
        return (static_cast<float>(value) -
                static_cast<float>(zero_point)) *
               scale;
    }
};

/**
 * @brief In-place, numerically stable softmax over `size` values.
 */
inline void softmax(float* values, int size) {
    float maximum = values[0];
    for (int i = 1; i < size; ++i) {
        maximum = std::max(maximum, values[i]);
    }

    float sum = 0.0F;
    for (int i = 0; i < size; ++i) {
        sum += std::exp(values[i] - maximum);
    }

    for (int i = 0; i < size; ++i) {
        values[i] = std::exp(values[i] - maximum) / sum;
    }
}

/**
 * @brief Distribution Focal Loss decode: for each of the four box sides,
 * softmax over `dfl_length` bins, then the expected bin index becomes the
 * side's distance in `box`.
 */
inline void compute_dfl(float* values, int dfl_length, float* box) {
    for (int side = 0; side < 4; ++side) {
        float* distribution = values + side * dfl_length;
        softmax(distribution, dfl_length);

        box[side] = 0.0F;
        for (int bin = 0; bin < dfl_length; ++bin) {
            box[side] +=
                distribution[bin] * static_cast<float>(bin);
        }
    }
}

/**
 * @brief IoU of two boxes given their corner coordinates.
 */
inline float calculate_overlap(
    float xmin0,
    float ymin0,
    float xmax0,
    float ymax0,
    float xmin1,
    float ymin1,
    float xmax1,
    float ymax1) {
    const float width = std::fmax(
        0.0F, std::fmin(xmax0, xmax1) - std::fmax(xmin0, xmin1) + 1.0F);
    const float height = std::fmax(
        0.0F, std::fmin(ymax0, ymax1) - std::fmax(ymin0, ymin1) + 1.0F);
    const float intersection = width * height;
    const float union_area =
        (xmax0 - xmin0 + 1.0F) * (ymax0 - ymin0 + 1.0F) +
        (xmax1 - xmin1 + 1.0F) * (ymax1 - ymin1 + 1.0F) - intersection;
    return union_area <= 0.0F ? 0.0F : intersection / union_area;
}

/**
 * @brief Greedy per-class NMS over the score-sorted `order`; candidates of
 * `class_id` overlapping a kept box by more than `threshold` are marked -1.
 *
 * `boxes` stores five floats per candidate, the first four being
 * xmin, ymin, width, height.
 */
inline void suppress_class(
    const std::vector<float>& boxes,
    const std::vector<int>& class_ids,
    std::vector<int>& order,
    int class_id,
    float threshold) {
    for (std::size_t i = 0; i < order.size(); ++i) {
        const int current = order[i];
        if (current == -1 ||
            class_ids[static_cast<std::size_t>(current)] != class_id) {
            continue;
        }

        const std::size_t current_box =
            static_cast<std::size_t>(current) * 5U;
        const float xmin0 = boxes[current_box];
        const float ymin0 = boxes[current_box + 1U];
        const float xmax0 = xmin0 + boxes[current_box + 2U];
        const float ymax0 = ymin0 + boxes[current_box + 3U];

        for (std::size_t j = i + 1U; j < order.size(); ++j) {
            const int candidate = order[j];
            if (candidate == -1 ||
                class_ids[static_cast<std::size_t>(candidate)] != class_id) {
                continue;
            }

            const std::size_t candidate_box =
                static_cast<std::size_t>(candidate) * 5U;
            const float xmin1 = boxes[candidate_box];
            const float ymin1 = boxes[candidate_box + 1U];
            const float xmax1 = xmin1 + boxes[candidate_box + 2U];
            const float ymax1 = ymin1 + boxes[candidate_box + 3U];

            if (calculate_overlap(
                    xmin0,
                    ymin0,
                    xmax0,
                    ymax0,
                    xmin1,
                    ymin1,
                    xmax1,
                    ymax1) > threshold) {
                order[j] = -1;
            }
        }
    }
}

/**
 * @brief Stable bottom-up merge sort of `indices` by descending score, using
 * `scratch` to avoid per-call allocation.
 */
inline void stable_sort_indices(
    const std::vector<float>& scores,
    std::vector<int>& indices,
    std::vector<int>& scratch) {
    bool source_is_indices = true;

    for (std::size_t run_size = 1U;
         run_size < indices.size();
         run_size *= 2U) {
        const std::vector<int>& source =
            source_is_indices ? indices : scratch;
        std::vector<int>& destination =
            source_is_indices ? scratch : indices;

        for (std::size_t begin = 0U;
             begin < indices.size();
             begin += 2U * run_size) {
            const std::size_t middle =
                std::min(begin + run_size, indices.size());
            const std::size_t end =
                std::min(begin + 2U * run_size, indices.size());
            std::size_t left = begin;
            std::size_t right = middle;

            for (std::size_t out = begin; out < end; ++out) {
                const bool take_left =
                    right == end ||
                    (left < middle &&
                     !(scores[static_cast<std::size_t>(source[right])] >
                       scores[static_cast<std::size_t>(source[left])]));
                destination[out] =
                    take_left ? source[left++] : source[right++];
            }
        }

        source_is_indices = !source_is_indices;
    }

    if (!source_is_indices) {
        std::copy_n(scratch.begin(), indices.size(), indices.begin());
    }
}

/**
 * @brief Clamps `value` to [minimum, maximum] and truncates to int.
 */
inline int clamp_to_int(float value, int minimum, int maximum) {
    return value > static_cast<float>(minimum)
        ? (value < static_cast<float>(maximum)
               ? static_cast<int>(value)
               : maximum)
        : minimum;
}

}  // namespace rknn_yolo

#endif  // LIBRKNN_YOLO__POSTPROCESS_COMMON_HPP_
