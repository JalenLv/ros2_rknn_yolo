#ifndef LIBRKNN_YOLOV8_POSE__LETTERBOX_PREPROCESSOR_HPP_
#define LIBRKNN_YOLOV8_POSE__LETTERBOX_PREPROCESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

namespace rknn_yolo {

enum class PixelFormat {
    RGB888,
    BGR888,
    RGBA8888,
    BGRA8888,
    YUYV422,
    UYVY422,
};

struct SrcView {
    const std::uint8_t *data{nullptr};
    int width{0};
    int height{0};
    std::size_t step_bytes{0};
    PixelFormat format{PixelFormat::RGB888};
};

struct Letterbox {
    int x_pad{0};
    int y_pad{0};
    float scale{1.0F};
};

/**
 * @brief Transforms an arbitrary ROS image into the model's fixed-size packed
 * RGB buffer while preserving aspect ratio (letterboxing).
 */
class LetterboxPreprocessor {
  public:
    LetterboxPreprocessor();
    ~LetterboxPreprocessor();

    LetterboxPreprocessor(const LetterboxPreprocessor &) = delete;
    LetterboxPreprocessor &operator=(const LetterboxPreprocessor &) = delete;
    LetterboxPreprocessor(LetterboxPreprocessor &&) noexcept;
    LetterboxPreprocessor &operator=(LetterboxPreprocessor &&) noexcept;

    int init(std::uint8_t *destination, int destination_width,
             int destination_height);
    int process(const SrcView &source, Letterbox &letterbox);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rknn_yolo

#endif // LIBRKNN_YOLOV8_POSE__LETTERBOX_PREPROCESSOR_HPP_
