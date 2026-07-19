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
 *
 * The preprocessor will attempt to use the RGA hardware accelerator if
 * available, but will fall back to OpenCV if necessary.
 */
class LetterboxPreprocessor {
  public:
    LetterboxPreprocessor();
    ~LetterboxPreprocessor();

    LetterboxPreprocessor(const LetterboxPreprocessor &) = delete;
    LetterboxPreprocessor &operator=(const LetterboxPreprocessor &) = delete;
    LetterboxPreprocessor(LetterboxPreprocessor &&) noexcept;
    LetterboxPreprocessor &operator=(LetterboxPreprocessor &&) noexcept;

    int init(int destination_width, int destination_height,
             bool use_rga = true);
    int process(const SrcView &source, Letterbox &letterbox);

    /**
     * @return the pointer to the persistent model input buffer.
     */
    std::uint8_t *destination() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rknn_yolo

#endif // LIBRKNN_YOLOV8_POSE__LETTERBOX_PREPROCESSOR_HPP_
