#ifndef LIBRKNN_YOLO__LETTERBOX_PREPROCESSOR_HPP_
#define LIBRKNN_YOLO__LETTERBOX_PREPROCESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

namespace rknn_yolo {

/**
 * @brief Source pixel layouts accepted by the preprocessor.
 */
enum class PixelFormat {
    RGB888,
    BGR888,
    RGBA8888,
    BGRA8888,
    YUYV422,
    UYVY422,
};

/**
 * @brief Non-owning view of one source image buffer.
 */
struct SrcView {
    const std::uint8_t *data{nullptr};
    int width{0};
    int height{0};
    std::size_t step_bytes{0};
    PixelFormat format{PixelFormat::RGB888};
};

/**
 * @brief Mapping produced by letterboxing: the source is resized by `scale`
 * and then offset by `x_pad`/`y_pad` inside the model input.
 */
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
 * available, but will fall back to OpenCV if necessary. Passing `use_rga =
 * false` will force the use of OpenCV.
 *
 * The class first tries to allocate a DMA32 buffer for the model input. If the
 * allocation fails or the buffer cannot be imported to RGA, it will fall back
 * to a heap allocation. If the destination buffer still could not be imported,
 * the RGA path will be disabled and OpenCV will be used for processing.
 */
class LetterboxPreprocessor {
  public:
    /**
     * @brief Constructs an idle preprocessor; call `init()` before use.
     */
    LetterboxPreprocessor();
    /**
     * @brief Releases the model input buffer and RGA resources.
     */
    ~LetterboxPreprocessor();

    // Non-copyable; movable (transfers buffer ownership).
    LetterboxPreprocessor(const LetterboxPreprocessor &) = delete;
    LetterboxPreprocessor &operator=(const LetterboxPreprocessor &) = delete;
    LetterboxPreprocessor(LetterboxPreprocessor &&) noexcept;
    LetterboxPreprocessor &operator=(LetterboxPreprocessor &&) noexcept;

    /**
     * @brief Allocates the persistent model input buffer and selects the
     * RGA or OpenCV path.
     * @param use_rga Prefer the RGA hardware; falls back to OpenCV when it
     * is unavailable.
     * @return 0 on success, negative on failure.
     */
    int init(int destination_width, int destination_height,
             bool use_rga = true);
    /**
     * @brief Letterboxes `source` into the model input buffer.
     * @param letterbox Receives the scale and padding used, needed to map
     * detections back to source coordinates.
     * @return 0 on success, negative on failure.
     */
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

#endif // LIBRKNN_YOLO__LETTERBOX_PREPROCESSOR_HPP_
