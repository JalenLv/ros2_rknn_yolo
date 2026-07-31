#ifndef LIBRKNN_YOLO__CORE_ENGINE_HPP_
#define LIBRKNN_YOLO__CORE_ENGINE_HPP_

#include "letterbox_preprocessor.hpp"

#include <rknn_api.h>

#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rknn_yolo {

/**
 * @brief Shared inference core owned by each backend.
 *
 * Owns the RKNN context, the class labels, the letterbox preprocessor, and
 * the IO descriptors. It covers the task-agnostic half of the pipeline:
 * model loading, input-tensor validation, preprocessing, and running one
 * frame on the NPU. Backends validate the task-specific output layout and
 * decode the results.
 */
class CoreEngine {
  public:
    /**
     * @brief Loads the class labels, creates the RKNN context, queries the
     * IO tensor attributes, and validates the input tensor.
     * @throws std::runtime_error on any failure.
     */
    CoreEngine(const std::string &model_path, const std::string &label_path,
               rclcpp::Logger logger);
    /**
     * @brief Releases the RKNN context.
     */
    ~CoreEngine();

    // Non-copyable: owns the RKNN context.
    CoreEngine(const CoreEngine &)            = delete;
    CoreEngine &operator=(const CoreEngine &) = delete;

    /**
     * @brief Completes IO setup after the backend has validated the outputs:
     * sizes the persistent model input buffer, initializes the letterbox
     * preprocessor, and prepares the input/output descriptors.
     * @param use_rga Prefer the RGA hardware for preprocessing.
     * @param is_quantized Determined by the backend; quantized models keep
     * their raw INT8 outputs, otherwise floating-point outputs are requested.
     */
    void finalize_io(bool use_rga, bool is_quantized);

    /**
     * @brief Runs the task-agnostic front half of inference for one frame:
     * builds the source view, letterboxes it into the model input buffer,
     * and executes the model.
     *
     * On success the raw outputs are available via `outputs()` and must be
     * released with `release_outputs()` after decoding.
     * @param image The input image message.
     * @param letterbox Receives the letterbox mapping used for this frame.
     * @return 0 on success, negative on failure.
     */
    int run_frame(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                  Letterbox                                     &letterbox);

    /** @brief Releases the output buffers to RKNN after decoding. */
    void release_outputs() noexcept;

    /** @return the raw output tensors of the last `run_frame()`. */
    const std::vector<rknn_output>      &outputs() const noexcept;
    /** @return the model's output tensor attributes. */
    const std::vector<rknn_tensor_attr> &output_attrs() const noexcept;
    /** @return the class labels loaded from the label file. */
    const std::vector<std::string>      &labels() const noexcept;

    /** @return the model input width in pixels. */
    int model_width() const noexcept;
    /** @return the model input height in pixels. */
    int model_height() const noexcept;
    /** @return the model input channel count. */
    int model_channels() const noexcept;

    /** @return the logger used for library diagnostics. */
    rclcpp::Logger logger() const noexcept;

    /** @brief Logs the message and throws `std::runtime_error`. */
    [[noreturn]] void fail(const std::string &message);

  private:
    /**
     * @brief Loads class names for postprocessing.
     */
    void load_labels(const std::string &label_path);
    /**
     * @brief Creates the RKNN context, queries the IO tensor counts/attributes,
     * and validates the input tensor.
     */
    void initialize_model(const std::string &model_path);
    /**
     * @brief Validates the model's input tensor attributes
     *
     * input 0: a 640x640 image of shape [1 (batch size), 3 (channel), 640, 640]
     * (NCHW) or [1, 640, 640, 3] (NHWC)
     */
    void validate_input();
    /**
     * @brief Destroys the RKNN context if it was created.
     */
    void destroy_context() noexcept;

    rclcpp::Logger                logger_;
    rknn_context                  rknn_ctx_{0};
    rknn_input_output_num         io_num_{};
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;

    bool context_initialized_{false};
    int  model_width_{0};
    int  model_height_{0};
    int  model_channels_{0};
    bool is_quantized_{false};

    std::vector<std::string> labels_;
    LetterboxPreprocessor    preprocessor_;
    std::vector<rknn_input>  inputs_;
    std::vector<rknn_output> outputs_;
};

} // namespace rknn_yolo

#endif // LIBRKNN_YOLO__CORE_ENGINE_HPP_
