#include "core_engine.hpp"

#include <rknn_api.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace rknn_yolo {
namespace {

constexpr int kExpectedInputCount = 1;

std::optional<PixelFormat>
pixel_format_for_encoding(const std::string &encoding) {
    using namespace sensor_msgs::image_encodings;

    if (encoding == RGB8) {
        return PixelFormat::RGB888;
    }
    if (encoding == BGR8) {
        return PixelFormat::BGR888;
    }
    if (encoding == RGBA8) {
        return PixelFormat::RGBA8888;
    }
    if (encoding == BGRA8) {
        return PixelFormat::BGRA8888;
    }
    if (encoding == YUV422_YUY2) {
        return PixelFormat::YUYV422;
    }
    if (encoding == YUV422) {
        return PixelFormat::UYVY422;
    }
    return std::nullopt;
}

bool image_storage_is_valid(const sensor_msgs::msg::Image &image) {
    if (image.width == 0U || image.height == 0U || image.step == 0U) {
        return false;
    }

    const std::size_t height = static_cast<std::size_t>(image.height);
    const std::size_t step   = static_cast<std::size_t>(image.step);
    return step <= std::numeric_limits<std::size_t>::max() / height &&
           image.data.size() >= step * height;
}

} // namespace

CoreEngine::CoreEngine(const std::string &model_path,
                       const std::string &label_path, rclcpp::Logger logger)
    : logger_(std::move(logger)) {
    try {
        load_labels(label_path);
        initialize_model(model_path);
    } catch (...) {
        destroy_context();
        throw;
    }
}

CoreEngine::~CoreEngine() { destroy_context(); }

void CoreEngine::load_labels(const std::string &label_path) {
    std::ifstream stream(label_path);
    if (!stream.is_open()) {
        fail("failed to open label file: " + label_path);
    }

    std::string label;
    while (std::getline(stream, label)) {
        if (!label.empty() && label.back() == '\r') {
            label.pop_back();
        }
        labels_.push_back(label);
    }
    if (stream.bad()) {
        fail("failed while reading label file: " + label_path);
    }
    if (labels_.empty()) {
        fail("label file is empty: " + label_path);
    }
}

void CoreEngine::initialize_model(const std::string &model_path) {
    // Creates the RKNN context
    // The resulting hanndle is stored in `rknn_ctx_`
    int ret = rknn_init(
        &rknn_ctx_, static_cast<void *>(const_cast<char *>(model_path.c_str())),
        0, 0, nullptr);
    if (ret < 0) {
        rknn_ctx_ = 0;
        fail("rknn_init failed for " + model_path + ": " + std::to_string(ret));
    }
    context_initialized_ = true;

    // Queries IO tensor counts
    // Requires 1 input tensor; the backend validates the output count
    ret =
        rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != RKNN_SUCC) {
        fail("rknn_query(RKNN_QUERY_IN_OUT_NUM) failed: " +
             std::to_string(ret));
    }
    if (io_num_.n_input != kExpectedInputCount) {
        fail("expected " + std::to_string(kExpectedInputCount) +
             " input(s), got " + std::to_string(io_num_.n_input));
    }

    // Queries IO tensor attributes
    // Later validated in `validate_input()` and the backend's
    // `validate_outputs()`.
    // See detailed attributes in `rknn_api.h` for `rknn_tensor_attr`.
    input_attrs_.resize(io_num_.n_input);
    for (std::uint32_t i = 0; i < io_num_.n_input; ++i) {
        rknn_tensor_attr &attr = input_attrs_[i];
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;
        ret = rknn_query(rknn_ctx_, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
        if (ret != RKNN_SUCC) {
            fail("rknn_query(RKNN_QUERY_INPUT_ATTR, " + std::to_string(i) +
                 ") failed: " + std::to_string(ret));
        }
    }

    output_attrs_.resize(io_num_.n_output);
    for (std::uint32_t i = 0; i < io_num_.n_output; ++i) {
        rknn_tensor_attr &attr = output_attrs_[i];
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;
        ret =
            rknn_query(rknn_ctx_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
        if (ret != RKNN_SUCC) {
            fail("rknn_query(RKNN_QUERY_OUTPUT_ATTR, " + std::to_string(i) +
                 ") failed: " + std::to_string(ret));
        }
    }

    // Validates the model's input tensor attributes
    validate_input();
}

void CoreEngine::validate_input() {
    // 4 dimensions, batch size 1
    const rknn_tensor_attr &input = input_attrs_.front();
    if (input.n_dims != 4U || input.dims[0] != 1U) {
        fail("model input must be a four-dimensional, batch-one tensor");
    }

    // NCHW or NHWC layout
    std::uint32_t width    = 0U;
    std::uint32_t height   = 0U;
    std::uint32_t channels = 0U;
    if (input.fmt == RKNN_TENSOR_NCHW) {
        channels = input.dims[1];
        height   = input.dims[2];
        width    = input.dims[3];
    } else if (input.fmt == RKNN_TENSOR_NHWC) {
        height   = input.dims[1];
        width    = input.dims[2];
        channels = input.dims[3];
    } else {
        fail("model input must use NCHW or NHWC layout");
    }

    // Nonzero and representable dimensions
    if (width == 0U || height == 0U ||
        width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        fail("model input dimensions are invalid");
    }
    // 3 channels
    if (channels != 3U) {
        fail("model input must have exactly three channels");
    }

    // Populates members for later use
    model_width_    = static_cast<int>(width);
    model_height_   = static_cast<int>(height);
    model_channels_ = static_cast<int>(channels);
}

void CoreEngine::finalize_io(bool use_rga, bool is_quantized) {
    // Computes the persistent model input buffer size
    const std::uint64_t input_size =
        static_cast<std::uint64_t>(model_width_) *
        static_cast<std::uint64_t>(model_height_) *
        static_cast<std::uint64_t>(model_channels_);
    if (input_size > std::numeric_limits<std::size_t>::max() ||
        input_size > std::numeric_limits<std::uint32_t>::max()) {
        fail("model input tensor is too large");
    }

    // Init preprocessing
    // The preprocessor maintains a persistent model input buffer
    if (preprocessor_.init(model_width_, model_height_, use_rga) != 0) {
        fail("failed to initialize letterbox preprocessor");
    }
    RCLCPP_INFO(logger_, "preprocessing: %s",
                use_rga ? "RGA hardware" : "CPU (OpenCV)");

    is_quantized_ = is_quantized;
    // Prepares the input descriptor
    inputs_.resize(io_num_.n_input);
    inputs_[0]       = {};
    inputs_[0].index = 0;
    inputs_[0].type  = RKNN_TENSOR_UINT8;
    inputs_[0].fmt   = RKNN_TENSOR_NHWC;
    inputs_[0].size  = input_size;
    // Gets the pointer to input buffer from the preprocessor
    inputs_[0].buf   = preprocessor_.destination();

    // Prepares the output state
    outputs_.resize(io_num_.n_output);
}

int CoreEngine::run_frame(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                          Letterbox &letterbox) {
    SrcView                          source;
    cv_bridge::CvImageConstPtr       converted_image;
    // Query the pixel format for the image encoding from the message
    const std::optional<PixelFormat> direct_format =
        pixel_format_for_encoding(image->encoding);

    // Populates the source view for the preprocessor
    if (direct_format.has_value()) {
        if (!image_storage_is_valid(*image) ||
            image->width >
                static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
            image->height >
                static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
            RCLCPP_ERROR(logger_, "invalid %s image buffer",
                         image->encoding.c_str());
            return -1;
        }

        source.data       = image->data.data();
        source.width      = static_cast<int>(image->width);
        source.height     = static_cast<int>(image->height);
        source.step_bytes = static_cast<std::size_t>(image->step);
        source.format     = *direct_format;
    } else {
        try {
            converted_image =
                cv_bridge::toCvShare(image, sensor_msgs::image_encodings::RGB8);
        } catch (const cv_bridge::Exception &exception) {
            RCLCPP_ERROR(logger_, "cv_bridge conversion failed: %s",
                         exception.what());
            return -1;
        }

        const cv::Mat &converted = converted_image->image;
        if (converted.empty() || converted.data == nullptr) {
            RCLCPP_ERROR(logger_, "cv_bridge produced an empty image");
            return -1;
        }
        source.data       = converted.data;
        source.width      = converted.cols;
        source.height     = converted.rows;
        source.step_bytes = converted.step[0];
        source.format     = PixelFormat::RGB888;
    }

    // Runs the letterbox preprocessor
    if (preprocessor_.process(source, letterbox) != 0) {
        RCLCPP_ERROR(logger_, "letterbox preprocessing failed");
        return -1;
    }

    // Sets the input tensor for RKNN
    int ret = rknn_inputs_set(rknn_ctx_, io_num_.n_input, inputs_.data());
    if (ret < 0) {
        RCLCPP_ERROR(logger_, "rknn_inputs_set failed: %d", ret);
        return -1;
    }

    // Runs the RKNN model
    ret = rknn_run(rknn_ctx_, nullptr);
    if (ret < 0) {
        RCLCPP_ERROR(logger_, "rknn_run failed: %d", ret);
        return -1;
    }

    // Gets the output tensors from RKNN
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
        outputs_[i]            = {};
        outputs_[i].index      = static_cast<std::uint32_t>(i);
        outputs_[i].want_float = static_cast<std::uint8_t>(!is_quantized_);
    }
    ret =
        rknn_outputs_get(rknn_ctx_, io_num_.n_output, outputs_.data(), nullptr);
    if (ret < 0) {
        RCLCPP_ERROR(logger_, "rknn_outputs_get failed: %d", ret);
        return -1;
    }

    return 0;
}

void CoreEngine::release_outputs() noexcept {
    rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs_.data());
}

const std::vector<rknn_output> &CoreEngine::outputs() const noexcept {
    return outputs_;
}

const std::vector<rknn_tensor_attr> &CoreEngine::output_attrs() const noexcept {
    return output_attrs_;
}

const std::vector<std::string> &CoreEngine::labels() const noexcept {
    return labels_;
}

int CoreEngine::model_width() const noexcept { return model_width_; }

int CoreEngine::model_height() const noexcept { return model_height_; }

int CoreEngine::model_channels() const noexcept { return model_channels_; }

rclcpp::Logger CoreEngine::logger() const noexcept { return logger_; }

[[noreturn]] void CoreEngine::fail(const std::string &message) {
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    throw std::runtime_error(message);
}

void CoreEngine::destroy_context() noexcept {
    if (context_initialized_) {
        rknn_destroy(rknn_ctx_);
        rknn_ctx_            = 0;
        context_initialized_ = false;
    }
}

} // namespace rknn_yolo
