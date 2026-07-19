#include "letterbox_preprocessor.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <linux/dma-heap.h>
#include <new>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "im2d.h"

namespace rknn_yolo {
namespace {

constexpr std::uint8_t kPaddingValue = 114U;
constexpr int kRequiredRgaStrideAlignment = 16;
constexpr char kDma32HeapPath[] = "/dev/dma_heap/system-uncached-dma32";

struct DmaHeapAllocation {
    int fd{-1};
    std::uint8_t *address{nullptr};
    std::size_t size{0U};
};

bool page_round(std::size_t size, std::size_t &rounded_size) {
    const long page_size_value = sysconf(_SC_PAGESIZE);
    if (page_size_value <= 0) {
        return false;
    }

    const auto page_size = static_cast<std::size_t>(page_size_value);
    if (size > std::numeric_limits<std::size_t>::max() - (page_size - 1U)) {
        return false;
    }

    rounded_size = ((size + page_size - 1U) / page_size) * page_size;
    return true;
}

bool allocate_dma32(std::size_t size, DmaHeapAllocation &allocation) {
    const int heap_fd = open(kDma32HeapPath, O_RDWR | O_CLOEXEC);
    if (heap_fd < 0) {
        return false;
    }

    dma_heap_allocation_data data{};
    data.len = size;
    data.fd_flags = O_RDWR | O_CLOEXEC;
    const int result = ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &data);
    close(heap_fd);
    if (result < 0) {
        return false;
    }

    void *const address = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                               MAP_SHARED, static_cast<int>(data.fd), 0);
    if (address == MAP_FAILED) {
        close(static_cast<int>(data.fd));
        return false;
    }

    allocation.fd = static_cast<int>(data.fd);
    allocation.address = static_cast<std::uint8_t *>(address);
    allocation.size = size;
    return true;
}

std::size_t bytes_per_pixel(PixelFormat format) {
    switch (format) {
    case PixelFormat::RGB888:
    case PixelFormat::BGR888:
        return 3U;
    case PixelFormat::RGBA8888:
    case PixelFormat::BGRA8888:
        return 4U;
    case PixelFormat::YUYV422:
    case PixelFormat::UYVY422:
        return 2U;
    }

    return 0U;
}

int rga_format(PixelFormat format) {
    switch (format) {
    case PixelFormat::RGB888:
        return RK_FORMAT_RGB_888;
    case PixelFormat::BGR888:
        return RK_FORMAT_BGR_888;
    case PixelFormat::RGBA8888:
        return RK_FORMAT_RGBA_8888;
    case PixelFormat::BGRA8888:
        return RK_FORMAT_BGRA_8888;
    case PixelFormat::YUYV422:
        return RK_FORMAT_YUYV_422;
    case PixelFormat::UYVY422:
        return RK_FORMAT_UYVY_422;
    }

    return -1;
}

int cv_type(PixelFormat format) {
    switch (format) {
    case PixelFormat::RGB888:
    case PixelFormat::BGR888:
        return CV_8UC3;
    case PixelFormat::RGBA8888:
    case PixelFormat::BGRA8888:
        return CV_8UC4;
    case PixelFormat::YUYV422:
    case PixelFormat::UYVY422:
        return CV_8UC2;
    }

    return -1;
}

int cv_to_rgb_code(PixelFormat format) {
    switch (format) {
    case PixelFormat::BGR888:
        return cv::COLOR_BGR2RGB;
    case PixelFormat::RGBA8888:
        return cv::COLOR_RGBA2RGB;
    case PixelFormat::BGRA8888:
        return cv::COLOR_BGRA2RGB;
    case PixelFormat::YUYV422:
        return cv::COLOR_YUV2RGB_YUY2;
    case PixelFormat::UYVY422:
        return cv::COLOR_YUV2RGB_UYVY;
    case PixelFormat::RGB888:
        return -1;
    }

    return -1;
}

} // namespace

struct LetterboxPreprocessor::Impl {
    struct Geometry {
        Letterbox letterbox{};
        im_rect source_rect{};
        im_rect destination_rect{};
        bool has_padding{false};
    };

    ~Impl() { release_destination(); }

    std::uint8_t *destination() const { return destination_; }

    int init(int destination_width, int destination_height) {
        // Resets previous state
        release_destination();

        destination_width_ = 0;
        destination_height_ = 0;
        destination_size_ = 0U;
        initialized_ = false;
        rga_disabled_ = false;
        fill_with_memset_ = false;
        fallback_reported_ = false;
        geometry_valid_ = false;
        cached_source_width_ = 0;
        cached_source_height_ = 0;
        converted_rgb_.release();

        // Checks for non-positive dimensions
        if (destination_width <= 0 || destination_height <= 0) {
            return -1;
        }

        // Checks for overflowing destination size
        const auto width = static_cast<std::size_t>(destination_width);
        const auto height = static_cast<std::size_t>(destination_height);
        if (width > std::numeric_limits<std::size_t>::max() / height ||
            width * height > std::numeric_limits<std::size_t>::max() / 3U) {
            return -1;
        }

        // Stores the destination geometry
        destination_width_ = destination_width;
        destination_height_ = destination_height;
        destination_size_ = width * height * 3U;

        std::size_t allocation_size = 0U;
        DmaHeapAllocation allocation;
        if (page_round(destination_size_, allocation_size) &&
            allocation_size <=
                static_cast<std::size_t>(std::numeric_limits<int>::max()) &&
            allocate_dma32(allocation_size, allocation)) {
            destination_fd_ = allocation.fd;
            destination_ = allocation.address;
            destination_allocation_size_ = allocation.size;
            owns_dma_destination_ = true;
            destination_handle_ =
                importbuffer_fd(destination_fd_,
                                static_cast<int>(destination_allocation_size_));
            if (destination_handle_ == 0U) {
                release_destination_storage();
            }
        }

        if (destination_handle_ == 0U) {
            try {
                fallback_storage_.resize(destination_size_);
            } catch (const std::bad_alloc &) {
                release_destination_storage();
                return -1;
            }
            destination_ = fallback_storage_.data();

            im_handle_param_t parameters{};
            parameters.width = static_cast<std::uint32_t>(destination_width_);
            parameters.height = static_cast<std::uint32_t>(destination_height_);
            parameters.format = RK_FORMAT_RGB_888;
            destination_handle_ =
                importbuffer_virtualaddr(destination_, &parameters);
        }

        initialized_ = true;
        if (destination_handle_ == 0U) {
            disable_rga("could not import the model input buffer");
            return 0;
        }

        destination_buffer_ = wrapbuffer_handle(
            destination_handle_, destination_width_, destination_height_,
            RK_FORMAT_RGB_888, destination_width_, destination_height_);
        return 0;
    }

    int process(const SrcView &source, Letterbox &letterbox) {
        if (!validate_source(source)) {
            return -1;
        }

        if (!geometry_valid_ || source.width != cached_source_width_ ||
            source.height != cached_source_height_) {
            Geometry geometry;
            if (!compute_geometry(source.width, source.height, geometry)) {
                return -1;
            }
            fill_padding(geometry);
            geometry_ = geometry;
            cached_source_width_ = source.width;
            cached_source_height_ = source.height;
            geometry_valid_ = true;
        }

        letterbox = geometry_.letterbox;

        int source_stride_pixels = 0;
        if (rga_stride(source, source_stride_pixels)) {
            if (process_rga(source, source_stride_pixels) == 0) {
                return 0;
            }
        } else if (!rga_disabled_) {
            report_fallback("source row stride cannot be represented by RGA or "
                            "is not 16-byte aligned");
        }

        return process_cpu(source);
    }

  private:
    void release_destination() {
        release_destination_handle();
        release_destination_storage();
    }

    void release_destination_handle() {
        if (destination_handle_ != 0U) {
            releasebuffer_handle(destination_handle_);
            destination_handle_ = 0U;
        }
        destination_buffer_ = {};
    }

    void release_destination_storage() {
        if (owns_dma_destination_) {
            if (destination_ != nullptr && destination_allocation_size_ != 0U) {
                munmap(destination_, destination_allocation_size_);
            }
            if (destination_fd_ >= 0) {
                close(destination_fd_);
            }
        }

        destination_ = nullptr;
        destination_fd_ = -1;
        destination_allocation_size_ = 0U;
        owns_dma_destination_ = false;
        std::vector<std::uint8_t>().swap(fallback_storage_);
    }

    void report_fallback(const char *reason) {
        if (!fallback_reported_) {
            std::fprintf(stderr,
                         "LetterboxPreprocessor: using OpenCV fallback (%s)\n",
                         reason);
            fallback_reported_ = true;
        }
    }

    void disable_rga(const char *reason) {
        rga_disabled_ = true;
        report_fallback(reason);
    }

    void disable_rga(const char *operation, IM_STATUS status) {
        rga_disabled_ = true;
        if (!fallback_reported_) {
            std::fprintf(stderr,
                         "LetterboxPreprocessor: using OpenCV fallback (%s "
                         "failed: %s)\n",
                         operation, imStrError_t(status));
            fallback_reported_ = true;
        }
    }

    bool validate_source(const SrcView &source) const {
        if (!initialized_ || source.data == nullptr || source.width <= 0 ||
            source.height <= 0) {
            return false;
        }

        const std::size_t pixel_size = bytes_per_pixel(source.format);
        if (pixel_size == 0U) {
            return false;
        }

        const auto width = static_cast<std::size_t>(source.width);
        if (width > std::numeric_limits<std::size_t>::max() / pixel_size ||
            source.step_bytes < width * pixel_size) {
            return false;
        }

        if ((source.format == PixelFormat::YUYV422 ||
             source.format == PixelFormat::UYVY422) &&
            (source.width & 1) != 0) {
            return false;
        }

        return true;
    }

    bool compute_geometry(int source_width, int source_height,
                          Geometry &geometry) const {
        // Keep the model-zoo letterbox calculations unchanged. In particular,
        // scale describes the pre-alignment resize while the resize dimensions
        // receive the vendor's 4/2 adjustment.
        int resize_width = destination_width_;
        int resize_height = destination_height_;
        int left_offset = 0;
        int top_offset = 0;
        float scale = 1.0F;

        im_rect source_rect{0, 0, source_width, source_height};
        im_rect destination_rect{0, 0, destination_width_, destination_height_};

        const float scale_width = static_cast<float>(destination_width_) /
                                  static_cast<float>(source_width);
        const float scale_height = static_cast<float>(destination_height_) /
                                   static_cast<float>(source_height);
        if (scale_width < scale_height) {
            scale = scale_width;
            resize_height =
                static_cast<int>(static_cast<float>(source_height) * scale);
        } else {
            scale = scale_height;
            resize_width =
                static_cast<int>(static_cast<float>(source_width) * scale);
        }

        if (resize_width % 4 != 0) {
            resize_width -= resize_width % 4;
        }
        if (resize_height % 2 != 0) {
            resize_height -= resize_height % 2;
        }
        if (resize_width <= 0 || resize_height <= 0) {
            return false;
        }

        const int padding_height = destination_height_ - resize_height;
        const int padding_width = destination_width_ - resize_width;
        if (scale_width < scale_height) {
            destination_rect.y = padding_height / 2;
            if (destination_rect.y % 2 != 0) {
                destination_rect.y -= destination_rect.y % 2;
                if (destination_rect.y < 0) {
                    destination_rect.y = 0;
                }
            }
            destination_rect.height = resize_height;
            top_offset = destination_rect.y;
        } else {
            destination_rect.x = padding_width / 2;
            if (destination_rect.x % 2 != 0) {
                destination_rect.x -= destination_rect.x % 2;
                if (destination_rect.x < 0) {
                    destination_rect.x = 0;
                }
            }
            destination_rect.width = resize_width;
            left_offset = destination_rect.x;
        }

        geometry.letterbox = Letterbox{left_offset, top_offset, scale};
        geometry.source_rect = source_rect;
        geometry.destination_rect = destination_rect;
        geometry.has_padding = destination_rect.width != destination_width_ ||
                               destination_rect.height != destination_height_;
        return true;
    }

    void fill_padding(const Geometry &geometry) {
        if (!geometry.has_padding) {
            return;
        }

        if (!rga_disabled_ && !fill_with_memset_ && destination_handle_ != 0U) {
            constexpr int color = static_cast<int>(kPaddingValue) |
                                  (static_cast<int>(kPaddingValue) << 8) |
                                  (static_cast<int>(kPaddingValue) << 16) |
                                  (static_cast<int>(kPaddingValue) << 24);
            const im_rect whole_destination{0, 0, destination_width_,
                                            destination_height_};
            const IM_STATUS status =
                imfill(destination_buffer_, whole_destination, color);
            if (status > IM_STATUS_FAILED) {
                return;
            }
            // Color fill runs only on the RGA2-Enhance core, whose 32-bit IOMMU
            // cannot address virtual-address buffers backed above 4 GB. Keep
            // RGA resize/color conversion enabled when fill cannot be assigned.
            fill_with_memset_ = true;
            std::fprintf(stderr,
                         "LetterboxPreprocessor: RGA padding fill failed (%s); "
                         "filling padding with memset\n",
                         imStrError_t(status));
        }

        std::memset(destination_, kPaddingValue, destination_size_);
    }

    bool rga_stride(const SrcView &source, int &source_stride_pixels) const {
        if (rga_disabled_ || destination_handle_ == 0U ||
            source.step_bytes % kRequiredRgaStrideAlignment != 0U ||
            (static_cast<std::size_t>(destination_width_) * 3U) %
                    kRequiredRgaStrideAlignment !=
                0U) {
            return false;
        }

        const std::size_t pixel_size = bytes_per_pixel(source.format);
        if (pixel_size == 0U || source.step_bytes % pixel_size != 0U) {
            return false;
        }

        const std::size_t stride_pixels = source.step_bytes / pixel_size;
        if (stride_pixels >
            static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            return false;
        }

        source_stride_pixels = static_cast<int>(stride_pixels);
        return true;
    }

    int process_rga(const SrcView &source, int source_stride_pixels) {
        const int format = rga_format(source.format);
        if (format < 0) {
            disable_rga("unsupported RGA source format");
            return -1;
        }

        im_handle_param_t parameters{};
        parameters.width = static_cast<std::uint32_t>(source_stride_pixels);
        parameters.height = static_cast<std::uint32_t>(source.height);
        parameters.format = static_cast<std::uint32_t>(format);
        const rga_buffer_handle_t source_handle = importbuffer_virtualaddr(
            const_cast<std::uint8_t *>(source.data), &parameters);
        if (source_handle == 0U) {
            disable_rga("could not import the source image");
            return -1;
        }

        const rga_buffer_t source_buffer =
            wrapbuffer_handle(source_handle, source.width, source.height,
                              format, source_stride_pixels, source.height);
        const rga_buffer_t pattern{};
        const im_rect pattern_rect{};
        const IM_STATUS status = improcess(
            source_buffer, destination_buffer_, pattern, geometry_.source_rect,
            geometry_.destination_rect, pattern_rect, 0);
        releasebuffer_handle(source_handle);

        if (status <= IM_STATUS_FAILED) {
            disable_rga("RGA resize/color conversion", status);
            return -1;
        }
        return 0;
    }

    int process_cpu(const SrcView &source) {
        try {
            cv::Mat source_image(
                source.height, source.width, cv_type(source.format),
                const_cast<std::uint8_t *>(source.data), source.step_bytes);
            cv::Mat destination_image(
                destination_height_, destination_width_, CV_8UC3, destination_,
                static_cast<std::size_t>(destination_width_) * 3U);
            const cv::Rect destination_roi(geometry_.destination_rect.x,
                                           geometry_.destination_rect.y,
                                           geometry_.destination_rect.width,
                                           geometry_.destination_rect.height);
            cv::Mat resized_destination = destination_image(destination_roi);

            if (source.format == PixelFormat::RGB888) {
                cv::resize(source_image, resized_destination,
                           resized_destination.size(), 0.0, 0.0,
                           cv::INTER_LINEAR);
            } else {
                cv::cvtColor(source_image, converted_rgb_,
                             cv_to_rgb_code(source.format));
                cv::resize(converted_rgb_, resized_destination,
                           resized_destination.size(), 0.0, 0.0,
                           cv::INTER_LINEAR);
            }
        } catch (const cv::Exception &) {
            return -1;
        }

        return 0;
    }

    std::uint8_t *destination_{nullptr};
    int destination_fd_{-1};
    int destination_width_{0};
    int destination_height_{0};
    std::size_t destination_size_{0U};
    std::size_t destination_allocation_size_{0U};
    bool owns_dma_destination_{false};
    std::vector<std::uint8_t> fallback_storage_;
    bool initialized_{false};

    rga_buffer_handle_t destination_handle_{0U};
    rga_buffer_t destination_buffer_{};
    bool rga_disabled_{false};
    bool fill_with_memset_{false};
    bool fallback_reported_{false};

    bool geometry_valid_{false};
    int cached_source_width_{0};
    int cached_source_height_{0};
    Geometry geometry_{};

    cv::Mat converted_rgb_;
};

LetterboxPreprocessor::LetterboxPreprocessor()
    : impl_(std::make_unique<Impl>()) {}

LetterboxPreprocessor::~LetterboxPreprocessor() = default;

LetterboxPreprocessor::LetterboxPreprocessor(
    LetterboxPreprocessor &&) noexcept = default;

LetterboxPreprocessor &
LetterboxPreprocessor::operator=(LetterboxPreprocessor &&) noexcept = default;

int LetterboxPreprocessor::init(int destination_width, int destination_height) {
    return impl_ == nullptr
               ? -1
               : impl_->init(destination_width, destination_height);
}

std::uint8_t *LetterboxPreprocessor::destination() const {
    return impl_ == nullptr ? nullptr : impl_->destination();
}

int LetterboxPreprocessor::process(const SrcView &source,
                                   Letterbox &letterbox) {
    return impl_ == nullptr ? -1 : impl_->process(source, letterbox);
}

} // namespace rknn_yolo
