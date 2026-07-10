#include "rknn_yolo/yolo_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>

YoloNode::YoloNode(const std::string &node_name) : Node(node_name) {
    RCLCPP_INFO(this->get_logger(), "Initializing YoloNode...");

    // Construct the YOLO object
    std::string share_dir = ament_index_cpp::get_package_share_directory("librknn_yolov8_pose");
    this->declare_parameter("model_path", share_dir + "/model/yolov8_pose.rknn");
    this->declare_parameter("label_path", share_dir + "/model/yolov8_pose_labels_list.txt");
    std::string model_path = this->get_parameter("model_path").as_string();
    std::string label_path = this->get_parameter("label_path").as_string();
    yolo = std::make_unique<rknn_yolo::YoloV8Pose>(model_path, label_path, this->get_logger());

    // Subscribe to the camera image topic
    // Incoming images call `image_callback`
    std::string image_topic("image_raw");
    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&YoloNode::image_callback, this, std::placeholders::_1)
    );

    // Publisher for YOLO detections
    std::string detections_topic("detections");
    detections_publisher = this->create_publisher<yolo_msgs::msg::Detections>(
        detections_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
    );

    // Maximum inference rate
    // Frames that arrive while inference is busy or rate-limited
    // replace the pending frame so control sees the latest image.
    this->declare_parameter("fps", 0);
    int fps = this->get_parameter("fps").as_int();
    if (fps > 0) {
        min_inference_interval_ = std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / static_cast<double>(fps)));
    } else {
        min_inference_interval_ = std::chrono::steady_clock::duration::zero();
    }

    // Starts the inference worker thread
    inference_thread_ = std::thread(&YoloNode::inference_loop, this);

    RCLCPP_INFO(this->get_logger(), "YoloNode initialized.");
}

YoloNode::~YoloNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down YoloNode...");
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        stop_worker_ = true;
    }
    image_cv_.notify_one();
    if (inference_thread_.joinable()) {
        inference_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "YoloNode shut down.");
}

void YoloNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg
) {
    std::lock_guard<std::mutex> lock(image_mutex_);

    // Skips duplicates by checking whether the image
    // is already pending, active, or processed.
    if (is_duplicate_locked(msg)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Skipping duplicate image stamp %d.%09u",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec);
        return;
    }

    // If another image is already pending, it will be replaced by this one.
    // If there is no pending image, this one will be the pending one.
    if (has_pending_image_) {
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Replacing pending YOLO frame with newer image stamp %d.%09u",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec);
    }

    pending_image_ = msg;
    has_pending_image_ = true;
    // Wakes the worker if it is waiting
    image_cv_.notify_one();
}

void YoloNode::inference_loop() {
    while (true) {
        // Local variable to hold the image to be processed
        sensor_msgs::msg::Image::ConstSharedPtr job;

        {
            // We use `unique_lock` instead `lock_guard` because
            // condition variables need a lock that can temporarily
            // unlock and re-lock while waiting.
            std::unique_lock<std::mutex> lock(image_mutex_);
            // Tracks whether the worker has successfully obtained a job to process
            bool have_job = false;

            while (!stop_worker_ && !have_job) {
                // Waits for a new image to be available or
                // for the worker to be stopped
                if (!has_pending_image_) {
                    image_cv_.wait(
                        lock,
                        [this]() {
                            return stop_worker_ || has_pending_image_;
                        });
                    continue;
                }

                // Enforces the FPS limit by waiting until
                // the next allowed inference time, unless shutdown happens.
                if (
                    has_last_inference_time_ &&
                    min_inference_interval_ >
                    std::chrono::steady_clock::duration::zero()
                ) {
                    const auto next_allowed =
                        last_inference_time_ + min_inference_interval_;
                    const auto now = std::chrono::steady_clock::now();
                    if (now < next_allowed) {
                        image_cv_.wait_until(
                            lock,
                            next_allowed,
                            [this]() { return stop_worker_; });
                        continue;
                    }
                }

                job = pending_image_;
                has_pending_image_ = false;

                if (is_duplicate_locked(job)) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "Skipping duplicate pending image stamp %d.%09u",
                        job->header.stamp.sec,
                        job->header.stamp.nanosec);
                    continue;
                }

                active_image_ = job;
                has_active_image_ = true;
                last_inference_time_ = std::chrono::steady_clock::now();
                has_last_inference_time_ = true;
                have_job = true;
            }

            if (stop_worker_) {
                break;
            }
        } // Unlock

        // Create output message
        auto detections_msg = std::make_shared<yolo_msgs::msg::Detections>();

        // Run inference
        if (yolo->infer(job, detections_msg) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Inference failed");
        } else {
            detections_msg->header.stamp = this->now();
            detections_msg->header.frame_id = job->header.frame_id;
            detections_publisher->publish(*detections_msg);
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Published YOLO result for image stamp %d.%09u",
                job->header.stamp.sec,
                job->header.stamp.nanosec);
        }

        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            has_active_image_ = false;
            last_processed_image_ = job;
            has_last_processed_image_ = true;
        }
    }
}

bool YoloNode::is_duplicate_locked(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg
) const {
    const auto &stamp = msg->header.stamp;
    if (has_nonzero_stamp(stamp)) {
        if (
            has_last_processed_image_ &&
            has_same_stamp(stamp, last_processed_image_->header.stamp)
        ) {
            return true;
        }
        if (
            has_active_image_ &&
            has_same_stamp(stamp, active_image_->header.stamp)
        ) {
            return true;
        }
        if (
            has_pending_image_ &&
            has_same_stamp(stamp, pending_image_->header.stamp)
        ) {
            return true;
        }
        return false;
    }

    return (
        (has_last_processed_image_ && last_processed_image_ == msg) ||
        (has_active_image_ && active_image_ == msg) ||
        (has_pending_image_ && pending_image_ == msg)
    );
}

bool YoloNode::has_same_stamp(
    const builtin_interfaces::msg::Time &lhs,
    const builtin_interfaces::msg::Time &rhs
) const {
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

bool YoloNode::has_nonzero_stamp(
    const builtin_interfaces::msg::Time &stamp
) const {
    return stamp.sec != 0 || stamp.nanosec != 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloNode>());
    rclcpp::shutdown();
    return 0;
}
