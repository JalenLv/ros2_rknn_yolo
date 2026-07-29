#ifndef RKNN_YOLO_YOLO_NODE_HPP
#define RKNN_YOLO_YOLO_NODE_HPP

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <builtin_interfaces/msg/time.hpp>
#include <librknn_yolo/yolo.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detections.hpp>

/**
 * @brief ROS 2 node wrapper for RKNN YOLO inference.
 *
 * This node subscribes to images, runs latest-frame inference, and publishes
 * YOLO detections.
 */
class YoloNode : public rclcpp::Node {
  public:
    /**
     * @brief Sets up topics and parameters, constructs and initializes the
     * selected YOLO backend, and starts the inference worker thread.
     */
    YoloNode(const std::string &node_name = "yolo_node");
    /**
     * @brief Stops and joins the inference worker thread.
     */
    ~YoloNode();

  private:
    /**
     * @brief Runs when a camera image arrives.
     *        Stores the newest valid image and wakes the worker thread.
     */
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    /**
     * @brief Worker thread loop. Waits for images,
     *        enforces FPS, runs inference, publishes results.
     */
    void inference_loop();
    /**
     * @brief Checks whether an image is already pending, active, or processed.
     *
     * Must be called while holding.
     */
    bool is_duplicate_locked(
        const sensor_msgs::msg::Image::ConstSharedPtr &msg) const;
    /**
     * @brief Compares two ROS timestamps for equality.
     */
    bool has_same_stamp(const builtin_interfaces::msg::Time &lhs,
                        const builtin_interfaces::msg::Time &rhs) const;
    /**
     * @brief Checks whether an image has a usable timestamp.
     */
    bool has_nonzero_stamp(const builtin_interfaces::msg::Time &stamp) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Publisher<yolo_msgs::msg::Detections>::SharedPtr
        detections_publisher;

    // Selected RKNN YOLO inference backend.
    std::unique_ptr<rknn_yolo::Yolo> yolo;

    // Protects shared state between the ROS callback thread
    // and the inference worker thread.
    std::mutex              image_mutex_;
    // Condition variable to wake the worker thread when a new image arrives.
    std::condition_variable image_cv_;
    std::thread             inference_thread_;

    // Set during destruction so the worker thread exits cleanly.
    bool stop_worker_              = false;
    // True when `pending_image_` contains a new image wating for inference.
    bool has_pending_image_        = false;
    // True while the worker is currently running inference on `active_image_`.
    bool has_active_image_         = false;
    // True once at least one frame has already been processed.
    bool has_last_processed_image_ = false;

    // Newest image waiting to be processed.
    // If another image arrives while inference is busy, it replaces this one.
    sensor_msgs::msg::Image::ConstSharedPtr pending_image_;
    // Image currently being inferred.
    sensor_msgs::msg::Image::ConstSharedPtr active_image_;
    // Last image processed.
    sensor_msgs::msg::Image::ConstSharedPtr last_processed_image_;

    // Time when the lastest inference started. Used for FPS limiting.
    std::chrono::steady_clock::time_point last_inference_time_;
    // False before the first inference; true after that.
    bool                                  has_last_inference_time_ = false;
    // Minimum duration between inference starts.
    // For FPS=15, this is 66.7 ms.
    // If non-positive, there is no limit on inference rate.
    std::chrono::steady_clock::duration   min_inference_interval_ =
        std::chrono::steady_clock::duration::zero();
};

#endif // RKNN_YOLO_YOLO_NODE_HPP
