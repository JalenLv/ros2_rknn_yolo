#include <librknn_yolo/yolo.hpp>

#include "detect_backend.hpp"
#include "pose_backend.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace rknn_yolo {

std::unique_ptr<Yolo> make_yolo(rclcpp::Node &node) {
    const std::string backend =
        node.declare_parameter<std::string>("backend", "");
    if (backend.empty()) {
        throw std::runtime_error(
            "backend parameter is required; select a backend with the "
            "launch 'backend' argument");
    }

    if (backend == "pose") {
        return std::make_unique<PoseBackend>();
    }
    if (backend == "detect") {
        return std::make_unique<DetectBackend>();
    }

    const std::string message =
        "unknown backend '" + backend + "'; supported: pose, detect";
    RCLCPP_ERROR(node.get_logger(), "%s", message.c_str());
    throw std::invalid_argument(message);
}

} // namespace rknn_yolo
