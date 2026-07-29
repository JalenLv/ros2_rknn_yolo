#include "backend_params.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <stdexcept>
#include <string>

namespace rknn_yolo {
namespace {

using ament_index_cpp::get_package_share_directory;
using std::filesystem::path;

std::string resolve_package_path(const std::string &value) {
    const path path_(value);
    if (path_.is_absolute()) {
        return path_.string();
    }

    // clang-format off
    return (path(get_package_share_directory("librknn_yolo"))
            / path("model") / path_).string();
    // clang-format on
}

} // namespace

CommonParams declare_common_params(rclcpp::Node &node) {
    CommonParams params;
    params.model_path = node.declare_parameter<std::string>("model_path", "");
    params.label_path = node.declare_parameter<std::string>("label_path", "");
    params.use_rga    = node.declare_parameter<bool>("use_rga", true);

    if (params.model_path.empty() || params.label_path.empty()) {
        throw std::runtime_error(
            "model_path and label_path are required; specify a model with the "
            "launch 'model_path' and 'label_path' arguments");
    }

    params.model_path = resolve_package_path(params.model_path);
    params.label_path = resolve_package_path(params.label_path);
    return params;
}

} // namespace rknn_yolo
