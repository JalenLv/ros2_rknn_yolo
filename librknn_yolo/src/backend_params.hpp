#ifndef LIBRKNN_YOLO__BACKEND_PARAMS_HPP_
#define LIBRKNN_YOLO__BACKEND_PARAMS_HPP_

#include <string>

#include <rclcpp/node.hpp>

namespace rknn_yolo {

/**
 * @brief Parameters shared by every backend.
 */
struct CommonParams {
    // Absolute path to the `.rknn` model file.
    std::string model_path;
    // Absolute path to the class label list.
    std::string label_path;
    // Prefer the RGA hardware for preprocessing.
    bool        use_rga;
};

/**
 * @brief Declares and reads the parameters shared by every backend:
 * `model_path`, `label_path` (required), and `use_rga` (default true).
 *
 * Relative model/label paths are resolved against the `model/` directory of
 * the `librknn_yolo` package share.
 * @param node used for parameter declaration
 * @return the resolved parameter values
 * @throws std::runtime_error if `model_path` or `label_path` is missing.
 */
CommonParams declare_common_params(rclcpp::Node &node);

} // namespace rknn_yolo

#endif // LIBRKNN_YOLO__BACKEND_PARAMS_HPP_
