cmake_minimum_required(VERSION 3.15)
project(librknn_yolov8_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(LIBRKNNRT
  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rknpu2/Linux/aarch64/librknnrt.so")
set(LIBRKNNRT_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rknpu2/include")
set(LIBRGA
  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/librga/Linux/aarch64/librga.so")
set(LIBRGA_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/librga/include")

add_library(rknn_yolov8_pose SHARED
  src/rknn_yolov8_pose.cpp
  src/letterbox_preprocessor.cpp
  src/detection_postprocessor.cpp
)
target_compile_features(rknn_yolov8_pose PUBLIC cxx_std_17)
target_include_directories(rknn_yolov8_pose
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${LIBRKNNRT_INCLUDES}
    ${LIBRGA_INCLUDES}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(rknn_yolov8_pose
  PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${yolo_msgs_TARGETS}
  PRIVATE
    cv_bridge::cv_bridge
    opencv_core
    opencv_imgproc
    ${LIBRKNNRT}
    ${LIBRGA}
)

install(TARGETS rknn_yolov8_pose
  EXPORT export_rknn_yolov8_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  ${LIBRKNNRT}
  ${LIBRGA}
  DESTINATION lib
)

install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

ament_export_targets(export_rknn_yolov8_pose HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs yolo_msgs)
ament_package()
