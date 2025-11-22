cmake_minimum_required(VERSION 3.8)
project(rknn_yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bboxes_kpoints_msgs REQUIRED)
find_package(librknn_yolov8_pose REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

add_executable(yolo_node src/yolo_node.cpp)
target_include_directories(yolo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(yolo_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(yolo_node
  rclcpp
  sensor_msgs
  bboxes_kpoints_msgs
  cv_bridge
  message_filters
)

target_link_libraries(yolo_node
  librknn_yolov8_pose::rknn_yolov8_pose
)

install(TARGETS yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()
