cmake_minimum_required(VERSION 3.15)
project(librknn_yolov8_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bboxes_kpoints_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

add_definitions(-DLIBRGA_IM2D_HANDLE)

add_library(imageutils STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/image_utils.c
)
set_target_properties(imageutils PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(imageutils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${LIBRGA_INCLUDES}>
)
target_link_libraries(imageutils
  ${LIBRGA}
)

add_library(rknn_yolov8_pose SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rknn_yolov8_pose.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/postprocess.cc
)

ament_target_dependencies(rknn_yolov8_pose
  rclcpp
  sensor_msgs
  bboxes_kpoints_msgs
  cv_bridge
)

target_link_libraries(rknn_yolov8_pose
  imageutils
  ${LIBRKNNRT}
  dl
  ${OpenCV_LIBS}
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(rknn_yolov8_pose Threads::Threads)

target_include_directories(rknn_yolov8_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${LIBRKNNRT_INCLUDES}>
  $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
)

install(TARGETS rknn_yolov8_pose
  EXPORT export_rknn_yolov8_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS imageutils
  EXPORT export_rknn_yolov8_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  ${LIBRKNNRT_INCLUDES}/rknn_api.h
  DESTINATION include/librknn_yolov8_pose
)

install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(EXPORT export_rknn_yolov8_pose
  FILE rknn_yolov8_poseTargets.cmake
  NAMESPACE rknn_yolov8_pose::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_rknn_yolov8_pose HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
