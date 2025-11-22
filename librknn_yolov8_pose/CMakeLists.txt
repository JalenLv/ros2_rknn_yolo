cmake_minimum_required(VERSION 3.15)
project(librknn_yolov8_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bboxes_kpoints_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Set default build options
if (NOT DEFINED TARGET_SOC)
  set(TARGET_SOC "rk3588")
endif()
if (NOT DEFINED CMAKE_SYSTEM_NAME)
  set(CMAKE_SYSTEM_NAME Linux)
endif()
if (NOT DEFINED CMAKE_SYSTEM_PROCESSOR)
  set(CMAKE_SYSTEM_PROCESSOR aarch64)
endif()
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if (NOT DEFINED ENABLE_ASAN)
  set(ENABLE_ASAN OFF)
endif()
if (NOT DEFINED DISABLE_RGA)
  set(DISABLE_RGA OFF)
endif()
if (NOT DEFINED DISABLE_LIBJPEG)
  set(DISABLE_LIBJPEG OFF)
endif()

# Import 3rd party libraries
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

# Build utils
add_library(fileutils STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/file_utils.c
)
set_target_properties(fileutils PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fileutils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_library(imagedrawing STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/image_drawing.c
)
set_target_properties(imagedrawing PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(imagedrawing PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(DISABLE_RGA AND NOT (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103" OR TARGET_SOC STREQUAL "rv1103b"))
  add_definitions(-DDISABLE_RGA)
endif ()

# only RGA on rv1106 and rk3588 support handle
if (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rk3588")
  add_definitions(-DLIBRGA_IM2D_HANDLE)
endif()

add_library(imageutils STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/image_utils.c
)
set_target_properties(imageutils PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(imageutils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${STB_INCLUDES}>
  $<BUILD_INTERFACE:${LIBRGA_INCLUDES}>
)

target_link_libraries(imageutils
  ${LIBRGA}
)

if (DISABLE_LIBJPEG)
  add_definitions(-DDISABLE_LIBJPEG)
else()
  target_link_libraries(imageutils
    ${LIBJPEG}
  )
  target_include_directories(imageutils PUBLIC
    $<BUILD_INTERFACE:${LIBJPEG_INCLUDES}>
  )
endif()

add_library(audioutils STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/audio_utils.c
)
set_target_properties(audioutils PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(audioutils
  ${LIBSNDFILE}
)

target_include_directories(audioutils PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBSNDFILE_INCLUDES}
)

# Build yolov8 pose library
add_library(rknn_yolov8_pose SHARED
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/yolov8-pose.cc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/postprocess.cc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rknn_yolov8_pose.cc>
)

target_compile_definitions(rknn_yolov8_pose PRIVATE
  LABEL_NAME_TXT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model/yolov8_pose_labels_list.txt"
  MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model/yolov8_pose.rknn"
)

ament_target_dependencies(rknn_yolov8_pose
  sensor_msgs
  bboxes_kpoints_msgs
  cv_bridge
)

target_link_libraries(rknn_yolov8_pose
  imageutils
  fileutils
  # imagedrawing
  # audioutils
  ${LIBRKNNRT}
  dl
  ${OpenCV_LIBS}
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
  target_link_libraries(rknn_yolov8_pose Threads::Threads)
endif()

target_include_directories(rknn_yolov8_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${LIBRKNNRT_INCLUDES}>
  $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
)
# foreach(_dir ${OpenCV_INCLUDE_DIRS})
#   install(DIRECTORY ${_dir}/
#     DESTINATION include
#   )
#   target_include_directories(rknn_yolov8_pose PUBLIC
#     $<BUILD_INTERFACE:${_dir}>
#     $<INSTALL_INTERFACE:include>
#   )
# endforeach()

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

install(TARGETS fileutils
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

install(EXPORT export_rknn_yolov8_pose
  FILE rknn_yolov8_poseTargets.cmake
  NAMESPACE rknn_yolov8_pose::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_rknn_yolov8_pose HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
