cmake_minimum_required(VERSION 3.15)
project(rknn_3rdparty_libs)

set(RKNN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/rknpu2)
set(LIBRKNNRT ${RKNN_PATH}/Linux/aarch64/librknnrt.so PARENT_SCOPE)
set(LIBRKNNRT_INCLUDES ${RKNN_PATH}/include PARENT_SCOPE)
install(PROGRAMS ${RKNN_PATH}/Linux/aarch64/librknnrt.so DESTINATION lib)

set(RGA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/librga)
set(LIBRGA ${RGA_PATH}/Linux/aarch64/librga.a PARENT_SCOPE)
set(LIBRGA_INCLUDES ${RGA_PATH}/include PARENT_SCOPE)
install(PROGRAMS ${RGA_PATH}/Linux/aarch64/librga.so DESTINATION lib)
