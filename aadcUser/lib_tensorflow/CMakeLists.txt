# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx


# --------------------- OpenCV ----------------------
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIR})
# ---------------------------------------------------

# build util classes as library
set(TF_WRAPPER ${CMAKE_SOURCE_DIR}/lib_tensorflow/)

include_directories("/home/aadc/ADTF/Libs/tensorflow-1.4.0/include/")
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})

add_library(frAIburg_tensorflow ${TF_WRAPPER}tensorflow_controller.h
                                ${TF_WRAPPER}tensorflow_controller.cpp
                                ${TF_WRAPPER}tensorflow_adtf_bridge.h
                                ${TF_WRAPPER}tensorflow_adtf_bridge.cpp
                                ${TF_WRAPPER}tensorflow_opencv_bridge.h
                                ${TF_WRAPPER}tensorflow_opencv_bridge.cpp
                                ${TF_WRAPPER}tensorflow_cpp_wrapper.h
                                ${TF_WRAPPER}tensorflow_cpp_wrapper.cpp
)

target_link_libraries(frAIburg_tensorflow /home/aadc/ADTF/Libs/tensorflow-1.4.0/lib/libtensorflow.so ${OpenCV_LIBS})
