# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
#set project name at beginning this is cleaner than just a folder
project(frAIburg_ThriftClient)
set(FILTER_NAME ${PROJECT_NAME})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx


# --------------------- adtf slim -------------------
set(SLIM_DIR ${CMAKE_SOURCE_DIR}/lib_adtf_slim/)
include_directories(${SLIM_DIR})
link_directories(${SLIM_DIR})
# ---------------------------------------------------

# ------------------ librealsense -------------------
#c++11 support for librealsense
#if we use std=c++11 we have some warnings from ADTF an we supress them here
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-literal-suffix")
include_directories(${REALSENSE_INCLUDE_DIRS})
link_directories(${REALSENSE_LIBRARY_DIRS})
# -------

# ------------------ utils library ------------------
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})
link_directories(${UTILS_DIR})
link_directories(${CMAKE_SOURCE_DIR}/../../_build_user/utils)
link_directories(/home/aadc/ADTF/_build_user/utils)
# ---------------------------------------------------

# ------------------- OpenCV ------------------------
set(CMAKE_PREFIX_PATH /opt/opencv/3.2.0/share/OpenCV/)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
# ---------------------------------------------------

# ------------------ map library --------------------
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${MAP_DIR})
link_directories(${CMAKE_BINARY_DIR}/lib_map/)
# ---------------------------------------------------


include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR} 
    ${Thrift_INCLUDE_DIR}
    gen-cpp
)

adtf_add_filter(${FILTER_NAME}
    ../stdafx.h 
    thrift_client.h
    thrift_client.cpp
    gen-cpp/ExtIf_constants.h
    gen-cpp/ExtIf_constants.cpp
    gen-cpp/ExtIf_types.h
    gen-cpp/ExtIf_types.cpp
    gen-cpp/ExtService.h
    gen-cpp/ExtService.cpp
)

target_link_libraries(${FILTER_NAME}
    adtfslim
    utils
    global_map
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${Thrift_LINK_LIBS}
    ${OpenSSL_LINK_LIBS}
)

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
