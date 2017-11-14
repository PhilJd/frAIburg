set(CMAKE_PREFIX_PATH /opt/opencv/3.2.0/share/OpenCV/)
set(FILTER_NAME frAIburg_ObstacleDetection)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx



# --------------------- adtf slim -------------------
set(SLIM_DIR ${CMAKE_SOURCE_DIR}/lib_adtf_slim/)
include_directories(${SLIM_DIR})
link_directories(${SLIM_DIR})
# ---------------------------------------------------

# --------------------- OpenCV ----------------------
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIR})
# ---------------------------------------------------

# ------------------ utils library ------------------
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})
link_directories(${UTILS_DIR})
# ---------------------------------------------------


# ------------------ map library --------------------
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${MAP_DIR})
link_directories(${CMAKE_BINARY_DIR}/lib_map/)
# ---------------------------------------------------


# ------------------ librealsense -------------------
#c++11 support for librealsense
#if we use std=c++11 we have some warnings from ADTF an we supress them here
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-literal-suffix")
include_directories(${REALSENSE_INCLUDE_DIRS})
link_directories(${REALSENSE_LIBRARY_DIRS})
# ---------------------------------------------------

adtf_add_filter(${FILTER_NAME}
	../stdafx.h
    obstacle_detection.h
    obstacle_detection.cpp
)

target_link_libraries(${FILTER_NAME} adtfslim
	                                 utils
	                                 global_map
	                                 ${OpenCV_LIBS}
	                                 ${REALSENSE_LIBS})

adtf_set_folder(${FILTER_NAME} obstacle_detection) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
