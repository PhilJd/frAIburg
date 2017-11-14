set(FILTER_NAME frAIburg_ObjectDetection)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx


# --------------------- adtf slim -------------------
set(SLIM_DIR ${CMAKE_SOURCE_DIR}/lib_adtf_slim/)
include_directories(${SLIM_DIR})
link_directories(${SLIM_DIR})
# ---------------------------------------------------

# ------------------- OpenCV ------------------------
set(CMAKE_PREFIX_PATH /opt/opencv/3.2.0/share/OpenCV/)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
# ---------------------------------------------------

# ------------------ utils library ------------------
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})
link_directories(${UTILS_DIR})
link_directories(${CMAKE_SOURCE_DIR}/../../_build_user/utils)
link_directories(/home/aadc/ADTF/_build_user/utils)
# ---------------------------------------------------

# ---------------- tensorflow library ---------------
set(TF_WRAPPER_DIR ${CMAKE_SOURCE_DIR}/lib_tensorflow/)
include_directories(${TF_WRAPPER_DIR})
link_directories(${TF_WRAPPER_DIR})
link_directories(${CMAKE_BINARY_DIR}/tensorflow)

set(TF_DIR ${CMAKE_SOURCE_DIR}/../../Libs/tensorflow-1.4.0/)
include_directories(${TF_DIR}/include)
link_directories(${TF_DIR}/lib)
# ---------------------------------------------------

# ------------------ map library --------------------
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${MAP_DIR})
link_directories(${CMAKE_BINARY_DIR}/lib_map/)
# ---------------------------------------------------

# --------------------- boost -----------------------
find_package(Boost COMPONENTS thread REQUIRED)
# ---------------------------------------------------

adtf_add_filter(${FILTER_NAME}
	object_detection.h
	object_detection.cpp
	bounding_box_visualization.h
	bounding_box_visualization.cpp
)

target_link_libraries(${FILTER_NAME}
	adtfslim
	global_map
	utils
	frAIburg_tensorflow
	tensorflow
	${OpenCV_LIBS}
	${Boost_THREAD_LIBRARY}
)

adtf_set_folder(${FILTER_NAME} object_detection) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

