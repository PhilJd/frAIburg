# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx


# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME frAIburg_SteeringController)


# --------------------- adtf slim -------------------
set(SLIM_DIR ${CMAKE_SOURCE_DIR}/lib_adtf_slim/)
include_directories(${SLIM_DIR})
link_directories(${SLIM_DIR})
# ---------------------------------------------------

# ---------------- use utils library ----------------
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})
link_directories(${UTILS_DIR})
# ---------------------------------------------------

adtf_add_filter(${FILTER_NAME}
    ../stdafx.h
    SteeringController.h
    SteeringController.cpp
)

adtf_set_folder(${FILTER_NAME} controller) 
target_link_libraries(${FILTER_NAME} adtfslim utils)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

