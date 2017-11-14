# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx
find_library(/home/aadc/ADTF/_build_user/map map)



# declare our filter
set(FILTER_NAME frAIburg_RoadSignFilter)


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

# ------------------ map library --------------------
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${MAP_DIR})
link_directories(${CMAKE_BINARY_DIR}/lib_map/)
# ---------------------------------------------------



adtf_add_filter(${FILTER_NAME}
    ../stdafx.h
    roadsign_filter.h
    roadsign_filter.cpp
)


# -------------- link to utils library --------------
target_link_libraries(${FILTER_NAME} adtfslim global_map utils)
# ---------------------------------------------------

adtf_set_folder(${FILTER_NAME} roadsign)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
