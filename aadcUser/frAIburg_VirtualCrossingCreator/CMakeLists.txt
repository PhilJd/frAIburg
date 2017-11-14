set(FILTER_NAME frAIburg_VirtualCrossingCreator)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx





set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -O3 -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__")


INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


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
# --------------- use Libs --------------------------
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libs)

# ------------------ map library --------------------
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${MAP_DIR})
link_directories(${CMAKE_BINARY_DIR}/lib_map/)
# ---------------------------------------------------

adtf_add_filter(${FILTER_NAME}
    ../stdafx.h
    crossing_creator.h
    crossing_creator.cpp
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

link_directories(${LIBDIR}/qpOASES-3.2.1/bin)
adtf_set_folder(${FILTER_NAME} crossingdetector) 
target_link_libraries(${FILTER_NAME} utils
                      ${LIBDIR}/qpOASES-3.2.1/bin/libqpOASES.a)
# MAP-------------- link to utils library --------------
target_link_libraries(${FILTER_NAME} adtfslim global_map utils)
# ---------------------------------------------------


# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})


