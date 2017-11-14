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

# ---------------- use utils library ----------------
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})
link_directories(${UTILS_DIR})
# ---------------------------------------------------

# build util classes as library
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(global_map SHARED ${MAP_DIR}global_map.cpp
                       ${MAP_DIR}map_element_types.cpp
                       ${MAP_DIR}map_element.cpp
                       ${MAP_DIR}map_helper.cpp
)


if(ADTF_ADDITIONAL_UTILS_LIBS)
    # if ADTF_ADDITIONAL_UTILS_LIBS is set from the outside
    # we have to forward this setting to A_UTILSConfig.cmake
    set(A_UTILS_ADDITIONAL_UTILS_LIBS ${ADTF_ADDITIONAL_UTILS_LIBS})
endif(ADTF_ADDITIONAL_UTILS_LIBS)

find_package(Boost COMPONENTS system filesystem REQUIRED)
#boost  Geometry is an header-only library so it's not needed
message("Boost_SYSTEM_LIBRARY: ${Boost_SYSTEM_LIBRARY} ")
message("BUILD_LIBS: ${BUILD_LIBS}")
message("ADTF_OPENGL_LIBRARY: ${ADTF_OPENGL_LIBRARY}")
message("ADTF_ADDITIONAL_UTILS_LIBS: ${ADTF_ADDITIONAL_UTILS_LIBS}")
# -------------- link to utils library and boost-----
target_link_libraries(global_map PUBLIC ${Boost_SYSTEM_LIBRARY} ${BUILD_LIBS}
                          ${ADTF_OPENGL_LIBRARY}
                          ${Boost_FILESYSTEM_LIBRARY}
                          ${ADTF_ADDITIONAL_UTILS_LIBS}
                          ${OpenCV_LIBS}
                          utils
                          adtfslim)
