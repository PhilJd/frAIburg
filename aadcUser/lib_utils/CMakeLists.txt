
# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx

# ------------------ librealsense -------------------
#c++11 support for librealsense
#if we use std=c++11 we have some warnings from ADTF an we supress them here
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-literal-suffix")
include_directories(${REALSENSE_INCLUDE_DIRS})
link_directories(${REALSENSE_LIBRARY_DIRS})
# ---------------------------------------------------


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

# build util classes as library
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)


add_library(utils STATIC ${UTILS_DIR}/../stdafx.h
                  ${UTILS_DIR}log_tools.h
                  ${UTILS_DIR}log_tools.cpp
                  ${UTILS_DIR}xml_helper.hpp
                  ${UTILS_DIR}circular_buffer.hpp
                  ${UTILS_DIR}circular_buffer_ultrasonic.hpp
                  ${UTILS_DIR}camera_transformations.h
                  ${UTILS_DIR}camera_transformations.cpp
                  ${UTILS_DIR}r200_transformations.h
                  ${UTILS_DIR}r200_transformations.cpp

)

find_package(Boost COMPONENTS system filesystem REQUIRED)

if(ADTF_ADDITIONAL_UTILS_LIBS)
    # if ADTF_ADDITIONAL_UTILS_LIBS is set from the outside
    # we have to forward this setting to A_UTILSConfig.cmake
    set(A_UTILS_ADDITIONAL_UTILS_LIBS ${ADTF_ADDITIONAL_UTILS_LIBS})
endif(ADTF_ADDITIONAL_UTILS_LIBS)
set(ADTF_ADDITIONAL_UTILS_LIBS ${A_UTILS_ADDITIONAL_UTILS_LIBS})
target_link_libraries(utils ${BUILD_LIBS}
                            ${QT_LIBRARIES}
                            ${ADTF_OPENGL_LIBRARY}
                            ${ADTF_ADDITIONAL_UTILS_LIBS}
                            ${OpenCV_LIBS}
                            ${Boost_FILESYSTEM_LIBRARY}
                            ${Boost_SYSTEM_LIBRARY}
                            ${REALSENSE_LIBS})
