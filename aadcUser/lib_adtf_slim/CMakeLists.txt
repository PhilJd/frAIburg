# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx


# ------------------- OpenCV ------------------------
set(CMAKE_PREFIX_PATH /opt/opencv/3.2.0/share/OpenCV/)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
# ---------------------------------------------------

# build adtf slim classes as library
set(SLIM_DIR ${CMAKE_SOURCE_DIR}/lib_adtf_slim/)


add_library(adtfslim STATIC ${SLIM_DIR}../stdafx.h
                            ${SLIM_DIR}adtf_tools.h
                            ${SLIM_DIR}adtf_tools.cpp
                            ${SLIM_DIR}slim_pins.h
                            ${SLIM_DIR}slim_pins.cpp
                            ${SLIM_DIR}opencv_tools.h
                            ${SLIM_DIR}opencv_tools.cpp
                            ${SLIM_DIR}adtf_log_macros.h
)

find_package(Boost COMPONENTS system filesystem REQUIRED)

if(ADTF_ADDITIONAL_UTILS_LIBS)
    # if ADTF_ADDITIONAL_UTILS_LIBS is set from the outside
    # we have to forward this setting to A_UTILSConfig.cmake
    set(A_UTILS_ADDITIONAL_UTILS_LIBS ${ADTF_ADDITIONAL_UTILS_LIBS})
endif(ADTF_ADDITIONAL_UTILS_LIBS)
set(ADTF_ADDITIONAL_UTILS_LIBS ${A_UTILS_ADDITIONAL_UTILS_LIBS})
target_link_libraries(adtfslim ${BUILD_LIBS}
                               ${QT_LIBRARIES}
                               ${ADTF_OPENGL_LIBRARY}
                               ${ADTF_ADDITIONAL_UTILS_LIBS}
                               ${OpenCV_LIBS}
                               ${Boost_FILESYSTEM_LIBRARY}
                               ${Boost_SYSTEM_LIBRARY})
