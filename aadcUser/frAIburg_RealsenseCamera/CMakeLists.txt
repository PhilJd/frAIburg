set(FILTER_NAME frAIburg_realsense)

# add the include directory of the filter
include_directories(../include
			${REALSENSE_INCLUDE_DIRS}
			${GLFW_INCLUDE_DIRS}
			${OpenCV_INCLUDE_DIRS}
			"${CMAKE_CURRENT_SOURCE_DIR}/../") # stdafx

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

# ------------------ utils library ------------------
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})
link_directories(${UTILS_DIR})
link_directories(${CMAKE_SOURCE_DIR}/../../_build_user/utils)
link_directories(/home/aadc/ADTF/_build_user/utils)
# ---------------------------------------------------
		
adtf_add_filter(${FILTER_NAME}
	../stdafx.h
	realsense_camera.h
	realsense_camera.cpp
	realsense_helper.h
)
	
adtf_set_folder(${FILTER_NAME} camera) 

target_link_libraries(${FILTER_NAME} utils adtfslim ${REALSENSE_LIBS} ${OPENGL_LIBRARIES} ${GLFW_LIBS} ${LIBUSB1_LIBRARIES} ${OpenCV_LIBS})
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
