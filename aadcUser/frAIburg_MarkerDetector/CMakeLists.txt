
set(FILTER_NAME frAIburg_MarkerDetector)

#set include directories
include_directories(${OpenCV_INCLUDE_DIR}) 
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx



# --------------------- boost -----------------------
find_package(Boost COMPONENTS thread REQUIRED)
# ---------------------------------------------------

# --------------------- adtf slim -------------------
set(SLIM_DIR ${CMAKE_SOURCE_DIR}/lib_adtf_slim/)
include_directories(${SLIM_DIR})
link_directories(${SLIM_DIR})
# ---------------------------------------------------

# add the source
adtf_add_filter(${FILTER_NAME}
                aruco_helpers.h
				cMarkerDetector.h
				cMarkerDetector.cpp
		)

# linking libs
target_link_libraries(${FILTER_NAME} adtfslim ${OpenCV_LIBS} ${Boost_THREAD_LIBRARY})


# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
