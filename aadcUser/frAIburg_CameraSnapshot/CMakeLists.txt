set(CMAKE_PREFIX_PATH /opt/opencv/3.2.0/share/OpenCV/)
set(FILTER_NAME camera_snapshot)

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

# ---------------- use utils library ----------------
set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
include_directories(${UTILS_DIR})
link_directories(${UTILS_DIR})
# ---------------------------------------------------




adtf_add_filter(${FILTER_NAME}
	../stdafx.h
    camera_snapshot.h
    camera_snapshot.cpp
)

target_link_libraries(${FILTER_NAME} utils ${OpenCV_LIBS})

adtf_set_folder(${FILTER_NAME} camerasnapshot) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})



#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
    
    install(FILES "${Qt_DIR}/bin/QtTest4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo) 
    install(FILES "${Qt_DIR}/bin/QtTestd4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug) 
endif(WIN32)
