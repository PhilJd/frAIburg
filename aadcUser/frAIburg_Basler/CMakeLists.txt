
if(AADC_PYLON_FOUND)
    # This is required by CMake
    cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx
    project(frAIburg_Basler)
    set(FILTER_NAME ${PROJECT_NAME})
    set(MAIN_FOLDER "plugins")
    



    include_directories(
        ${PROJECT_SOURCE_DIR}
        ../../include
        SYSTEM
    )

    # --------------------- adtf slim -------------------
    set(SLIM_DIR ${CMAKE_SOURCE_DIR}/lib_adtf_slim/)
    include_directories(${SLIM_DIR})
    link_directories(${SLIM_DIR})
    # ---------------------------------------------------

    # ---------------- use utils library ----------------
    # (used for camera transformations)
    set(UTILS_DIR ${CMAKE_SOURCE_DIR}/lib_utils/)
    include_directories(${UTILS_DIR})
    link_directories(${UTILS_DIR})
    # ---------------------------------------------------

    # ---------------- use pylon library ----------------
    include_directories(${PYLON_INCLUDE_DIRS})
    link_directories(${PYLON_LIBRARY_DIRS})
    # ---------------------------------------------------
    
    # --------------------- boost -----------------------
    find_package(Boost COMPONENTS system REQUIRED)
    # ---------------------------------------------------

    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

    adtf_add_filter(${FILTER_NAME}
        basler_camera.h
        basler_camera.cpp
    )

    target_link_libraries(${FILTER_NAME}
        adtfslim
        utils
        ${PYLON_LIBS}
        ${OpenCV_LIBS}
        ${Boost_SYSTEM_LIBRARY}
    )

    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

endif(AADC_PYLON_FOUND)
