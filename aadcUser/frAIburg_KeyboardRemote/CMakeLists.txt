# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx

set(FILTER_NAME frAIburg_keyboardRemoteFilter )

adtf_use_qt(QtCore QtGui)

#we build only if qt was found
if(QT_FOUND)

    set(HEADER_MOC
        displaywidget.h
        keyboard_remote_filter.h
        )

    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )

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
                    displaywidget.cpp
                    keyboard_remote_filter.cpp
                    ${HEADER_MOC}
    )

    # -------------- link to utils library --------------
    target_link_libraries(${FILTER_NAME} adtfslim utils global_map)

    # ---------------------------------------------------

    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} helper)

    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

endif(QT_FOUND)


