# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx


# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME frAIburg_dynamic_csv_writers)


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


adtf_add_filter(${FILTER_NAME}
    ../stdafx.h
    dynamic_csv_writer_filter.h
    dynamic_csv_writer_filter.cpp
)

find_package(Boost COMPONENTS system filesystem REQUIRED)

# -------------- link to utils library and boost-----
target_link_libraries(${FILTER_NAME} adtfslim utils
                ${Boost_FILESYSTEM_LIBRARY}
                ${Boost_SYSTEM_LIBRARY})
# ---------------------------------------------------

adtf_set_folder(${FILTER_NAME} template)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

