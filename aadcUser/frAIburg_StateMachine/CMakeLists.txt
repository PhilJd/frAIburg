# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx

# set to cpp11 to have typeid().hash_code()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-literal-suffix")

# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME frAIburg_StateMachine_new)


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
# ---------------- use map library for testing----------------
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${MAP_DIR})
link_directories(${MAP_DIR})
# ---------------------------------------------------
# ---------------- BOOST ----------------
#find boost
set (Boost_USE_STATIC_LIBS OFF CACHE BOOL "use static libraries from Boost")
set (Boost_USE_MULTITHREADED ON)
find_package(Boost 1.58 REQUIRED COMPONENTS
		system filesystem thread chrono date_time atomic timer)
# ---------------------------------------------------
adtf_add_filter(${FILTER_NAME}
    ../stdafx.h
    machine.h
    state_machine.h
    event_struct.h
    state_machine.cpp
)

# -------------- link to utils library --------------
target_link_libraries(${FILTER_NAME} adtfslim utils global_map ${Boost_LIBRARIES})
# ---------------------------------------------------

adtf_set_folder(${FILTER_NAME} state_machine_new)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
