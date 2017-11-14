# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")  # stdafx


# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME frAIburg_Planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-sign-compare -O3 -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__")



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
# --------------- use Libs --------------------------
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libs)
include_directories(SYSTEM "${LIBDIR}/qpOASES-3.2.1/include"
		            SYSTEM "${LIBDIR}/qpOASES-3.2.1/bin"
                    SYSTEM "${LIBDIR}/Eigen")


# ------------------ map library --------------------
set(MAP_DIR ${CMAKE_SOURCE_DIR}/lib_map/)
include_directories(${MAP_DIR})
link_directories(${CMAKE_BINARY_DIR}/lib_map/)
# ---------------------------------------------------

adtf_add_filter(${FILTER_NAME}
    ../stdafx.h
    PlannerFilter.h
    PlannerFilter.cpp
    TrajectoryPlanner.h
    TrajectoryPlanner.cpp
    ManeuverPlanner.h
    ManeuverPlanner.cpp
    PathPlanner.h
    PathPlanner.cpp
    SpeedPlanner.h
    SpeedPlanner.cpp
    GoalPointGenerator.h
    GoalPointGenerator.cpp
    Spline.h
    Spline.cpp
    ManeuverGoal.cpp
    ManeuverGoal.h
    ${LIBDIR}/qpOASES-3.2.1/src/BLASReplacement.o
    ${LIBDIR}/qpOASES-3.2.1/src/LAPACKReplacement.o
)

link_directories(${LIBDIR}/qpOASES-3.2.1/bin)
adtf_set_folder(${FILTER_NAME} pathplanner) 
target_link_libraries(${FILTER_NAME} utils
                      ${LIBDIR}/qpOASES-3.2.1/bin/libqpOASES.a)

# MAP-------------- link to utils library --------------
target_link_libraries(${FILTER_NAME} adtfslim global_map utils)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

