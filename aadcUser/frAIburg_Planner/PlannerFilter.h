/**********************************************************************
Copyright (c) 2017, team frAIburg
Licensed under BSD-3.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************/

#ifndef AADCUSER_FRAIBURG_PLANNER_PLANNERFILTER_H_
#define AADCUSER_FRAIBURG_PLANNER_PLANNERFILTER_H_

#define OID_ADTF_FILTER_DEF "adtf.frAIburg_planner"
#define ADTF_FILTER_PLANNER_FILTER_NAME "frAIburg Planner filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "frAIburg Planner"
#define ADTF_FILTER_VERSION_SUB_NAME "PlannerFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "Filter to plan trajectory"

#include <iostream>
#include <vector>
#include "TrajectoryPlanner.h"
#include "ManeuverPlanner.h"
#include "slim_pins.h"
#include "customtypes.h"
#include <boost/assign/list_of.hpp>
#include "stdafx.h"
#include "aadc_structs.h"
#include<list>
//MAP

#include "global_map.hpp"
#include "map_element.hpp"
#include "map_event_listener.hpp"

/* This class takes a reference to a Spline object and sets up optim. problem
 * to solve for spline params.
 *
 * Cases: - Add to exising problem
 * - setup new problem
 *
 * */
// as defined in AADC description:

//using namespace frAIburg::map;

struct tPosition {
    tFloat32 f32x;
    tFloat32 f32y;
    tFloat32 f32radius;
    tFloat32 f32speed;
    tFloat32 f32heading;
};

class PlannerFilter : public adtf::cFilter{

    /*! This macro does all the plugin setup stuff */
  ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC,
                      adtf::OBJCAT_Auxiliary, ADTF_FILTER_VERSION_SUB_NAME,
                      ADTF_FILTER_VERSION_Major, ADTF_FILTER_VERSION_Minor,
                      ADTF_FILTER_VERSION_Build, ADTF_FILTER_VERSION_LABEL);

 private:
    bool LOG_DEBUG_;
    float DELTA_T_REPLAN_;
    bool behav_completed_sent_;
    frAIburg::map::GlobalMap* map_;
    slim::InputPin tLaneElement_vert_input_pin_,
                  tLaneElement_horiz_input_pin_,
                  tLaneElement_parking_input_pin_,
                  tPos_input_pin_,
                  ds_input_pin_,
                  behaviour_input_pin_;

    slim::OutputPin vtracking_output_pin_,
                   goalpoint_output_pin_,
                   status_output_pin_,
                   enable_parking_rear_view_output_pin_,
                   enable_front_view_output_pin_,
                   light_controller_output_pin_,
                   enable_us_right_output_pin_,
                   jury_obstacle_output_pin_;

    std::vector<Lane> lane_vert_vec_,
                      lane_horiz_vec_;

    TrajectoryPlanner trajectory_planner_;
    ManeuverPlanner maneuver_planner_;
    tTimeStamp last_time_;
    tTimeStamp init_time_;

    tPose pose_curr_;
    tPose pose_prev_;
    tPoint last_goal_;
    float speed_curr_;
    tBehaviour behaviour_prev_;
    tBehaviour behaviour_curr_;
    bool isPositionInit_;
    float v_tracking_curr_;
    float v_tracking_prev_;
    bool processed_new_lanevec_;
    bool is_maneuver_finished_;
    cCriticalSection update_lane_mutex_;
    cCriticalSection update_pose_mutex_;

    planner_status actOnCurrentBehaviour();
    bool isReplanNecessary();
    float getSpeedValue();
    tPoint getGoalpoint();
    planner_status doFollowLane();
    planner_status doObjectApproach();
    planner_status doCrossing();
    planner_status doParking();
    planner_status doDemoTask();
    void correctRepositioningJump();
    void setLightForBehaviour();
    void CheckTransmitObstaclePosition();
    // IO
    bool processLaneElement(IMediaSample* media_sample);
    bool processPose(IMediaSample* media_sample);
    bool processDeltaDistance(IMediaSample* media_sample, float *ds);
    bool processBehaviour(IMediaSample* media_sample);
    void transmitGoalpoint(const tPoint &goal_pnt);
    void transmitPlannerStatus(const planner_status &status);
    void transmitVTracking(float v_tracking);
    void doManeuverFinishedActions();
    void TransmitEnableParkingRearView(const tBool enable_rear);
    void TransmitEnableFrontView(const tBool enable_front);
    tResult TransmitLightCommand(tInt32 light_id, tBool switch_bool);
    void TransmitEnableUSRight(const tBool enable_us_right);
    void TransmitJuryObstacle(const frAIburg::map::tSptrMapElement & el_obstacle);

    void addPointToMap(frAIburg::map::MapElementType type,
                       frAIburg::map::tMapData x,
                       frAIburg::map::tMapData y,
                       frAIburg::map::tMapData angle);

    std::vector<int> map_el_id_buff_;


    void InitTimer();
    tResult Init(tInitStage stage, __exception);
    tResult Shutdown(tInitStage stage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    tResult SetPinIDs();
    tResult OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                    tInt param2, IMediaSample* media_sample);

    void getAllProperties(void);
    void setAllProperties(void);
    tResult UpdateProperties(ucom::IException** __exception_ptr = NULL);
    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;
    tResult Run(tInt nActivationCode,
                const tVoid* pvUserData, tInt szUserDataSize,
                ucom::IException** __exception_ptr);
    void DeleteTimer();

 public:
    PlannerFilter(const tChar* __info);

};

#endif  // AADCUSER_FRAIBURG_PLANNER_PLANNERFILTER_H_
