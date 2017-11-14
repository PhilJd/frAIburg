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

#ifndef AADCUSER_FRAIBURG_PLANNER_MANEUVERPLANNER_H_
#define AADCUSER_FRAIBURG_PLANNER_MANEUVERPLANNER_H_

#include "stdafx.h"
#include "aadc_structs.h"
#include <iostream>
#include <vector>
//#include "slim_pins.h"
#include "customtypes.h"
#include "nonpin_types.h"
#include "PathPlanner.h"
#include <boost/thread/mutex.hpp>
//MAP

#include "global_map.hpp"
#include "map_element.hpp"
#include "map_element_types.h"
#include "map_event_listener.hpp"

enum behaviour_state_parent_enum {BS_PARENT_OBSTACLE, BS_PARENT_NORMALDRIVING,
                                  BS_PARENT_PARKING, BS_PARENT_CARAHEAD, BS_PARENT_LANECHANGE,
                                  BS_PARENT_DEMO, BS_PARENT_PULLRIGHT};

enum behaviour_state_child_enum {BS_OBSTACLE_AHEAD, BS_OBSTACLE_BREAK, BS_OBSTACLE_DRIVEBACK, BS_OBSTACLE_DRIVEAROUND, BS_OBSTACLE_STOPAT,
                                 BS_NORMALDRIVING_STATEMACHINE,
                                 BS_CARAHEAD_FOLLOW, BS_CARAHEAD_OVERTAKE,
                                 BS_PARKING_GOTOSWITCH, BS_PARKING_PARKBACKWARDS, BS_PARKING_STOP,
                                 BS_LANECHANGE_TOLEFT, BS_LANECHANGE_TORIGHT, BS_LANECHANGE_TOCLEAR,
                                 BS_DEMO_GOTOPERSON, BS_DEMO_GOTOINIT,
                                 BS_PULLRIGHT_FWD, BS_PULLRIGHT_BWD};

enum obstacle_type_enum {OT_CAR_BACK, OT_PERSON, OT_UNKNOWN, OT_CLEAR}; // OT_FULLBLOCK

class ManeuverPlanner {
 private:

    struct memory_state_struct {
      behaviour_state_parent_enum parent_state;
      behaviour_state_child_enum child_state;
    };

    std::list<memory_state_struct> behaviour_return_list_;
    PathPlanner* path_planner_;
    bool LOG_DEBUG_;
    bool deadlock_detected_;
    tTimeStamp time_since_stopped_;
    tTimeStamp current_time_;
    boost::mutex update_behaviour_mutex_;

    void TransitionToState(behaviour_state_parent_enum new_parent,
                           behaviour_state_child_enum new_child);
    void TransitionToPreviousState();
    void enterParentStateNormaldriving();
    void enterParentStateLanechange();
    void enterParentStateObstacle();
    void enterParentStateParking();
    void enterParentStateCarAhead();
    void enterParentStateDemo();
    void enterParentStatePullout();
    void addChildParentStatesToList();
    void setObstacleAheadBehaviour();
    obstacle_type_enum getObstacleAheadType(int object_id, float *distance_to_obst = NULL);
    void overwriteDisableEMBehaviour();
    bool wasNormalDrivingBefore();
    void executeCurrentState();
    void printBehaviourIfChanged();
    void printFullState();
    bool isFinished(const tBehaviour& behaviour);

 public:
    // only write using the setBehaviour method!!
    tBehaviour behaviour_prev_;
    tBehaviour behaviour_curr_;
    tBehaviour behaviour_finished_;
    tBehaviour behaviour_statemachine_;
    tBehaviour behaviour_emergency_;
    tBehaviour behaviour_demo_;

    behaviour_state_parent_enum behaviour_state_parent_;
    behaviour_state_child_enum behaviour_state_child_;
    bool is_maneuver_finished_;


    ManeuverPlanner(PathPlanner* path_planner = NULL);
    void setDebugMode(bool debug_active);

    tBehaviour getUpdatedBehaviour();
    void updateTimestamp(tTimeStamp time);
    void setBehaviour(tInt32 behaviour_id, tInt32 behaviour_next_id, tInt32 object_id,
                      tInt32 speed_id, tBehaviour* target);
    void setBehaviour(tBehaviour& behaviour_new, tBehaviour* target);
    bool isBehaviourEqual(const tBehaviour& behaviour1, const tBehaviour& behaviour2);

};

#endif  // AADCUSER_FRAIBURG_PLANNER_ManeuverPlanner_H_
