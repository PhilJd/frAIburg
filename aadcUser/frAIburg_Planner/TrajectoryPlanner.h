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

#ifndef AADCUSER_FRAIBURG_PLANNER_TRAJECTORYPLANNER_H_
#define AADCUSER_FRAIBURG_PLANNER_TRAJECTORYPLANNER_H_

#include "stdafx.h"
#include "aadc_structs.h"
#include <iostream>
#include <vector>
//#include "slim_pins.h"
//#include "customtypes.h"
#include "nonpin_types.h"
#include "PathPlanner.h"
#include "SpeedPlanner.h"

//MAP

#include "global_map.hpp"
#include "map_element.hpp"
#include "map_event_listener.hpp"

class TrajectoryPlanner {
 private:

    // no. of ignored goal point jumps until jump is accepted:
    int MAX_JUMP_CNT_;
    bool LOG_DEBUG_;
    int obj_id_curr_;
    float distance_from_init_;
    //frAIburg::map::GlobalMap* map_;
    float getSpeedProposal(int speed_id) const;

    bool processIdBehaviour(const tBehaviour& behaviour,
                            const tPose &pose_curr,
                            tPath* waypoints, float* goal_orientation,
                            float* dist_to_object, 
                            std::vector<Lane>* lane_vert_vec,
                            bool* skip_pathplanning);
    tPoint getTPoint(float x, float y) const;

 public:
    TrajectoryPlanner();
    PathPlanner path_planner_;
    SpeedPlanner speed_planner_;
    tBehaviour behaviour_curr_;
    planner_status requestTrajectories(const tBehaviour& behaviour, 
                                       const tPose &pose_global,
                                       float v_current, 
                                       std::vector<Lane>* lane_vert_vec = NULL);
    void updateDistance(float ds);
    bool isManeuverFinished(const tPose &pose_curr, float v_curr,
                            const tBehaviour& behaviour);
    void setDebugMode(bool debug_active);

};

#endif  // AADCUSER_FRAIBURG_PLANNER_TRAJECTORYPLANNER_H_
