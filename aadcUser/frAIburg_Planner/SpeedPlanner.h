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

#ifndef AADCUSER_FRAIBURG_PLANNER_SPEEDPLANNER_H_
#define AADCUSER_FRAIBURG_PLANNER_SPEEDPLANNER_H_

#include <cmath>
#include <iostream>
#include <vector>
#include <cstdio>
#include "nonpin_types.h"


class SpeedPlanner {
 public:
    SpeedPlanner();
    void setGoalPoint(float v_goal, float s_goal);
    void updateCarState(float ds, float v_current_vel);
    float getVTracking();
    void getAccelerationDist(float v_goal, float *s_distance) const;
    void getSampledSpeedPath(float sample_distance,
                         float s0,
                         std::vector<float>* s_vec,
                         std::vector<float>* v_vec);
    void setDebugMode(bool debug_active);
    
 private:
    float v_goal_curr_;
    float s_goal_curr_;
    float v_goal_last_;
    float s_goal_last_;
    float v_tracking_;
    float v_current_vel_;
    float v_goal_tmp_;
    float s_goal_tmp_;
    std::vector<float> path_coeff_;  // v(s) = a+bs+cs^2+ds^3
    float s_current_distance_;  // distance on current path
    bool steady_state_;  // fixed v_tracking, v' ~= 0
    bool threshold_passed_;
    bool isGoalSet_;
    bool IS_DEBUG_MODE_;
    bool use_tmp_goal_;
    float V_MIN_THRESHOLD_; // init speed for getting car to move
    void printStats() const;
    void setPathCoeff(float v0, float v_goal, float s_goal,
                      float dvds_curr, float dvds_goal);
    float getCurrentDirection(float v_goal);
};

#endif  // AADCUSER_FRAIBURG_PLANNER_SPEEDPLANNER_H_
