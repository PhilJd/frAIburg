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
#ifndef AADCUSER_FRAIBURG_PLANNER_MANEUVERGOAL_H_
#define AADCUSER_FRAIBURG_PLANNER_MANEUVERGOAL_H_
//#include "stdafx.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <vector>
#include "nonpin_types.h"
//#include "customtypes.h"



class ManeuverGoal {
 public:
    // ManeuverGoal();
    void SetManeuverGoal(float goal_global_x, float goal_global_y,
                    float goal_global_heading, int behaviour_id);
    void SetManeuverGoal(const tPose &pose_curr, float goal_local_x, float goal_local_y, 
                    float goal_local_relative_angle, int behaviour_id);

    bool isManeuverGoalPassed(const tPose &pose_curr, bool use_flipped_goal = false,
                              float dist_offset = 0., float angle_alignment_margin = 0.25*PI);
    bool isGoalSet(int behaviour_id) const;
    float minDiffAngle(float angle1, float angle2) const;
    bool isCarAlignedWithGoal(const tPose &pose_curr,
                              float max_error_rad);
    tPose getGoalInLocalFrame(const tPose &pose_curr) const;
    bool isLocalHeadingParallelToGoal(const tPose &pose_curr,
                                      float angle_local,
                                      float max_error_rad) const;
    void updateGoalPoseAfterJump(float dx_global, float dy_global);
    void orthogonalGoalShift(float dy);
    const tPose& getGoalPose() const;
    void normalizeAngleTo2PI(float* target) const;
    // virtual ~ManeuverGoal();
    tPose goal_pose_;
    bool is_active_;
    int behaviour_id_current_;

 private:
    bool IS_DEBUG_MODE_;
};

#endif  // AADCUSER_FRAIBURG_PLANNER_ManeuverGoal_H_
