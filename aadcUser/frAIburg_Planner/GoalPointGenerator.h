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
#ifndef AADCUSER_FRAIBURG_PLANNER_GOALPOINTGENERATOR_H_
#define AADCUSER_FRAIBURG_PLANNER_GOALPOINTGENERATOR_H_

#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>
#include "nonpin_types.h"
#include <stdio.h>

class GoalPointGenerator {
 private:
  tPose pose_global_t0_;
  tPose pose_global_prev_;
  std::vector<float> distance_vec_;
  int getClosestPointIdx(const tPoint &pos_local) const;
  int getLookaheadPointIdx(int start_idx, float lookaheaddist) const;
  tPoint getPointInInitialFrame(const tPose &pose_curr) const;
  bool isInit_;
  float path_length_;
  bool isPathSet_;
  bool IS_DEBUG_MODE_;
  bool use_rear_anchor_;
 public:
  GoalPointGenerator();
  tPath* current_path_;
  void setNewPath(tPath* path, const tPose &pose_curr);
  void updateInitialPoseAfterJump(float dx_global, float dy_global);
  bool getUpdatedGoal(const tPose &pose_curr, float lookaheaddist, 
                      tPoint* const goal_point, bool use_rear_anchor = false);
  bool getPathLength(float* const path_length) const;

  bool getPerpendicularPoint(const tPose &pose_curr, const tPoint& target_pnt,
                             tPoint* result) const;
  void setDebugMode(bool debug_active);
};
#endif  // AADCUSER_FRAIBURG_PLANNER_GOALPOINTGENERATOR_H_
