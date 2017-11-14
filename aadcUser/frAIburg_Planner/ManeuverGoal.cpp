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

#include "ManeuverGoal.h"

// using namespace frAIburg::map;

// todo: use timeout

// ctor for global goal
void ManeuverGoal::SetManeuverGoal(float goal_global_x, float goal_global_y,
                                   float goal_global_heading,
                                   int behaviour_id) {
    if (isGoalSet(behaviour_id)) return;
    is_active_ = true;
    behaviour_id_current_ = behaviour_id;
    goal_pose_.x = goal_global_x;
    goal_pose_.y = goal_global_y;
    goal_pose_.heading = goal_global_heading;
    normalizeAngleTo2PI(&goal_pose_.heading);
}

bool ManeuverGoal::isGoalSet(int behaviour_id) const {
  return (is_active_ && behaviour_id == behaviour_id_current_);
}

// ctor for local goal, which is transformed into global frame
void ManeuverGoal::SetManeuverGoal(const tPose &pose_curr, float goal_local_x,
                                   float goal_local_y,
                                   float goal_local_relative_angle,
                                   int behaviour_id) {
    if (is_active_ && behaviour_id == behaviour_id_current_) return;
    is_active_ = true;
    behaviour_id_current_ = behaviour_id;
    // transform goal into global frame
    goal_pose_.x = (goal_local_x)*cos(pose_curr.heading) -
                   (goal_local_y)*sin(pose_curr.heading) + pose_curr.x;
    goal_pose_.y = (goal_local_x)*sin(pose_curr.heading) +
                   (goal_local_y)*cos(pose_curr.heading) + pose_curr.y;
    printf("Set ManeuverGoal with local goal x y %f %f\n", goal_local_x,
           goal_local_y);
    printf("Set ManeuverGoal with pose curr x y %f %f\n", pose_curr.x,
           pose_curr.y);
    printf("Set ManeuverGoal with global goal x y %f %f\n", goal_pose_.x,
           goal_pose_.y);
    goal_pose_.heading = pose_curr.heading + goal_local_relative_angle;
    //printf("ManeuverGoal heading before normalizing %f \n", goal_pose_.heading);
    normalizeAngleTo2PI(&goal_pose_.heading);
    //printf("ManeuverGoal heading after normalizing %f \n", goal_pose_.heading);
}

bool ManeuverGoal::isManeuverGoalPassed(const tPose &pose_curr, bool use_flipped_goal,
                                        float dist_offset, float angle_alignment_margin) {

    if (!is_active_) return false;
    float dx_global = goal_pose_.x - pose_curr.x;
    float dy_global = goal_pose_.y - pose_curr.y;
    //printf("ManeuverGoal dx dy to goal is %f %f\n", dx_global, dy_global);
    float dist = sqrt(pow(dx_global, 2) + pow(dy_global, 2));
    float angle_to_goal_relative =
        atan2(dy_global, dx_global) - pose_curr.heading;
    //printf("ManeuverGoal angle_to_goal_relative is %f cos() %f\n",
    //       angle_to_goal_relative, cos(angle_to_goal_relative));
    //printf("ManeuverGoal mindiff curr goal is %f\n",
    //       minDiffAngle(goal_pose_.heading, pose_curr.heading));
    // goal point must be "behind" current position and angle must be aligned +-
    // ANGLE_ALIGNMENT_MARGIN
    bool dist_criterion;
    bool alignment_criterion;
    float min_diff = minDiffAngle(goal_pose_.heading, pose_curr.heading);
    if (use_flipped_goal) {
        dist_criterion = cos(angle_to_goal_relative)*dist > -dist_offset;
        alignment_criterion = min_diff > PI - angle_alignment_margin;
    } else {
        dist_criterion = cos(angle_to_goal_relative)*dist < dist_offset;
        alignment_criterion = min_diff < angle_alignment_margin;
    }
    if (dist_criterion && alignment_criterion) {
        printf("ManeuverGoal completed!\n");
        is_active_ = false;
        return true;
    }
    return false;
}

const tPose& ManeuverGoal::getGoalPose() const {
    return goal_pose_;
}

void ManeuverGoal::normalizeAngleTo2PI(float *target) const {
    if (*target > 2 * PI)
        *target -= 2 * PI;
    else if (*target < 0.)
        *target += 2 * PI;
    return;
}

// returns the abs() of the smallest angle difference;
float ManeuverGoal::minDiffAngle(float angle1, float angle2) const {
    normalizeAngleTo2PI(&angle1);
    normalizeAngleTo2PI(&angle2);
    float diff = fabs(angle1 - angle2);
    if (diff > PI) return fabs(2 * PI - diff);
    return diff;
}

bool ManeuverGoal::isCarAlignedWithGoal(const tPose &pose_curr,
                                        float max_error_rad) {
    if (!is_active_) return false;
    return minDiffAngle(goal_pose_.heading, pose_curr.heading) < max_error_rad;
}

bool ManeuverGoal::isLocalHeadingParallelToGoal(const tPose &pose_curr,
                                                float angle_local,
                                                float max_error_rad) const {
    if (!is_active_) return false;
    float target_global = angle_local +  pose_curr.heading;
    float mindiff = minDiffAngle(target_global, goal_pose_.heading);
    return (mindiff > (PI-max_error_rad)) || (mindiff < max_error_rad);
}

tPose ManeuverGoal::getGoalInLocalFrame(const tPose &pose_curr) const {
    // transform global goal into local frame
    float dx = goal_pose_.x - pose_curr.x;
    float dy = goal_pose_.y - pose_curr.y;
    tPose goal_local;
    goal_local.x =
        (dx)*cos(-pose_curr.heading) - (dy)*sin(-pose_curr.heading);
    goal_local.y =
        (dx)*sin(-pose_curr.heading) + (dy)*cos(-pose_curr.heading);
    goal_local.heading = goal_pose_.heading - pose_curr.heading;
    return goal_local;
}

void ManeuverGoal::updateGoalPoseAfterJump(float dx_global, float dy_global) {
    goal_pose_.x += dx_global;
    goal_pose_.y += dy_global;
}

void ManeuverGoal::orthogonalGoalShift(float dy) {
  goal_pose_.x += dy*cos(goal_pose_.heading + 0.5*PI);
  goal_pose_.y += dy*sin(goal_pose_.heading + 0.5*PI);
}
