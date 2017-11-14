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
#include "GoalPointGenerator.h"
GoalPointGenerator::GoalPointGenerator() {
    isPathSet_ = false;
    current_path_ = NULL;
    IS_DEBUG_MODE_ = false;
    use_rear_anchor_ = false;
}

int GoalPointGenerator::getClosestPointIdx(const tPoint& pos_local) const {
    int max_iter = current_path_->size();
    // start from index of prev. closest point
    int iter = 0;
    int min_dist_idx = 0;
    float min_dist = 9999999.0;
    //float anchor_offset = use_rear_anchor_?CAR_ANCHOR_OFFSET_BWD_:CAR_ANCHOR_OFFSET_;

    while (iter < max_iter) {
        float dist = 
            sqrt(pow(pos_local.x - ((*current_path_)[iter].x), 2.0) +
                 pow(pos_local.y - (*current_path_)[iter].y, 2.0));
        if (dist < min_dist) {
            min_dist_idx = iter;
            min_dist = dist;
        }
        iter++;
    }
    return min_dist_idx;
}

int GoalPointGenerator::getLookaheadPointIdx(int start_idx,
                                             float lookaheaddist) const {
    int max_iter = distance_vec_.size();
    int iter = start_idx;
    float dist_sum = 0;
    // find index of point with given distance along the path
    while (iter < max_iter && dist_sum < lookaheaddist) {
        dist_sum += distance_vec_[iter];
        iter++;
    }
    iter = std::min(iter, (int)(current_path_->size()) - 1);
    // printf("closest point is %f %f \n", (*current_path_)[iter].x,
    // (*current_path_)[iter].y);
    return iter;
}

void GoalPointGenerator::setNewPath(tPath* path, const tPose& pose_curr) {
    if (path == NULL || path->size() == 0) {
        printf("Warning: GPGen failed to set new path\n");
        return;
    }
    if (IS_DEBUG_MODE_)
        printf("GPGen: New path with pose (x y heading) %f %f %f\n",
               pose_curr.x, pose_curr.y, pose_curr.heading);
    current_path_ = path;
    pose_global_t0_ = pose_curr;
    pose_global_prev_ = pose_curr;
    distance_vec_.clear();
    distance_vec_.reserve(path->size());
    path_length_ = 0.;
    // compute all distances once
    for (int i = 0; i < current_path_->size() - 1; ++i) {
        //printf("path (x, y) %f %f \n", (*current_path_)[i].x, (*current_path_)[i].y); // remove todo
        distance_vec_.push_back(
            sqrt(pow((*current_path_)[i + 1].x - (*current_path_)[i].x, 2.0) +
                 pow((*current_path_)[i + 1].y - (*current_path_)[i].y, 2.0)));
        path_length_ += distance_vec_.back();
    }
    isPathSet_ = true;
}

// return lookahead point from POV of anchor point of car

bool GoalPointGenerator::getUpdatedGoal(const tPose& pose_curr,  // make bool
                                        float lookaheaddist,
                                        tPoint* const goal_point,
                                         bool use_rear_anchor) {
    static int timeout_cnt = 0;
    static bool reached_last = false;
    static int cnt = 0;

    use_rear_anchor_ = use_rear_anchor;
    if (!current_path_) {
        // printf("GoalPointGenerator::getUpdatedGoal failed, path NULL\n");
        return false;
    }
    cnt++;
    // avoid spamming the log
    if (timeout_cnt == 40) {
        printf(
            "GPGen: No Path set. Cannot update goal point. "
            "Check if path set and position init.\n");
        timeout_cnt = 0;
    }
    if (!isPathSet_ || goal_point == NULL) {
        timeout_cnt += 1;
        return false;
    } else {
        timeout_cnt = 0;
    }

    // first get anchor point in coordinates of old frame in which path was
    // generated
    tPoint pos_local = getPointInInitialFrame(pose_curr);
    // now get anchor point pos. that we want to know closest point to:
    float dphi = pose_curr.heading - pose_global_t0_.heading;
    float anchor_offset = use_rear_anchor?CAR_ANCHOR_OFFSET_BWD_:CAR_ANCHOR_OFFSET_;
    pos_local.x -= anchor_offset*cos(dphi);
    pos_local.y -= anchor_offset*sin(dphi);
    // from POV of updated position get closest point in old coordinates
    int lk_idx =
        getLookaheadPointIdx(getClosestPointIdx(pos_local), lookaheaddist);

    reached_last = (lk_idx == current_path_->size() -1);

    tPoint goalpoint_curr_frame;

    goalpoint_curr_frame = (*current_path_)[lk_idx];
    if (!reached_last) { // get point at lookahead index
        goalpoint_curr_frame = (*current_path_)[lk_idx];
    }
    else if (current_path_->size() > 1) { // linearly extrapolate when last idx reached
        float dx = (*current_path_)[current_path_->size()-1].x -
                   (*current_path_)[current_path_->size()-2].x;
        float dy = (*current_path_)[current_path_->size()-1].y -
                   (*current_path_)[current_path_->size()-2].y;
        float dx_unit_length_increase = fabs(dx/sqrt(pow(dx,2)+pow(dy,2)));
        float lookahead_ext = 0.4; // length of new path segment
        float x_dir = dx > 0?1.:-1.; // which x-direction to go into?
        float x_step = x_dir*lookahead_ext*dx_unit_length_increase;
        goalpoint_curr_frame.x = (*current_path_)[current_path_->size()-1].x + x_step;
        goalpoint_curr_frame.y = (*current_path_)[current_path_->size()-1].y + dy/dx * x_step;
        // printf("x at lookahead is %f", (*current_path_)[current_path_->size()-1].x);
	    // printf("y at lookahead is %f", (*current_path_)[current_path_->size()-1].y);
        // printf("xstep is %f, y step %f\n", x_step, dy/dx*x_step);
    } else {
        return false;
    }

    // Transform these coordinates to new frame
    tPoint goalpoint_old_frame;
    // assume phi positive around z axis and yaw in [rad]:
    goalpoint_old_frame.x = (goalpoint_curr_frame.x - pos_local.x) * cos(-dphi) -
                            (goalpoint_curr_frame.y - pos_local.y) * sin(-dphi);
    goalpoint_old_frame.y = (goalpoint_curr_frame.x - pos_local.x) * sin(-dphi) +
                            (goalpoint_curr_frame.y - pos_local.y) * cos(-dphi);
    *goal_point = goalpoint_old_frame;

    if (IS_DEBUG_MODE_) {
        printf("GPGen: current (dx dy dphi) since planning: %f %f %f\n",
               pos_local.x, pos_local.y, -dphi);
        printf("GPGen: lookahead pnt (x y) in planning frame %f %f\n",
               goalpoint_curr_frame.x, goalpoint_curr_frame.y);
        printf("GPGen: New lookahead pnt (x y) %f %f\n", goalpoint_old_frame.x,
               goalpoint_old_frame.y);
    }

    return true;
}

tPoint GoalPointGenerator::getPointInInitialFrame(
    const tPose& pose_curr) const {  // do inline!?
    tPoint pos_local;
    // dx, dy in global frame
    float dx = pose_curr.x - pose_global_t0_.x;
    float dy = pose_curr.y - pose_global_t0_.y;

    // dx, dy in old frame, rotate shifted current frame cw by
    // pose_global_t0_.heading
    pos_local.x =
        (dx)*cos(-pose_global_t0_.heading) - (dy)*sin(-pose_global_t0_.heading);
    pos_local.y =
        (dx)*sin(-pose_global_t0_.heading) + (dy)*cos(-pose_global_t0_.heading);
    return pos_local;
}

bool GoalPointGenerator::getPathLength(float* const path_length) const {
    if (!isPathSet_) return false;
    *path_length = path_length_;
    return true;
}

void GoalPointGenerator::setDebugMode(bool debug_active) {
    IS_DEBUG_MODE_ = debug_active;
}

/*
bool GoalPointGenerator::getPerpendicularPoint(const tPose& pose_curr,
                                               const tPoint& target_pnt,
                                               tPoint* result) const {
    if (!current_path_ || isPathSet_) {
        printf("GoalPointGenerator::getPerpendicularPoint failed, path NULL\n");
        return false;
    }
    int result_pnt_idx = -1;
    tPoint pos_local = getPointInInitialFrame(pose_curr);
    float min_angle_abs = 1e9;
    for (int i = 0; i < current_path_->size(); ++i) {
        float angle_abs = fabs(atan2(target_pnt.y - (*current_path_)[i].y,
                                     target_pnt.x - (*current_path_)[i].x));
        if (fabs(angle_abs - 0.5 * PI) < min_angle_abs) {
            result_pnt_idx = i;
            min_angle_abs = angle_abs;
        }
    }
    if (result_pnt_idx != -1) {
        *result = (*current_path_)[result_pnt_idx];
        return true;
    } else {
        return false;
    }
}*/

void GoalPointGenerator::updateInitialPoseAfterJump(float dx_global,
                                                    float dy_global) {
    pose_global_t0_.x += dx_global;
    pose_global_t0_.y += dy_global;
}
