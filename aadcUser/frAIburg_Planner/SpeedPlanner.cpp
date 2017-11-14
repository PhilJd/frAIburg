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
#include "SpeedPlanner.h"

SpeedPlanner::SpeedPlanner() {
    V_MIN_THRESHOLD_ = 0.5 * SPEED_LOW_;
    s_current_distance_ = 0;
    v_goal_curr_ = 0.;
    s_goal_curr_ = 0.;
    // v_goal_last_ = 0.;
    // s_goal_last_ = 0.;
    v_tracking_ = 0.;
    v_current_vel_ = 0.;
    v_goal_tmp_ = 0.;
    s_goal_tmp_ = 0.;
    // we'll use 3rd order polynomial for v(s) trajectory
    path_coeff_.resize(4, 0.);
    steady_state_ = true;
    isGoalSet_ = false;
    use_tmp_goal_ = false;
    threshold_passed_ = false;
    IS_DEBUG_MODE_ = false;
}

void SpeedPlanner::setGoalPoint(float v_goal, float s_goal) {
    /******************************************************
                    Conditions for planning goal
     *****************************************************/
    if (s_goal <= 0.) {
        if (IS_DEBUG_MODE_) {
            printf("SpeedPlanner: s_goal <= 0, abort planning! \n");
        }
        if (v_goal == 0.) {
            path_coeff_.resize(4, 0.);
            v_goal_curr_ = 0.;
        }
        return;
    }
    // initiate v0 correctly with eiher last tracking value
    // or current velocity
    float v0;
    if (fabs(v_current_vel_) > fabs(v_tracking_)) {
        if (fabs(v_goal) > fabs(v_current_vel_))
            v0 = v_current_vel_;
        else
            v0 = v_tracking_;
    } else {
        if (fabs(v_goal) > fabs(v_current_vel_))
            v0 = v_tracking_;
        else
            v0 = v_current_vel_;
    }
    s_goal = std::max(0.05, 1. * s_goal);
    float goal_change = (s_goal_curr_ - s_current_distance_) - s_goal;
    // average change of velocity over distance as init. value
    float dvds_curr = (v_goal - v_current_vel_) / s_goal;

    /* Don't replan when we accelerate; trade accuracy for smoothness */
    if (v_goal_curr_ == v_goal && fabs(v_goal) >= V_MIN_THRESHOLD_ &&
        !use_tmp_goal_) {
        // printf("Abort speedplanning 87\n"); printStats();
        return;
    }
    /* If we constantly set new goal, the path cannot be sampled correctly;
       This case handles breaking.
       therefore skip update when: */
    if (fabs(goal_change) < 0.1 && v_goal_curr_ == v_goal && !use_tmp_goal_) {
        // printf("Abort speedplanning 94\n"); printStats();
        return;
    }

    /* If we're driving too fast, slightly lower v0 and decelerating
       slope */
    if (fabs(v_current_vel_) > 1.1 * SPEED_ULTRAHIGH_) {
        float curr_direction_sign = getCurrentDirection(v_goal);
        v0 = curr_direction_sign * SPEED_HIGH_;
        dvds_curr = -curr_direction_sign * 0.1;
    }
    /******************************************************
                      Actual goal computation
     *****************************************************/
    v_goal_tmp_ = v_goal;
    s_goal_tmp_ = s_goal;
    // if not otherwise specified, try to finish path with zero slope
    float dvds_goal = 0.;

    /* Avoid breaking too early; plan temporary goal until to
       breaking distance */
    if (v_goal == 0.) {
        float dist_to_goal = s_goal_curr_ - s_current_distance_;
        float breaking_dist =
            0.5;  // todo: might need adjustment, depends on state machine
        // if we stopped to early but there's distance to cover, set tmp goal to
        // accelerate:
        if (!threshold_passed_ && dist_to_goal > 0.2) { // 3.11: before 0.1
            s_goal_tmp_ = 0.5 * dist_to_goal;
            v_goal_tmp_ = getCurrentDirection(v_goal) * SPEED_LOW_; // f 2.11: before 0.5*
            dvds_curr = (v_goal_tmp_ - v_current_vel_) / s_goal_tmp_;
            use_tmp_goal_ = true;
        }
        // 0.5m before stop we want to reach low velocity
        else if (dist_to_goal > breaking_dist + 0.1) {
            s_goal_tmp_ = dist_to_goal - breaking_dist;
            v_goal_tmp_ =
                getCurrentDirection(v_goal) *
                SPEED_LOW_;  // todo: needs adjustment for backwards driving
            dvds_curr = (v_goal_tmp_ - v_current_vel_) / s_goal_tmp_;
            use_tmp_goal_ = true;
        } else {
            dvds_goal =
                -0.1 * ((v_current_vel_ >= 0.) ? 1. : -1.);  //  7.10 CHANGE: 0.05
                                                              //  before 0.01;
                                                              //  leads to more
                                                              //  abrupt
                                                              //  stopping
            use_tmp_goal_ = false;
        }

        if (IS_DEBUG_MODE_ && use_tmp_goal_)
            printf(
                "*** SpeedPlanner: Use temp. goal"
                " to accelerate (s,v) %f, %f\n",
                s_goal_tmp_, v_goal_tmp_);
    }

    // if (IS_DEBUG_MODE_) {
    //   printf("set speed path with v0 %f dvdscurr %f dvdsgoal %f\n",
    //   v_current_vel_, dvds_curr, dvds_goal);
    // }
    setPathCoeff(v0, v_goal_tmp_, s_goal_tmp_, dvds_curr, dvds_goal);

    /* update goal */
    // s_goal_last_ = s_goal_curr_;
    // v_goal_last_ = v_goal_curr_;
    s_goal_curr_ = s_goal;
    v_goal_curr_ = v_goal;
    s_current_distance_ = 0.;
    steady_state_ = false;
    isGoalSet_ = true;

    if (IS_DEBUG_MODE_) {
        printStats();
    }
}

float SpeedPlanner::getVTracking() {
    if (!isGoalSet_) {
        // std::printf("SpeedPlanner: Cannot update v_tracking_; no goal
        // set.\n");
        return 0.;
    }
    float v_goal_tracking;
    float dist_to_goal;
    if (use_tmp_goal_) {
        dist_to_goal = s_goal_tmp_ - s_current_distance_;
        v_goal_tracking = v_goal_tmp_;
    } else {
        v_goal_tracking = v_goal_curr_;
        dist_to_goal = s_goal_curr_ - s_current_distance_;
    }

    float s_offset = 0.1;
    if (!threshold_passed_ &&
        (/*(dist_to_goal > 0.2) ||*/ (fabs(v_goal_tracking) > 0.))) {
        // adaptive offset/lookahead for smooth acceleration
        s_offset =
            std::max(0., 1. - 1. * fabs(v_current_vel_ / V_MIN_THRESHOLD_)) *
            s_offset;
        s_offset = std::min(1. * dist_to_goal, 1. * s_offset);
        // printf("SpeedPlanner: Use offset of %f\n", s_offset);
    } else
        s_offset = 0.;

    v_tracking_ = 0.;
    if (!steady_state_) {
        for (unsigned int i = 0; i < path_coeff_.size(); ++i) {
            v_tracking_ +=
                path_coeff_[i] * pow(s_current_distance_ + s_offset, i);
        }
    } else {
        v_tracking_ = v_goal_curr_;
    }

    // todo: filter out jumps in v_tracking, just in case of weird timing issues

    if (IS_DEBUG_MODE_) {
        printf("SpeedPlanner: Use offset of %f\n", s_offset);
        printf("SpeedPlanner: v_tr is %f, s_curr_dist %f, v_goal %f\n",
               v_tracking_, s_current_distance_, v_goal_curr_);
        // assert(!(v_goal_curr_>= 0 && use_tmp_goal_));
        // assert(v_goal_curr_*v_tracking_ >= 0.);
    }
    return v_tracking_;
}

void SpeedPlanner::updateCarState(float ds, float v_current_vel) {
    s_current_distance_ += ds;
    float dist_to_goal = s_goal_curr_ - s_current_distance_;
    if ((dist_to_goal <= 0.05) ||
        (dist_to_goal < 0.2 && fabs(v_current_vel_) <= SPEED_STOPPED_)) { // 3.11: before 0.1
        steady_state_ = true;
        // if (IS_DEBUG_MODE_) printf("Speed Planner reached Steady State \n");
    }
    /* threshold important to determine when car is moving; for starting from
       v=0,
       v_tracking will be modified */
    v_current_vel_ = v_current_vel;
    if (fabs(v_current_vel_) > V_MIN_THRESHOLD_) threshold_passed_ = true; // before fabs.. didnt work for backwards driving
    if (fabs(v_current_vel_) < SPEED_STOPPED_) threshold_passed_ = false;
}

void SpeedPlanner::setPathCoeff(float v0, float v_goal, float s_goal,
                                float dvds_curr, float dvds_goal) {
    if (s_goal == 0.) {
        printf("SpeedPlanner: setPathCoeff failed, s_goal = 0.\n");
        return;
    }
    /* v(s) = a+bs+cs^2+ds^3
     * v(0) = v_current;
     * v(s_goal) = v_goal;
     * v'(s=0) = dv(s)/ds * dv/dt = a_curr
     * v'(T) = 0 // we want to end at constant velocity
     * note that ' is time derivative
     * See matlab-script for symbolic derivation
     * */
    if (fabs(dvds_curr) > ACC_MAX_) {
        if (dvds_curr >= 0)
            dvds_curr = ACC_MAX_;
        else
            dvds_curr = -ACC_MAX_;
    }
    path_coeff_[0] = v0;
    path_coeff_[1] = dvds_curr;
    path_coeff_[2] =
        -(3 * v0 - 3 * v_goal + dvds_goal * s_goal + 2 * dvds_curr * s_goal) /
        pow(s_goal, 2);
    path_coeff_[3] =
        (2 * v0 - 2 * v_goal + dvds_goal * s_goal + dvds_curr * s_goal) /
        pow(s_goal, 3);
}

float SpeedPlanner::getCurrentDirection(float v_goal) {
    // todo: moving average of lasts speed to determine driving direction
    float current_direction = 1.;
    /*if (fabs(v_goal) > 0.) {
      current_direction = v_goal > 0. ? 1. : -1.;
    } else if (fabs(v_current_vel_) > 0.1) {
      current_direction = v_current_vel_ > 0. ? 1. : -1.;
    } else {
      current_direction = v_tracking_ >= 0. ? 1. : -1.;
    }*/
    return current_direction;
}

/********************************************************************
 *********************** HELPER ROUTINES ***************************/

void SpeedPlanner::printStats() const {
    printf("_____\n");
    printf("s_goal_curr | v_goal_curr are: %fm | %fm/s \n", s_goal_curr_,
           v_goal_curr_);
    // printf("s_goal_last | v_goal_last are: %fm | %fm/s \n", s_goal_last_,
    // v_goal_last_);
    printf("s_current_distance_ is %f \n", s_current_distance_);
    printf("Steady state is %s \n", steady_state_ ? "reached" : "not reached");
    printf("Threshold is %s \n", threshold_passed_ ? "passed" : "not passed");
    printf("Path coeff are: | ");
    for (int i = 0; i < path_coeff_.size(); ++i) {
        printf("%f | ", path_coeff_[i]);
    }
    printf("\n");
    printf("v_current is %f \n", v_current_vel_);
    printf("_____\n");
}

void SpeedPlanner::setDebugMode(bool debug_active) {
    IS_DEBUG_MODE_ = debug_active;
}

void SpeedPlanner::getSampledSpeedPath(float sample_distance, float s0,
                                       std::vector<float>* s_vec,
                                       std::vector<float>* v_vec) {
    if (!s_vec || !v_vec || !isGoalSet_) {
        printf("getSampledSpeedPath failed \n");
        return;
    }
    s_vec->resize(0);
    v_vec->resize(0);
    float s_curr = 0;
    float ds = 0.01;
    while (s_curr < sample_distance) {
        float v_tracking = 0.;
        if (s_curr < fabs(s_goal_curr_)) {
            for (int i = 0; i < path_coeff_.size(); ++i) {
                v_tracking += path_coeff_[i] * pow(s_curr, i);
            }
        } else {
            v_tracking = v_goal_curr_;
        }
        s_vec->push_back(s_curr + s0);
        v_vec->push_back(v_tracking);
        s_curr += ds;
    }
    // printf("SampleSpeedPath, return path size is %i\n", s_vec->size());
    // printStats();
}

/* Depending on the goal and current speed, this method computed the distance
   until
   which the speed should be reached when accelerating */
void SpeedPlanner::getAccelerationDist(float v_goal, float* s_distance) const {
    *s_distance =
        std::max(0.1, 1. * fabs((v_current_vel_ - v_goal) / ACC_MAX_));
}

/* Limit avg. acceleration to predefined value,
   increase s if necessary (leading to possibly
   shorter stopping distances in total, since slip
   is minimized)*/
/* calculate mean acceleration dv/dt required for goal
   s = 1/(2a)*(dv^2 + v0*dv) [use: s = 0.5at^2 + v0*t; v = at + v0] */
/*  float v_term = pow((v_current_vel-v_goal), 2) + v_goal*v_current_vel;
float acc_mean = 0.5*v_term/s_goal;
// printf("mean acc is %f\n", acc_mean);
if (fabs(acc_mean) > ACC_MAX_) {
  s_goal = 0.5*fabs(v_term)/ACC_MAX_;
  printf("Acc. too high! Increased s_goal to %f\n", s_goal);
}*/
