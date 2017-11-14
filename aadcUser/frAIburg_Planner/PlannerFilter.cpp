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

/* This filter deals with incoming and outgoing information, commands the globa
   planner to compute new trajectories and feeds control set point to speed and
   steering
   controller */

#include "PlannerFilter.h"

using namespace frAIburg::map;

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, PlannerFilter)

/* string arrays below for debugging; MAKE SURE IT'S SYNCHED with enums in
 * nonpin_types.h*/

const char* planner_status_string[] = {"PS_STAT_OBSTACLE", "PS_DYN_OBSTACLE",
                                       "PS_SUCCESSFUL",    "PS_FAILED",
                                       "PS_NO_LANES",      "PS_COMPLETED"};

PlannerFilter::PlannerFilter(const tChar* __info)
    : cFilter(__info),
      LOG_DEBUG_(true),
      DELTA_T_REPLAN_(0.05),
      behav_completed_sent_(true),
      maneuver_planner_(&trajectory_planner_.path_planner_),
      speed_curr_(0.),
      isPositionInit_(false),
      v_tracking_curr_(0.),
      v_tracking_prev_(0.),
      processed_new_lanevec_(true) {
    pose_curr_.x = 0.;
    pose_curr_.y = 0.;
    pose_curr_.heading = 0.;
    pose_prev_ = pose_curr_;
    maneuver_planner_.setBehaviour(STOP, BT_UNKNOWN, -1, ST_STOP,
                                   &behaviour_curr_);

    setAllProperties();
}

/******************* Main thread loop ************************************/

tResult PlannerFilter::Run(tInt nActivationCode, const tVoid* pvUserData,
                           tInt szUserDataSize,
                           ucom::IException** __exception_ptr) {
    if (running_ok_) {
        // add goal point visualisation
       //  if (LOG_DEBUG_) {
       //      ManeuverGoal& goal = trajectory_planner_.path_planner_.
       //                           goal_global_;
       //      if (goal.isGoalSet(behaviour_curr_.behaviour_id)) {
       //        addPointToMap(DEBUG_POINT, goal.getGoalInLocalFrame(pose_curr_).x,
       //                      goal.getGoalInLocalFrame(pose_curr_).y,
    			// goal.getGoalInLocalFrame(pose_curr_).heading);
       //      }
       //  }
        // update behaviour and replan
        if (isPositionInit_) {
            //printf("Pf: update behaviour curr\n");

            behaviour_curr_ = maneuver_planner_.getUpdatedBehaviour();
            /*if (behaviour_curr_.behaviour_id != maneuver_planner_.behaviour_curr_.behaviour_id) {
                printf("WARNING! STATES HAVE DIVERGED!!! (PF: %i MP: %i)\n", behaviour_curr_.behaviour_id,
                    maneuver_planner_.behaviour_curr_.behaviour_id);
            }*/
            if (isReplanNecessary()) {
                //printf("Pf: isReplanNecessary true \n");
                actOnCurrentBehaviour(); // maybe something goes missing here?
            }
            // set lights
            setLightForBehaviour();
            CheckTransmitObstaclePosition();
        }
        // update maneuver status
        if (behaviour_curr_.behaviour_id != FOLLOW_LANE) {
            __synchronized_obj(update_pose_mutex_);
            maneuver_planner_.is_maneuver_finished_ =
                trajectory_planner_.isManeuverFinished(pose_curr_, speed_curr_,
                                                       behaviour_curr_);
            if (maneuver_planner_.is_maneuver_finished_) {
		        //printf("Pf: entered maneuver finished, try to transmit:\n");
                doManeuverFinishedActions();
                //printf("Pf: entered maneuver finished, update behaviour:\n");
                //behaviour_curr_ = maneuver_planner_.getUpdatedBehaviour(); // new 7.11
                //maneuver_planner_.is_maneuver_finished_ = false; // also new 5.11
                //printf("Pf: entered maneuver finished, behaviour updated\n");

            }
        }

        // update time stamp for drawing paths
        trajectory_planner_.path_planner_.updateTimestamp(
            _clock->GetStreamTime());
        maneuver_planner_.updateTimestamp(_clock->GetStreamTime());
    }
    RETURN_NOERROR;
}

/********************* Main thread loop for external
 * triggers/events*************/

tResult PlannerFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                  tInt param2, IMediaSample* media_sample) {
    RETURN_IF_POINTER_NULL(media_sample);
    RETURN_IF_POINTER_NULL(source);
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        /* ----- process lanes only if in lanefollow-mode and
                 last lane of batch has been received */
        if (tLaneElement_vert_input_pin_.isSource(source)) {
            processLaneElement(media_sample);
            /* ----- update car's speed and position in global frame
                     get and transmit v_tracking to speed controller*/
        } else if (tPos_input_pin_.isSource(source)) {
            if (!processPose(media_sample)) {
                printf("PlannerFilter couldn't update pose from mediasample");
                RETURN_NOERROR;
            }
            isPositionInit_ = true;
            
            transmitGoalpoint(
                getGoalpoint());  // todo: return reference instead tPoint

            /* ----- update distance driven since last planning
                     check if maneuver finished
                     replan if necessary */
            correctRepositioningJump();
        } else if (ds_input_pin_.isSource(source) && isPositionInit_) {
            float ds;
            if (processDeltaDistance(media_sample, &ds)) {
                trajectory_planner_.updateDistance(ds);
                trajectory_planner_.speed_planner_.updateCarState(ds,
                                                                  speed_curr_);
            }
            // send 0 to init. motor
            if (_clock->GetStreamTime() - init_time_ > 4. * 1e6) {
                transmitVTracking(getSpeedValue());
            } else {
                transmitVTracking(0.);
            }
            // ----- update current behaviour and act
        } else if (behaviour_input_pin_.isSource(source)) {
            processBehaviour(media_sample);
        }
    }
    RETURN_NOERROR;
}

bool PlannerFilter::isReplanNecessary() {
    switch (behaviour_curr_.behaviour_id) {
        case FOLLOW_LANE:
        case FOLLOW_LANE_TO_CLEAR:
        case FOLLOW_CAR:
        case GO_TO_OBJECT:
        case STOP_AT_OBJECT:
        case PARK_AT_OBJECT:
        case GO_STRAIGHT:
        case TURN_LEFT:
        case TURN_RIGHT:
        case CHANGELANE_TO_LEFT:
        case CHANGELANE_TO_RIGHT:
        //case GO_BACKWARDS:
        case DEMO_GO_TO_PERSON:
        case DEMO_GO_TO_INIT:
        case PULL_RIGHT_BWD:
            return true;
        default:
            return (!maneuver_planner_.isBehaviourEqual(
                    maneuver_planner_.behaviour_curr_,
                    trajectory_planner_.behaviour_curr_));
    }
}

planner_status PlannerFilter::actOnCurrentBehaviour() {
    /* Requests global planner to plan path depending on requested behaviour */

    // enable ultrasonic before lanechange; deactive when finished
    if (behaviour_curr_.behaviour_id == FOLLOW_LANE_TO_CLEAR /*||
        behaviour_curr_.behaviour_id == CHANGELANE_TO_LEFT*/) //CHANGELANE_TO_LEFT
        TransmitEnableUSRight(true);

    __synchronized_obj(update_pose_mutex_);
    if (behaviour_curr_.behaviour_id == FOLLOW_LANE ||
        behaviour_curr_.behaviour_id == FOLLOW_LANE_TO_CLEAR ||
        behaviour_curr_.behaviour_id == GO_BACKWARDS ||
        behaviour_curr_.behaviour_id == FOLLOW_CAR ||
        behaviour_curr_.behaviour_id == CHANGELANE_TO_LEFT ||
        behaviour_curr_.behaviour_id == CHANGELANE_TO_RIGHT) {
        return doFollowLane();
        // GO_TO/STOP_AT_OBJECT: give lanes if possible
    } else if ((behaviour_curr_.behaviour_id == STOP_AT_OBJECT ||
                behaviour_curr_.behaviour_id == GO_TO_OBJECT) ||
               behaviour_curr_.behaviour_id == GO_AROUND_OBJECT) {
        return doObjectApproach(); // todo: hand over lane_vec
    } else if (behaviour_curr_.behaviour_id == TURN_LEFT ||
               behaviour_curr_.behaviour_id == TURN_RIGHT ||
               behaviour_curr_.behaviour_id == GO_STRAIGHT) {
        return doCrossing();
    } else if (behaviour_curr_.behaviour_id == STOP_AT_PARKING_INFRONT ||
               behaviour_curr_.behaviour_id == PARK_AT_OBJECT) {
        doParking();
    } /*else if (behaviour_curr_.behaviour_id == DEMO_GO_TO_INIT ||
               behaviour_curr_.behaviour_id == DEMO_GO_TO_PERSON) {
        doDemoTask();
    }*/
    else { // todo: switch, list all behaviours!
        // pull_out and other behaviours
        return trajectory_planner_.requestTrajectories(
            behaviour_curr_, pose_curr_, speed_curr_, NULL);
    }
    return PS_SUCCESSFUL;
}

planner_status PlannerFilter::doParking() {
    if (behaviour_curr_.behaviour_id == STOP_AT_PARKING_INFRONT) {
        return trajectory_planner_.requestTrajectories(
            behaviour_curr_, pose_curr_, speed_curr_, NULL);
    } else if (behaviour_curr_.behaviour_id == PARK_AT_OBJECT) {
        TransmitEnableParkingRearView(true);
        TransmitEnableFrontView(false);
        return trajectory_planner_.requestTrajectories(
            behaviour_curr_, pose_curr_, speed_curr_, &lane_vert_vec_);
    }
    return PS_SUCCESSFUL;
}


planner_status PlannerFilter::doCrossing() {

    __synchronized_obj(update_lane_mutex_);
    processed_new_lanevec_ = true;
    return trajectory_planner_.requestTrajectories(
        behaviour_curr_, pose_curr_, speed_curr_, &lane_vert_vec_);
}

planner_status PlannerFilter::doFollowLane() {
    static bool is_lane_timeout = false;
    static tTimeStamp time_first_timeout = _clock->GetStreamTime();

    planner_status return_status = PS_SUCCESSFUL;
    if (!processed_new_lanevec_) {
        __synchronized_obj(update_lane_mutex_);
        return_status = trajectory_planner_.requestTrajectories(
            behaviour_curr_, pose_curr_, speed_curr_, &lane_vert_vec_);
        processed_new_lanevec_ = true;
        if (return_status == PS_NO_LANES || return_status == PS_FAILED) {
            if (is_lane_timeout == false)
                time_first_timeout = _clock->GetStreamTime();
            is_lane_timeout = true;
            printf("Start timeout\n");
        } else
            is_lane_timeout = false;
        if (is_lane_timeout &&
            (_clock->GetStreamTime() - time_first_timeout) > 1. * 1e6) {
            printf(
                "PlannerFilter: lane timeout triggered after %f sec. without "
                "lane.\n",
                1e-6 * (_clock->GetStreamTime() - time_first_timeout));
            return_status = PS_NO_LANES;
        } else {
            return_status = PS_SUCCESSFUL;
        }
    }
    return return_status;
}

planner_status PlannerFilter::doObjectApproach() {
    if (behaviour_curr_.behaviour_id == STOP_AT_OBJECT ||
        behaviour_curr_.behaviour_id == GO_TO_OBJECT) {
        if (!processed_new_lanevec_) {
            __synchronized_obj(update_lane_mutex_);
            trajectory_planner_.requestTrajectories(
                behaviour_curr_, pose_curr_, speed_curr_, &lane_vert_vec_);
            processed_new_lanevec_ = true;
        } else {
            trajectory_planner_.requestTrajectories(behaviour_curr_, pose_curr_,
                                                    speed_curr_, NULL);
        }
        return PS_SUCCESSFUL;
    } else if (behaviour_curr_.behaviour_id == GO_AROUND_OBJECT) {
        return trajectory_planner_.requestTrajectories(
            behaviour_curr_, pose_curr_, speed_curr_, NULL);
    }
    return PS_SUCCESSFUL;
}

// planner_status PlannerFilter::doDemoTask() {

// }

/* Based on current behaviour, this method gets speed value from either
    SpeedPlanner or sets it directly (e.g. for following car) */
float PlannerFilter::getSpeedValue() {
    //const float ACC_CENTRIPETAL_MAX = 0.5;

    if (maneuver_planner_.behaviour_statemachine_.behaviour_id == STOP) {
   	v_tracking_curr_ = 0.;
        return v_tracking_curr_;
    }


    v_tracking_prev_ = v_tracking_curr_;
    switch (maneuver_planner_.behaviour_curr_.behaviour_id) {
	case STOP_AT_PARKING_INFRONT:
    case PULL_RIGHT_FWD:
            v_tracking_curr_ = 0.3;
            break;
    case PULL_RIGHT_BWD:
    case PARK_AT_OBJECT:
    case GO_BACKWARDS:
         v_tracking_curr_ = -0.25;
         if (maneuver_planner_.is_maneuver_finished_) {
                v_tracking_curr_ = 0.;
         }
         break;
    case STOP_AT_OBJECT:
        if (maneuver_planner_.is_maneuver_finished_) {
            v_tracking_curr_ = 0.;
        } else {
 	        v_tracking_curr_ =
                trajectory_planner_.speed_planner_.getVTracking();
                if (speed_curr_ < 0.05)
 		             v_tracking_curr_ = 0.1;
            }
	    return v_tracking_curr_;
    default:
	        v_tracking_curr_ =
            trajectory_planner_.speed_planner_.getVTracking();
            if (maneuver_planner_.behaviour_curr_.behaviour_id == FOLLOW_LANE || 
                (maneuver_planner_.behaviour_curr_.behaviour_id) == CHANGELANE_TO_LEFT) {
                    v_tracking_curr_ = std::max(0.05f, v_tracking_curr_);
            }
            break;
    }
    // limit velocity in curves:
    /*if (speed_curr_ > 0.8*SPEED_HIGH_) {
        // A 1m radius we want to drive with 0.7m/s for better crossing detection
        float curvature_steering = fabs(last_goal_.y)/
                (2*(pow(last_goal_.x, 2.0) + pow(last_goal_.y, 2.0)));
        float acc_centripetal  = speed_curr_*speed_curr_*curvature_steering;
        if (acc_centripetal > ACC_CENTRIPETAL_MAX && curvature_steering != 0.) {
            v_tracking_curr_ =
                std::min(1.0*v_tracking_curr_,
                    0.5*(v_tracking_curr_+sqrt(ACC_CENTRIPETAL_MAX/curvature_steering)));
            printf("Limited speed due to high centripetal forces\n");
        }
    }*/
    return v_tracking_curr_;
}

tPoint PlannerFilter::getGoalpoint() {
    tPoint goal;
    float lookahead = 0.2;
    //float anchor_offset;
    bool use_backward_anchor = false;

    if (maneuver_planner_.behaviour_curr_.behaviour_id == GO_BACKWARDS) {
      goal.x = 1.;
      goal.y = 0.;
      return goal;
    }
    //printf("Pf behaviour curr is %i\n", behaviour_curr_.behaviour_id);
    //printf("MP behaviour curr is %i\n", maneuver_planner_.behaviour_curr_.behaviour_id);
    if (maneuver_planner_.behaviour_curr_.behaviour_id == PARK_AT_OBJECT ||
	    maneuver_planner_.behaviour_curr_.behaviour_id == PULL_RIGHT_BWD /*||
        maneuver_planner_.behaviour_curr_.behaviour_id == GO_BACKWARDS*/) { //behaviour_curr_.behaviour_id == GO_BACKWARDS
            printf("Use backward goal point");
	        //anchor_offset = CAR_ANCHOR_OFFSET_BWD_;
            lookahead = 0.2;
            use_backward_anchor = true;
            //printf("goal rel to anchor is %f %f\n", goal.x, goal.y);
     } else {
        lookahead = 0.2;
         //if (maneuver_planner_.behaviour_curr_.behaviour_id == FOLLOW_LANE &&
         //   speed_curr_ > 1.5 * SPEED_LOW_) {
           //lookahead = 0.3;
           // adaptive lookahead; damping for high speeds:
           lookahead += 0.1 * (fabs(speed_curr_) / SPEED_HIGH_);
        //}
        use_backward_anchor = false;
     }

     if (trajectory_planner_.path_planner_.gp_gen_.getUpdatedGoal(
        pose_curr_, lookahead, &goal, use_backward_anchor)) {
        //anchor_offset = CAR_ANCHOR_OFFSET_;
        last_goal_ = goal;
        return goal;
    } else {
        return last_goal_;
    }

}

void PlannerFilter::correctRepositioningJump() {
    const float JUMP_DELTA_L = 0.1;
    __synchronized_obj(update_pose_mutex_);
    float dx_global_prev = pose_curr_.x - pose_prev_.x;
    float dy_global_prev = pose_curr_.y - pose_prev_.y;
    if ((pow(dx_global_prev, 2) + pow(dy_global_prev, 2)) > JUMP_DELTA_L*JUMP_DELTA_L) {
        trajectory_planner_.path_planner_.gp_gen_.
            updateInitialPoseAfterJump(dx_global_prev, dy_global_prev);
        trajectory_planner_.path_planner_.goal_global_.
            updateGoalPoseAfterJump(dx_global_prev, dy_global_prev);
        if (LOG_DEBUG_)
            printf("PlannerFilter detected repos. jump by dx dy %f %f\n",
                    dx_global_prev, dy_global_prev);
    }
    pose_prev_ = pose_curr_;
}

void PlannerFilter::setLightForBehaviour() {
    const float DT_RESPONSE = 0.1;
    static tTimeStamp time_since_change = _clock->GetStreamTime();
    static bool change_detected = false;

    tTimeStamp current_time = _clock->GetStreamTime();
    // break lights for stopping
    if (fabs(v_tracking_curr_) < fabs(v_tracking_prev_) && fabs(speed_curr_) > 0.1) {
        if (!change_detected) {
            time_since_change = current_time;
            change_detected = true;
        } else if (current_time - time_since_change > DT_RESPONSE*1e6) {
            TransmitLightCommand(BRAKE_LIGHTS, true);
        }
    } else {
        TransmitLightCommand(BRAKE_LIGHTS, false);
    }

    // hazard lights for emergency break
    if ((maneuver_planner_.behaviour_emergency_.behaviour_id == EM_ENABLED &&
        current_time-maneuver_planner_.behaviour_emergency_.timestamp > 3.*1e6) ||
        maneuver_planner_.behaviour_curr_.behaviour_id == GO_BACKWARDS) {
        printf("Set hazard lights\n");
        TransmitLightCommand(HAZZARD_LIGHTS, true);
    } else {
        TransmitLightCommand(HAZZARD_LIGHTS, false);
    }
    // turning lights for lane changing
    switch (maneuver_planner_.behaviour_curr_.behaviour_id) {
        case CHANGELANE_TO_LEFT:
        case TURN_LEFT:
        case PULL_RIGHT_BWD:
            printf("Set turn light left\n");
            TransmitLightCommand(TURN_SIGNAL_LEFT, true);
            break;
        case CHANGELANE_TO_RIGHT:
        case TURN_RIGHT:
        case PULL_RIGHT_FWD:
            TransmitLightCommand(TURN_SIGNAL_RIGHT, true);
            break;
        default:
            TransmitLightCommand(TURN_SIGNAL_LEFT, false);
            TransmitLightCommand(TURN_SIGNAL_RIGHT, false);
            break;
    }

}

void PlannerFilter::doManeuverFinishedActions() {
    if (behaviour_curr_.behaviour_id == PARK_AT_OBJECT) {
        TransmitEnableParkingRearView(false);
        TransmitEnableFrontView(true);
    } else if (behaviour_curr_.behaviour_id == FOLLOW_LANE_TO_CLEAR) {
        TransmitEnableUSRight(false);
    }
    transmitPlannerStatus(PS_COMPLETED);
}

void PlannerFilter::CheckTransmitObstaclePosition() {
    static tPose car_pose_last_transmit = pose_curr_;

    if ((maneuver_planner_.behaviour_emergency_.behaviour_id == EM_ENABLED &&
        (_clock->GetStreamTime()-maneuver_planner_.behaviour_emergency_.timestamp) > 3.*1e6) ||
        (maneuver_planner_.behaviour_curr_.behaviour_id = CHANGELANE_TO_LEFT)) {
        float dist_last_transmit = sqrt(pow(pose_curr_.x - car_pose_last_transmit.x, 2) +
                                        pow(pose_curr_.y - car_pose_last_transmit.y, 2));
        if (dist_last_transmit > 1.) {
            // get closest obstacle, otherwise return default position in front
            tMapID coll_id = trajectory_planner_.path_planner_.getClosestCollidingIDWithBox(0.4, 0.);
            tSptrMapElement coll_obj = map_->GetElement(coll_id);
            TransmitJuryObstacle(coll_obj);
            car_pose_last_transmit = pose_curr_;
        }
    }
}

/**************************************************************************
                     Routines for I/O processing
**************************************************************************/

void PlannerFilter::transmitPlannerStatus(const planner_status& status) {
    // static bool isLaneFollowInit = false;

    // send PS_COMPLETE only once; PS_FAILED etc. can be send
    // repeatedly, though
    if (status == PS_SUCCESSFUL) return;
    if (maneuver_planner_.behaviour_curr_.behaviour_id == FOLLOW_LANE) {
        printf(
            "PlannerFilter wants to transmit PS_COMPLETED after lane follow. "
            "THIS SHOULD NOT HAPPEN! \n");
        return;
    }
    behaviour_state_child_enum childstate = maneuver_planner_.behaviour_state_child_;

    if ((childstate != BS_NORMALDRIVING_STATEMACHINE &&
         maneuver_planner_.behaviour_finished_.behaviour_id != maneuver_planner_.behaviour_statemachine_.behaviour_id) //&&
        //(childstate != BS_PULLRIGHT_BWD) &&
        //(childstate != BS_PARKING_STOP)) {
     ){
        //printf("Dont transmit planner status for childstate %i \n", childstate);
        return;
    }

    if (status == PS_COMPLETED) {
        if (behav_completed_sent_) {
            //printf("Completed already sent\n");
            return;
        }
        else {
            behav_completed_sent_ = true;
            // printf("set behav_completed_sent_ true \n");
        }
    }
    //if (behaviour_curr_.behaviour_id == STOP_AT_OBJECT) {
    //   maneuver_planner_.behaviour_statemachine_.behaviour_id = STOP;}
    if (LOG_DEBUG_) {
        printf("Planner transmit: %s\n", planner_status_string[status]);
        fflush(stdout);
    }
    // if (LOG_DEBUG_) {printf("Behaviour status: %s\n",
    // behaviour_type_string[behaviour_curr_.behaviour_id]);}
    vector<const tVoid*> vals = boost::assign::list_of((const tVoid*)&status);
    status_output_pin_.Transmit(vals, _clock->GetStreamTime());
}

bool PlannerFilter::processBehaviour(IMediaSample* media_sample) {
    /* Receive integer ID of enum element; some enums are combined with an
       ID of a map object (e.g. for behaviour "stop_at_object")*/
    tBehaviour* pSampleData = NULL;
    if (IS_OK(behaviour_input_pin_.ReadNoID_start(
            media_sample, (const tVoid**)&pSampleData, sizeof(tBehaviour)))) {
        /* If interrupted by emergency filter, keep prev. state:
            ToDO: Use list/queue instead */
        tBehaviour* target_behaviour = NULL;
        switch (pSampleData->behaviour_id) {
            case EM_ENABLED:
            case EM_DISABLED:
            case EM_OBSTACLE_AHEAD:
                //std::cout << "Received behaviour from EM" << pSampleData->behaviour_id << std::endl;
                target_behaviour = &maneuver_planner_.behaviour_emergency_;
                break;
            case DEMO_GO_TO_PERSON:
            case DEMO_GO_TO_INIT:
                target_behaviour = &maneuver_planner_.behaviour_demo_;
                break;
            default: // state machine
                //std::cout << "Received behaviour from SM " << pSampleData->behaviour_id << std::endl;
                //printf("received behaviour %i\n", pSampleData->behaviour_id);
                target_behaviour = &maneuver_planner_.behaviour_statemachine_;
                behav_completed_sent_ = false;
                //printf("PlannerFIlter received behaviour %i from state machine\n", pSampleData->behaviour_id);
                break;
        }
        maneuver_planner_.setBehaviour(*pSampleData, target_behaviour);
  
        // fflush(stdout);
        // if (LOG_DEBUG_) {printf("Planner: Received behaviour %s and ID %i\n",
        //                  behaviour_type_string[pSampleData->behaviour_id],
        //                                        pSampleData->object_id);
        //                                        fflush(stdout);}
        behaviour_input_pin_.ReadNoID_end(media_sample,
                                          (const tVoid**)pSampleData);
        return true;
    }
    printf("PlannerFilter::processBehaviour failed \n");
    return false;
}

bool PlannerFilter::processLaneElement(IMediaSample* media_sample) {
    tLaneElement* pSampleData = NULL;

    __synchronized_obj(update_lane_mutex_);
    // add incoming data to buffer vector;
    if (IS_OK(tLaneElement_vert_input_pin_.ReadNoID_start(
            media_sample, (const tVoid**)&pSampleData, sizeof(tLaneElement)))) {
        if ((*pSampleData).idx == 0) {
            lane_vert_vec_.resize((*pSampleData).count);
            // printf("received first lane out of %i \n", (*pSampleData).count);
        }
        if (lane_vert_vec_.size() > (*pSampleData).idx) {
            lane_vert_vec_[(*pSampleData).idx].coeff.clear();
            lane_vert_vec_[(*pSampleData).idx].coeff.push_back(
                (*pSampleData).coeff0);
            lane_vert_vec_[(*pSampleData).idx].coeff.push_back(
                (*pSampleData).coeff1);
            lane_vert_vec_[(*pSampleData).idx].coeff.push_back(
                (*pSampleData).coeff2);
            lane_vert_vec_[(*pSampleData).idx].dist = (*pSampleData).dist;
            lane_vert_vec_[(*pSampleData).idx].no_of_samples =
                (*pSampleData).no_of_samples;
            if ((*pSampleData).idx == (*pSampleData).count - 1) {
                processed_new_lanevec_ = false;
                // printf("processLaneElement received last element\n");
            }
            // printf("received lane with idx %i \n", (*pSampleData).idx);
        } else {
            printf("Couldn't add received lane, idx out of bounds \n");
        }
        tLaneElement_vert_input_pin_.ReadNoID_end(media_sample,
                                                  (const tVoid**)pSampleData);
    }
    return true;
}

bool PlannerFilter::processPose(IMediaSample* media_sample) {
    tPosition* pSampleData = NULL;
    if (IS_OK(tPos_input_pin_.ReadNoID_start(
            media_sample, (const tVoid**)&pSampleData, sizeof(tPosition)))) {
        __synchronized_obj(update_pose_mutex_);
        pose_curr_.x = pSampleData->f32x;
        pose_curr_.y = pSampleData->f32y;
        pose_curr_.heading = fmod(pSampleData->f32heading, 2*PI);

        if (pose_curr_.heading < 0.)
		pose_curr_.heading += 2*PI;
        speed_curr_ = pSampleData->f32speed;
        tPos_input_pin_.ReadNoID_end(media_sample, (const tVoid**)pSampleData);
        // if (LOG_DEBUG_) {printf("Current pose is x=%f y=%f phi=%f\n",
        // pose_curr_.x, pose_curr_.y, pose_curr_.heading);}
        return true;
    }
    printf("PlannerFilter::processPose failed \n");
    return false;
}

void PlannerFilter::transmitGoalpoint(const tPoint& goal_pnt) {
    /* update goal point based on car movement since path creation */
    //if (fabs(speed_curr_) <= SPEED_STOPPED_)
    //    return;
    vector<const tVoid*> vals = boost::assign::list_of(
        (const tVoid*)&goal_pnt.x)((const tVoid*)&goal_pnt.y);
    goalpoint_output_pin_.Transmit(vals, _clock->GetStreamTime());
}

void PlannerFilter::transmitVTracking(float v_tracking) {
    /* sample and send tracking point v to the speed controller */
    tTimeStamp time = _clock->GetStreamTime();
    vector<const tVoid*> vals =
        boost::assign::list_of((const tVoid*)&v_tracking)((const tVoid*)&time);
    vtracking_output_pin_.Transmit(vals, time);
}

bool PlannerFilter::processDeltaDistance(IMediaSample* media_sample,
                                         float* ds) {
    tSignalValue* sample_data = NULL;
    if (IS_OK(ds_input_pin_.ReadNoID_start(
            media_sample, (const tVoid**)&sample_data, sizeof(tSignalValue)))) {
        *ds = sample_data->f32Value;
        ds_input_pin_.ReadNoID_end(media_sample, (const tVoid**)sample_data);
        return true;
    }
    printf("PlannerFilter::processDeltaDistance failed \n");
    return false;
}

void PlannerFilter::TransmitEnableParkingRearView(const tBool enable_rear) {
    vector<const tVoid*> vals =
        boost::assign::list_of((const tVoid*)&enable_rear);
    if (IS_FAILED(enable_parking_rear_view_output_pin_.Transmit(
            vals, _clock->GetStreamTime()))) {
        LOG_ERROR("Failed sending enable rear view.");
    }
}

void PlannerFilter::TransmitEnableFrontView(const tBool enable_front) {
    if (IS_FAILED(enable_front_view_output_pin_.Transmit<tBool>(
            enable_front, _clock->GetStreamTime()))) {
        LOG_ERROR("Failed sending enable front view.");
    }
}


tResult PlannerFilter::TransmitLightCommand(tInt32 light_id, tBool switch_bool) {
    vector<const void*> vals1 = boost::assign::list_of((const void*)&light_id)(
        (const void*)&switch_bool);
    if (IS_FAILED(light_controller_output_pin_.Transmit(
            vals1, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF("Pf: failed sending light command\n");
    }
    RETURN_NOERROR;
}

void PlannerFilter::TransmitEnableUSRight(const tBool enable_us_right) {
    static bool last_transmit = true;
    if (enable_us_right == last_transmit) return;
    last_transmit = enable_us_right;
    if (IS_FAILED(enable_us_right_output_pin_.Transmit<tBool>(
            enable_us_right, _clock->GetStreamTime()))) {
        LOG_ERROR("Failed sending enable us right.");
    }
}


void PlannerFilter::TransmitJuryObstacle(const tSptrMapElement & el_obstacle)
{
    tMapPoint center;

    if (!el_obstacle) {
        center.set<0>(pose_curr_.x + 0.5*cos(pose_curr_.heading));
        center.set<1>(pose_curr_.y + 0.5*sin(pose_curr_.heading));
    } else {
        center = el_obstacle->GetGlobalPolyCenter();
    }
    jury_obstacle_output_pin_.Transmit<float, float>(
    center.get<0>(), center.get<1>(), _clock->GetStreamTime());
    printf("PlannerFilter transmitted obstacle to jury with x y (%f %f)", center.get<0>(), center.get<1>());
    // if (1) {
    //     LOG_ERROR("Failed sending enable jury_obstacle.");
    // }
}


/**************************************************************************
                     Routines for setting up ADTF
**************************************************************************/

tResult PlannerFilter::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))
    // in StageFirst you can create and register your static pins.

    tResult nResult = ERR_NOERROR;

    if (stage == StageFirst) {
        nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");

    } else if (stage == StageNormal) {
        // In this stage you would do further initialisation and/or create your
        // dynamic pins. Please take a look at the demo_dynamicpin example for
        // further reference.
    } else if (stage == StageGraphReady) {
        // All pin connections have been established in this stage so you can
        // query your pins about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further
        // reference.
        map_ = frAIburg::map::getInstance();
        trajectory_planner_.path_planner_.setMap(map_);
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");
        InitTimer();  // init and start timer after a dely
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult PlannerFilter::Shutdown(tInitStage stage, __exception) {
    if (stage == StageGraphReady) {
        DeleteTimer();
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult PlannerFilter::CreateInputPins(__exception) {
    slim::register_pin_func func = &PlannerFilter::RegisterPin;

    // This pin also handles the lanes that come from parking
    RETURN_IF_FAILED(tLaneElement_vert_input_pin_.FirstStageCreate(
        this, func, "VerticalLane", "tLaneElement"));
    RETURN_IF_FAILED(tLaneElement_horiz_input_pin_.FirstStageCreate(
        this, func, "HorizontalLane", "tLaneElement"));
    RETURN_IF_FAILED(
        tPos_input_pin_.FirstStageCreate(this, func, "Position", "tPosition"));
    RETURN_IF_FAILED(
        ds_input_pin_.FirstStageCreate(this, func, "delta_s", "tSignalValue"));
    RETURN_IF_FAILED(behaviour_input_pin_.FirstStageCreate(
        this, func, "behaviour", "tBehaviour"));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult PlannerFilter::CreateOutputPins(__exception) {
    slim::register_pin_func func = &PlannerFilter::RegisterPin;

    RETURN_IF_FAILED(vtracking_output_pin_.FirstStageCreate(this, func, "v_set",
                                                            "tSignalValue"));
    RETURN_IF_FAILED(goalpoint_output_pin_.FirstStageCreate(
        this, func, "goal_point", "tPoint"));
    RETURN_IF_FAILED(status_output_pin_.FirstStageCreate(
        this, func, "planner_status", "tPlannerStatus"));

    RETURN_IF_FAILED(enable_parking_rear_view_output_pin_.FirstStageCreate(
        this, func, "enable_parking_rear_view", "tBoolSignalValue"));
    RETURN_IF_FAILED(enable_front_view_output_pin_.FirstStageCreate(
        this, func, "enable_front_view", "tBoolSignalValue"));
    RETURN_IF_FAILED(light_controller_output_pin_.FirstStageCreate(
         this, func, "light_output", "tLightCommand"));
    RETURN_IF_FAILED(enable_us_right_output_pin_.FirstStageCreate(
         this, func, "enable_us_right", "tBoolSignalValue"));
    RETURN_IF_FAILED(jury_obstacle_output_pin_.FirstStageCreate(
         this, func, "jury_obstacle", "tObstacle"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult PlannerFilter::SetPinIDs() {
    vector<string> ids1 =
        boost::assign::list_of("f32Value")("ui32ArduinoTimestamp");
    RETURN_IF_FAILED(vtracking_output_pin_.StageGraphReadySetIDOrder(ids1));
    vector<string> ids2 = boost::assign::list_of("x")("y");
    RETURN_IF_FAILED(goalpoint_output_pin_.StageGraphReadySetIDOrder(ids2));
    vector<string> ids3 = boost::assign::list_of("status");
    RETURN_IF_FAILED(status_output_pin_.StageGraphReadySetIDOrder(ids3));
    // camera on/off
    vector<string> id_enable_front_view = boost::assign::list_of("bValue");
    RETURN_IF_FAILED(enable_front_view_output_pin_.StageGraphReadySetIDOrder(
        id_enable_front_view));
    vector<string> id_enable_rear_view = boost::assign::list_of("bValue");
    RETURN_IF_FAILED(
        enable_parking_rear_view_output_pin_.StageGraphReadySetIDOrder(
            id_enable_rear_view));
    // light command
    vector<string> id5s = boost::assign::list_of("light_id")("switch_bool");
    RETURN_IF_FAILED(
        light_controller_output_pin_.StageGraphReadySetIDOrder(id5s));
    // enable ultrasonic right
    RETURN_IF_FAILED(
        enable_us_right_output_pin_.StageGraphReadySetIDOrder(
            id_enable_rear_view));

    vector<string> id6s = boost::assign::list_of("f32x")("f32y");
    RETURN_IF_FAILED(jury_obstacle_output_pin_.StageGraphReadySetIDOrder(id6s));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult PlannerFilter::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    last_time_ = _clock->GetStreamTime();
    init_time_ = _clock->GetStreamTime();
    running_ok_ = true;
    getAllProperties();
    TransmitEnableUSRight(false);
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult PlannerFilter::Stop(__exception) {
    running_ok_ = false;
    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}

void PlannerFilter::getAllProperties(void) {
    LOG_DEBUG_ = GetPropertyBool("Filter communication: debug mode");
    trajectory_planner_.setDebugMode(LOG_DEBUG_);
    trajectory_planner_.path_planner_.setDebugMode(
        GetPropertyBool("Path planner: debug mode"));
    trajectory_planner_.path_planner_.spline_.setDebugMode(
        GetPropertyBool("Path planner: debug mode"));
    trajectory_planner_.speed_planner_.setDebugMode(
        GetPropertyBool("Speed planner: debug mode"));
    trajectory_planner_.path_planner_.gp_gen_.setDebugMode(
        GetPropertyBool("Goal point generation: debug mode"));
}

// ____________________________________________________________________________
void PlannerFilter::setAllProperties(void) {
    SetPropertyBool("Filter communication: debug mode", false);
    SetPropertyBool("Path planner: debug mode", false);
    SetPropertyBool("Speed planner: debug mode", false);
    SetPropertyBool("Goal point generation: debug mode", false);
    // SetPropertyBool("Draw current path", true);
}

tResult PlannerFilter::UpdateProperties(ucom::IException** __exception_ptr) {
    RETURN_IF_FAILED(cFilter::UpdateProperties(__exception_ptr));
    getAllProperties();
    RETURN_NOERROR;
}

/**************************************************************************
                            Helper routines
**************************************************************************/

void PlannerFilter::addPointToMap(MapElementType type, tMapData x, tMapData y,
                                  tMapData angle) {
    frAIburg::map::GlobalMap* map = frAIburg::map::getInstance();
    if (!map) return;
    std::vector<tMapPoint> v1 = boost::assign::list_of(tMapPoint(x, y));
    // el1.
    tSptrMapElement el_sign(
        new MapElement(type, v1, _clock->GetStreamTime(), angle));
    el_sign->EnableTimeOfLife(0.1 * 1e6);
    el_sign->user_tag_ui_ = "GP";
    //printf("Added pnt %f %f to map \n", x, y);
    map->AddElement(el_sign);  // add ele new to map
}

// ____________________________________________________________________________
void PlannerFilter::InitTimer() {
    timer_ = NULL;
    tUInt32 nFlags = 0;
    // Create our cyclic timer.
    cString strTimerName = OIGetInstanceName();
    strTimerName += ".rttimerStateMachineTimer";
    tTimeStamp tmPeriod = 1e6 * DELTA_T_REPLAN_;
    tTimeStamp tmstartdelay = 1e6 * 0.2;
    timer_ = _kernel->TimerCreate(tmPeriod,
                                  tmstartdelay,  // dely for the first start
                                  static_cast<IRunnable*>(this),  // call run
                                  NULL, NULL, 0, nFlags, strTimerName);
    if (!timer_) {
        LOG_ERROR("PlannerFilter unable to create timer");
    }
}

// ____________________________________________________________________________
void PlannerFilter::DeleteTimer() {
    if (timer_) {
        running_ok_ = false;
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
    }
}
