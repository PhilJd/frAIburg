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

#include "ManeuverPlanner.h"

using namespace frAIburg::map;

const char* behaviour_childstate_string[] = {
    "BS_OBSTACLE_AHEAD",     "BS_OBSTACLE_BREAK",
    "BS_OBSTACLE_DRIVEBACK", "BS_OBSTACLE_DRIVEAROUND",
    "BS_OBSTACLE_STOPAT",    "BS_NORMALDRIVING_STATEMACHINE",
    "BS_CARAHEAD_FOLLOW",    "BS_CARAHEAD_OVERTAKE",
    "BS_PARKING_GOTOSWITCH", "BS_PARKING_PARKBACKWARDS", "BS_PARKING_STOP",
    "BS_LANECHANGE_TOLEFT",  "BS_LANECHANGE_TORIGHT",
    "BS_LANECHANGE_TOCLEAR",
    "BS_DEMO_GOTOPERSON", "BS_DEMO_GOTOINIT",
    "BS_PULLRIGHT_FWD", "BS_PULLRIGHT_BWD"};

const char* behaviour_type_string[] = {"BT_UNKNOWN",
                                       "STOP",
                                       "EM_ENABLED",
                                       "EM_DISABLED",
                                       "EM_OBSTACLE_AHEAD",
                                       "FOLLOW_LANE",
                                       "GO_TO_OBJECT",
                                       "STOP_AT_OBJECT",
                                       "GO_AROUND_OBJECT",
                                       "TURN_LEFT",
                                       "TURN_RIGHT",
                                       "GO_STRAIGHT",
                                       "PARK_AT_OBJECT",
                                       "PULL_LEFT",
                                       "PULL_RIGHT",
                                       "PULL_RIGHT_FWD",
                                       "PULL_RIGHT_BWD",
                                       "GO_BACKWARDS",
                                       "STOP_AT_PARKING_INFRONT",
                                       "PARK",
                                       "CHANGELANE_TO_LEFT",
                                       "CHANGELANE_TO_RIGHT",
                                       "CHANGELANE_TO_CLEAR",
                                       "FOLLOW_CAR",
                                       "DEMO_GO_TO_PERSON",
                                       "DEMO_GO_TO_INIT"};

ManeuverPlanner::ManeuverPlanner(PathPlanner* path_planner) {
    path_planner_ = path_planner;
    setBehaviour(STOP, BT_UNKNOWN, -1, ST_STOP, &behaviour_curr_);
    behaviour_prev_ = behaviour_curr_;
    behaviour_statemachine_ = behaviour_curr_;
    behaviour_emergency_ = behaviour_curr_;
    behaviour_emergency_.behaviour_id = EM_DISABLED;
    behaviour_state_child_ = BS_NORMALDRIVING_STATEMACHINE;
    behaviour_state_parent_ = BS_PARENT_NORMALDRIVING;
    is_maneuver_finished_ = false;
    deadlock_detected_ = false;
    time_since_stopped_ = 0;
}

/* -Only enter..() methods are allowed to change the behaviour_curr,
  - behaviour_prev is changed only once every external call
  - Only TransitionToState is allowed to change states */

// this method is called once, externally
tBehaviour ManeuverPlanner::getUpdatedBehaviour() {
    //boost::mutex::scoped_lock scoped_lock(update_behaviour_mutex_);
    /* Behaviour for deadlock, return to prev. state after 45 sec.*/
    // if (fabs(path_planner_->v_current_) < SPEED_STOPPED_) {
    //     if (!deadlock_detected_) {
    //         time_since_stopped_ = path_planner_->current_time_;
    //         deadlock_detected_ = true;
    //     }
    //     if (path_planner_->current_time_ - time_since_stopped_ > 45*1e6) {
    //         setBehaviour(behaviour_statemachine_, &behaviour_prev_);
    //         deadlock_detected_ = false;
    //         time_since_stopped_ = path_planner_->current_time_;
    //     }
    // } else {
    //     deadlock_detected_ = false;
    // }
    /* Normal behaviour */
    setBehaviour(behaviour_curr_, &behaviour_prev_);
    if (is_maneuver_finished_) {
      setBehaviour(behaviour_curr_, &behaviour_finished_);
    }
    executeCurrentState();
    printBehaviourIfChanged();
    //printFullState();
    return behaviour_curr_;
}

// this method can be called multiple times internally in one update step
void ManeuverPlanner::executeCurrentState() {
    switch (behaviour_state_parent_) {
        case BS_PARENT_NORMALDRIVING:
            enterParentStateNormaldriving();
            break;
        case BS_PARENT_OBSTACLE:
            enterParentStateObstacle();
            break;
        case BS_PARENT_PARKING:
            enterParentStateParking();
            break;
        case BS_PARENT_CARAHEAD:
            enterParentStateCarAhead();
            break;
        case BS_PARENT_LANECHANGE:
            printf("enterParentStateLanechange\n");
            enterParentStateLanechange();
            break;
        case BS_PARENT_DEMO:
            enterParentStateDemo();
            break;
        case BS_PARENT_PULLRIGHT:
            enterParentStatePullout();
	       break;
        default:
            printf("ManeuverPlanner: ENUM not handled in switch!\n");
            break;
    }
}

// this method changes the states and executes the new state
void ManeuverPlanner::TransitionToState(behaviour_state_parent_enum new_parent,
                                        behaviour_state_child_enum new_child) {
    bool is_state_changed = false;  // this will avoid algebraic loops
    if (behaviour_state_parent_ != new_parent ||
        behaviour_state_child_ != new_child) {
        is_state_changed = true;
    }
    // todo: summarize cases w/o special behaviour
    switch (new_parent) {
        case BS_PARENT_NORMALDRIVING:
            behaviour_state_parent_ = BS_PARENT_NORMALDRIVING;
            behaviour_state_child_ = new_child;
            behaviour_return_list_.clear();  // no state to return to
            break;
        case BS_PARENT_OBSTACLE:
            if (behaviour_state_parent_ !=
                BS_PARENT_OBSTACLE)
                addChildParentStatesToList();
            behaviour_state_parent_ = BS_PARENT_OBSTACLE;
            behaviour_state_child_ = new_child;
            break;
        case BS_PARENT_PARKING:
        case BS_PARENT_CARAHEAD:
        case BS_PARENT_LANECHANGE:
        case BS_PARENT_DEMO:
        case BS_PARENT_PULLRIGHT:
            behaviour_state_parent_ = new_parent;
            behaviour_state_child_ = new_child;
            break;
        default:
            break;
    }
    if (is_state_changed) executeCurrentState();
}

void ManeuverPlanner::TransitionToPreviousState() {
    LOG_INFO("ManeuverPlanner: TransitionToPreviousState() called\n");
    printf("child state is %s\n", behaviour_childstate_string[behaviour_state_child_]);

    if (behaviour_return_list_.empty()) {
        TransitionToState(BS_PARENT_NORMALDRIVING,
                          BS_NORMALDRIVING_STATEMACHINE);
        LOG_INFO(
            "ManeuverPlanner: TransitionToPreviousState: "
            "Behaviour return list was empty!\n");
        return;
    }
    // to be on the safe side, first pop before calling
    // the transition; might be unnecessary
    behaviour_state_parent_enum tmp_parent =
      behaviour_return_list_.back().parent_state;
    behaviour_state_child_enum tmp_child =
      behaviour_return_list_.back().child_state;
    behaviour_return_list_.pop_back();
    printf("jump back to child state %s\n", behaviour_childstate_string[tmp_child]);
    TransitionToState(tmp_parent, tmp_child);
}

/*This is the sub-statemachine for dealing with obstacles */
void ManeuverPlanner::enterParentStateObstacle() {
    const float wait_time = 10.;

      printf("enterParentStateObstacle\n ");
      fflush(stdout);
    // when in drive back mode, ignore EM signal, otherwise going back might be
    // hindered
    if ((behaviour_emergency_.behaviour_id == EM_ENABLED) &&
        behaviour_state_child_ != BS_OBSTACLE_DRIVEBACK) {
        TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_BREAK);
    }
    switch (behaviour_state_child_) {
        case BS_OBSTACLE_AHEAD:
            setObstacleAheadBehaviour();
            break;
        case BS_OBSTACLE_STOPAT:
            if (behaviour_finished_.behaviour_id == STOP_AT_OBJECT &&
                is_maneuver_finished_) {
                TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_BREAK);
            } else {
                printf("MP: BS_OBSTACLE_STOPAT l 211\n");
                setBehaviour(STOP_AT_OBJECT, BT_UNKNOWN,
                             behaviour_emergency_.object_id, ST_STOP,
                             &behaviour_curr_);
            }
            break;
        case BS_OBSTACLE_BREAK:
            //printf("dt since trigger is %f\n", (path_planner_->current_time_ - behaviour_emergency_.timestamp) * 1e-6);
            setBehaviour(STOP, BT_UNKNOWN, -1, ST_STOP, &behaviour_curr_);
            if (behaviour_emergency_.behaviour_id == EM_DISABLED ||
                (path_planner_->current_time_ - behaviour_emergency_.timestamp > 30. * 1e6)) {
                TransitionToPreviousState();
                return;
            // drive back only after waiting and if transitioned to EM Break from normal driving
            // this excludes maneuvers like lanechange
            } else if ((behaviour_emergency_.behaviour_id == EM_ENABLED) &&
                       (path_planner_->current_time_ - behaviour_emergency_.timestamp > wait_time * 1e6) &&
                       (getObstacleAheadType(behaviour_emergency_.object_id) != OT_PERSON) &&
                        wasNormalDrivingBefore()) {
                printf("Transition to DRIVEBACK\n");
                TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_DRIVEBACK);
            }
            break;
        case BS_OBSTACLE_DRIVEBACK:
            // todo: front EM might still be enabled..
            if (behaviour_finished_.behaviour_id == GO_BACKWARDS &&
                is_maneuver_finished_) {
                // is road clear?
                if (path_planner_->getClosestCollidingIDWithBox(0.8, 0) == -1) {
                    printf("Driveback, road ahead is clear\n");
                    TransitionToPreviousState();
                } else if (path_planner_->isLaneChangeAllowed()) {
                    printf("Driveback, lanechange allowed, do it\n");
                    TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_DRIVEAROUND);
                } else {
                    printf("Driveback, lanechange not allowed, go to obst. break\n");
                    TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_BREAK);
                }
            } else {
                setBehaviour(GO_BACKWARDS, BT_UNKNOWN, -1, ST_BACKWARDS,
                             &behaviour_curr_);
            }
            break;
        case BS_OBSTACLE_DRIVEAROUND:
            // if (behaviour_statemachine_.behaviour_id == FOLLOW_LANE &&
            //     path_planner_->isLaneChangeAllowed() &&
            //     wasNormalDrivingBefore()) {
            //     TransitionToState(BS_PARENT_LANECHANGE, BS_LANECHANGE_TOLEFT);
            // } else {
            //     TransitionToPreviousState();
            // }
            printf("Entered BS_OBSTACLE_DRIVEAROUND\n");
            TransitionToState(BS_PARENT_LANECHANGE, BS_LANECHANGE_TOLEFT);
            break;
        default:
            break;
    }
}

// this method mainly excludes trying to changelane a second time during
// a previous, unfinished langechange maneuver (e.g. when blocked again)
bool ManeuverPlanner::wasNormalDrivingBefore() {
    if (!behaviour_return_list_.empty()) {
        if (behaviour_return_list_.back().child_state ==
            BS_NORMALDRIVING_STATEMACHINE) {
            return true;
        } else {
            printf("ManeuverPlanner: wasNormalDrivingBefore() called, but false\n");
        }
    }
    printf("wasNormalDrivingBefore called, but empty return list (WHY?) \n");
    return false;
}

void ManeuverPlanner::setObstacleAheadBehaviour() {
    // this method is called for EM_OBSTACLE_AHEAD:
    static tTimeStamp first_detection_time;
    static bool action_triggered = false;

    const float MIN_DECISION_DIST = 0.8;
    float distance_to_obst = 2.;
    obstacle_type_enum obstacle_type =
        getObstacleAheadType(behaviour_emergency_.object_id,
                             &distance_to_obst);

    // reset time if either last action too long ago or first trigger.
    // In case of return, the behaviour could change to EM_ENABLED,
    // and action_triggered wouldn't be reset when entering the next time

    if (!action_triggered ||
        (path_planner_->current_time_-first_detection_time) > 5.*1e6) {
        first_detection_time = path_planner_->current_time_;
        action_triggered = true;
    }
    // if object too far away, just slow down and return todo: check angle  and use distannce instead time
    if ((distance_to_obst > MIN_DECISION_DIST) &&
        (obstacle_type != OT_CLEAR)) {
        setBehaviour(behaviour_curr_.behaviour_id,
                    behaviour_curr_.behaviour_next_id,
                    behaviour_curr_.object_id, ST_SPEED_LOW,
                    &behaviour_curr_);
        return;
    }
    // if action needed, but not enough time passed, return
   /* if ( (obstacle_type != OT_CLEAR) &&
         (path_planner_->current_time_-first_detection_time < 0.1*1e6)) {
        printf("setObstacleAheadBehaviour to little time passed in ahead mode, return\n");
        return;
    }*/

    // ACT!
    action_triggered = false;

    switch (obstacle_type) {
        case OT_CLEAR:
            TransitionToPreviousState();
            break;
        case OT_PERSON:
            TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_STOPAT);
            break;
        case OT_CAR_BACK:
            if (behaviour_statemachine_.behaviour_id == FOLLOW_LANE &&
                wasNormalDrivingBefore()) {
                TransitionToState(BS_PARENT_CARAHEAD, BS_CARAHEAD_FOLLOW);
            }
            break;
        case OT_UNKNOWN:
            /*if (behaviour_statemachine_.behaviour_id == FOLLOW_LANE &&
                path_planner_->isLaneChangeAllowed() &&
                wasNormalDrivingBefore()) {
                if (distance_to_obst > 0.6) {
                  TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_DRIVEAROUND);
                } else {
                  TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_STOPAT);
                }
            }*/
            //TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_STOPAT);
                    setBehaviour(behaviour_curr_.behaviour_id,
                    behaviour_curr_.behaviour_next_id,
                    behaviour_curr_.object_id, ST_SPEED_LOW,
                    &behaviour_curr_);
            break;
    }
}


void ManeuverPlanner::enterParentStateCarAhead() {

    if (behaviour_emergency_.behaviour_id == EM_ENABLED) {
        std::cout << "enterParentStateCarAhead transition to break "<< std::endl;
        TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_BREAK);
        return;
    }
    float v_car_ahead = 0;
    bool found_car = false;
    switch (behaviour_state_child_) {
        case BS_CARAHEAD_FOLLOW:
            // if clear, return
            float v_rel;
            float distance;
            found_car = path_planner_->getDistSpeedCarAhead(&v_rel, &distance);
            // this MUST NOT CONTRADICT the EM_OBST_AHEAD signal, otherwise stuck in loop:
            if (!found_car) {
                std::cout << "enterParentStateCarAhead transition to normaldriving, no car found"<< std::endl;
                TransitionToState(BS_PARENT_NORMALDRIVING,
                                  BS_NORMALDRIVING_STATEMACHINE);
                return;
            }    
            // Car ahead, check its velocity:
            v_car_ahead = fabs(v_rel + path_planner_->v_current_);
            // car moving in correct direction and not stopped:
            if (v_car_ahead > 0.2) {
                setBehaviour(FOLLOW_CAR, BT_UNKNOWN,
                             behaviour_emergency_.object_id, ST_SPEED_LOW,
                             &behaviour_curr_);
                if (v_car_ahead < 0.8 && distance < 0.7 && behaviour_statemachine_.behaviour_id == FOLLOW_LANE &&
                    path_planner_->isLaneChangeAllowed()) {
                    std::cout << "enterParentStateCarAhead transition to overtake since lanechange allowed"<< std::endl;
 					TransitionToState(BS_PARENT_CARAHEAD, BS_CARAHEAD_OVERTAKE);
                }
                return;
            } 
            printf("Car ahead is too slow \n");
            // Car ahead, but unsure if moving or moving correct direction:
           /* if (behaviour_statemachine_.behaviour_id == FOLLOW_LANE &&
                    path_planner_->isLaneChangeAllowed()) {
                    std::cout << "enterParentStateCarAhead transition to overtake since lanechange allowed"<< std::endl;
 					TransitionToState(BS_PARENT_CARAHEAD, BS_CARAHEAD_OVERTAKE);
            } else {
                TransitionToPreviousState();
            }*/
            TransitionToPreviousState();
            break;
        case BS_CARAHEAD_OVERTAKE:
            std::cout << "ManeuverPlanner: Try to overtake!" << std::endl;
            TransitionToState(BS_PARENT_LANECHANGE, BS_LANECHANGE_TOLEFT);
            break;
        default:
            break;
    }
}

void ManeuverPlanner::enterParentStateLanechange() {
    if (behaviour_emergency_.behaviour_id == EM_ENABLED) {
        TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_BREAK);
        return;
    }
    switch (behaviour_state_child_) {
        case BS_LANECHANGE_TOLEFT:
            if (behaviour_finished_.behaviour_id == CHANGELANE_TO_LEFT &&
                is_maneuver_finished_) {
                printf("transition to changelane to clear\n");
                TransitionToState(BS_PARENT_LANECHANGE, BS_LANECHANGE_TOCLEAR);
            } else {
                setBehaviour(CHANGELANE_TO_LEFT, BT_UNKNOWN, -1, ST_SPEED_LOW,
                             &behaviour_curr_);
            }
            break;
        case BS_LANECHANGE_TOCLEAR:
            // if (is_maneuver_finished_) printf("is_maneuver_finished_ true
            // 1\n");
            if (behaviour_finished_.behaviour_id == FOLLOW_LANE_TO_CLEAR &&
                is_maneuver_finished_) {
                // if (is_maneuver_finished_) printf("is_maneuver_finished_ true
                // 2\n");
                TransitionToState(BS_PARENT_LANECHANGE, BS_LANECHANGE_TORIGHT);
            } else {
                setBehaviour(FOLLOW_LANE_TO_CLEAR, BT_UNKNOWN,
                             behaviour_emergency_.object_id,
                             ST_SPEED_HIGH,
                             &behaviour_curr_);
            }
            break;
        case BS_LANECHANGE_TORIGHT:
            if (behaviour_finished_.behaviour_id == CHANGELANE_TO_RIGHT &&
                is_maneuver_finished_) {
                TransitionToState(BS_PARENT_NORMALDRIVING,
                                  BS_NORMALDRIVING_STATEMACHINE);
            } else {
                setBehaviour(CHANGELANE_TO_RIGHT, BT_UNKNOWN, -1, ST_SPEED_HIGH,
                             &behaviour_curr_);
            }
            break;
        default:
            break;  // todo: add obstacle em_enabled
    }
}

/*This is the sub-statemachine for normal driving */
void ManeuverPlanner::enterParentStateNormaldriving() {
    //printf("enterParentStateNormaldriving l 415\n");
    switch (behaviour_state_child_) {
        case BS_NORMALDRIVING_STATEMACHINE:
            if (behaviour_emergency_.behaviour_id == EM_ENABLED) {
                TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_BREAK);
            } else if (behaviour_emergency_.behaviour_id == EM_OBSTACLE_AHEAD) {
                TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_AHEAD);
            } else if (behaviour_statemachine_.behaviour_id == PARK && !behaviour_statemachine_.is_finished) { // 5.11: behaviour_curr_
    	        printf("enterParentStateNormaldriving PARK l 423\n");
                TransitionToState(BS_PARENT_PARKING, BS_PARKING_GOTOSWITCH);
            } else if (behaviour_statemachine_.behaviour_id == PULL_RIGHT && !behaviour_statemachine_.is_finished) {// 5.11: behaviour_curr_
                //printf("enterParentStateNormaldriving PULLRIGHT l 425\n");
                TransitionToState(BS_PARENT_PULLRIGHT, BS_PULLRIGHT_FWD);
            } else if (behaviour_demo_.behaviour_id == DEMO_GO_TO_PERSON) {
                TransitionToState(BS_PARENT_DEMO, BS_DEMO_GOTOPERSON);
            } else if (!behaviour_statemachine_.is_finished) {
                printf("enterParentStateNormaldriving; SM behav not finished, set behav. curr l 430\n");
                setBehaviour(behaviour_statemachine_, &behaviour_curr_);
                TransitionToState(BS_PARENT_NORMALDRIVING, BS_NORMALDRIVING_STATEMACHINE);
            }
            break;
        default:
            break;
    }
}

/*This is the sub-statemachine for dealing with parking */
void ManeuverPlanner::enterParentStateParking() { // todo: activate emergency stop!?
    switch (behaviour_state_child_) {
        case BS_PARKING_GOTOSWITCH:
            printf("entered parking goto switch \n");
            if ((behaviour_finished_.behaviour_id == STOP_AT_PARKING_INFRONT) &&
                is_maneuver_finished_) {
                TransitionToState(BS_PARENT_PARKING, BS_PARKING_PARKBACKWARDS);
            } else {
                setBehaviour(STOP_AT_PARKING_INFRONT, BT_UNKNOWN, behaviour_statemachine_.object_id,
                             ST_SPEED_LOW, &behaviour_curr_);
            }
            break;
        case BS_PARKING_PARKBACKWARDS:
            if ((behaviour_finished_.behaviour_id == PARK_AT_OBJECT) &&
                is_maneuver_finished_) {
                TransitionToState(BS_PARENT_PARKING,
                                  BS_PARKING_STOP);
            } else {
                setBehaviour(PARK_AT_OBJECT, BT_UNKNOWN, behaviour_statemachine_.object_id,
                             ST_BACKWARDS, &behaviour_curr_);
            }
            break;
        case BS_PARKING_STOP:
                if ((behaviour_finished_.behaviour_id == STOP) &&
                     is_maneuver_finished_) {
                behaviour_statemachine_.is_finished = true;
                printf("Parking stop finished, transition to state: state machine\n");
                TransitionToState(BS_PARENT_NORMALDRIVING,
                                  BS_NORMALDRIVING_STATEMACHINE);
            } else {
                setBehaviour(STOP, BT_UNKNOWN, behaviour_statemachine_.object_id,
                             ST_STOP, &behaviour_curr_);
            }
            break;

        default:
            break;
    }
}

/*This is the sub-statemachine for dealing with pull out right */
void ManeuverPlanner::enterParentStatePullout() {  // todo: activate emergency stop!?
    switch (behaviour_state_child_) {
        printf("MP enter parentstatepullout\n");
        case BS_PULLRIGHT_FWD:
            if ((behaviour_finished_.behaviour_id == PULL_RIGHT_FWD) && // 5.11: before behav_curr
                is_maneuver_finished_) {
                TransitionToState(BS_PARENT_PULLRIGHT, BS_PULLRIGHT_BWD);
            } else {
 		//printf("setbehav to pullrighfwd\n");
                setBehaviour(PULL_RIGHT_FWD, BT_UNKNOWN, -1,
                             ST_SPEED_LOW, &behaviour_curr_);
            }
            break;
        case BS_PULLRIGHT_BWD:
            if ((behaviour_finished_.behaviour_id == PULL_RIGHT_BWD) &&
                is_maneuver_finished_) {
                printf(" PULL_RIGHT_BWD transition to SM l 500 \n");
                behaviour_statemachine_.is_finished = true;
                TransitionToState(BS_PARENT_NORMALDRIVING,
                                  BS_NORMALDRIVING_STATEMACHINE);
            } else {
                setBehaviour(PULL_RIGHT_BWD, BT_UNKNOWN, -1,
                             ST_BACKWARDS, &behaviour_curr_);
            }
            break;
        default:
            break;
    }
}


void ManeuverPlanner::enterParentStateDemo() {
    if (behaviour_emergency_.behaviour_id == EM_ENABLED) {
        TransitionToState(BS_PARENT_OBSTACLE, BS_OBSTACLE_BREAK);
        return;
    }
    std::cout << "ManeuverPlanner reached demo state; no behaviours implemented" << std::endl;
}

void ManeuverPlanner::addChildParentStatesToList() {
    memory_state_struct tmp;
    tmp.parent_state = behaviour_state_parent_;
    tmp.child_state = behaviour_state_child_;
    behaviour_return_list_.push_back(tmp);
}

void ManeuverPlanner::setBehaviour(tInt32 behaviour_id,
                                   tInt32 behaviour_next_id, tInt32 object_id,
                                   tInt32 speed_id, tBehaviour* target) {
    boost::mutex::scoped_lock scoped_lock(update_behaviour_mutex_);
    target->behaviour_id = behaviour_id;
    target->behaviour_next_id = behaviour_next_id;
    target->object_id = object_id;
    target->speed_id = speed_id;
    target->timestamp = current_time_;
    //target->is_finished = false; // 5.11: new
}

void ManeuverPlanner::setBehaviour(tBehaviour& behaviour_new,
                                   tBehaviour* target) {
    boost::mutex::scoped_lock scoped_lock(update_behaviour_mutex_);
    *target = behaviour_new;
}

bool ManeuverPlanner::isBehaviourEqual(const tBehaviour& behaviour1,
                                       const tBehaviour& behaviour2) {
    boost::mutex::scoped_lock scoped_lock(update_behaviour_mutex_);
    return ((behaviour1.behaviour_id == behaviour2.behaviour_id) &&
            (behaviour1.speed_id == behaviour2.speed_id) &&
            (behaviour1.object_id == behaviour2.object_id));
}

void ManeuverPlanner::setDebugMode(bool debug_active) {
    LOG_DEBUG_ = debug_active;
}

void ManeuverPlanner::updateTimestamp(tTimeStamp time) { current_time_ = time; }


/* This method returns the obstacle type and its distance to the car */
obstacle_type_enum ManeuverPlanner::getObstacleAheadType(int object_id, float *distance_to_obst) {
    // try to find ID in map
    frAIburg::map::GlobalMap* map = frAIburg::map::getInstance();
    if (!map) return OT_UNKNOWN;
    tSptrMapElement obstacle_element =
        map->GetElement(static_cast<tMapID>(object_id));
    if (!obstacle_element) {
        tMapID obstacle_id = path_planner_->getClosestCollidingIDWithPlannerPath(); // todo use box instead
        obstacle_element = map->GetElement(obstacle_id);
    }
    if (!obstacle_element) {
        overwriteDisableEMBehaviour();
        return OT_CLEAR;
    }
    if (distance_to_obst) {
        path_planner_->getDistanceToObject(distance_to_obst,
                                           object_id, true);
    }
    MapElementType obstacle_type = obstacle_element->GetType();
    //MapElementCar* car_el;
    switch (obstacle_type) {
        case CAR:
	    return OT_CAR_BACK;
            // todo: if car from back: ..
          /*  car_el = dynamic_cast<MapElementCar*>(obstacle_element.get());
            if (car_el->GetCarOrientation() == AWAY) 
                return OT_CAR_BACK;
            else {printf("MP: Car not from back detected\n"); return OT_UNKNOWN;} // todo: only if not moving*/
            // return obstacle
        case PEDESTRIAN_ADULT:
        case PEDESTRIAN_CHILD:
        case PEDESTRIAN_REAL:
            return OT_PERSON;
        case OBSTACLE: // todo: correct?
        case DEPTH:
            return OT_UNKNOWN;
        default:
            printf("ManeuverPlanner::getObst..: default switch\n");
            overwriteDisableEMBehaviour();
            return OT_CLEAR;
    }
}

void ManeuverPlanner::printBehaviourIfChanged() {
    if (!isBehaviourEqual(behaviour_curr_, behaviour_prev_)) {
        printf("ManeuverPlanner: Transition to child state %s\n",
               behaviour_childstate_string[behaviour_state_child_]);
        fflush(stdout);
        printf("Transition from behav %s to %s \n ",
               behaviour_type_string[behaviour_prev_.behaviour_id],
               behaviour_type_string[behaviour_curr_.behaviour_id]);
        fflush(stdout);
    }
}

void ManeuverPlanner::printFullState() {
   printf("....\n");
	printf("Parentstate %i\n", behaviour_state_parent_);
	printf("Childstate %s\n",
               behaviour_childstate_string[behaviour_state_child_]);
        printf("ManeuverPlanner: Transition to child state %s\n",
               behaviour_childstate_string[behaviour_state_child_]);
	printf("behaviour_curr_ %s\n ",
		       behaviour_type_string[behaviour_curr_.behaviour_id]);
	printf("behaviour_statemachine_ %s\n ",
		       behaviour_type_string[behaviour_statemachine_.behaviour_id]);
	printf("behaviour_finished_ %s\n ",
		       behaviour_type_string[behaviour_finished_.behaviour_id]);
	printf("SM behaviour is %i\n", behaviour_statemachine_.is_finished);
   printf("....\n");
}

/* call this if OT_CLEAR detected in case of mismatch between
   what defines an "obstacle ahead", which leads to infinite loops */
void ManeuverPlanner::overwriteDisableEMBehaviour() {
    printf("overwriteDisableEMBehaviour called \n");
    setBehaviour(EM_DISABLED, BT_UNKNOWN, -1,
                 ST_SPEED_LOW, &behaviour_emergency_);
}

bool ManeuverPlanner::isFinished(const tBehaviour& behaviour) {
	return (behaviour_finished_.behaviour_id == behaviour.behaviour_id);
}

/*
pathplanner->isCarAhead(object id, v*, s*)
- just to check if car ahead; if yes, switch behaviour to car follow mode;
- trajectoryplanner calls isCarAhead to get v, s and adapt speed path
*/
