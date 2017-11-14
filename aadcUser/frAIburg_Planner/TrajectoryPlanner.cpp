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

/* This filter decides the mid-level behaviour of the car and commands
   speed and path planner depending on the current behaviour */

#include "TrajectoryPlanner.h"

using namespace frAIburg::map;

TrajectoryPlanner::TrajectoryPlanner() {
    // frAIburg::map::GlobalMap* map_ = frAIburg::map::getInstance();

    LOG_DEBUG_ = true;
    distance_from_init_ = 0.0;
}

/* Replan speed and optionally path; tBehaviour always contains a suggested
   speed
   given by the state machine and optionally an object ID to act on*/
planner_status TrajectoryPlanner::requestTrajectories(
    const tBehaviour& behaviour, const tPose& pose_global, float v_current,
    std::vector<Lane>* lane_vert_vec) {
    // printf("Entered requestTrajectories\n");
    planner_status status_request = PS_SUCCESSFUL;
    behaviour_curr_ = behaviour;
    tPath waypoints;
    float goal_orientation;
    float v_goal = getSpeedProposal(behaviour.speed_id);
    behaviour_type behaviour_id =
        static_cast<behaviour_type>(behaviour.behaviour_id);
    // Try to achieve suggested speeds at this distance:
    float speedplanner_dist;
    speed_planner_.getAccelerationDist(v_goal, &speedplanner_dist);
    bool skip_pathplanning = false;

    path_planner_.updateStatePointer(behaviour, pose_global);
    path_planner_.updateSpeed(v_current);
    path_planner_.updateLaneVecPointer(lane_vert_vec);

    // printf("requestTrajectories: Behav.|Object|Speed ID are %i %i %i \n",
    //        behaviour.behaviour_id, behaviour.object_id, behaviour.speed_id);
    switch (behaviour_id) {
        /* Lane following */
        case FOLLOW_LANE:
            if (!lane_vert_vec ||
                !path_planner_.getLaneFollowPath(&waypoints,
                                                 &goal_orientation,
                                                 LL_LANE_CENTER)) {
                return PS_NO_LANES;
            }
            break;
        case CHANGELANE_TO_LEFT:
        case CHANGELANE_TO_RIGHT:
            if (!lane_vert_vec ||
                !path_planner_.getLaneSwitchPath(&waypoints,
                                                 &goal_orientation)) {
                return PS_NO_LANES;
            }
            break;
        case FOLLOW_LANE_TO_CLEAR:
            if (lane_vert_vec)
                path_planner_.getLaneToClearPath(&waypoints,
                                                 &goal_orientation);
            break;
        /* Hard stop */
        case STOP:
        case EM_ENABLED:
            speedplanner_dist = 0.01;  // stop immediately
            skip_pathplanning = true;
            break;
        /* Object related */
        case STOP_AT_OBJECT:
        case GO_TO_OBJECT:
            processIdBehaviour(behaviour, pose_global, &waypoints,
                               &goal_orientation, &speedplanner_dist,
                               lane_vert_vec, &skip_pathplanning);
            break;
        /* Crossing related*/
        case TURN_LEFT:
        case TURN_RIGHT:
        case GO_STRAIGHT:
            path_planner_.getCrossingPath(&waypoints, &goal_orientation);
            break;
        /* Pulling out from parking spot */
        case PULL_LEFT:
        //case PULL_RIGHT:
        case PULL_RIGHT_FWD:
        case PULL_RIGHT_BWD:
            path_planner_.getPullOutPath(&waypoints, &goal_orientation);
	   break;
        /* Parking */
        case STOP_AT_PARKING_INFRONT:
            path_planner_.getPathToParkingSwitch(&waypoints, &goal_orientation);
            break;
        case PARK_AT_OBJECT:
            path_planner_.getPathToPark(&waypoints, &goal_orientation);
            break;
        /* Obstacle avoidance */
        case GO_BACKWARDS:
            path_planner_.getPathBackwards(&waypoints,
                                           &goal_orientation);
            break;
        case FOLLOW_CAR:
            path_planner_.getCarFollowTrajectory(&waypoints, &goal_orientation,
                                                 v_current, &v_goal, &speedplanner_dist);
            break;
        case GO_AROUND_OBJECT:
            printf(
                "DON'T USE GO_AROUND_OBJECT!! Remove from Liors state machine "
                "as well\n");
            path_planner_.getGoAroundPath(&waypoints, &goal_orientation);
        case DEMO_GO_TO_PERSON:
        case DEMO_GO_TO_INIT:
            path_planner_.getDemoPath(&waypoints, &goal_orientation);
        default:
            printf(
                "WARNING: TrajectoryPlanner couldn't decide what to do."
                " Return PS_FAILED. (e.g. Check if lanes found.) behaviour_id "
                "is %i\n",
                behaviour_id);
            return PS_FAILED;
    }
    // always set proposed speed, independent of path success
    speed_planner_.setGoalPoint(v_goal, speedplanner_dist);

    // if (LOG_DEBUG_)
    //   printf("TrajectoryPlanner: ");

    if (LOG_DEBUG_)
        //printf("behavspeed id is %i, speed is %f\n", behaviour.speed_id,
        //       getSpeedProposal(behaviour.speed_id));

    if (skip_pathplanning) return PS_SUCCESSFUL;

    status_request =
        path_planner_.PlanPath(waypoints, goal_orientation);
    if (status_request == PS_SUCCESSFUL) {
        // distance_from_init_ will be compared to path length to
        // determine whether maneuver was finished
        distance_from_init_ = 0.;
    }

    return status_request;
}

/* Processing STOP_AT_ID, GO_TO_OBJECT */
bool TrajectoryPlanner::processIdBehaviour(
    const tBehaviour& behaviour, const tPose& pose_curr, tPath* waypoints,
    float* goal_orientation, float* dist_to_object,
    std::vector<Lane>* lane_vert_vec, bool* skip_pathplanning) {
    *skip_pathplanning = true;
    frAIburg::map::GlobalMap* map_ = frAIburg::map::getInstance();
    if (!map_) return false;
    tSptrMapElement el =
        map_->GetElement(static_cast<tMapID>(behaviour.object_id));
    if (!el) {
        printf("TrajectoryPlanner: Failed to find object at id %i\n",
               behaviour.object_id);
        return false;
    }
    // map_->Print();
    MapElementType el_type = el->GetType();
    *dist_to_object =
        path_planner_.getStopDistToObject(behaviour.object_id, true, true);
    if (*dist_to_object == 0.) {
        return false;
    }
    /* If too close to stopping object, skip lane following and approach
       stopping with a small angle for better turn performance */
    if ((el_type >= STREET_MARKER_STOP_LINE &&
         el_type <= STREET_MARKER_CROSSING_X) &&
        *dist_to_object <
            1.) {  // todo: put to 0.5 so that always go straight!?
        // check map_types.h to see all elements in range:
        if (behaviour.behaviour_next_id == TURN_LEFT ||
            behaviour.behaviour_next_id == TURN_RIGHT ||
            behaviour.behaviour_next_id == GO_STRAIGHT) {
            path_planner_.getCrossingApproachPath(waypoints, goal_orientation);
            /*waypoints->clear();
            waypoints->push_back(getTPoint(0, 0));
            waypoints->push_back(getTPoint(*dist_to_object, .0));
            *goal_orientation = 0;*/

            if (LOG_DEBUG_) printf("Try to do stopping approach at angle\n");
            // path_planner_.getCrossingApproachPath(behaviour, waypoints,
            //                                      goal_orientation,
            //                                      pose_curr);
            *skip_pathplanning = false;
            return true;
        }
    }

    /* In all other cases, to normal lane following, if possible */
    if (lane_vert_vec &&
        path_planner_.getLaneFollowPath(waypoints, goal_orientation, LL_LANE_CENTER)) {
        *skip_pathplanning = false;
    } else {
        *skip_pathplanning = true;
    }
    return true;
}

bool TrajectoryPlanner::isManeuverFinished(const tPose& pose_curr, float v_curr,
                                           const tBehaviour& behaviour) {

    // assume that we started stopping from v > 0.01 m/s, deadlock otherwise
    // printf("GlPlanner: Received behaviour %i \n", behaviour);
    switch (behaviour.behaviour_id) {
        // never-completed
        case FOLLOW_LANE:
        case FOLLOW_CAR:
            return false;
        // stopping
        case STOP:
        case EM_ENABLED:
            return (fabs(v_curr) < SPEED_STOPPED_);
        case STOP_AT_OBJECT:
            return (fabs(v_curr) <= SPEED_STOPPED_) &&
                   ((path_planner_.getStopDistToObject(behaviour.object_id,
                                                       true, false) < 0.4));
        case GO_TO_OBJECT:
            return (path_planner_.getStopDistToObject(behaviour.object_id, true,
                                                      false) < 0.1);
        // passing an obstacle
        case FOLLOW_LANE_TO_CLEAR:
             return path_planner_.goal_global_.isManeuverGoalPassed(pose_curr, false)
                     //&& path_planner_.getClosestCollidingIDWithPlannerPath(LL_LANE_RIGHT) == MAP_DEFAULT_ID // no obstacle
                     && path_planner_.getClosestCollidingIDWithBox(0.3, -0.3) == MAP_DEFAULT_ID; //box to check behind the car 0.6
        // standard maneuvers
        case CHANGELANE_TO_LEFT:
        case CHANGELANE_TO_RIGHT:
        case GO_STRAIGHT:
        case PULL_LEFT:
        case PULL_RIGHT_FWD:
        case STOP_AT_PARKING_INFRONT:
            return path_planner_.goal_global_.isManeuverGoalPassed(pose_curr, false);
        // needs to have smaller goal angle deviation
        case TURN_LEFT:
        case TURN_RIGHT:
            return path_planner_.goal_global_.isManeuverGoalPassed(pose_curr, false, 10./180. * PI); // use smaller
        // backward driving
        case GO_BACKWARDS:
        case PULL_RIGHT_BWD:
        case PARK_AT_OBJECT:
            return path_planner_.goal_global_.isManeuverGoalPassed(pose_curr, true);
        // demo task
        case DEMO_GO_TO_PERSON:
        case DEMO_GO_TO_INIT:
            printf("isManeuverFinished not implmented for demo\n");
            break;
        default:  // turn_left, turn_right.. something else?
            printf("TrajPlanner isManeuverFinished reached DEFAULT; ERROR!!! return false\n");
            //float path_length;
            //path_planner_.gp_gen_.getPathLength(&path_length);
            //return (path_length < distance_from_init_);
	       return false;
    }
    return false;
}

float TrajectoryPlanner::getSpeedProposal(int speed_id) const {
    switch (static_cast<speed_type>(speed_id)) {
        case ST_BACKWARDS:
            return -SPEED_LOW_;
        case ST_STOP:
            return 0.0;
        case ST_SPEED_LOW:
            return SPEED_LOW_;
        case ST_SPEED_MEDIUM:
            return SPEED_MEDIUM_;
        case ST_SPEED_HIGH:
            return SPEED_HIGH_;
        case ST_SPEED_ULTRAHIGH:
            return SPEED_ULTRAHIGH_;
        default:
            printf(
                "WARNING: TrajectoryPlanner: Invalid speed ID. Assume "
                "SPEED_LOW_\n");
            return SPEED_LOW_;
    }
}

void TrajectoryPlanner::updateDistance(float ds) { distance_from_init_ += ds; }

tPoint TrajectoryPlanner::getTPoint(float x, float y) const {
    tPoint res;
    res.x = x;
    res.y = y;
    return res;
}

void TrajectoryPlanner::setDebugMode(bool debug_active) {
    LOG_DEBUG_ = debug_active;
}
