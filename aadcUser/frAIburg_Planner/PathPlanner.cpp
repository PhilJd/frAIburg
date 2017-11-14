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

/* This filter is the low-level path planner that contains methods for
   way point generation and also a main method that commands the "Spline.cpp
   to return a sampled path */

#include "PathPlanner.h"

using namespace frAIburg::map;

PathPlanner::PathPlanner() {
    MAX_LANE_WIDTH_ERROR_ = 0.1;
    LOG_DEBUG_ = true;
    // request_rear_cam_lanes_ = false;
    goal_global_.is_active_ = false;
    pose_curr_ = NULL;
    behaviour_curr_ = NULL;
    v_current_ = 0.;
}
/* ###########################################################################
   Main method for planning path from given way points
   ##########################################################################*/

planner_status PathPlanner::PlanPath(const tPath& way_points,
                                     float goal_orientation) {
    /* way_points and goal orientation.
       A spline is calculated, sampled and stored in member
       variable current_path_. Finally, the GoalPointGenerator is called and the
       sampled
       path passed.
       Goal point updates based on odometry can be obtained by
       gpgen.getUpdatedGoalgetPathLength() */
    float straight_ext_length = 0.0;
    if (way_points.size() == 0 ||
        !spline_.getSampledPath(way_points, goal_orientation, &current_path_,
                                straight_ext_length)) {
    //   if(LOG_DEBUG_)
		//printf("Spline failed to return path. Given waypointsvec has size %i\n", way_points.size());
        return PS_FAILED;
    }
    DrawCurrentPath(3.0);
    // DrawCurrentway_points(3.0, way_points);
    //  DO COLLISION CHECKING HERE:

    //  if(CheckCollision(current_path_, &obstacle_id_))
    //    return PS_STAT_OBSTACLE;

    //  printf("current path has size %i\n", current_path_.size());
    // printf("Set new path for gpgen with current pos. (%f %f) \n",
    // pose_curr_->x, pose_curr_->y);
    gp_gen_.setNewPath(&current_path_, *pose_curr_);
    return PS_SUCCESSFUL;
    //  store collision points and angles internally
}

/* ###########################################################################
   In this section methods for computing way_points for different maneuvers are
   located.
   ##########################################################################*/

/* This method computes waypoint to the point from which we want to turn into
   backward mode in order to park; Two cases: Either we are at similar height to
   current parking spot, or we've already passed it.
   - Assumption (unchecked) Parking spots always on the right, 90deg to road,
     and car always parallel to road */

void PathPlanner::getDemoPath(tPath* way_points,
                              float* goal_orientation) {
  printf("getDemoPath called, but not implemented \n");
}

void PathPlanner::getCarFollowTrajectory(tPath* way_points,
                                         float* goal_orientation,
                                         float v_current,
                                         float* v_tracking, float* dist_car) {
    const float MIN_DIST_TO_CAR_FOLLOW = 1.;
    const float MAX_DIST_TO_CAR_FOLLOW = 1.5;

    /* do normal lane following */

    getLaneFollowPath(way_points, goal_orientation,
                      LL_LANE_CENTER);


    /* set tracking velocity */
    /*
    tSptrMapElement obstacle_el =
        map_->GetElement(static_cast<tMapID>(behaviour_curr_->object_id));
    if (!obstacle_el) {
        obstacle_el = map_->GetElement(getClosestCollidingObjectID()); // todo:element in range
    }*/
    float dist_to_car = 1.;
    float v_rel = 0.5;
    if(!getDistSpeedCarAhead(&v_rel, &dist_to_car)) return;

    *v_tracking = v_current + v_rel;

    if (dist_to_car < MIN_DIST_TO_CAR_FOLLOW) {
        // drive slower
        //if (fabs(*v_tracking) < 0.3 && dist_to_car > 0.5) {
        //  *v_tracking = 0.3;
        //} else {
          *v_tracking = 0.9 * (*v_tracking);
        //}
        printf("Pf: Dist. to car < %f, lower v_tracking\n",
               MIN_DIST_TO_CAR_FOLLOW);
    } else if (dist_to_car > MAX_DIST_TO_CAR_FOLLOW) {
	    //if (fabs(*v_tracking) < 0.3 && dist_to_car > 0.5) {
        //  *v_tracking = 0.3;
        //} else {
          *v_tracking = 1.1 * (*v_tracking);
        //}
    }
    // distance when to reach desired tracking vel.
    *dist_car = 0.5*dist_to_car;//0.5*dist_to_car;
    //  v_tracking should be in range 0..1.5m/s
    *v_tracking = 1. * std::min(1. * (*v_tracking), 1.2);
    printf("Following car with v_tracking = %f\n", *v_tracking);
}

bool PathPlanner::getDistSpeedCarAhead(float* v_rel, float* distance) {
    const float MAX_DIST_TO_CAR_FOLLOW = 2.;

    tSptrMapElement el;
    std::vector<tSptrMapElement> detected_cars;
    const std::vector<MapElementType> get_types =
        boost::assign::list_of(CAR);
    map_->GetAllElementsWithTypes(get_types, detected_cars);
    BOOST_FOREACH (const tSptrMapElement& s, detected_cars) { // todo: get closest..
        *distance = s->GetDistanceToCar();
        if (*distance < MAX_DIST_TO_CAR_FOLLOW &&
            fabs(sin(s->GetOrientationAngleToCar()) * (*distance)) < 1.3) { // todo: orientation not put correclty in map
            el = s; // get()?
        }
        std::cout << "PP: dist to car is " << *distance <<  " orientation angle "<<  s->GetOrientationAngleToCar() << std::endl;
    }
    if (!el) {std::cout << "PP: no car found" << std::endl; return false;}

    MapElementCar* car_el = dynamic_cast<MapElementCar*>(el.get());
    // far away: use only bounding box
    if (*distance > 1.5) {
        *v_rel = car_el->GetRelativeLocalXSpeedObjectDetection();
    } else { // near range: Also use depth
        *v_rel = 0.5*(car_el->GetRelativeLocalXSpeedObjectDepth() +
                      car_el->GetRelativeLocalXSpeedObjectDetection());
    }
    return true;
}

bool PathPlanner::isLaneChangeAllowed() {
    // any object is blocking left lane, or parking spot, crossing ahead
    if (current_path_.size() > 4) {
      float dx = current_path_[current_path_.size() -1].x -
                 current_path_[current_path_.size() -5].x;
      float dy = current_path_[current_path_.size() -1].y -
                 current_path_[current_path_.size() -5].y;
      if (dx != 0.) {
        float slope  = dy/dx;
        if (fabs(slope) > 0.7) {printf("isLaneChangeAllowed false, curved path\n"); return false;} else {
          printf("lanechange allowed with slope %f \n", slope);
        }
      }
    }
    const float STREETSIGN_BLOCK_DIST_X = 1.;
    const std::vector<MapElementType> exclude_types =
        boost::assign::list_of(STREET_MARKER_ZEBRA)
                               (STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT)
                               (STREET_TRAFFIC_SIGN_STOP)
                               (STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC)
                               (STREET_TRAFFIC_SIGN_JUST_STRAIGHT)
                               (STREET_TRAFFIC_SIGN_GIVE_WAY)  // yield sign
                               (STREET_TRAFFIC_SIGN_CROSSWALK)
                               (STREET_TRAFFIC_SIGN_PARKING)
                               (STREET_TRAFFIC_SIGN_CIRCLE)
                               (STREET_TRAFFIC_SIGN_NO_TAKE_OVER)
                               (STREET_TRAFFIC_ROAD_WORKS)
                               (LM_STREET_PARKING_SPOT);

    tSptrMapElement el;
    std::vector<tSptrMapElement> detected_objects;
    map_->GetAllElementsWithTypes(exclude_types, detected_objects);

    BOOST_FOREACH (const tSptrMapElement& s, detected_objects) {
        float x_distance = s->GetDistanceToCar()*cos(s->GetOrientationAngleToCar());
        float y_distance = s->GetDistanceToCar()*sin(s->GetOrientationAngleToCar());
        switch (s->GetType()) {
            // hard obstacles
            case PEDESTRIAN_ADULT:
            case PEDESTRIAN_CHILD:
            case PEDESTRIAN_REAL:
            case CAR:
            case DEPTH:
            case ULTRRASONIC:
                if (y_distance < 1.5*LANEWIDTH_ && y_distance > 0.3*LANEWIDTH_) {
                    printf("isLaneChangeAllowed false: "
                           "Blocked by hard obstacle\n");
                    return false;
                }
                break;
            case STREET_TRAFFIC_SIGN_NO_TAKE_OVER:
            case STREET_TRAFFIC_ROAD_WORKS:
                if (x_distance > -5. && x_distance < 2.) {
                    printf("isLaneChangeAllowed false: "
                           "NO_TAKE_OVER or ROAD_WORKS\n");
                    return false;
                }
                break;
            // street signs for crossings etc:
            default:
                if (y_distance < 0.7 && y_distance > -1.2 &&
                    x_distance < 1.7 && x_distance >-0.5) {
                    printf("isLaneChangeAllowed false: StreetSigns too"
                           " close (possible crossing) %s\n", s->ToString().c_str());
	            //map_->Print();.
                    return false;
                }
                break;
        }
    }
    printf("isLaneChangeAllowed true\n");
    return true;
}

void PathPlanner::getPathBackwards(tPath* way_points, float* goal_orientation) {
    // flip x-val of lane follow path or set a hard goal
    const float X_MIN = -0.5;
    /*if (getLaneFollowPath(way_points, goal_orientation, LL_LANE_CENTER)) {
        for (int i = 0; i < way_points->size(); ++i) {
            (*way_points)[i].x = - (*way_points)[i].x;
            (*way_points)[i].y = - (*way_points)[i].y;
        }
    } else if (!goal_global_.isGoalSet(behaviour_curr_->behaviour_id)) {
    	  way_points->clear();
        way_points->push_back(getTPoint(0., 0));
        way_points->push_back(getTPoint(X_MIN, 0));
        *goal_orientation = 0.;
    }*/

    if (!goal_global_.isGoalSet(behaviour_curr_->behaviour_id)) {
          way_points->clear();
        way_points->push_back(getTPoint(0., 0));
        way_points->push_back(getTPoint(-1., 0)); // for better stability
        *goal_orientation = 0.;
        goal_global_.SetManeuverGoal(*pose_curr_, X_MIN,
                                 way_points->back().y, *goal_orientation + PI,
                                 behaviour_curr_->behaviour_id);
    }

    // and just flip the relative goal

}
void PathPlanner::getPathToParkingSwitch(tPath* way_points,
                                         float* goal_orientation) {
    const float CENTER_DIST = 0.4;
    tSptrMapElement el =
        map_->GetElement(static_cast<tMapID>(behaviour_curr_->object_id));
    if (!el) {
        printf(
            "WARNING: PP: getPathToParkingSwitch() failed, no parking spot found "
            "with the given ID.\n");
        return;
    }
    way_points->clear();
    way_points->push_back(getTPoint(0., 0.));

    // car rear pos. rel. to parking spot (road facing point = shifted center)
    tPose car_rear_rel_to_spot;
    float y_dist = sin(el->GetOrientationAngleToCar()) * (el->GetDistanceToCar());
    float global_angle_parkspot = el->GetGlobalOrientation();
    /* parking spot on the right */
    if (y_dist < 0) {
        printf("parking spot on the right\n");
        car_rear_rel_to_spot.x = 0.5*LANEWIDTH_;
        car_rear_rel_to_spot.y = -1.5*LANEWIDTH_;
        car_rear_rel_to_spot.heading = -20.*PI/180;
    } else {
    /* parking spot on the left */
        car_rear_rel_to_spot.x = 0.5*LANEWIDTH_;
        car_rear_rel_to_spot.y = 1.3*LANEWIDTH_;
        car_rear_rel_to_spot.heading = 20.*PI/180;
    }
    /* calc. car position ( = front bumper) */
    car_rear_rel_to_spot.x += cos(car_rear_rel_to_spot.heading)*CAR_LENGTH_ + CENTER_DIST;
    car_rear_rel_to_spot.y += sin(car_rear_rel_to_spot.heading)*CAR_LENGTH_;
    /*Goal at road facing side of parking spot, add center dist.*/
    tMapPoint goalpnt_global = el->GetGlobalPolyCenter();
    tPose goal_pose;
    goal_pose.heading = global_angle_parkspot + car_rear_rel_to_spot.heading;
    goal_pose.x = goalpnt_global.get<0>() + (car_rear_rel_to_spot.x)*cos(global_angle_parkspot) - (car_rear_rel_to_spot.y)*sin(global_angle_parkspot);
    goal_pose.y = goalpnt_global.get<1>() + (car_rear_rel_to_spot.x)*sin(global_angle_parkspot) + (car_rear_rel_to_spot.y)*cos(global_angle_parkspot);

    printf("park x y global %f %f \n", goalpnt_global.get<0>(), goalpnt_global.get<1>());
    printf("global park angle is %f \n",global_angle_parkspot);
    printf("goal rel. x y phi %f %f %f \n", car_rear_rel_to_spot.x, car_rear_rel_to_spot.y, car_rear_rel_to_spot.heading);
    printf("goal global. x y phi %f %f %f \n", goal_pose.x, goal_pose.y, goal_pose.heading);
    getPathToGlobalCoord(goal_pose, way_points, goal_orientation);
    /* Simple case: Car is a lane width or less past the parking spot */

    goal_global_.SetManeuverGoal(goal_pose.x, goal_pose.y, goal_pose.heading,
                                 behaviour_curr_->behaviour_id);
    return;
}

void PathPlanner::getPathToPark(tPath* way_points,
                                float* goal_orientation) {
    const float CENTER_DIST = 0.4;
    tSptrMapElement el =
        map_->GetElement(static_cast<tMapID>(behaviour_curr_->object_id));
    if (!el) {
        printf(
            "WARNING: GP: getPathToPark() failed, no obstacle found with the"
            " given ID.\n");
        return;
    }
    tPose goal_pose_global;
    goal_pose_global.heading = (el->GetGlobalOrientation());
    tMapPoint goalpnt_global = el->GetGlobalPolyCenter();
    float dy = -0.15;
    
    // add dist. to parking spot center; we want to be at the road facing side of the spot
    goal_pose_global.x = goalpnt_global.get<0>() + cos(goal_pose_global.heading)*CENTER_DIST - dy*sin(goal_pose_global.heading);
    goal_pose_global.y = goalpnt_global.get<1>() + sin(goal_pose_global.heading)*CENTER_DIST + dy*cos(goal_pose_global.heading);
    goal_global_.SetManeuverGoal(goal_pose_global.x, goal_pose_global.y,
                                 goal_pose_global.heading + PI,
                                 behaviour_curr_->behaviour_id);

    if (LOG_DEBUG_) std::cout << "entered getPathToPark\n" << std::endl;
    bool backward_lanes_available = false;
    //  try to use detectecd lane markings:
    if (goal_global_.isCarAlignedWithGoal(*pose_curr_, 30. * 180. / PI)) {
        backward_lanes_available = false;
            //getBackwardsLanePath(way_points, goal_orientation);
        // printf("Parking could %s find parking lanes\n",
        // backward_lanes_available?"":"NOT");
    }
    if (backward_lanes_available) {
        printf("Found way_points to park using lanes!\n");
        return;
    } else {
        //  blind driving:
        getPathToGlobalCoord(goal_pose_global, way_points, goal_orientation, -CAR_LENGTH_);
        return;
    }
}


void PathPlanner::getPathToGlobalCoord(const tPose& pose_goal,
                                       tPath* way_points,
                                       float* goal_orientation,
				       float x_extension) const {
    //  compute diff. vec. in global frame
    float dx = pose_goal.x - pose_curr_->x;// + x_extension*cos(pose_goal);
    float dy = pose_goal.y - pose_curr_->y;// + x_extension*sin(pose_goal);
    tPoint goal_point;
    //  rotate point clockwise by current heading
    goal_point.x = (dx)*cos(-pose_curr_->heading) - (dy)*sin(-pose_curr_->heading);
    goal_point.y = (dx)*sin(-pose_curr_->heading) + (dy)*cos(-pose_curr_->heading);
    way_points->clear();
    way_points->push_back(getTPoint(0., 0.));
    way_points->push_back(goal_point);
    if (x_extension != 0.) { // todo:decide whether to put in ffront or after
        float dphi = pose_goal.heading - pose_curr_->heading;
        float x_tmp = goal_point.x + x_extension*cos(dphi);
        float y_tmp = goal_point.y + x_extension*sin(dphi);
	way_points->push_back(getTPoint(x_tmp, y_tmp));
    }
    *goal_orientation = fmod(pose_goal.heading - pose_curr_->heading, 2*PI);
    if (*goal_orientation > PI) { *goal_orientation -= 2*PI;}
   else if (*goal_orientation < -PI) { *goal_orientation += 2*PI;}


    return;
}

bool PathPlanner::getCrossingPath(tPath* way_points,
                                  float* goal_orientation) {
    if (!goal_global_.isGoalSet(behaviour_curr_->behaviour_id)) {
        way_points->clear();
        way_points->push_back(getTPoint(0., 0.));
        tPoint goal_rel_from_center;
        float angle_offset = 0.;  //  to be added
        if (behaviour_curr_->behaviour_id == TURN_LEFT) {
            goal_rel_from_center.x = 0.45 * LANEWIDTH_;
            goal_rel_from_center.y = 2. * LANEWIDTH_;
            angle_offset = 0.5 * PI;  //  0.4
        } else if (behaviour_curr_->behaviour_id == TURN_RIGHT) {
            goal_rel_from_center.x = -0.5 * LANEWIDTH_;
            goal_rel_from_center.y = -2.2 * LANEWIDTH_;
            //way_points->push_back(getTPoint(0.1, 0.));
            angle_offset = -0.5 * PI;
        } else if (behaviour_curr_->behaviour_id == GO_STRAIGHT) {
            goal_rel_from_center.x = 0.8 * LANEWIDTH_;
            goal_rel_from_center.y = -0.5 * LANEWIDTH_;
            angle_offset = 0.;
        } else {
            printf("Invalid behaviour for getCrossingPath\n");
        }
        tPoint goal_from_car;
        float dphi_crossing = 0.;
        getPointOnCrossing(behaviour_curr_->object_id, goal_rel_from_center,
                           &goal_from_car, &dphi_crossing);

        *goal_orientation = dphi_crossing + angle_offset;
         if (behaviour_curr_->behaviour_id == GO_STRAIGHT) {
            way_points->push_back(getTPoint(0.5*goal_from_car.x, goal_from_car.y));
            *goal_orientation = 0.;
         }
        goal_global_.SetManeuverGoal(*pose_curr_, goal_from_car.x, goal_from_car.y,
                                     *goal_orientation, behaviour_curr_->behaviour_id);
        way_points->push_back(goal_from_car);
    } else if(!getGoalAlignedLanePath( way_points,
                                       goal_orientation)) {
        return false;
    }

    // printf("Turning Path: dphi %f | goal_car_frame_x %f | goal_car_frame_y %f
    // | goalorient %f\n", dphi_crossing, goal_from_car.x, goal_from_car.y,
    // *goal_orientation);
    return true;
}

bool PathPlanner::getCrossingApproachPath(tPath* way_points,
                                          float* goal_orientation) {
    int behaviour_id_nxt = behaviour_curr_->behaviour_next_id;

    tPoint goal_rel_from_center;
    float angle_offset = 0.;  //  to be added
    if (behaviour_id_nxt == TURN_LEFT) {
        // printf("Approach for turning left\n");
        goal_rel_from_center.x = -LANEWIDTH_ - STOPLINE_OFFSET_;
        goal_rel_from_center.y = -0.5 * LANEWIDTH_;
        angle_offset = 0. * PI / 180.;
    } else if (behaviour_id_nxt == TURN_RIGHT) {
        // printf("Approach for turning right\n");
        goal_rel_from_center.x = -LANEWIDTH_ - STOPLINE_OFFSET_;
        goal_rel_from_center.y = -0.5 * LANEWIDTH_;
        angle_offset = -0. * PI / 180.;
    } else if (behaviour_id_nxt == GO_STRAIGHT) {
        // printf("Approach for going straight\n");
        goal_rel_from_center.x = -LANEWIDTH_ - STOPLINE_OFFSET_;
        goal_rel_from_center.y = -0.5 * LANEWIDTH_;
        angle_offset = 0.;
    } else {
        printf("getCrossingApproachPath failed, invalid behaviour_id (%i)\n",
               behaviour_curr_->behaviour_id);
    }
    tPoint goal_from_car;
    float dphi_crossing = 0.;
    getPointOnCrossing(behaviour_curr_->object_id, goal_rel_from_center,
                       &goal_from_car, &dphi_crossing);

    *goal_orientation = dphi_crossing + angle_offset;
    if (fabs(*goal_orientation) >= PI) {
        *goal_orientation += (*goal_orientation < 0) ? 2. * PI : -2. * PI;
    }
    way_points->clear();
    way_points->push_back(getTPoint(0., 0.));
    way_points->push_back(goal_from_car);
    goal_global_.SetManeuverGoal(way_points->back().x,
                                 way_points->back().y, *goal_orientation,
                                 behaviour_curr_->behaviour_id);
    return true;
}

bool PathPlanner::getPointOnCrossing(int crossing_id,
                                     const tPoint& point_relative_crossing,
                                     tPoint* pnt_car_frame,
                                     float* dphi_crossing) const {
    tSptrMapElement el = map_->GetElement(static_cast<tMapID>(crossing_id)); // todo: failsafe, just assume values
    if (!el) {
        printf("WARNING: getTurningPath: Coulnd't find crossing at ID %i\n",
               crossing_id);
        return false;
    }
    printf("Crossing global orient. is %f, current heading %f\n", el->GetGlobalOrientation(), pose_curr_->heading);
    *dphi_crossing  = el->GetGlobalOrientation() - pose_curr_->heading - PI; // todo: this is untested
    // map to -PI..PI
    *dphi_crossing = fmod(*dphi_crossing, 2*PI);
    if (*dphi_crossing >= PI) {
        *dphi_crossing -= 2*PI;
    } else if (*dphi_crossing <= -PI) {
      *dphi_crossing += 2*PI;
    }
    //  assume crossing is at multiple of 90deg; just round current
    //  car heading to multiple of 90deg
  /*  *dphi_crossing = fmod(pose_curr_->heading, 0.5 * PI); // todo: use crossing orientation directly
    if (*dphi_crossing >= 0.25 * PI) *dphi_crossing -= 0.5 * PI;
    if (*dphi_crossing <= -0.25 * PI) *dphi_crossing += 0.5 * PI;
    *dphi_crossing = -*dphi_crossing;*/

    float x_dist_center =
        cos(el->GetOrientationAngleToCar()) * (el->GetDistanceToCar());
    float y_dist_center =
        sin(el->GetOrientationAngleToCar()) * (el->GetDistanceToCar());
    // if (x_dist_center > 0.8)
    //   x_dist_center = 0.8;
    // if (x_dist_center < 0.5)
    //   x_dist_center = 0.5;
    // y_dist_center = 0.25*LANEWIDTH_;

    (*pnt_car_frame).x = (point_relative_crossing.x) * cos(*dphi_crossing) -
                         (point_relative_crossing.y) * sin(*dphi_crossing) +
                         x_dist_center;
    (*pnt_car_frame).y = (point_relative_crossing.x) * sin(*dphi_crossing) +
                         (point_relative_crossing.y) * cos(*dphi_crossing) +
                         y_dist_center;
    if (LOG_DEBUG_) {
        printf("Crossing dphi = %f, car heading %f \n", *dphi_crossing,
               pose_curr_->heading);
        printf("Crossing angle relative to car is: %f\n",
               el->GetOrientationAngleToCar());
        printf("Crossing center (x, y) = %f %f \n", x_dist_center,
               y_dist_center);
        printf("Pnt on Crossing in car frame (x, y) = %f %f \n",
               (*pnt_car_frame).x, (*pnt_car_frame).y);
    }

    return true;
}

bool PathPlanner::getPullOutPath(tPath* way_points,
                                 float* goal_orientation) {
    static tPose pose_init;
    // goal relative from starting pose backward goal depends on it
    tPose goal_right_fwd;

    way_points->clear();
    switch (behaviour_curr_->behaviour_id) {
        case PULL_LEFT:
            way_points->push_back(getTPoint(0, 0));
            way_points->push_back(getTPoint(1.5 * LANEWIDTH_, 2 * LANEWIDTH_));
            *goal_orientation = 0.5 * PI;
             goal_global_.SetManeuverGoal(*pose_curr_, way_points->back().x,
                                 way_points->back().y, *goal_orientation,
                                 behaviour_curr_->behaviour_id);
            break;
        case PULL_RIGHT_FWD:
            // save global goal for PULL_RIGHT_BWD
      	    pose_init = *pose_curr_;
            //printf("global init pull fwd is %f %f %f \n", pose_init.x, pose_init.y, pose_init.heading);
	          // forward goal: pos of back of car
	    goal_right_fwd.x = 0.8*LANEWIDTH_ + 0.2; // 0.7
    	    goal_right_fwd.y = -0.4*LANEWIDTH_;
    	    goal_right_fwd.heading = -0.3*PI;
           *goal_orientation = goal_right_fwd.heading;
            way_points->push_back(getTPoint(0, 0));
            way_points->push_back(
            getTPoint(goal_right_fwd.x + cos(*goal_orientation)*CAR_LENGTH_,
                      goal_right_fwd.y + sin(*goal_orientation)*CAR_LENGTH_));

            goal_global_.SetManeuverGoal(*pose_curr_, way_points->back().x,
                                 way_points->back().y, *goal_orientation,
                                 behaviour_curr_->behaviour_id);
            break;
        case PULL_RIGHT_BWD:
            // goal from initial pose
            //tPose goal_right_bwd;
            //goal_right_bwd.x = 0.5*LANEWIDTH_;
            //goal_right_bwd.y = 1.*LANEWIDTH_;
            //goal_right_bwd.heading = -0.5*PI;
            // set goal for finish
            float dx = 0.5*LANEWIDTH_ + 0.35; // before +0.2
	    float dy = -0.8*LANEWIDTH_;

            tPose goal_pose;
            goal_pose.x = pose_init.x + dx*cos(pose_init.heading) - dy*sin(pose_init.heading);
            goal_pose.y = pose_init.y + dx*sin(pose_init.heading) + dy*cos(pose_init.heading);
            goal_pose.heading = pose_init.heading - 0.5*PI;
            goal_global_.SetManeuverGoal(goal_pose.x, goal_pose.y,
                                         goal_pose.heading + PI,
                                         behaviour_curr_->behaviour_id);
            // extend car to the back for pathplanning
            //pose_init.x += -CAR_LENGTH_*cos(pose_init.heading);
            //pose_init.y += -CAR_LENGTH_*sin(pose_init.heading); //CAR_LENGTH_
	          getPathToGlobalCoord(goal_pose, way_points, goal_orientation, -1.);
//printf("goal orient is %f goal heading is %f current heading is %f\n", *goal_orientation, pose_init.heading, pose_curr_->heading);
           // printf("goal global pull bwd is %f %f %f \n", pose_init.x, pose_init.y, pose_init.heading);
            // transform into new frame
            // rotate point clockwise by current heading
            //float dx = goal_right_bwd.x - goal_right_fwd.x;
            //float dy = goal_right_bwd.y - goal_right_fwd.y;
            //float x_goal = -0.3;//dx*cos(-goal_right_fwd.heading) - dy*sin(-goal_right_fwd.heading);
            //float y_goal = 0;//dx*sin(-goal_right_fwd.heading) + dy*cos(-goal_right_fwd.heading);

            //*goal_orientation = -0.5*PI - goal_right_fwd.heading;
            //way_points->push_back(getTPoint(0, 0));
            //way_points->push_back(getTPoint(x_goal, y_goal));

            // goal_global_.SetManeuverGoal(*pose_curr_, way_points->back().x,
            //                      way_points->back().y, *goal_orientation + PI,
            //                      behaviour_curr_->behaviour_id);
          break;
       // default:
       //     return false;
    }
    return true;
}

/* ###########################################################################
   In this section all lane-related methods are implemented
   ##########################################################################*/
bool PathPlanner::getLaneToClearPath(tPath* way_points, float* goal_orientation) {
    //  set goal in local frame; this will happen only once
    // todo: don't do this , just follow lane; in Traj.Planner, check if clear
    float x_dist = 0.4;//getStopDistToObject(behaviour_curr_->object_id, true, true);
    *goal_orientation = 0.;

    if (getLaneFollowPath(way_points, goal_orientation, LL_LANE_CENTER)) {
	x_dist = std::min(1. * way_points->back().x, 0.4);
    }
    goal_global_.SetManeuverGoal(*pose_curr_, x_dist, sin(*goal_orientation)*x_dist,
			        *goal_orientation, behaviour_curr_->behaviour_id);
    return true;
}

bool PathPlanner::getBackwardsLanePath(tPath* way_points,
                                     float* goal_orientation) const {
    /* This method uses lanes given by rear cam; only difference to normal
       lane follow mode is that lane polynomial is given in POV of rear cam;
       subtract car width in x-direction and flip sign of y and we're done! */
    bool res = getLaneFollowPath(way_points, goal_orientation, LL_LANE_CENTER);
    if (!res) return false;
    for (int i = 0; i < way_points->size(); ++i) {
        (*way_points)[i].x -= CAR_LENGTH_;
        (*way_points)[i].y = -(*way_points)[i].y;
    }
    //  goal_orientation doesn't change
    return true;
}

bool PathPlanner::getLaneSwitchPath(tPath* way_points, float* goal_orientation) {
    bool is_goal_set = goal_global_.isGoalSet(behaviour_curr_->behaviour_id);
    if (is_goal_set) {
      getGoalAlignedLanePath(way_points, goal_orientation);
      return true;
    }
    lane_to_lock lane_goal;
    if (behaviour_curr_->behaviour_id == CHANGELANE_TO_LEFT) {
        lane_goal = LL_LANE_LEFT;
    } else {
        lane_goal = LL_LANE_RIGHT;
    }
    // if entering this method the first time and no global
    // goal set, just do a a fixed lanechange
    /*if (!getLaneFollowPath(way_points, goal_orientation, lane_goal) &&
         !goal_global_.isGoalSet(behaviour_curr_->behaviour_id)) { // not sure if we should even do this..
      switch (lane_goal) {
        way_points->clear();
        case LL_LANE_CENTER:
          return false;
        case LL_LANE_LEFT:
          way_points->push_back(getTPoint(0., 0.));
          way_points->push_back(getTPoint(0.6, 1. * LANEWIDTH_));
          break;
        case LL_LANE_RIGHT:
          way_points->push_back(getTPoint(0., 0.));
          way_points->push_back(getTPoint(0.6, -1. * LANEWIDTH_));
          break;
      }
      *goal_orientation = 0.;
    }*/
    if (getLaneFollowPath(way_points, goal_orientation, lane_goal)) { // not sure if we should even do this..
	 
         goal_global_.SetManeuverGoal(*pose_curr_, way_points->back().x,
                                 way_points->back().y, *goal_orientation,
                                 behaviour_curr_->behaviour_id);
    }
    return true;
}

bool PathPlanner::getGoalAlignedLanePath(tPath* way_points, float* goal_orientation) {
    const float DY_GOAL_LANE_MARGIN = 0.*LANEWIDTH_; // 0.1 with orthogonal shift; 3.11
    if (!lane_vert_vec_ptr_) return false;
    std::vector<Lane>& lane_vert_vec = *lane_vert_vec_ptr_;
    if (!goal_global_.isGoalSet(behaviour_curr_->behaviour_id) ||
         lane_vert_vec.empty()) return false;
    tPose goal_local = goal_global_.getGoalInLocalFrame(*pose_curr_);
    /* now look for which lanes are in range and sample from them;
       lanes are already ordered (from right to left) */
    Lane* lane_left = NULL;
    Lane* lane_right = NULL;
    for (int i = 0; i < lane_vert_vec.size(); ++i) {
        Lane* lane_tmp = &lane_vert_vec[i];
        float phi_lane = atan(getDerivAtX(goal_local.x, lane_tmp->coeff));
        bool is_parallel = goal_global_.isLocalHeadingParallelToGoal(
            *pose_curr_, phi_lane, 20.*PI/180.);
        if (!is_parallel) {continue;}
        float y_at_x_lane = getValAtX(goal_local.x, lane_tmp->coeff);
        float y_deviation = fabs(y_at_x_lane - goal_local.y);
        float dy_left_lane = 0.;
        float dy_right_lane = 0.;
        float sum_lanes = 0;
        /* goal too close to lane; unclear assignment
           if one lane further right, take this one */
        if ((y_deviation < DY_GOAL_LANE_MARGIN) && (i > 0)) {
          lane_left = &lane_vert_vec[i];
          lane_right = &lane_vert_vec[i-1];
          printf("Lane was too close to goal; take this one and lane right\n");
          break;
          //  todo: shift goal point a little
        /* check if goal is centered */
        } else if (isInRange(goal_local.y, y_at_x_lane - 0.9*LANEWIDTH_,
                      y_at_x_lane, false)) {
            lane_left = lane_tmp;  // is goal left from a right lane?
            dy_left_lane = y_deviation - 0.5*LANEWIDTH_;
            sum_lanes++;
            printf("Found lane right to ManeuverGoal with dy %f\n", goal_local.y-y_at_x_lane);
        } else if (isInRange(goal_local.y, y_at_x_lane,
                             y_at_x_lane + 0.9*LANEWIDTH_, false)) {
            lane_right = lane_tmp;  // is goal right from a left lane?
            dy_right_lane = 0.5*LANEWIDTH_-y_deviation;
            sum_lanes++;
            printf("Found lane left to ManeuverGoal with dy %f\n", y_at_x_lane-goal_local.y);
        } else {printf("lane not in range\n");}
        //if (sum_lanes > 0)
        //  goal_global_.orthogonalGoalShift(0.1*(dy_left_lane+dy_right_lane)/sum_lanes); // move to center
    }
    if (lane_left == NULL && lane_right == NULL) {
      return false;
    }
    sampleFromLanes(lane_left, lane_right,
                    way_points, goal_orientation);
    return true;
}

bool PathPlanner::getLaneFollowPath(tPath* way_points, float* goal_orientation,
                                    lane_to_lock lane_goal) const {
    // todo: add method for automatic lanechange
    // static float last_goal_angle = 1000.0;
    // static int jump_cnt = 0;

    /* This method considers car pos. relative to detected lane markings,
       determines
       which markings should be used as guidance and calculates 5 way_points up
       to 2 meters ahead, using information on marking quality
       (sample size; fitting error also possible) */
    const float MIN_DIST_VALID_LANE = 0.0;
    static float last_angle = 0.;
    static int jump_cnt = 0.;

    if (!lane_vert_vec_ptr_) return false;
    std::vector<Lane>& lane_vert_vec = *lane_vert_vec_ptr_;
    float closest_left_dist = 1e9, closest_right_dist = -1e9;
    int closest_left_idx = -1, closest_right_idx = -1;
    //  Figure out closest left and right lane marking
    // printf("GLPl: Lane with dist: ");

    for (int i = 0; i < lane_vert_vec.size(); ++i) {
        // printf("%f | ", lane_vert_vec[i].dist);
        if (lane_vert_vec[i].dist > 0 &&
            lane_vert_vec[i].dist < closest_left_dist &&
            fabs(lane_vert_vec[i].dist) > MIN_DIST_VALID_LANE) { //MIN_DIST_VALID_LANE

            closest_left_dist = lane_vert_vec[i].dist;
            closest_left_idx = i;
        }
        if (lane_vert_vec[i].dist < 0 &&
            lane_vert_vec[i].dist > closest_right_dist &&
            fabs(lane_vert_vec[i].dist) > MIN_DIST_VALID_LANE) { //MIN_DIST_VALID_LANE
            closest_right_dist = lane_vert_vec[i].dist;
            closest_right_idx = i;
        }
    }
    //  Define driving corridor between left and right lane marking if possible,
    //  otherwise use single lane marking as guideline
    Lane* lane_left = NULL;
    Lane* lane_right = NULL;

    switch (lane_goal) {
        // std::cout << "entered lane follow path switch" << std::endl;
        case LL_LANE_LEFT:
            // std::cout << "case: LL_LANE_LEFT" << std::endl;
            if (closest_left_idx != -1) {
                lane_right = &lane_vert_vec[closest_left_idx];
                if (closest_left_idx + 1 < lane_vert_vec.size())
                    lane_left = &lane_vert_vec[closest_left_idx + 1];
            }
            if (lane_left &&
                !isInRange(lane_left->dist, LANEWIDTH_, 2. * LANEWIDTH_, true))
                lane_left = NULL;
            if (lane_right &&
                !isInRange(lane_right->dist, 0., LANEWIDTH_, true))
                lane_right = NULL;
            // if (lane_left) printf("lane left dist is %f\n", lane_left->dist);
            // if (lane_right) printf("lane right dist is %f\n",
            // lane_right->dist);
            break;
        case LL_LANE_CENTER:
            // std::cout << "case: LL_LANE_CENTER" << std::endl;
            if (closest_left_idx != -1) {
                lane_left = &lane_vert_vec[closest_left_idx];
            }
            if (closest_right_idx != -1) {
                lane_right = &lane_vert_vec[closest_right_idx];
            }
            if (lane_left && !isInRange(lane_left->dist, 0, LANEWIDTH_, false)) {
                //  printf("set left lane to 0, dist is %f\n", lane_left->dist);
                lane_left = NULL;
            }
            if (lane_right &&
                !isInRange(lane_right->dist, 0., LANEWIDTH_, true)) {
                lane_right = NULL;
                //  printf("set right lane to NULL\n");
            }
            // if (lane_left && lane_left && fabs(lane_left->dist) < 0.1) {
            //     lane_left == NULL;
            //     printf("LaneFollowPath ignored left lane that was too close\n");
            // }
            break;
        case LL_LANE_RIGHT:
            // std::cout << "case: LL_LANE_RIGHT" << std::endl;
            if (closest_right_idx != -1) {
                lane_left = &lane_vert_vec[closest_right_idx];
                if (closest_right_idx > 0) {
                    lane_right = &lane_vert_vec[closest_right_idx - 1];
                }
            }
            if (lane_left && !isInRange(lane_left->dist, 0, LANEWIDTH_, true))
                lane_left = NULL;
            if (lane_right &&
                !isInRange(lane_right->dist, LANEWIDTH_, 2 * LANEWIDTH_, true))
                lane_right = NULL;
            break;
    }

    if (lane_left == NULL && lane_right == NULL) {
      printf("PathPlanner l. 708 Couldn't compute way_points from lanes. \n");
      return false;
    }
    /*  Sample lane and compute goal orientation tangential to lane; */

    sampleFromLanes(lane_left, lane_right,
                    way_points, goal_orientation);

    // todo: if lane center, check if plausible radius
   /* if (lane_goal == LL_LANE_CENTER && way_points->size() > 0) {
            float radius =
        2*(pow(way_points->back().x, 2.0) + pow(way_points->back().y, 2.0))/way_points->back().y;
        if (fabs(radius) < 0.8) {
            way_points->clear();
            printf("Discard lane path, curvature unreasonable!\n");
            return false;
        }
    }

    if (*goal_orientation < -20.*PI/180.) {
        if ((*goal_orientation - last_angle) < -20.*PI/180.) { // more than 20deg jump to right
            jump_cnt++;
        } else {
            jump_cnt = 0.;
            last_angle = *goal_orientation;
        }
        // eval. number of counts
        if (jump_cnt > 20) { // apparently the jump tells the truth
            last_angle = *goal_orientation;
            jump_cnt = 0.;
        }
        if (jump_cnt > 0.) {
            way_points->clear();
            printf("Detected and ignored jumps in lane\n");
        }
    }*/
    return true;
}

void PathPlanner::sampleFromLanes(Lane* lane_left, Lane* lane_right,
                                  tPath* way_points, float* goal_orientation) const {
    //const float X_MAX_LANE = 1.0;
    const float X_INCREMENT = 0.2;
    const float MAX_LENGTH = 1.0;

    //int n_way_points = X_MAX_LANE / X_INCREMENT;
    way_points->clear();
    way_points->push_back(getTPoint(0., 0.));
    tPoint res;
    tPoint prev_point;
    prev_point.x = 0; prev_point.y = 0;
    float total_dist = 0.;
    float x_curr = 0;

    while (total_dist < MAX_LENGTH) {
        x_curr += X_INCREMENT;
        if (getLaneCenterPoint(x_curr, &res, lane_left, lane_right) &&
            res.x >= 0.7*X_INCREMENT) {
            way_points->push_back(res);
            total_dist += sqrt(pow(res.x-prev_point.x, 2) +
                               pow(res.y-prev_point.y, 2));
            prev_point = res;
        }
    }
    // for (int i = 1; i <= n_way_points; ++i) {
    //     float x_dist = X_INCREMENT * i;
    //     if (getLaneCenterPoint(x_dist, &res, lane_left, lane_right)) {
    //         //  x values of lane centers must be increasing! and big enough
    //         if ( ((*way_points)[i - 1].x < res.x) &&
    //              (res.x >= 0.7*X_INCREMENT)) {
    //             way_points->push_back(res);
    //         }
    //     }
    // }
    //  Compute goal heading := tangent to last waypoint
    if (lane_left != NULL && lane_right != NULL) {
        *goal_orientation =
            0.5 *
            (atan(getDerivAtX(x_curr, lane_left->coeff)) +
             atan(getDerivAtX(x_curr, lane_right->coeff)));
    } else if (lane_left != NULL) {
        *goal_orientation =
            atan(getDerivAtX(x_curr, lane_left->coeff));
    } else if (lane_right != NULL) {
        *goal_orientation =
            atan(getDerivAtX(x_curr, lane_right->coeff));
    }
}

bool PathPlanner::isInRange(float val, float min, float max,
                            bool use_abs) const {
    if (use_abs) val = fabs(val);
    return (val > min && val < max);
}

bool PathPlanner::getLaneCenterPoint(float x_dist, tPoint* center_point,
                                     Lane* lane_left, Lane* lane_right) const {
    /* This method evaluates one or two lane objects and returns waypoint for
     * given x-distance */

    float car_center_dist = 0.5 * LANEWIDTH_;
    float normal_angle;
    float lane_y_at_x;
    if ((lane_left == NULL) && (lane_right == NULL)) return false;
    /***** single lane marking given *****/
    if ((lane_left == NULL) ^ (lane_right == NULL)) {
        Lane* lane_tmp;
        if (lane_left != NULL)
            lane_tmp = lane_left;
        else
            lane_tmp = lane_right;
        normal_angle = atan(getDerivAtX(x_dist, lane_tmp->coeff));
        if (lane_tmp == lane_left) {  //  marking left to car
            normal_angle += -0.5 * PI;
            car_center_dist = 0.5 * LANEWIDTH_;
        } else {  //  marking right to car
            normal_angle += 0.5 * PI;
            car_center_dist = 0.5 * LANEWIDTH_;
        }
        lane_y_at_x = getValAtX(x_dist, lane_tmp->coeff);
        center_point->x = x_dist + car_center_dist * cos(normal_angle);
        center_point->y = lane_y_at_x + car_center_dist * sin(normal_angle);
        //  printf(" single lane orientation, center: x = %f, y = %f\n",
        //  center_point->x, center_point->y);
        return true;
    }

    /***** two lane markings given: ******/
    /*
      Lane *lane_left, *lane_right;
      if (lane1->dist > lane2->dist) {
        lane_left = lane1;
        lane_right = lane2;
      } else {
        lane_left = lane2;
        lane_right = lane1;
      } */
    float normal_angle_l =
        atan(getDerivAtX(x_dist, lane_left->coeff)) - 0.5 * PI;
    //  todo:
    // float normal_angle_r = atan(getDerivAtX(x_dist, lane_right->coeff)) + 0.5
    // * PI;
    // tPoint center_l;
    // tPoint center_r;
    // printf("norm angle %f\n", normal_angle);
    // float lane_dist = approxLaneDistance(lane_left, lane_right, x_dist);
    // printf("lane dist %f\n", lane_dist);
    //  start from center, and remaining distance is split/added according to
    //  no.
    //  of samples
    float dist_from_left_lane = 0.4 * LANEWIDTH_; /*std::max(
        (lane_dist - LANEWIDTH_) * (lane_left->no_of_samples) /
            (lane_left->no_of_samples + lane_right->no_of_samples) +
          0.5 * lane_dist, 0.5 * CAR_WIDTH_);*/
    center_point->x = x_dist + dist_from_left_lane * cos(normal_angle_l);
    center_point->y = getValAtX(x_dist, lane_left->coeff) +
                      dist_from_left_lane * sin(normal_angle_l);

    //  center_l.x = x_dist + car_center_dist * cos(normal_angle_l);
    //  center_l.y = getValAtX(x_dist, lane_left->coeff) +
    //               car_center_dist * sin(normal_angle_l);
    //  center_r.x = x_dist + car_center_dist * cos(normal_angle_r);
    //  center_r.y = getValAtX(x_dist, lane_right->coeff) +
    //               car_center_dist * sin(normal_angle_r);

    //  center_point->x = 0.5*(center_l.x + center_r.x);
    //  center_point->y = 0.5*(center_l.y + center_r.y);
    //  printf("2 lanes, center: x = %f, y = %f\n", res.x, res.y);
    return true;
}

// this method is currently unused
float PathPlanner::approxLaneDistance(Lane* lane1, Lane* lane2,
                                      float x_eval) const {
    /* This method calculates the approximate distance between two lanes */
    //  is lane1 left to lane2?
    if (lane1 == NULL || lane2 == NULL) {
        printf("approxLaneDistance failed.\n");
        return LANEWIDTH_;
    }
    float normal_angle1, normal_angle2;
    if (lane1->dist > lane2->dist) {
        normal_angle1 = -0.5 * PI;
        normal_angle2 = 0.5 * PI;
    } else {
        normal_angle1 = 0.5 * PI;
        normal_angle2 = -0.5 * PI;
    }
    normal_angle1 += getDerivAtX(x_eval, lane1->coeff);
    normal_angle2 += getDerivAtX(x_eval, lane2->coeff);
    //  -- Initial guess on distance
    float y0 = getValAtX(x_eval, lane1->coeff);
    //  go expected distance towards lane2, assuming that lane markings
    //  have constant distance
    float x_expected = x_eval + LANEWIDTH_ * cos(normal_angle1);
    float y1_at_x_exp = getValAtX(x_expected, lane2->coeff);
    //  -- Gradient descent with fixed step number
    float dx = 0.1;
    int max_steps = 5;
    int step = 0;

    float dist0 = (pow(x_expected - x_eval, 2.0) + pow(y0 - y1_at_x_exp, 2.0));
    //  return dist0;
    float dist_dx = (pow(x_expected - x_eval + dx, 2.0) +
                     pow(y0 - getValAtX(x_expected + dx, lane2->coeff), 2.0));
    if (dist_dx > dist0) {
        dx = dist0 > LANEWIDTH_ ? -dx : dx;
    } else {
        dx = dist0 > LANEWIDTH_ ? dx : -dx;
    }
    float min_dist = dist0;
    while (step < max_steps) {
        dist_dx =
            (pow(x_expected - x_eval + step * dx, 2.0) +
             pow(y0 - getValAtX(x_expected + step * dx, lane2->coeff), 2.0));
        if (dist_dx <= min_dist) min_dist = dist_dx;
        //  else
        //  break;
        step++;
    }
    min_dist = sqrt(min_dist);
    // printf("dist0 is %f\n", sqrt(dist0));
    return min_dist;
}

float PathPlanner::getValAtX(float x, const std::vector<float>& coeff) const {
    float res = 0;
    for (int i = 0; i < coeff.size(); ++i) {
        res += coeff[i] * pow(x, i);
    }
    return res;
}

float PathPlanner::getDerivAtX(float x, const std::vector<float>& coeff) const {
    float res = 0;
    for (int i = 1; i < coeff.size(); ++i) {
        res += coeff[i] * pow(x, i - 1);
    }
    return res;
}


/* ###########################################################################
   In this section all collision avoidance methods are implemented
   ##########################################################################*/

/* This routine plans a path around a given obstacle ID;
   Assumption: Car always centered on right lane */

// if path available, take this, otherwise create box
// get global polygon points
// if offset set, shift these
// check collision

tSptrMapElement PathPlanner::createCollisionBox(float x_center, float y_shift) const {
  const float config_collision_box_local_center_x_ = x_center;
  const float config_collision_box_local_center_y_ = y_shift;
  const float config_collision_box_size_half_x_ = 0.5;
  const float config_collision_box_size_half_y_ = 0.15;

  tSptrMapElement map_collision_check_box = MapHelper::CreatBox(
      DEBUG_POLYGON,
      config_collision_box_local_center_x_,
      config_collision_box_local_center_y_,
      config_collision_box_size_half_x_,
      config_collision_box_size_half_y_,
      current_time_);
    tMapCarPosition car_pos;
    car_pos.x = pose_curr_->x;
    car_pos.y = pose_curr_->y;
    car_pos.heading = pose_curr_->heading;
    car_pos.update_time_microseconds = current_time_;
   map_collision_check_box->LocalToGlobal(car_pos);
  return map_collision_check_box;
}


tSptrMapElement PathPlanner::getPlannerCollisionPath(lane_to_lock lane_offset) const {
  // tTimeStamp start_time_micro_s = _clock->GetStreamTime();


  // create collision box in global frame based on current or virtual path
  tSptrMapElement map_collision_check_box = map_->GetElement(path_id_);
  if (!map_collision_check_box) {
      printf("PP: ERROR: map_collision_check_box empty (no path found)\n");
      map_collision_check_box = createCollisionBox( 0.5, 0.);
  }
  tMapPolygon collbox_polygon = map_collision_check_box->GetGlobalPoly();

  if (collbox_polygon.outer().empty()) {printf("PP: collbox_polygon is empty\n");}
  // optional: shift this to left/right (e.g. for overtake)
  float y_offset_local;
  switch (lane_offset) {
    case LL_LANE_LEFT:  y_offset_local = 0.5*LANEWIDTH_; break;
    case LL_LANE_RIGHT:  y_offset_local = -LANEWIDTH_; break;
    default: y_offset_local = 0.; break;
  }

  if (y_offset_local != 0.) {
      std::cout << "Try collision check with shifted coll. box" << std::endl;
      // shift polygon in local y_direction
      float dx_global = y_offset_local * cos(pose_curr_->heading + 0.5*PI);
      float dy_global = y_offset_local * sin(pose_curr_->heading + 0.5*PI);

      std::cout << "dx_global: " << dx_global << std::endl;
      std::cout << "dy_global: "<< dy_global << std::endl;
      // float dx_global = pose_curr_->x + y_offset_local*cos(pose_curr_->heading + 0.5*PI);
      // float dy_global = pose_curr_->y + y_offset_local*sin(pose_curr_->heading + 0.5*PI);
      tMapPolygon collbox_polygon_shifted;
      MapHelper::PolyTranslate(collbox_polygon, collbox_polygon_shifted,  // maybe different collbox polygons
                               dx_global, dy_global);

      std::cout << "collbox_polygon[0] x: "<<  collbox_polygon.outer().front().get<0>() << std::endl;
      std::cout << "collbox_polygon_shifted[0] x: " << collbox_polygon_shifted.outer().front().get<0>() << std::endl;
      tSptrMapElement map_collision_check_box_tmp(new MapElement(DEBUG_POLYGON,
                                      collbox_polygon_shifted.outer(),
                                      current_time_));
      map_collision_check_box = map_collision_check_box_tmp;


      //printf("Shifted polygon has %i points and id: %d\n",
      //  collbox_polygon_shifted.outer().size(), map_collision_check_box->GetID());
  }
  return map_collision_check_box;
}


tMapID PathPlanner::getClosestCollidingObjectID(tSptrMapElement &path) const {
    std::cout << "Entering getClosestCollidingObjectID \n" << std::endl;
    tMapID ret_id = MAP_DEFAULT_ID;
    static const std::vector<MapElementType> exclude_types =
        boost::assign::list_of(DEBUG_POINT)(DEBUG_POLYGON)(STREET_MARKER_LANE)(
            STREET_MARKER_LANE)(STREET_MARKER_STOP_LINE)(
            STREET_MARKER_CROSSING_T)(STREET_MARKER_CROSSING_X)(
            STREET_MARKER_ZEBRA)(STREET_PARKING_SPOT)(LM_STREET_LANE)(
            PLANNER_PATH)(LM_STREET_PARKING_SPOT);

    std::vector<tSptrMapElement> collision_els;
    // if debug elements added from step before a collision is detected
    //  ->fuse el to map before collision check or exclude_types
    bool is_collision = map_->CheckElementsCollision(
                      path,
                      collision_els,
                      2.,  // max range
                      -1,  // return all
                      &exclude_types);
                      tMapData distances_min = -1;

    if (LOG_DEBUG_){
        path->user_color_ui_ = is_collision ? "red" : "green";

        //TODO ONLY ADD IN DEBUG TO MAP
        if (path->GetID() == MAP_DEFAULT_ID){
          //add el to map if it was not added before
            path->EnableTimeOfLife(1. * 1e6);
            map_->AddElement(path, current_time_);

        }
    }
    BOOST_FOREACH(const tSptrMapElement& el, collision_els) {
        // set the return id to in element with the min distance to the car
        if (distances_min == -1 || el->GetDistanceToCar() < distances_min) {
            ret_id = el->GetID();
            distances_min = el->GetDistanceToCar();
        }
    }
    return ret_id;
}

tMapID PathPlanner::getClosestCollidingIDWithPlannerPath(lane_to_lock lane_offset) const {
  tSptrMapElement path = getPlannerCollisionPath(lane_offset);
  return getClosestCollidingObjectID(path);
}

tMapID PathPlanner::getClosestCollidingIDWithBox(float x_shift, float y_shift) const {
  tSptrMapElement box = createCollisionBox(x_shift,y_shift);
  return getClosestCollidingObjectID(box);
}


bool PathPlanner::getGoAroundPath(tPath* way_points,
                                  float* goal_orientation) const {
    /*
   General heuristic:
   - determine collision point and driving direction
   - go perpendicular with fixed distance increment (e.g. car_width)
   - pick either left or right point, depending on driving behaviour
   - check if path feasible by calling XminGoAround()
   - if yes: set new waypoint, and sample goal point&direction from
     old path (e.g. at twice the x-val of collision point)
   - maybe: shift goal point forward in goal direction
   */
    std::cout << "Entering getGoAroundPath \n" << std::endl;
    tMapID el_id = getClosestCollidingIDWithPlannerPath();
    tSptrMapElement el = map_->GetElement(el_id);
    if (!el) {
        printf(
            "WARNING: GP: getGoAroundPath() failed, no obstacle found with"
            " the given ID.\n");
        return false;
    }
    printf("getGoAroundPath element: %s", el->ToString().c_str());
    //  Get center from obstacle
    float obst_local_angle = el->GetOrientationAngleToCar();
    tPoint obstacle_center =
        getTPoint(cos(obst_local_angle) * (el->GetDistanceToCar()),
                  sin(obst_local_angle) * (el->GetDistanceToCar()));

    /* Find 90deg point on current path segment towards obst. center */
    //  tPoint perp_point;
    //  if (!gp_gen_.getPerpendicularPoint(pose_curr, obstacle_center,
    //  &perp_point))
    //    return false;
    //  std::cout << "getGoAroundPath: gotPerp point \n" << std::endl;
    bool go_left = true;
    float angle_direction = go_left ? 1. : -1.;
    //  Move on line from obstacle, until point is not in polygon
    //  float dy = (perp_point.y - obstacle_center.y);
    //  float dx = (perp_point.x - obstacle_center.x);
    //  float slope = dy/dx;
    //  //  how large dx to move 1m on line?
    //  float unitlength_x_inc = sqrt(pow(dx, 2.)+pow(dy, 2.))/dx;
    //  float length_inc = 0.01;
    tPoint border_pnt;
    //  float x_step;
    //bool overlap = false;
    /* Get Get obstacle polygon, transform into car frame and find
     * left/rightmost point */
    tMapPolygon obstacle_polygon_global = el->GetGlobalPoly();
    tMapPolygon obstacle_polygon_local;
    tMapPoint obstacle_center_global = el->GetGlobalPolyCenter();
    tMapData dx_global = pose_curr_->x - obstacle_center_global.get<0>();
    tMapData dy_global = pose_curr_->y - obstacle_center_global.get<1>();
    tMapData dphi_global = pose_curr_->heading - el->GetGlobalOrientation();
    tMapPoint max_left_pnt(0, -1e9);
    MapHelper::PolyTranslateRotate(obstacle_polygon_global,
                                   obstacle_polygon_local, dx_global, dy_global,
                                   dphi_global);

    BOOST_FOREACH(const tMapPoint& p, obstacle_polygon_local.outer()) {
        if (p.get<1>() > max_left_pnt.get<1>()) {
            max_left_pnt = p;  //  todo felix: don't copy around..
        }
    }
    border_pnt.x = max_left_pnt.get<0>();
    border_pnt.y = max_left_pnt.get<1>();
    float dist_obst_center_border =
        sqrt(pow(border_pnt.x - obstacle_center.x, 2.) +
             pow(border_pnt.y - obstacle_center.y, 2.));
    //  for (int i = 0; i < 20; ++i) {
    //    x_step = i*length_inc*unitlength_x_inc;
    //    border_pnt.x = obstacle_center.x + x_step;
    //    border_pnt.y = obstacle_center.y + angle_direction*fabs(slope*x_step);
    //    if (!(el->GlobalPolyDisjoint(tMapPoint(border_pnt.x, border_pnt.y))))
    //      break;
    //  }
    std::cout << "getGoAroundPath: got border pnt\n" << std::endl;
    /* From obst. center: Avoidance way_points on circle around obst. center
       with (distance obst. center to border) + (half car width) */
    bool go_around_possible =
        minGoAroundX(border_pnt.y, go_left) < border_pnt.x;
    std::cout << "getGoAroundPath: return from minGoAroundX \n" << std::endl;
    float avoidance_dist = 0.6 * CAR_WIDTH_ + dist_obst_center_border;
    //  angles from POV of obst. center
    float wp_angles[] = {0.7 * PI, 0.5 * PI};  // , 0.3*PI
    uint wp_angles_size = sizeof(wp_angles) / sizeof(wp_angles[0]);
    way_points->clear();
    way_points->push_back(getTPoint(0, 0));
    tPoint res;
    for (uint i = 0; i < wp_angles_size; ++i) {
        res.x = obstacle_center.x +
                avoidance_dist *
                    cos(angle_direction * wp_angles[i]);  // +obst_angle_local?
        res.y = obstacle_center.y +
                avoidance_dist * sin(angle_direction * wp_angles[i]);
        way_points->push_back(res);
    }
    *goal_orientation = wp_angles[wp_angles_size - 1] - 0.5 * PI;
    std::cout << "getGoAroundPath: got goal_orientation\n" << std::endl;
    if (LOG_DEBUG_) {
        printf("Collision: Obst. center at %f %f\n", obstacle_center.x,
               obstacle_center.y);
        //  printf("Collision: In curr. frame, current path is perpendicular at
        //  point %f %f\n",
        //         perp_point.x, perp_point.y);
        printf("Collision: Left border of obst. at %f %f \n", border_pnt.x,
               border_pnt.y);
        printf("Going around obstacle is %s possible!\n",
               go_around_possible ? "" : "not");
        printf("Collision: Avoidance dist from obst. is %f\n", avoidance_dist);
    }
    return true;
}

/* Given a y value of a collision point and desired turn type, this method
   computed the required x distance between collision point and car needed
   to safely pass this object. If distance too short, car needs to go back */
float PathPlanner::minGoAroundX(float y, bool goLeft) const {
    /* conditions for doing left/right move:
       l = R_min + 0.5*CARWIDTH
       l * cos(phi) = +-R_MIN - y
       l * sin(phi) = x = result */
    float l = RADIUS_MIN_ + 0.6 * CAR_WIDTH_;  // 0.6 = 0.5 + margin
    int sig = goLeft ? 1 : -1;
    //
    if (fabs((sig * RADIUS_MIN_ - y) / l) >= 1) return 0;

    //  check min distance (x) so that car touches edge on right side:
    float phi = acos((sig * RADIUS_MIN_ - y) / l);
    return l * sin(phi);
}


/* ***************************************************************************
   In this section misc. helper methods are implemented
   ***************************************************************************/

tPoint PathPlanner::getTPoint(float x, float y) const {
    tPoint res;
    res.x = x;
    res.y = y;
    return res;
}

tPath& PathPlanner::GetPathRef() { return current_path_; }

/*This method is only used since _clock not available in main filter class*/
void PathPlanner::updateTimestamp(tTimeStamp timestamp) {
    current_time_ = timestamp;
}

void PathPlanner::DrawCurrentPath(float lifetime_sec) {

    const float path_width_min = 0.4*CAR_WIDTH_;
    const float path_width_max = 0.1*CAR_WIDTH_;
    const float MAX_LENGTH = 1.5;


    tPath& current_path = GetPathRef();
    if (current_path.size() < 1) return;

    std::vector<tMapPoint> temp;
    std::vector<float> tangent_vec;
    tangent_vec.reserve(current_path.size());
    std::vector<float> length_vec;
    length_vec.reserve(current_path.size());

    //  calc. tangents and fill left part of the polygon, which must be
    //  given in clockwise order
    int i_max = 0;
    int i_dec = 1;
    float path_length = 0.;
    //  left side of the polygon; calc. tangents only once here
    for (int i = 0;
         i < current_path.size() - 1 && path_length < MAX_LENGTH;
         i += i_dec) {
        i_max = i;
        float dx = current_path[i + 1].x - current_path[i].x;
        float dy = current_path[i + 1].y - current_path[i].y;
        if (dx == 0.0) return;
        path_length += sqrt(pow(dx,2)+pow(dy,2));
	length_vec.push_back(path_length);
        float tangent = atan2(dy, dx);
       // float path_width = path_width_min + path_length*
        //                  (path_width_max-path_width_min)/MAX_LENGTH;
        float a = path_width_min/pow(MAX_LENGTH,2);
        float b = -2*a*MAX_LENGTH;
        float path_width = a*pow(path_length,2) + b*path_length + path_width_min;
        tangent_vec.push_back(tangent);
        temp.push_back(
            tMapPoint(current_path[i].x + path_width * cos(tangent + 0.5 * PI),
                      current_path[i].y + path_width * sin(tangent + 0.5 * PI)));
    }
    //  now fill right side of polygon; use precomputed tangents
    for (int i = i_max; i >= 0; i -= i_dec) {
        float tangent = tangent_vec[i / i_dec];
        float path_length = length_vec[i / i_dec];
          //float path_width = path_width_min + path_length*
          //                   (path_width_max-path_width_min)/MAX_LENGTH;
        float a = path_width_min/pow(MAX_LENGTH,2);
        float b = -2*a*MAX_LENGTH;
        float path_width = a*pow(path_length,2) + b*path_length + path_width_min;

        temp.push_back(
            tMapPoint(current_path[i].x + path_width * cos(tangent - 0.5 * PI),
                      current_path[i].y + path_width * sin(tangent - 0.5 * PI)));
    }
    // add to map
    tSptrMapElement path_poly(
        new MapElement(PLANNER_PATH, temp, current_time_));
    path_poly->user_color_ui_ = MAP_ELEMENT_COLOR_DEBUG;
    // no EnableTimeOfLife ->replace every found path in the map
    map_->AddFuseElement(path_poly, 999, -1, current_time_, MAP_FUSE_REPLACE);
    path_id_ = path_poly->GetID();
}

void PathPlanner::DrawCurrentway_points(float lifetime_sec,
                                       const tPath& way_points) {
    frAIburg::map::GlobalMap* map_ = frAIburg::map::getInstance();
    if (!map_) return;
    std::vector<tMapPoint> temp;
    for (int i = 0; i < way_points.size(); ++i) {
        temp.push_back(tMapPoint(way_points[i].x, way_points[i].y));
    }
    tSptrMapElement path_poly(new MapElement(DEBUG_POINT, temp, current_time_));
    path_poly->user_color_ui_ = MAP_ELEMENT_COLOR_DEBUG;
    path_poly->EnableTimeOfLife(lifetime_sec * 1e6);
    map_->AddElement(path_poly, current_time_);
    //  map_->AddFuseElement(path_poly,
    //    999,// no EnableTimeOfLife ->replace every found path in the map
    //     -1, current_time_, MAP_FUSE_REPLACE);
}

void PathPlanner::setDebugMode(bool debug_active) { LOG_DEBUG_ = debug_active; }

float PathPlanner::getArcLength(const tPoint& goal_pnt,
                                float goal_heading) const {
    float sin_heading = sin(goal_heading);
    if (fabs(sin_heading) < 1e-5) return fabs(goal_pnt.x);
    return fabs(goal_heading / sin_heading * goal_pnt.x);
}

/* if project_on_x true, output will be in range (0..dist) */
float PathPlanner::getStopDistToObject(int object_id, bool project_on_x,
                                       bool zero_cutoff) const {
    //  get element distance from map..
    //  get element type; stop at stopline, stop sign;
    /* Projecting on x has the advantage, that the distance is negative once the
       object
       is behind us. This does not work when we want to go to e.g. a parking
       spot,
       in which case we take the real distance. */
    tSptrMapElement el = map_->GetElement(static_cast<tMapID>(object_id));
    if (!el) {
        printf(
            "WARNING: PathPlanner: getStopDistToObject() failed. Return 0.5m "
            "[failsafe] FIX THE CAUSE!!!\n");
        return 0.5;
    }
    //  Distances in map always w.r.t. to object center; add offset:
    float dist_offset = 0.;
    MapElementType el_type = el->GetType();
    switch (el_type) {
        case STREET_MARKER_CROSSING_T:
            dist_offset = -LANEWIDTH_ - STOPLINE_OFFSET_;
            break;
        case STREET_MARKER_CROSSING_X:
            dist_offset = -LANEWIDTH_ - STOPLINE_OFFSET_;
            break;
        case STREET_MARKER_ZEBRA:
            dist_offset = -0.5;
            break;
        case PEDESTRIAN_ADULT:
        case PEDESTRIAN_CHILD:
        case PEDESTRIAN_REAL:
            dist_offset = -0.4;
            break;
        default:
            dist_offset = 0.0;
            break;
    }

    float dist = 0.5;
    getDistanceToObject(&dist, object_id, project_on_x);
    dist += dist_offset;
    if (zero_cutoff)
        return std::max(0.0, 1. * dist);
    else
        return dist;
}

bool PathPlanner::getDistanceToObject(float* dist, int object_id,
                                      bool project_on_x) const {
    tSptrMapElement el = map_->GetElement(static_cast<tMapID>(object_id));
    if (!el) {
        printf(
            "WARNING: PathPlanner: getDistanceToObject() failed \n");
        return false;
    }
    if (project_on_x) {
        *dist = cos(el->GetOrientationAngleToCar()) * (el->GetDistanceToCar());
    } else {
        *dist = el->GetDistanceToCar();
    }
    return true;
}

void PathPlanner::setMap(frAIburg::map::GlobalMap* map) { map_ = map; }

void PathPlanner::updateSpeed(float v_current) {
  v_current_ = v_current;
}

void PathPlanner::updateStatePointer(const tBehaviour& behaviour_current,
                                     const tPose& pose_global) {
  pose_curr_ = &pose_global;
  behaviour_curr_ = &behaviour_current;
}

void PathPlanner::updateLaneVecPointer(std::vector<Lane>* lane_vert_vec_ptr) {
  lane_vert_vec_ptr_ = lane_vert_vec_ptr;
}
