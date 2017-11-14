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


#ifndef AADCUSER_FRAIBURG_PLANNER_PATHPLANNER_H_
#define AADCUSER_FRAIBURG_PLANNER_PATHPLANNER_H_


#include <iostream>
#include <vector>
#include "GoalPointGenerator.h"
#include "Spline.h"
#include "ManeuverGoal.h"
#include "nonpin_types.h"
#include "stdafx.h"
#include <boost/assign/list_of.hpp>
//MAP
#include "global_map.hpp"
#include "map_element.hpp"
#include "map_element_types.h"
#include "map_helper.hpp"
#include "map_event_listener.hpp"

enum lane_to_lock {LL_LANE_LEFT, LL_LANE_CENTER, LL_LANE_RIGHT};

class PathPlanner {
 private:
  frAIburg::map::GlobalMap* map_;
  // allowed error so that neighbouring lane markings describe same lane:
  float MAX_LANE_WIDTH_ERROR_;
  bool LOG_DEBUG_;
  bool request_rear_cam_lanes_;
  const tBehaviour *behaviour_curr_;
  const tPose* pose_curr_;
  std::vector<Lane>* lane_vert_vec_ptr_;

  bool getLaneCenterPoint(float x_dist, tPoint* center_point,
                          Lane* lane_left = NULL, Lane* lane_right = NULL) const;
  float approxLaneDistance(Lane* lane1, Lane* lane2,
                           float x_eval) const;
  float getDerivAtX(float x, const std::vector<float> &coeff) const;
  float getValAtX(float x, const std::vector<float> &coeff) const;
  tPoint getTPoint(float x, float y) const;
  float minGoAroundX(float y, bool goLeft) const;
  bool CheckCollision(const tPath& path,
            frAIburg::map::tMapID* collision_id);
  bool GetPointAtX(float x_max, tPoint* res_pnt,
                   float * res_heading) const;


 public:

  ManeuverGoal goal_global_;
  float v_current_;
  PathPlanner();
  void updateStatePointer(const tBehaviour& behaviour_current,
                          const tPose& pose_global);
  void updateLaneVecPointer(std::vector<Lane>* lane_vert_vec_ptr); // this ptr isn't const.
  void updateSpeed(float v_current);
  void setMap(frAIburg::map::GlobalMap* map);
  Spline spline_;
  GoalPointGenerator gp_gen_;
  frAIburg::map::tMapID obstacle_id_;
  frAIburg::map::tMapID path_id_;
  tPath current_path_;
  tTimeStamp current_time_;
  planner_status PlanPath(const tPath &way_points,
                          float goal_orientation);

  float getStopDistToObject(int object_id, bool project_on_x, bool zero_cutoff) const;
  bool getDistanceToObject(float* dist, int object_id, bool project_on_x) const;
  bool getDistSpeedCarAhead(float* v_rel, float* distance);
  /*! return the closest el which collide with the path, return map defautl id
  if no collision detected */
private:
  frAIburg::map::tMapID getClosestCollidingObjectID(
                        frAIburg::map::tSptrMapElement &path) const;
  /* returns a rect map debug poly element, with a fixed size*/
  frAIburg::map::tSptrMapElement getPlannerCollisionPath(
                        lane_to_lock lane_offset ) const;
  frAIburg::map::tSptrMapElement createCollisionBox(float x_center, float y_center) const;
public:
  /* check a collision with the planer path or a fixed size rect, if no collision map defautl id is retured */
  frAIburg::map::tMapID getClosestCollidingIDWithPlannerPath(lane_to_lock lane_offset = LL_LANE_CENTER) const;
  frAIburg::map::tMapID getClosestCollidingIDWithBox(float x_center, float y_center) const;

  const tPath& getCurrentPath() const;

  void getDemoPath(tPath* way_points,
                   float* goal_orientation);

  bool getGoAroundPath(tPath *way_points,
                       float* goal_orientation) const;
  tPath& GetPathRef();

  void getCarFollowTrajectory(tPath *way_points,
                              float* goal_orientation,
                              float v_current,
                              float* v_tracking,
                              float* dist_car);
  bool isLaneChangeAllowed();
  void getPathBackwards(tPath *way_points,
                        float* goal_orientation);
  void getPathToParkingSwitch(tPath *way_points,
                              float* goal_orientation);
  void getPathToPark(tPath *way_points,
                     float* goal_orientation);

  bool getBackwardsLanePath(tPath* way_points,
                          float* goal_orientation) const;

  void getPathToGlobalCoord(const tPose& pose_goal,
                            tPath* way_points,
                            float* goal_orientation,
			    float x_extension = 0.) const;

  bool getCrossingApproachPath(tPath *way_points,
                               float *goal_orientation);
  bool getCrossingPath(tPath *way_points,
                       float *goal_orientation);
  bool getPointOnCrossing(int crossing_id,
                          const tPoint& point_relative_crossing,
                          tPoint *pnt_car_frame,
                          float *dphi_crossing) const;
  bool getPullOutPath(tPath *way_points, float* goal_orientation);

  bool getGoalAlignedLanePath(tPath* way_points, float* goal_orientation);

  bool getLaneToClearPath(tPath* way_points,
                          float* goal_orientation);

  bool getLaneSwitchPath(tPath* way_points,
                         float* goal_orientation);

  bool getLaneFollowPath(tPath* way_points,
                         float* goal_orientation,
                         lane_to_lock lane_goal) const;

  void sampleFromLanes(Lane* lane_left, Lane* lane_right,
                       tPath* way_points, float* goal_orientation) const;

  void setDebugMode(bool debug_active);

  void DrawCurrentPath(float lifetime_sec);
  void DrawCurrentway_points(float lifetime_sec, const tPath &way_points);
  float getArcLength(const tPoint &goal_pnt, float goal_heading) const;
  void updateTimestamp(tTimeStamp timestamp);
  bool isInRange(float val, float min, float max, bool use_abs) const;

};

#endif  // AADCUSER_FRAIBURG_PLANNER_PATHPLANNER_H_
