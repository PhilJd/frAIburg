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
#ifndef AADCUSER_FRAIBURG_LANEDETECTIONFILTER_LANEFINDER_H_
#define AADCUSER_FRAIBURG_LANEDETECTIONFILTER_LANEFINDER_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include "subproblem.h"
#include "customtypes.h"
#include <cassert>

struct LanePOI
{
  orientation_type orientation;
  float dist;
  float angle;
};

enum lane_gap_type_enum {LG_TO_FRONT, LG_TO_BACK, LG_TO_LEFT, LG_TO_RIGHT, LG_UNKNOWN};
enum corner_type_enum {C_BOTTOM_LEFT, C_BOTTOM_RIGHT, C_TOP_LEFT, C_TOP_RIGHT, C_UNKNOWN};

struct corner_proposal {
  tPose pose_corner;
  lane_gap_type_enum lane_gap_type;
  corner_type_enum corner_type;
  corner_proposal(float x, float y, float phi, lane_gap_type_enum gap_type,
                  corner_type_enum corner_type_) {
    pose_corner.x = x;
    pose_corner.y = y;
    pose_corner.heading = phi;
    lane_gap_type = gap_type;
    corner_type = corner_type_;
  }
};

class LaneFinder {
 private:
    bool cleanInputPaths(const std::vector<tPath>& paths_in,
                         std::vector<int>* path_assignments);

    bool solveQPs(const std::vector<tPath> &paths_in,
                  std::vector<Subproblem> *sub_vec_out, int order);

    bool mergePaths(const std::vector<tPath> &paths_in, std::vector<tPath> *paths_out,
                    std::vector<int>* path_assignments,
                    std::vector<weight_struct> *weight_vec = NULL,
                    std::vector<Subproblem> *sub_vec = NULL);

    float getPathWeight(const Subproblem& sub);

    bool getFirstOrderAssignment(
        const std::vector<tPath> &paths_in,
        const std::vector<Subproblem> &sub_vec,
        std::vector<int>* path_assignments);

    bool getSecondOrderAssignment(const std::vector<Subproblem> &sub_vec,
                                  std::vector<int>* path_assignments);

    bool processSubToLanes(const std::vector<Subproblem> &sub_vec,
                           std::vector<Lane> *lane_vert_vec,
                           std::vector<Lane> *lane_horiz_vec);

    bool setLimitPoints(const Subproblem &sub, tPoint *bottom_left,
                        tPoint *top_right);

    float detectStoplineDist(std::vector<Subproblem *> *sub_vert_vec,
                             std::vector<Subproblem *> *sub_horiz_vec,
                             std::vector<Lane> *lane_vert_vec,
                             std::vector<Lane> *lane_horiz_vec);

    bool setLane(Lane *lane, const Subproblem &sub);

    void removeOutlierLanes(std::vector<Lane> *lane_vec, bool isVertical);

    bool getIntersectingCorners(std::vector<Lane>& lane_vert_vec,
                                std::vector<Lane>& lane_horiz_vec,
                                tPath* corner_pnts,
                                float* direction_angle);
    int getNumberOfGaps(std::vector<Lane*>& lane_vert_vec,
                        float gap_dist) const;
    void getLaneDistFilteredPtr(std::vector<Lane>& lane_vec,
                                std::vector<Lane*>* lane_vecptr,
                                float dist_min);

    void MeanFuseAdjacentCorners(const tPath &temp_corners, tPath* corner_pnts);

    bool FindPartnerIP(const tPoint &ip,
                       const tPath &corner_candidates,
                       int candidates_index);
    bool IsAtSameDistAnd2LanesApart(const tPoint &p1, const tPoint &p2);

    bool ComputeIntersectionPoint(float coeff_a1, float coeff_b1,
                            float coeff_c1, float coeff_a2, float coeff_b2,
                            float coeff_c2, tPoint *ip);
    void ComputeCrossingCenter(const tPath &corner_pnts,
                                    tCrossing *detected_crossing);

    float max_view_x_;

    ////////////////////////////////////////////////////////////////////////////////////////////
    // dist. for 2nd order merging paths neighbouring together:
    float MIN_SEGMENT_DIST_;
    float MAX_ANGLE_ERROR_;
    // lower bound on dist. between adjacent lanes; used for
    // determining lane distance to car in terms of multiples of this distance:
    // If segments intersect using 1std order extrapolation
    // the slope angle at their ends must be lower than this value:
    float MAX_DELTA_ANGLE_RAD_;
    float MIN_SEGMENT_LENGTH_;
    float MIN_VERT_LANE_LENGTH_;
    float MIN_HORIZ_LANE_LENGTH_;
    float MAX_EXTRAPOLATION_ERROR_;
    float point_dist_squared_;
    bool dumpLog_;

 public:
    void SetMaxViewX(float view);
    float getValAtX(float x, const std::vector<float>& coeff) const;
    float getDerivAtX(float x, const std::vector<float> &coeff) const;
    enum turn_type { left_turn, right_turn };
    void setParams(bool is_parking_mode);
    bool getLanes(const std::vector<tPath> &paths_in,
                  std::vector<Lane>* vert_lanes_out,
                  std::vector<Lane>* horiz_lanes_out,
                  tCrossing* crossing_detected = NULL);
    // crossing detection
    bool detectCrossing(std::vector<Lane>& lane_vert_vec,
                        std::vector<Lane>& lane_horiz_vec,
                        tCrossing* crossing_res,
                        std::vector<tPoint> *corner_pnts);
    LaneFinder();
};

#endif  // AADCUSER_FRAIBURG_LANEDETECTIONFILTER_LANEFINDER_H_
