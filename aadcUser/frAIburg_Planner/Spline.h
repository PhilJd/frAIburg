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
#ifndef AADCUSER_FRAIBURG_PLANNER_SPLINE_H_
#define AADCUSER_FRAIBURG_PLANNER_SPLINE_H_
//#include "stdafx.h"
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <vector>
#include "../../../Libs/qpOASES-3.2.1/include/qpOASES.hpp"
#include "nonpin_types.h"
//#include "customtypes.h"


USING_NAMESPACE_QPOASES

class Spline {
 public:
    Spline();
    // virtual ~Spline();
    void printStats(bool printPntSegMap);
    bool getSampledPath(const tPath &way_points, float goal_heading_local,
                        tPath* result_path, float straight_extension_length);
    void setDebugMode(bool debug_active);


 private:
    bool IS_DEBUG_MODE_;
    int n_order_;  // 3rd order poly. for each segment. Use 4 later on
    int n_segments_;
    int n_points_;
    int n_vars_;
    int FLAG_REINIT_PROBLEM_;
    float HEADING_ERROR_FEASIBLE_;
    int n_constraints_;
    float goal_heading_;
    bool use_circular_extension_;
    float phi_min_circle_;
    float phi_max_circle_;
    float MAX_HEADING_;
    float X_SAMPLE_RES_;
    tPoint circle_center_;
    tPath path_support_;  // points that support the path (Stuetzstellen)
    std::vector<std::vector<float> > coeff_vec_;
    std::vector<int> point_segment_map_;  // maps point index to segment index

    // use Eigen matrices for easy problem setup/index access. In the end,
    // convert
    // to row_major array
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        H_mat, g_vec, A_mat, lbA_vec, ubA_vec, phi_test;

    real_t *H, *A, *g, *lb, *ub, *lbA, *ubA;
    float constant_term_;

    void preprocessWaypoints(tPath *waypoints);
    float getValAtX(float x, std::vector<float> coeff);
    void assignPointToSegment();
    bool sampleSpline(tPath* result_path, float x_resolution, 
                      float straight_extension_length);
    bool solveQP();
    void setObjectivefunc();
    void setConstraints();
    void setGoalPointToCircle(tPoint* goal_new, tPath* target_path);
};

#endif  // AADCUSER_FRAIBURG_PLANNER_SPLINE_H_
