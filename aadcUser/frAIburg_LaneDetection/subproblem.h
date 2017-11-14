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

#ifndef AADCUSER_FRAIBURG_LANEDETECTIONFILTER_SUBPROBLEM_H_
#define AADCUSER_FRAIBURG_LANEDETECTIONFILTER_SUBPROBLEM_H_

//#include "stdafx.h"
#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>
#include "../../../Libs/qpOASES-3.2.1/include/qpOASES.hpp"
//#include "customtypes.h"
#include "nonpin_types.h"

USING_NAMESPACE_QPOASES

struct weight_struct {
    std::vector<int> stop_idx;
    std::vector<float> weights;
    float weight_sum_;
    int curr_size;
    int n_segments;
    weight_struct() {
        weight_sum_ = 0;
        curr_size = 0;
        n_segments = 0;
    }
    void addStopidxWeight(int idx, float weight) {
        curr_size = curr_size + idx;
        n_segments++;
        stop_idx.push_back(curr_size - 1);
        weights.push_back(weight);
        weight_sum_ += weight;
        // std::cout << "index: "<< stop_idx[n_segments-1] << " weight: " <<
        // weight
        // << "\n" ;
    }
};

typedef std::vector<tPoint> tPath;

class Subproblem {
 public:
    
    Subproblem(const tPath &path_points, int order, weight_struct *weights = NULL);
    void solveQP();
    orientation_type detectOrientation();                                                /// MAKE PRIVATE and change calling methods..
    float getValAtX(float x) const;
    float getDerivAtX(float x) const;
    tPath *path_;
    int n_samples_;
    std::vector<float> coeff_;
    orientation_type orientation_;
    // if orient. = 1, store mean. x-value (approx. dist. to car)
    float vert_x_mean_;  
     // if orient. = 0, store y at x0 here; used for grouping
    float y_at_x0_; 
    float fitting_error_;
    float SLOPE_HORIZ_LANE_MIN_;
 private:

    void setObjectivefunc();
    float getMeanX() const;
    // only for subproblems with n_order = 2
    float getXatY0() const;
    float getWeightAtIdx(int idx) const;

    bool dumpLog_;
    float Y_AT_X0_MAX_;
    float WEIGHT_REGULARIZATION_;
    
    weight_struct *weights_;
    int n_order_;        // order of polynomial
    int n_vars_;         // will later become more complex (splines, parallel
                         // fitting..)
    int n_constraints_;  // equals no. of row of constraint matrix A
    // store regression matrix here
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        H_mat;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        g_vec;
    real_t *H, *A, *g, *lb, *ub, *lbA, *ubA;
    float constant_term_;

};
#endif  // AADCUSER_FRAIBURG_LANEDETECTIONFILTER_SUBPROBLEM_H_
