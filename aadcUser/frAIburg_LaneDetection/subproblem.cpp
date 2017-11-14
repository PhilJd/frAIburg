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

#include "subproblem.h"

Subproblem::Subproblem(const tPath& path_points, int order,
                       weight_struct* weights) {
    path_ = (tPath*)&path_points;  // not nice, but.. path_points is alive!
    n_samples_ = path_->size();
    n_order_ = order;
    n_vars_ = n_order_ + 1;
    coeff_.resize(n_vars_);
    weights_ = weights;
    n_constraints_ = 0;
    WEIGHT_REGULARIZATION_ = 1e-3;  // HAS BIG INFLUENCE! 3.11: 1e-3
    // vertical lanes must originate +-1m from car:
    Y_AT_X0_MAX_ = 1.8 * LANEWIDTH_;
    // in car frame: slope for horiz. lanes must be higher:
    SLOPE_HORIZ_LANE_MIN_ = 2.f;
    /* define matrices as row-array as required by qpOases
    H, A are not deep-copied by qpOases, make sure to not change during solve*/
    // H = new real_t[n_vars_*n_vars_];
    // g = new real_t[n_vars_];
    lb = 0;  // new real_t[n_order+1];
    ub = 0;  // new real_t[n_order+1];
    A = 0;   // contraint matrix
    lbA = 0;
    ubA = 0;
    dumpLog_ = false;
}

void Subproblem::solveQP() {
    setObjectivefunc();
    QProblemB optimproblem(n_vars_);  // , n_constraints_
    Options options;
    options.printLevel = PL_NONE;
    optimproblem.setOptions(options);
    int_t nWSR = 100;  // max iterations; alternative: max cpu-time
                       // optimproblem.init( H,g,A,lb,ub,lbA,ubA, nWSR );
    optimproblem.init(H, g, lb, ub, nWSR);

    real_t xOpt[n_vars_];
    optimproblem.getPrimalSolution(xOpt);
    coeff_.insert(coeff_.begin(), &xOpt[0], &xOpt[n_vars_]);
    // scale fitting error for better numerical stability
    fitting_error_ =
        1e6 * fabs(optimproblem.getObjVal() + constant_term_) / n_samples_;
    if (dumpLog_) {
        printf("\nxOpt = [ %f, %f, %f, %f ] \n", coeff_[0], coeff_[1],
               coeff_[2], coeff_[3]);
        // printf("Fitting Error is: %f\n", fitting_error_);
    }

    detectOrientation();
}

orientation_type Subproblem::detectOrientation() {
    float val_x0 = getValAtX(0.0);  // use small offset for better stability
    // lane must originate close to car and have less than 45deg slope
    if ((fabs(val_x0) < Y_AT_X0_MAX_) && (fabs(getDerivAtX(0.05)) < 1.5)) {
        orientation_ = vertical;
        y_at_x0_ = val_x0;
        // printf("%s: Part of vertical lane detected \n", __FUNCTION__);
    } else {
        // Else: get slope at avg. x-value of sampled points; if sufficiently
        // steep,
        // this curve describes a horizontal lane
        vert_x_mean_ = getMeanX();
        float slope = fabs(getDerivAtX(vert_x_mean_));
        if (slope > SLOPE_HORIZ_LANE_MIN_) {
            orientation_ = horizontal;
            // printf("%s: Part of horizontal lane detected \n", __FUNCTION__);
        } else {
            orientation_ = unknown;
            if (dumpLog_) {
                printf("lane orientiation couldn't be determined \n");
            }
        }
    }
    return orientation_;
}

void Subproblem::setObjectivefunc() {
    /* setting up objective function, store Eigen matrices and pass
        real_t pointer to row-major memory block as expected by qpOases */

    // Fills regression matrix so that in i-th row: x_i^0, x_i^1 ...
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> phi(
        n_samples_, n_vars_);
    phi.resize(n_samples_, n_vars_);

    for (int i = 0; i < n_samples_; ++i) {
        float weight = getWeightAtIdx(i);
        for (int j = 0; j < n_vars_; ++j) {
            phi(i, j) = weight * pow((*path_)[i].x, j);
        }
    }
    // set H = 2*Phi'*Phi
    H_mat.resize(n_vars_, n_vars_);
    H_mat = (2 * phi.transpose() * phi);
    // add regularization term to penalize 2nd+ order coefficients
    for (int i = 2; i < n_vars_; ++i) H_mat(i, i) += WEIGHT_REGULARIZATION_;
    H = H_mat.data();  // pointer to H_mat; pointing to row-major memory

    // set g = -2*Phi*y
    Eigen::MatrixXd y_tmp(n_samples_, 1);
    for (int i = 0; i < n_samples_; ++i) {
        float weight = getWeightAtIdx(i);
        y_tmp(i, 0) = weight * (*path_)[i].y;
    }
    g_vec.resize(n_vars_, 1);
    g_vec = -2 * phi.transpose() * y_tmp;
    g = g_vec.data();
    // constant term only needed to add to getObjectiveVal() to get error
    constant_term_ = (y_tmp.transpose() * y_tmp)(0, 0);
}

float Subproblem::getMeanX() const {
    float res = 0;
    for (int i = 0; i < n_samples_; ++i) res += (*path_)[i].x;
    res = res / n_samples_;
    return res;
}

float Subproblem::getValAtX(float x) const {
    float res = 0;
    for (int i = 0; i < n_vars_; ++i) {
        res += coeff_[i] * pow(x, i);
    }
    return res;
}

float Subproblem::getDerivAtX(float x) const {
    float res = 0;
    for (int i = 1; i < n_vars_; ++i) {
        res += coeff_[i] * pow(x, i - 1);
    }
    return res;
}

float Subproblem::getWeightAtIdx(int idx) const {
    /* weight_ points to a struct of type weight.
       We solved several subproblems before, each with its own fiting error.
       Corresponding 'sub-paths' are then merged together into final path.
      *weight_ then stores fitting errors and end indices of sub-paths */

    // if default constructor is used, weights_ points to NULL
    // and all points get equal weight
    if (weights_ == NULL) return 1;
    // otherwise for each segment there's a different weight
    if (weights_->n_segments <= 1) return 1;
    // for all segments the merged path consists of:
    for (int i = 0; i < weights_->n_segments; ++i) {
        if (idx <= (weights_->stop_idx[i])) {
            return fabs((weights_->weights[i]) / weights_->weight_sum_);
        }
    }
    // just in case something goes wrong and no suitable index found:
    return 1;
}

// only for subproblems with n_order = 2
// Returns X where Y=0 or if no intersection: X of Y apex
float Subproblem::getXatY0() const {
    if (n_order_ != 2) {
        printf("cannot getXatY0 for this poly. order");
        return 0;
    }
    float a = coeff_[0];
    float b = coeff_[1];
    float c = coeff_[2];
    float radical = b * b - 4 * a * c;
    float root_radical;
    float sol1;
    float sol2;
    if (fabs(c) < 1e-9) return -a / b;  // just a line
    if (radical < 0)
        return -b / (2 * c);  // no solution, return x = argmin(f(x))
    root_radical = sqrt(radical);
    sol1 = -b / (2 * c) - root_radical;
    sol2 = -b / (2 * c) + root_radical;
    if (sol1 * sol2 < 0.0f) {              // solutions have different signs
        return sol1 > 0.0f ? sol1 : sol2;  // return the positive x-value
    } else {                               // both have same sign;
        if (sol1 > 0.0f)
            return std::max(sol1, sol2);
        else
            return std::min(sol1, sol2);
    }
}
