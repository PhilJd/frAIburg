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

#include "Spline.h"

// check zero length paths etc.
// Spline needs to contain at least one nonzero waypoint

Spline::Spline() {
    X_SAMPLE_RES_ = 0.05;
    HEADING_ERROR_FEASIBLE_ = 5.0 * PI / 180.0;
    IS_DEBUG_MODE_ = false;
}

bool Spline::getSampledPath(const tPath& way_points, float goal_heading_local,
                            tPath* result_path,
                            float straight_extension_length) {
    n_order_ = 4;
    MAX_HEADING_ = 0.4 * 3.141;
    path_support_ = way_points;
    goal_heading_ = goal_heading_local;
    // transform to -PI..PI
    if (fabs(goal_heading_) >= PI) {
        goal_heading_ += (goal_heading_ < 0) ? 2. * PI : -2. * PI;
    }
    if (path_support_.size() > 0 && path_support_[0].x != 0.) {
        tPoint start;
        start.x = 0.;
        start.y = 0.;
        path_support_.insert(path_support_.begin(), start);
        printf("Spline: You forgot to add point (0, 0) to waypoints.\n");
    }
    // preprocessWaypoints(&path_support_);

    if (path_support_.size() <= 1) {
        printf("Spline: Path contains less than two waypoints. abort. \n");
        return false;
    }
    n_points_ = path_support_.size();

    use_circular_extension_ = false;
    // ||
    //(path_support_.back().x < 0 && fabs(goal_heading_-PI) > MAX_HEADING_)
    if ((fabs(goal_heading_) > MAX_HEADING_)) {
        use_circular_extension_ = true;
        setGoalPointToCircle(&path_support_.back(), &path_support_);
        if (IS_DEBUG_MODE_)
            printf(
                "goal heading was too large. Set new goal and added circle"
                " segment to sampled path\n");
    }
    point_segment_map_.resize(n_points_);
    n_segments_ = std::max(1, n_points_ - 1);
    // each segment consists of 1 polynomial
    n_vars_ = (n_order_ + 1) * n_segments_;
    coeff_vec_.resize(n_segments_);
    FLAG_REINIT_PROBLEM_ = 0;
    lbA = 0, ubA = 0, A = 0, H = 0, g = 0, lb = 0, ub = 0;
    assignPointToSegment();
    if (IS_DEBUG_MODE_) printStats(true);
    if (!solveQP()) {
        printf("Spline: Failed to solve QP. Abort and print stats. \n");
        printStats(true);
        return false;
    }
    return sampleSpline(result_path, X_SAMPLE_RES_, straight_extension_length);
    return true;
}

/* This methods removes conflicting waypoints that would interfere with
   the path quality. */
void Spline::preprocessWaypoints(tPath* waypoints) {
    tPath& wp = *waypoints;
    int idx = 0;
    float direction = waypoints->back().x >= 0. ? 1. : -1;
    // this conditions ensures that comparison with last point always possible
    // and last point won't be removed:
    while (idx < wp.size() - 1) {
        // x-vals should either be strictly increasing or decreasing towards the
        // goal
        if (direction * (wp[idx + 1].x - wp[idx].x) <= 0) {
            // special case: we're at point (0,0) that shouldn't ever be removed
            if (idx == 0) {
                printf("WARNING: Spline: Removed waypoint (%f %f)\n",
                       (wp.begin() + 1)->x, (wp.begin() + 1)->y);
                wp.erase(wp.begin() + 1);
            } else {
                printf("WARNING: Spline: Removed waypoint (%f %f)\n",
                       (wp.begin() + idx)->x, (wp.begin() + idx)->y);
                wp.erase(wp.begin() + idx);
                idx--;
            }
        } else
            idx++;
    }
}

void Spline::printStats(bool printPntSegMap) {
    printf("n points: %i\n", n_points_);
    printf("n segments: %i\n", n_segments_);
    printf("n order: %i\n", n_order_);
    printf("n vars: %i\n", n_vars_);
    if (printPntSegMap) {
        for (int i = 0; i < point_segment_map_.size(); ++i) {
            printf("point %i (%f,%f) mapped to segment %i \n", i,
                   path_support_[i].x, path_support_[i].y,
                   point_segment_map_[i]);
        }
    }
    printf("goal heading [rad2deg]: %f\n", goal_heading_ * 57.);
}

/* This method sets a new goal/Spline end point, that will be extended by a
   circle segment.
    For this extension, 2 angles and a circle center are computed*/
void Spline::setGoalPointToCircle(tPoint* goal_new, tPath* target_path) {
    float offset = target_path->back().x < 0 ? PI : 0.;
    float heading_sign = goal_heading_ / fabs(goal_heading_);
    float edge_case_sign;

    if (target_path->back().x > 0 || target_path->back().y < 0)
        // 1st, 3rd and 4th qudrant handled
        edge_case_sign = 1.;
    else
        // 1st quadrant, edge case
        edge_case_sign = -1.;

    phi_max_circle_ = goal_heading_ - heading_sign * 0.5 * PI + offset;
    phi_min_circle_ = edge_case_sign * heading_sign *
                      (MAX_HEADING_ - 0.5 * PI + edge_case_sign * offset);
    goal_heading_ =
        (edge_case_sign * goal_heading_ > 0) ? MAX_HEADING_ : -MAX_HEADING_;
    circle_center_.x =
        target_path->back().x - RADIUS_MIN_ * cos(phi_max_circle_);
    circle_center_.y =
        target_path->back().y - RADIUS_MIN_ * sin(phi_max_circle_);
    (*goal_new).x = circle_center_.x + RADIUS_MIN_ * cos(phi_min_circle_);
    (*goal_new).y = circle_center_.y + RADIUS_MIN_ * sin(phi_min_circle_);

    // printf("phimax[in PI]: %f phimin: %f cx %f cy %f goalheading %f\n",
    // phi_max_circle_/PI, phi_min_circle_/PI,
    //     circle_center_.x, circle_center_.y, goal_heading_/PI);
}

void Spline::setConstraints() {
    // ###################### CONSTRAINTS
    /* 1: initial position
       2: initial slope
       3: goal slope
       4-...end continuity constraints for p, p', p''
    */
    tPoint goal_point = path_support_.back();

    n_constraints_ = 3 + 3 * (n_segments_ - 1);

    int constraint_idx = 0;
    A_mat.resize(n_constraints_, n_vars_);
    A_mat.setZero();
    lbA_vec.resize(n_constraints_, 1);
    lbA_vec.setZero();
    ubA_vec.resize(n_constraints_, 1);
    ubA_vec.setZero();

    // ## p(x0=0) = 0, [params: only first coefficient non-zero]
    A_mat(constraint_idx, 0) = 1;
    lbA_vec(constraint_idx, 0) = 0;
    ubA_vec(constraint_idx, 0) = 0;
    // ## p'(x0=0) = 0, [params: only second coefficient non-zero]
    constraint_idx++;
    A_mat(constraint_idx, 1) = 1;
    ubA_vec(constraint_idx, 0) = 0.;
    lbA_vec(constraint_idx, 0) = 0.;
    // ## p'(xgoal) = tan(heading_goal+-allowederror); 10 deg error is good
    // enough
    constraint_idx++;
    int last_seg_coeff = (n_order_ + 1) * (n_segments_ - 1);

    // derivative
    for (int i = 0; i < n_order_ + 1; ++i) {
        A_mat(constraint_idx, last_seg_coeff + i) =
            i * pow(goal_point.x, (i - 1));
    }
    if (fabs(goal_heading_) > 0.5 * PI) {
        printf(
            "|goal_heading| > pi/2! Could be infeasible with too few spline"
            " segments\n");
    }
    // ToDo(Felix): just set max slope to 999999 remove below code
    float slope_high = tan(goal_heading_ + HEADING_ERROR_FEASIBLE_);
    float slope_low = tan(goal_heading_ - HEADING_ERROR_FEASIBLE_);
    // printf("sl H %f sl L %f\n", slope_high, slope_low);
    if ((slope_high >= 0 && slope_low >= 0) ||
        (slope_high <= 0 && slope_low <= 0)) {
        ubA_vec(constraint_idx, 0) = slope_high;
        lbA_vec(constraint_idx, 0) = slope_low;
    } else {
        if (slope_low > 0 && goal_heading_ > 0) {
            // printf("first case\n");
            ubA_vec(constraint_idx, 0) = 1e20;
            lbA_vec(constraint_idx, 0) = slope_low;
        } else {
            // printf("2nd case\n");
            ubA_vec(constraint_idx, 0) = slope_low;
            lbA_vec(constraint_idx, 0) = -1e20;
        }
    }
    // ##### now add (n_segments-1)*3 continuity conditions (p, p', p'')
    for (int i = 0; i < n_points_ - 1; i++) {
        // printf("idx i = %i, i+1 = %i\n", point_segment_map_[i],
        // point_segment_map_[i+1]);
        // end and start point detected?
        if (point_segment_map_[i] != point_segment_map_[i + 1]) {
            // x val. of point belonging to two segments
            float x_c = path_support_[i].x;
            // start column index in constraint matrix A
            int start_col_idx_seg1 = point_segment_map_[i] * (n_order_ + 1);
            // ######### add constraint ensuring position continuity
            constraint_idx++;
            for (int j = 0; j < 2 * (n_order_ + 1); j++) {
                // p_i(x_c) - p_(i+1)(x_n) must be 0
                int p_sign = (j < n_order_ + 1) ? 1 : -1;
                int idx_offset = (j < n_order_ + 1) ? 0 : (n_order_ + 1);
                A_mat(constraint_idx, start_col_idx_seg1 + j) =
                    p_sign * pow(x_c, j - idx_offset);
                ubA_vec(constraint_idx, 0) = 0;
                lbA_vec(constraint_idx, 0) = 0;
            }
            // #### add constraint ensuring tangent continuity
            constraint_idx++;
            for (int j = 0; j < 2 * (n_order_ + 1); j++) {
                // p_i(x_c) - p_(i+1)(x_n) must be 0
                int p_sign = (j < n_order_ + 1) ? 1 : -1;
                // reset index to zero for computing powers of next segment
                int idx_offset = (j < n_order_ + 1) ? 0 : (n_order_ + 1);
                A_mat(constraint_idx, start_col_idx_seg1 + j) =
                    p_sign * (j - idx_offset) * pow(x_c, (j - idx_offset - 1));
                ubA_vec(constraint_idx, 0) = 0;
                lbA_vec(constraint_idx, 0) = 0;
            }
            // #### add constraint ensuring curvture continuity
            constraint_idx++;
            for (int j = 0; j < 2 * (n_order_ + 1); j++) {
                // p_i(x_c) - p_(i+1)(x_n) must be 0
                int p_sign = (j < n_order_ + 1) ? 1 : -1;
                // reset index to zero for computing powers of next segment
                int idx_offset = (j < n_order_ + 1) ? 0 : (n_order_ + 1);
                A_mat(constraint_idx, start_col_idx_seg1 + j) =
                    p_sign * (j - idx_offset) * (j - idx_offset - 1) *
                    pow(x_c, (j - idx_offset - 2));
                ubA_vec(constraint_idx, 0) = 0;
                lbA_vec(constraint_idx, 0) = 0;
            }
        }  // end adding current continuity constraints
    }      // end iterating points
    // std::cout << "A is " << std::endl << A_mat << std::endl;
    // std::cout << "lbA is " << std::endl << lbA_vec << std::endl;
    // std::cout << "ubA is " << std::endl << ubA_vec << std::endl;
    A = A_mat.data();
    lbA = lbA_vec.data();
    ubA = ubA_vec.data();
}

void Spline::setObjectivefunc() {
    /* setting up objective function, store Eigen matrices and pass
        real_t pointer to row-major memory block as expected by qpOases */

    // ######################### Fills regression matrix so that in i-th row:
    // x_i^0, x_i^1 ...
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> phi(
        n_points_, n_vars_);
    phi.resize(n_points_, n_vars_);
    phi.setZero();
    // all waypoints except last have weight 1
    std::vector<float> weight_vec(n_points_, 1);
    weight_vec.back() = 1;
    for (int i = 1; i < n_points_; ++i) {  // exclude zero
        // each segment corresponds to another set of coefficients=^optimvars
        int column_start = point_segment_map_[i] * (n_order_ + 1);
        for (int j = 0; j < n_order_ + 1; ++j) {
            // goal point gets higher weight
            phi(i, j + column_start) =
                weight_vec[i] * pow(path_support_[i].x, j);
            // printf("entry in phi is x y%f %f\n", path_support_[i].x, phi(i, j
            // + column_start));
        }
    }

    // phi_test = phi; // ONLY FOR DEBUGGING

    // #########################  Set H = 2*Phi'*Phi
    H_mat.resize(n_vars_, n_vars_);
    H_mat = (2 * phi.transpose() * phi);

    // add regularization term to penalize 2nd+ order coefficients
    for (int i = 0; i < n_segments_; ++i) {
        // jump in blocks, skip every n_order_+1 rows and cols
        int row_col_start = i * (n_order_ + 1);
        // add weight to penalize
        for (int i_coeff = 2; i_coeff < n_order_ + 1; ++i_coeff) {
            H_mat(row_col_start + i_coeff, row_col_start + i_coeff) +=
                0.0001;  // try out different weights
        }
    }
    // Pointer to H_mat (pointing to row-major memory block)
    H = H_mat.data();
    // ######################### set g = -2*Phi*y
    Eigen::MatrixXd y_tmp(n_points_, 1);
    for (int i = 1; i < n_points_; ++i) {
        y_tmp(i, 0) = weight_vec[i] * path_support_[i].y;
    }
    g_vec.resize(n_vars_, 1);
    g_vec = -2 * phi.transpose() * y_tmp;
    g = g_vec.data();

    // constant term only needed for adding to QProblem.getObjectiveVal() to get
    // error
    constant_term_ = (y_tmp.transpose() * y_tmp)(0, 0);
    // std::cout  << "phi = \n" << phi << std::endl;
    // std::cout  << "H = \n" << H_mat << std::endl;
}

float Spline::getValAtX(float x, std::vector<float> coeff) {
    float res = 0;
    for (int i = 0; i < coeff.size(); ++i) {
        res += coeff[i] * pow(x, i);
    }
    return res;
}

bool Spline::solveQP() {
    // printf("try setting objective");
    setObjectivefunc();
    // printf("try setting constraints");
    setConstraints();

    QProblem optimproblem(n_vars_, n_constraints_);  // , n_constraints_
    Options options;
    options.printLevel = PL_NONE;
    optimproblem.setOptions(options);
    int_t nWSR = 200;  // max iterations; alternative: max cpu-time

    returnValue ret_val = optimproblem.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    if (ret_val != SUCCESSFUL_RETURN) {
        std::cout << "phi = \n" << phi_test << std::endl;

        return false;
    }
    real_t xOpt[n_vars_];
    optimproblem.getPrimalSolution(xOpt);
    for (int i = 0; i < n_segments_; ++i) {
        // printf("p%i = %f\n", i, xOpt[i]);
        int start_idx = i * (n_order_ + 1);
        coeff_vec_[i].clear();
        coeff_vec_[i].insert(coeff_vec_[i].begin(), &xOpt[start_idx],
                             &xOpt[start_idx + n_order_ + 1]);
    }
    return true;
    /*for (int i = 0; i<coeff_vec_[0].size();++i){
        printf("spline coeffvec0, coeff %i is %f\n", i, coeff_vec_[0][i]);
    }*/
}

bool Spline::sampleSpline(tPath* result_path, float x_resolution,
                          float straight_extension_length) {
    /* - no extrapolation (use case?). Also, max distance could be specified to
     save computations
     - note: segment ends with sampled point; next segment doesn't necessarily
     start with one*/
    if (result_path == NULL || path_support_.size() == 0) {
        printf("NULL or zero elements in sample Spline. return\n");
        return false;
    }
    result_path->resize(0);

    float x_sample = 0;
    int curr_segment_idx = 0;
    // result_path->reserve(path_support_.back().x / x_resolution);
    result_path->push_back(path_support_[0]);
    for (int i = 0; i < path_support_.size() - 1; i++) {
        float x_next = path_support_[i + 1].x;
        float direction_sign = (x_next - path_support_[i].x);
        // better use sign of first derivative;
        // could fail at U-turn & path extrapolation enabled
        direction_sign = (direction_sign < 0) ? -1 : 1;
        // new segment starts
        if (point_segment_map_[i] != point_segment_map_[i + 1]) {
            curr_segment_idx += 1;
        }

        while (fabs(x_next - x_sample) >= 0.5 * x_resolution) {
            x_sample += direction_sign * x_resolution;
            float y_sample = 0;
            std::vector<float> curr_coeff = coeff_vec_[curr_segment_idx];
            // calc. function val. y = p(x)
            for (int j = 0; j < n_order_ + 1; ++j) {
                y_sample += curr_coeff[j] * pow(x_sample, j);
            }
            tPoint res_pnt;
            res_pnt.x = x_sample;
            res_pnt.y = y_sample;

            // printf("sample; x=%f, y=%f\n", res_pnt.x, res_pnt.y);
            // printf("sample; x=%f, y=%f\n", res_pnt.x, res_pnt.y);
            if (result_path->back().x != res_pnt.x ||
                result_path->back().y != res_pnt.y)
                result_path->push_back(res_pnt);
        }
    }
    if (use_circular_extension_) {
        // printf("circle with phi_max %f, phi_min %f,  x0 = %f, y0 = %f\n",
        // phi_max_circle_*180./3.141, phi_min_circle_*180./3.141,
        // circle_center_.x,
        // circle_center_.y);

        float radius_adapt =
            sqrt(pow(circle_center_.x - result_path->back().x, 2) +
                 pow(circle_center_.y - result_path->back().y, 2));
        // remove last element to avoid overlap with circle; otherwise, problem
        // with setting up polygon / wrong order
        result_path->pop_back();
        for (int i = 1; i <= 5.; ++i) {
            tPoint res_pnt;
            float angle =
                phi_min_circle_ + i / 5.0 * (phi_max_circle_ - phi_min_circle_);
            // printf("angle is %f\n", angle);
            res_pnt.x = circle_center_.x + radius_adapt * cos(angle);
            res_pnt.y = circle_center_.y + radius_adapt * sin(angle);
            result_path->push_back(res_pnt);
            // printf("circ: x = %f, y = %f\n", res_pnt.x, res_pnt.y);
        }
    }

    // straight extension
    /*float straight_extension_length = 0.2;
    if (straight_extension_length > 0.0 && result_path->size() > 1) {
            int n_size = result_path->size();
            tPoint last_pnt = result_path->back();
            float dx = (*result_path)[n_size-1].x - (*result_path)[n_size-2].x;
            float dy = (*result_path)[n_size-1].y - (*result_path)[n_size-2].y;
            int n_points = 5;
            float norm = sqrt(pow(dx, 2) + pow(dy, 2));
            dx = straight_extension_length*dx/(norm*n_points);
            dy = straight_extension_length*dy/(norm*n_points);
            for (int i = 1; i<=n_points; ++i) {
                tPoint res;
                res.x = last_pnt.x + i*dx;
                res.y = last_pnt.y + i*dy;
                result_path->push_back(res);
            }
        }*/
    return true;
}

void Spline::assignPointToSegment() {
    /* determine which point is described by which spline segment
            considers 3 cases */

    if (n_segments_ == 1) {  // all points mapped to only segment, >= 2 points
                             // implicitly assumed
        // std::cout  << "Points assigned to single segment" << "\n";
        for (int i = 0; i < n_points_; i++) {
            point_segment_map_[i] = 0;
        }
    } else if (n_segments_ ==
               n_points_ - 1) {  // every point to index i-1, e.g.
                                 // point 2 (idx 1) to segment 1
                                 // (idx 0)
        // std::cout << "One segment less than points" << "\n";
        point_segment_map_[0] = 0;
        for (int i = 1; i < n_points_; i++) point_segment_map_[i] = i - 1;
    }
    // else: more than 1 segment and at least 2 more points than segments exist
    /*else{ // group points with similar curvature together, e.g. consider lane
   following situation with path: _____/
      // implicitly assumed: more than 3 points
        // std::cout << "at least 2 more points than segments";
       float sec_deriv[n_points-2]; // store "2nd derivatives" =^ differences of
   neighboring slopes
       for(int i=0; i<n_points-2; i++){
           float y1 = path_support[i+1].y-path_support[i].y;
           float y2 = path_support[i+2].y-path_support[i+1].y;
           float x1 = path_support[i+1].x-path_support[i].x;
           float x2 = path_support[i+2].x-path_support[i+1].x;
           sec_deriv[i] = (x2*y1-y2*x1)/(x1*x2);
           std::cout << sec_deriv[i] << "\n";
       }
   }
   else{
     // integer ceil(points/segments) for pos. numbers
     int split_interval = (n_points_+n_segments_-1)/n_segments_;
     std::cout << "Auto assignment of points to segments" << "\n";
     int rounding_overlap = n_points_ - split_interval*(n_segments_-1);
       // std::cout << rounding_overlap;
     for(int i = 0; i<n_segments_-1; i++){
       for(int k = 0; k<split_interval;k++){
         point_segment_map_[i*split_interval+k] = i;
          // std::cout << i << " " << (i*split_interval+k) << "\n";
       }
     }

     for(int i = 0; i<rounding_overlap; i++)
       point_segment_map_[n_points_-1-i] = n_segments_-1;

       // check if last point belongs to single segment. If yes, assign to
   previous segment
     if(point_segment_map_[n_points_-1]!=point_segment_map_[n_points_-2]){
       point_segment_map_[n_points_-1] = point_segment_map_[n_points_-2];
       n_segments_ -= 1;
       FLAG_REINIT_PROBLEM_ = 1;
       std::cout << "No. of segments had to be decremented" << "\n";
     }
     for(int i = 0; i<n_points_; i++)
       std::cout << i << " " << point_segment_map_[i] << "\n";
   }*/
}

void Spline::setDebugMode(bool debug_active) { IS_DEBUG_MODE_ = debug_active; }
