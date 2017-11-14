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

#include "lane_finder.h"

LaneFinder::LaneFinder() {
    /* WARNING: Distances below may vary between cars/cameras,
       since the calibration is not perfect!*/
    dumpLog_ = false;
    setParams(false);
}

/* iteratively fit, cluster & merge
NOTE: Subproblems and corresponding path must have same life time,
so deep copies of detected points can be avoided.
*/
bool LaneFinder::getLanes(const std::vector<tPath> &paths_in,
                          std::vector<Lane> *vert_lanes_out,
                          std::vector<Lane> *horiz_lanes_out,
                          tCrossing* crossing_res) {
    // store subproblems for each stage here
    std::vector<Subproblem> sub_vec;
    // store path-assignments for each stage here
    std::vector<int> idx_list_assignment;

    // ----------- Remove paths segments that are too small etc.

    std::vector<tPath> cleansed_paths;
    if (!cleanInputPaths(paths_in, &idx_list_assignment)) {
        return false;
    }

    if (!mergePaths(paths_in, &cleansed_paths, &idx_list_assignment))
        return false;

    // // ----------- FIRST STAGE: fit all input lists

    if (!solveQPs(cleansed_paths, &sub_vec, 2)) return false;
    if (!getFirstOrderAssignment(cleansed_paths, sub_vec, &idx_list_assignment))
        return false;
    std::vector<tPath> merged_paths1;
    if (!mergePaths(cleansed_paths, &merged_paths1, &idx_list_assignment))
        return false;

    // ----------- SECOND STAGE fit merged paths again with 2nd order

    if (!solveQPs(merged_paths1, &sub_vec, 2)) return false;
    std::vector<weight_struct> weight_vec;  // this is used in 3rd stage
    if (!getSecondOrderAssignment(sub_vec, &idx_list_assignment)) return false;
    std::vector<tPath> merged_paths2;
    if (!mergePaths(merged_paths1, &merged_paths2, &idx_list_assignment,
                    &weight_vec, &sub_vec))  //&weight_vec, &sub_vec
        return false;

    // ---------- THIRD STAGE: final fit using 3rd order and weighted points
    const std::vector<tPath> &resultpath = merged_paths2;
    sub_vec.clear();
    for (int i = 0; i < resultpath.size(); ++i) {
        Lane lane;
        // DO NOT CHANGE ORDER below (default: 2);
        // otherwise, ADTF sending routines need to be changed as well.
        Subproblem sub(resultpath[i], 2, &weight_vec[i]);  //, &weight_vec[i]
        sub.solveQP();
        sub_vec.push_back(sub);
    }
    if (!processSubToLanes(sub_vec, vert_lanes_out, horiz_lanes_out))
        return false;

    removeOutlierLanes(vert_lanes_out, true);

    return true;
}

void LaneFinder::setParams(bool is_parking_mode) {
    MIN_SEGMENT_LENGTH_ = 0.15;       // for init. path cleansing
    MAX_EXTRAPOLATION_ERROR_ = 0.05;  // for 1st order merge
    MIN_SEGMENT_DIST_ = 0.1;         // for 2nd order merge
    MAX_ANGLE_ERROR_ =
        10. * PI / 180;            // for 2nd order merge and outlier removal
    // MIN_VERT_LANE_LENGTH_ = 0.3;   // for final sub->lane
    MIN_HORIZ_LANE_LENGTH_ = 0.19;  // for final sub->lane  from 0.19 for crossings

    if (is_parking_mode) {
        MIN_VERT_LANE_LENGTH_ = 0.2;  // for final sub->lane
    } else {
        MIN_VERT_LANE_LENGTH_ = 0.3;  // for final sub->lane  from 0.35 for crossings
    }
    point_dist_squared_ = 0.02; // used for gap detection
}

bool LaneFinder::cleanInputPaths(const std::vector<tPath> &paths_in,
                                 std::vector<int> *path_assignments) {
    if (paths_in.empty() || !path_assignments) {
        printf(" LaneFinder: Path list is empty!\n");
        return false;
    }
    // printf("path in size %i\n",paths_in.size());
    path_assignments->clear();
    int curr_index = 0;
    int pnt_cnt = 0;
    float pnt_dist_sqared = 0;
    for (int i = 0; i < paths_in.size(); ++i) {
        bool isAdded = false;
        const tPath &current_path = paths_in[i];
        if (current_path.empty()) {
            isAdded = false;
        } else {
            float sq_dist =
                pow(current_path.front().x - current_path.back().x, 2) +
                pow(current_path.front().y - current_path.back().y, 2);
            if (current_path.size() > 2 &&
                sq_dist > MIN_SEGMENT_LENGTH_ * MIN_SEGMENT_LENGTH_) {
                isAdded = true;
                // printf("added segmnt with length %f\n", sqrt(sq_dist));
                pnt_cnt += current_path.size();
                pnt_dist_sqared += sq_dist; 
            }
        }
        if (isAdded) {
            path_assignments->push_back(curr_index);
            curr_index++;
        } else {
            path_assignments->push_back(-1);
        }
    }
    point_dist_squared_ = pnt_dist_sqared/(pnt_cnt+1);
    // printf("path out size %i\n", curr_index);
    // printf("_________________________\n");
    return true;
}

bool LaneFinder::solveQPs(const std::vector<tPath> &paths_in,
                          std::vector<Subproblem> *sub_vec_out, int order) {
    if (!sub_vec_out) {
        printf("LaneFinder::solveQPs failed.\n");
        return false;
    }
    sub_vec_out->clear();
    sub_vec_out->reserve(paths_in.size());
    for (int i = 0; i < paths_in.size(); ++i) {
        Subproblem sub(paths_in[i], order);
        sub.solveQP();
        sub_vec_out->push_back(sub);
    }
    return true;
}

/* Merge paths according to assignment list that maps
   paths to be merged onto same integer (0..max)
   and paths to be discarded to -1.
   NOTE: (Max. value + 1) determines number of output paths!*/
bool LaneFinder::mergePaths(const std::vector<tPath> &paths_in,
                            std::vector<tPath> *paths_out,
                            std::vector<int> *path_assignments,
                            std::vector<weight_struct> *weight_vec,
                            std::vector<Subproblem> *sub_vec) {
    if (!path_assignments || (paths_in.size() != path_assignments->size())) {
        printf("LaneFinder::mergePaths failed\n");
        return false;
    }

    int n_lanes =
        *max_element(path_assignments->begin(), path_assignments->end()) + 1;

    paths_out->resize(n_lanes);

    /* if weight_struct is given as arguement, we need weights
       of all previous subproblems in order to identify
       the indices of the merged path with the weights of the
       constituting subproblems */

    std::vector<float> weight_buff;
    if (weight_vec && sub_vec) {
        // a single weight_struct contains end indices and weights
        // to be applied until the respective end index
        weight_vec->resize(n_lanes);
        // weight_buff.resize(sub_vec->size(), 0);
        for (int i = 0; i < sub_vec->size(); ++i) {
            if ((*path_assignments)[i] == -1) continue;
            // don't compute weights twice.. store in buffer
            // weight_buff[i] = getPathWeight((*sub_vec)[i]);
            // printf("weight_buf %f\n", weight_buff[i]);
            // weight_sum += weight_buff[i];
        }
    }

    bool notEmpty = false;
    for (int i = 0; i < paths_in.size(); ++i) {
        if ((*path_assignments)[i] == -1) continue;
        int lane_idx = (*path_assignments)[i];
        if (weight_vec != NULL && sub_vec != NULL) {
            float weight = getPathWeight((*sub_vec)[i]);
            // printf("weight is %f\n",weight);
            (*weight_vec)[lane_idx].addStopidxWeight(paths_in[i].size(),
                                                     weight);
        }
        if (lane_idx >= paths_out->size()) {
            // printf("ERROR! lane idx out of bounds\n");
        } else
            (*paths_out)[lane_idx].insert((*paths_out)[lane_idx].end(),
                                          paths_in[i].begin(),
                                          paths_in[i].end());
        notEmpty = true;
        // printf("merged path %i into group %i \n", i, lane_idx);
    }

    return notEmpty;
}

float LaneFinder::getPathWeight(const Subproblem &sub) {
    if (sub.orientation_ == vertical) {
        // variance of prior +-sqrt(var) contains ~70% of prob. mass;
        float var = 0.2 * 0.2;
        // - use Gauss. mixture prior on lane position;
        //   rescale for better num. stability
        // - could also use prior on orientation
        float sum = 1e3 * sub.n_samples_ *
                    (exp(-0.5 / var * pow(sub.y_at_x0_ - 0.5 * LANEWIDTH_, 2)) +
                     exp(-0.5 / var * pow(sub.y_at_x0_ + 0.5 * LANEWIDTH_, 2)) +
                     exp(-0.5 / (3 * var) *
                         pow(sub.y_at_x0_ + 1.5 * LANEWIDTH_,
                             2)));  // higher variance for far left lane

        return sum;
    } else {
        return sub.n_samples_;
    }
}

// this method. tries to connect two lists by linearly extrapolating
// their end points. If both extensions "meet", the lists are assumed to belong
// together
bool LaneFinder::getFirstOrderAssignment(const std::vector<tPath> &paths_in,
                                         const std::vector<Subproblem> &sub_vec,
                                         std::vector<int> *path_assignments) {
    // printf("1std order ass.\n\n");

    if (!path_assignments) {
        printf("LaneFinder::getFirstOrderAssignment failed.\n");
        return false;
    }
    int n_paths = paths_in.size();
    path_assignments->resize(n_paths, -1);
    // i-->k, where i is index of the bottom segment/path
    std::vector<int> assignment_list(n_paths, -1);
    for (int i = 0; i < n_paths; i++) {
        // top point refers to point of segment with lowest x-value
        // this is compared to point with biggest x-val of another segment, the
        // 'bottom point'. The top point must me above bottom point.
        tPoint top_pnt;
        if (paths_in[i][0].x > paths_in[i].back().x)
            top_pnt = paths_in[i].back();
        else
            top_pnt = paths_in[i][0];

        for (int k = 0; k < n_paths; ++k) {
            if (i == k) break;
            tPoint bottom_pnt;
            if (paths_in[k][0].x < paths_in[k].back().x)
                bottom_pnt = paths_in[k].back();
            else
                bottom_pnt = paths_in[k][0];

            // "bottom" point must really be lower than top
            if (bottom_pnt.x - 0.01 < top_pnt.x) {
                float x_mean =
                    0.5 * (bottom_pnt.x + top_pnt.x);  // x of halfway-point
                 float slope_bottom = sub_vec[k].getDerivAtX(bottom_pnt.x);
                 float slope_top = sub_vec[i].getDerivAtX(top_pnt.x);
                // now extrapolate using a linear approximation based on the
                // 2nd-order model
                float y_error = sub_vec[i].getValAtX(x_mean) -
                                sub_vec[k].getValAtX(x_mean);  // EXPERIMENTAL

                //(top_pnt.y + (x_mean - top_pnt.x) * slope_top) -
                //(bottom_pnt.y + (x_mean - bottom_pnt.x) * slope_bottom);
                // printf("y_error is %f\n", y_error);
                if (fabs(y_error) <
                    MAX_EXTRAPOLATION_ERROR_   && fabs(atan(slope_top) -
                                                  atan(slope_bottom)) <
                                                  30*PI/180){
                    // bottom part points to path above with angle less than
                    // MAX_DELTA_ANGLE_RAD
                    assignment_list[k] = i;
                    if (dumpLog_) {
                        printf(
                            "1stOrder: Seg. %i below is pointing towards seg. "
                            "%i "
                            "\n",
                            k, i);
                    }
                    break;
                }
            }
        }
    }

    int segment_idx = 0;  // how many different segments remain after merging?
    std::vector<int> root_idx_vec;  // stores indices of segments that are root

    // now find the root segments (bottom-most), e.g. those that no other
    // segment is pointing to and that have a value different from -1
    for (int i = 0; i < n_paths; ++i) {
        bool isRoot = true;
        for (int k = 0; k < n_paths; ++k) {
            // is there a segment k that point to segment i?
            if (assignment_list[k] == i && k != i) {
                isRoot = false;
                break;
            }
        }
        if (isRoot) root_idx_vec.push_back(i);
    }
    // assign all segments their respective root index
    // assume no circular paths! (e.g. roots list is empty)
    for (int i = 0; i < root_idx_vec.size(); ++i) {
        int curr_segment_idx = root_idx_vec[i];
        // each root describes a new segment
        segment_idx += 1;
        while (curr_segment_idx != -1) {
            // assign current segment count to current segment (-1: starting
            // with 0)
            (*path_assignments)[curr_segment_idx] = segment_idx - 1;
            // get index of child segment
            curr_segment_idx = assignment_list[curr_segment_idx];
            // check for (extremely unlikely) loop
            if (curr_segment_idx == root_idx_vec[i]) {
                printf("1stOrderAssignment: loop detected. Skip\n");
                break;
            }
        }
    }
    return true;
}

/*Perform clustering of lanes. This is *activate buzzword-machine* "UNSUPERVISED
 * LEARNING" */
bool LaneFinder::getSecondOrderAssignment(
    const std::vector<Subproblem> &sub_vec,
    std::vector<int> *path_assignments) {
    if (!path_assignments) {
        printf("LaneFinder::getSecondOrderAssignment failed\n");
        return false;
    }
    int max_lane_idx = 0;
    // each POI is assigned to one cluster center
    std::vector<int> idx_list;
    // orientation and dist. of cluster centers stored here
    std::vector<LanePOI> center_list;
    int n_size = sub_vec.size();
    // first POI points to itself
    path_assignments->clear();
    path_assignments->push_back(0);
    std::vector<LanePOI> poi_list;
    // ---- Assign each path a POI, consisting of its
    // orientation and approx. distance to the car
    bool notEmpty = false;
    for (int i = 0; i < n_size; ++i) {
        LanePOI POI;
        if (sub_vec[i].orientation_ == vertical) {
            POI.orientation = vertical;
            POI.dist = sub_vec[i].y_at_x0_;
            POI.angle = atan(sub_vec[i].coeff_[1]);
            notEmpty = true;
            // printf("POI for vertical lane added with y (x=0) = %f\n",
            // sub_vec[i].y_at_x0_);
        } else if (sub_vec[i].orientation_ == horizontal) {
            POI.orientation = horizontal;
            POI.dist = sub_vec[i].vert_x_mean_;
            POI.angle = fabs(atan(sub_vec[i].coeff_[1]));
            notEmpty = true;
            // printf("POI for horizontal lane added with x (y=0) = %f\n",
            // sub_vec[i].vert_x_mean_);
        }
        poi_list.push_back(POI);
    }
    if (!notEmpty) return false;

    center_list.push_back(poi_list[0]);
    // ---- start with second POI and compare to all current roots
    for (int i = 1; i < n_size; ++i) {
        LanePOI &curr_element = poi_list[i];
        // std::cout << curr_element.y << "\n";
        bool isAssigned = false;
        int k = 0;  // start with first center
        while (!isAssigned && k <= max_lane_idx) {
            LanePOI &center_pnt = center_list[k];
            // curr. iterate must be of same orientation
            // and close enough to current center
            if ((center_pnt.orientation == curr_element.orientation) &&
                (fabs(center_pnt.dist - curr_element.dist) <
                 MIN_SEGMENT_DIST_) &&
                (fabs(center_pnt.angle - curr_element.angle) <
                 MAX_ANGLE_ERROR_)) {
                // update center
                center_pnt.dist = curr_element.dist;
                // associate current POI with center at index k
                path_assignments->push_back(k);
                isAssigned = true;
            }
            k++;
        }
        // if none of current center is close enough, point becomes its own
        // lane/root
        if (!isAssigned) {
            // add new lane with current POI as root
            max_lane_idx++;
            center_list.push_back(curr_element);
            path_assignments->push_back(max_lane_idx);
            isAssigned = 1;
        }
    }
    return true;
}

bool LaneFinder::processSubToLanes(const std::vector<Subproblem> &sub_vec,
                                   std::vector<Lane> *lane_vert_vec,
                                   std::vector<Lane> *lane_horiz_vec) {
    /* Sort subproblems right to left and bottom to top, creates corresponding
       lane object and fills pointed to lane vector with these lanes*/
    if (lane_vert_vec == NULL || lane_horiz_vec == NULL) {
        printf("LaneFinder::processSubToLanes failed\n");
        return false;
    }
    int n_size = sub_vec.size();
    std::vector<const Subproblem *> sub_vertical;
    std::vector<const Subproblem *> sub_horizontal;
    // separate by orientation and store address of subproblem
    for (int i = 0; i < n_size; ++i) {
        if (sub_vec[i].orientation_ == vertical) {
            sub_vertical.push_back(&(sub_vec[i]));
            // printf("class dist is %f\n", (*sub_vec)[i].y_at_x0_);
        } else if (sub_vec[i].orientation_ == horizontal) {
            sub_horizontal.push_back(&(sub_vec[i]));
        }
    }
    // sort vertical lanes from right to left. Insertion Sort efficient for
    // small n

    for (int i = 1; i < sub_vertical.size(); ++i) {
        const Subproblem *sub = sub_vertical[i];
        int j = i - 1;
        while (j >= 0 && (sub_vertical[j]->y_at_x0_ > sub->y_at_x0_)) {
            sub_vertical[j + 1] = sub_vertical[j];
            j--;
        }
        sub_vertical[j + 1] = sub;
    }
    // now sort horizontal lanes from bottom to top
    for (int i = 1; i < sub_horizontal.size(); ++i) {
        const Subproblem *sub = sub_horizontal[i];
        int j = i - 1;
        while (j >= 0 && sub_horizontal[j]->vert_x_mean_ > sub->vert_x_mean_) {
            sub_horizontal[j + 1] = sub_horizontal[j];
            j--;
        }
        sub_horizontal[j + 1] = sub;
    }
    // create lane objects
    for (int i = 0; i < sub_vertical.size(); ++i) {
        Lane lane;
        if (setLane(&lane, *sub_vertical[i])) {
            lane_vert_vec->push_back(lane);
            if (dumpLog_) printf("vertical lane with dist = %f\n", lane.dist);
        }
    }
    for (int i = 0; i < sub_horizontal.size(); ++i) {
        Lane lane;
        if (setLane(&lane, *sub_horizontal[i])) {
            if (dumpLog_) printf("horizontal lane with dist = %f\n", lane.dist);
            lane_horiz_vec->push_back(lane);
        }
    }
    // now check lane angle at origin and delete outlier
    // removeOutlierLanes(lane_vert_vec, true);
    // removeOutlierLanes(lane_horiz_vec, false);
    // float s1_mean = 0;
    // float s2_mean = 0;
    // int n = lane_vert_vec->size();

    // if (n > 0) {
    //   for (int  i = 0; i<lane_vert_vec->size(); ++i) {
    //     s1_mean +=(*lane_vert_vec)[i].coeff[1];
    //     s2_mean +=(*lane_vert_vec)[i].coeff[2];
    //   }
    //   s1_mean = s1_mean / n;
    //   s2_mean = s2_mean / n;
    //   for (int  i = 0; i<lane_vert_vec->size(); ++i) {
    //     (*lane_vert_vec)[i].coeff[1] = s1_mean;
    //     (*lane_vert_vec)[i].coeff[2] = s2_mean;
    //   }
    // }

    return (lane_horiz_vec->size() > 0 || lane_vert_vec->size() > 0);
    /*float stopline = detectStoplineDist(&sub_vertical, &sub_horizontal,
                                      lane_vert_vec, lane_horiz_vec);*/
    // printf("stopline detected at x = %f\n", stopline);
}


void LaneFinder::removeOutlierLanes(std::vector<Lane> *lane_vec,
                                    bool isVertical) {
    // If one lane found, don't remove it
    if (!lane_vec || lane_vec->size() <= 1) return;


   //lanes already sorted right to left
    if (isVertical) {
        for (int i = lane_vec->size() - 1; i >= 0 ; --i) {
            float y_dist = (*lane_vec)[i].dist;
            const tPoint& start_pnt = (*lane_vec)[i].start_pnt;
            if ( y_dist * start_pnt.y < 0 || start_pnt.x > 0.55) {
                lane_vec->erase(lane_vec->begin() + i);
                i--;
            }
        }
    }

   for (int i = lane_vec->size()-1; i > 0; --i) {
	float dist1_atdist = getValAtX(0.2, (*lane_vec)[i].coeff);
        float dist2_atdist = getValAtX(0.2, (*lane_vec)[i-1].coeff);
        //printf("dist 1 is %f, dist 2 %f\n", dist1_atdist, dist2_atdist);
        if (fabs(dist1_atdist-dist2_atdist) < 0.2) {
            //std::cout << "removeOutlierLanes: " << std::endl;
            float deriv1_atdist = getDerivAtX(0.5, (*lane_vec)[i].coeff);
            float deriv2_atdist = getDerivAtX(0.5, (*lane_vec)[i-1].coeff);
            if ( fabs(deriv1_atdist) > 
                 fabs(deriv2_atdist)) {
                lane_vec->erase(lane_vec->begin() + i);
            } else {
                 lane_vec->erase(lane_vec->begin() + i - 1);
            }
       }
    }
}

//fabs((*lane_vec)[i].angle_origin_rad) > 
                // fabs((*lane_vec)[i-1].angle_origin_rad)


bool LaneFinder::setLane(Lane *lane, const Subproblem &sub) {
    /* Sets attributes of lane object based on corresponding subproblem */
    if (lane == NULL) return false;
    lane->orientation = sub.orientation_;
    if (sub.orientation_ == vertical) {
        lane->dist = sub.y_at_x0_;
        lane->angle_origin_rad = atan(sub.getDerivAtX(0));
    } else {
        lane->dist = sub.vert_x_mean_;
        // use fabs to avoid angle jumps; -89deg ~= 91deg
        lane->angle_origin_rad = fabs(atan(sub.getDerivAtX(sub.vert_x_mean_)));
    }
    lane->coeff = sub.coeff_;
    lane->no_of_samples = sub.n_samples_;

    if (!setLimitPoints(sub, &(lane->start_pnt), &(lane->end_pnt))) {
        printf("setLane: failed setting lim points \n");
        return false;
    }
    if (dumpLog_) {
        printf("limit pnt bottom x is %f, top %f\n", lane->start_pnt.x,
               lane->end_pnt.x);
        printf("limit pnt bottom y is %f, top %f\n", lane->start_pnt.y,
               lane->end_pnt.y);
    }
    float length_sq = pow(lane->start_pnt.x - lane->end_pnt.x, 2) +
                      pow(lane->start_pnt.y - lane->end_pnt.y, 2);
    if ((sub.orientation_ == vertical &&
         length_sq > MIN_VERT_LANE_LENGTH_ * MIN_VERT_LANE_LENGTH_) ||
        (sub.orientation_ == horizontal &&
         length_sq > MIN_HORIZ_LANE_LENGTH_ * MIN_HORIZ_LANE_LENGTH_)) {
        return true;
    } else {
        if (dumpLog_) {
            printf("failed to get lane from subprob.; too short.\n");
        }
        return false;
    }
}

bool LaneFinder::setLimitPoints(const Subproblem &sub, tPoint *bottom_left,
                                tPoint *top_right) {
    /* in case parallel paths were merged into one using the 2ndOrderAssignment,
     first and last point in vector don't necessarily correspond
     to lane boundaries anymore. Not necessary after 1stOrderAssignment,
     though*/
    tPoint *pnt_min = NULL;  // &(sub->path_[0]);
    tPoint *pnt_max = NULL;  // &(sub->path_[0]);
    float min = 1e9;
    float max = -1e9;
    if (bottom_left == NULL || top_right == NULL) return false;
    tPath &sub_path = *sub.path_;
    if (sub.orientation_ == vertical) {
        for (int i = 0; i < sub_path.size(); ++i) {
            if (sub_path[i].x < min) {
                min = sub_path[i].x;
                pnt_min = &(sub_path[i]);
            }
            if (sub_path[i].x > max) {
                max = sub_path[i].x;
                pnt_max = &(sub_path[i]);
            }
        }
    } else if (sub.orientation_ == horizontal) {
        for (int i = 0; i < sub_path.size(); ++i) {
            // printf("min = %f, max = %f, y[i] = %f\n", min, max,
            // sub->path_[i].y);
            if (sub_path[i].y < min) {
                min = sub_path[i].y;
                pnt_max = &(sub_path[i]);
            }
            if (sub_path[i].y > max) {
                max = sub_path[i].y;
                pnt_min = &(sub_path[i]);
            }
        }
    }
    if (pnt_min != NULL && pnt_max != NULL) {
        *bottom_left = *pnt_min;
        *top_right = *pnt_max;
        return true;
    } else {
        std::cout << "setLimitPoints: Warning! no limit point assigned.\n";
        return false;
    }
}

float LaneFinder::detectStoplineDist(std::vector<Subproblem *> *sub_vert_vec,
                                     std::vector<Subproblem *> *sub_horiz_vec,
                                     std::vector<Lane> *lane_vert_vec,
                                     std::vector<Lane> *lane_horiz_vec) {
    /* Stopline found: returns distance to stopline. Else: -1.
       Subproblem and Lane indices correspond. */

    // check if at least horiz. lane detected

    if (lane_horiz_vec->size() == 0) return -1;

    // Find lane right next to car/
    float y_max = -1e9;
    int lane_right_idx = -1;
    for (int i = lane_vert_vec->size() - 1; i >= 0; --i) {
        float distance = (*lane_vert_vec)[i].dist;
        if (distance > y_max && distance < 0) {
            y_max = distance;
            lane_right_idx = i;
        }
    }

    if (lane_right_idx == -1) return -1;
    // x value of the stop line candidate
    float x_stop = (*lane_horiz_vec)[0].dist;
    // y_value of the intersection
    // if(&x_stop == NULL)printf("xstop = nulll ptrn\n");
    // printf("lane right idx = %i\n", lane_right_idx);
    float y_corner_intersect =
        (*sub_vert_vec)[lane_right_idx]->getValAtX(x_stop);
    // now check horiz. lane: if there is a connected region with y > y_corner
    // of certain length, this is our stopline!

    tPath &horizontal_path = (*(*sub_horiz_vec)[0]->path_);
    float y_dist_corner = 1e9;
    int closest_y_pnt_idx = -1;
    for (int i = 0; i < horizontal_path.size(); ++i) {
        if (fabs(horizontal_path[i].y - y_corner_intersect) <
            fabs(y_dist_corner)) {
            y_dist_corner = horizontal_path[i].y - y_corner_intersect;
            closest_y_pnt_idx = i;
        }
    }
    // printf("corner intersect at y =  %f, x_stop = %f\n", y_corner_intersect,
    // x_stop);
    // now we know closest point.y to intersection
    int k = closest_y_pnt_idx;
    // in neighbourhood of k: in which direction does y increase?
    // Assumption: Stop line goes from corner point to increasingly positive
    // y-value
    int direction = 0;
    if (k < horizontal_path.size() - 1)  // k is not end point
        direction =
            (horizontal_path[k + 1].y - horizontal_path[k].y > 0) ? 1 : -1;
    else if (k > 0)
        direction =
            (horizontal_path[k].y - horizontal_path[k - 1].y > 0) ? 1 : -1;
    // printf("inc y direct = %i", direction);
    float dist_sum = 0;
    float x_sum =
        0;  // stop line might be a bit offset from the whole detected lane
    int n_stop_samples = 0;
    float distance_inc = 0;

    int next_idx = k + direction;
    while (next_idx >= 0 && next_idx < horizontal_path.size() &&
           distance_inc < 0.1) {
        distance_inc =
            sqrt(pow(horizontal_path[next_idx].x - horizontal_path[k].x, 2) +
                 pow(horizontal_path[next_idx].y - horizontal_path[k].y, 2));
        // printf("distance inc is %f\n",distance_inc);
        if (distance_inc < 0.1) {
            dist_sum += distance_inc;
            n_stop_samples++;
            x_sum += horizontal_path[next_idx].x;
        }
        k = next_idx;
        next_idx = k + direction;
    }
    // max. stop_line length!?
    if (n_stop_samples == 0) return -1;
    if (dist_sum < 0.2) return x_sum / n_stop_samples;
    return 0;
}

/*
bool LaneFinder::detectCrossing(std::vector<Subproblem *> &sub_vert_vec,
                                std::vector<Subproblem *> &sub_horiz_vec,
                                tPose *crossing_proposal) {
    if (sub_horiz_vec.empty()) return false;
    // in this vector just put in all the gap points
    std::vector<corner_proposal> gap_corner_vec;
    // in this vec. add merged gaps (into a single corner)
    std::vector<corner_proposal> corner_vec;
    // tPath &horizontal_path = (*(sub_horiz_vec[0]->path_));
    // In first run, just determine position and orientation of gaps in detected
    // points list
    std::vector<corner_proposal> gap_corner_horiz_vec_return;
    std::vector<corner_proposal> gap_corner_vert_vec_return;
    // get corners from horizontal lanes
    for (int i = 0; i < sub_horiz_vec.size(); ++i) {
        getGapFromPath(*(sub_horiz_vec[i]->path_), horizontal,
                       &gap_corner_horiz_vec_return);
    }
    if (gap_corner_horiz_vec_return.empty()) return false;
    // get corners from vertical lanes
    for (int i = 0; i < sub_vert_vec.size(); ++i) {
        getGapFromPath(*(sub_vert_vec[i]->path_), vertical,
                       &gap_corner_vert_vec_return);
    }
    // compare all gaps with each other and if same position, determine corner
    // type
    std::cout << "LaneFinder: Deteced " << gap_corner_vert_vec_return.size()
              << "corners from vertical lanes";
    std::cout << "LaneFinder: Deteced " << gap_corner_horiz_vec_return.size()
              << "corners from horizontal lanes";
    return true;
}*/


/* How to debug:

   - Check if correct lanes are filtered out via distance
   - Check if dist. of points is computed correctly (for gap detection), adjust gap size
   - Check if mean of corners is valid for center calc.; fails if not symmetric
   - Check if dist. to horizontal lanes is close to intersection point; otherwise linearize
     around pseudo-intersection and refine by solving linear eq.*/
bool LaneFinder::detectCrossing(std::vector<Lane>& lane_vert_vec,
                                std::vector<Lane>& lane_horiz_vec,
                                tCrossing* detected_crossing,
                                std::vector<tPoint> *return_corner_pnts) {
    tPath corner_pnts;
    float direction_angle;
    if (!getIntersectingCorners(lane_vert_vec, lane_horiz_vec,
                                &corner_pnts, &direction_angle)) {
        detected_crossing->accuracy = 0.;
        return false;
    }

    detected_crossing->x = 0.;
    detected_crossing->y = 0.;
    detected_crossing->heading = direction_angle;
    ComputeCrossingCenter(corner_pnts, detected_crossing);

    detected_crossing->accuracy = 1.;
    //printf("LaneFinder: detected crossing with (x,y,phi) (%f %f %f)\n",
    //        detected_crossing->x, detected_crossing->y, detected_crossing->heading);
    for (int i = 0; i < corner_pnts.size(); ++i) {
    //    printf("corner %i: %fx, %fy \n", i, corner_pnts[i].x, corner_pnts[i].y);
    }
    (*return_corner_pnts) = corner_pnts;
    return true;
}

// works for two corner points
void LaneFinder::ComputeCrossingCenter(const tPath &corner_pnts,
                                    tCrossing *detected_crossing) {
    for (int i = 0; i < corner_pnts.size(); ++i) {
        detected_crossing->x += corner_pnts[i].x;
        detected_crossing->y += corner_pnts[i].y;
    }
    detected_crossing->x = detected_crossing->x / corner_pnts.size();
    detected_crossing->y = detected_crossing->y / corner_pnts.size();
    float dx = (corner_pnts[0].x - corner_pnts[1].x);
    float dy = (corner_pnts[0].y - corner_pnts[1].y);
    float alpha = std::atan(dx/dy);

    detected_crossing->x += 0.5 * std::cos(alpha);
    detected_crossing->y += 0.5 * std::sin(alpha);
}


// works only with x-limited ROI, so that max. 2 corner points can be detected
bool LaneFinder::getIntersectingCorners(std::vector<Lane>& lane_vert_vec,           // TODO(Jan) remove printfs
                                        std::vector<Lane>& lane_horiz_vec,
                                        tPath* corner_pnts,
                                        float* direction_angle) {
    tPath corner_candidates;
    std::vector<float> deriv_candidates_vec;
    float const CROSSING_DIST = 1.8*LANEWIDTH_;
    float const GAP_OVERLAP = 0.1;
    //float const SQR_DIST_MIN = CROSSING_DIST*CROSSING_DIST;
    if (lane_vert_vec.empty() || lane_horiz_vec.empty() || !corner_pnts ||
        !direction_angle) return false;

    std::vector<Lane*> laneptr_vert_vec;
    std::vector<Lane*> laneptr_horiz_vec;
    std::vector<float> deriv_vec;
    // filter out lanes that are too far away
    getLaneDistFilteredPtr(lane_vert_vec, &laneptr_vert_vec, CROSSING_DIST);
    getLaneDistFilteredPtr(lane_horiz_vec, &laneptr_horiz_vec, CROSSING_DIST);

    // calculate all intersection points
    for (int i = 0; i < laneptr_horiz_vec.size(); ++i) {
        // intersect_pnt.x = laneptr_horiz_vec[i]->dist;
        for (int k = 0; k < laneptr_vert_vec.size(); ++k) {
            tPoint intersect_pnt;
            if(!ComputeIntersectionPoint(laneptr_horiz_vec[i]->coeff[0],
                                    laneptr_horiz_vec[i]->coeff[1],
                                    laneptr_horiz_vec[i]->coeff[2],
                                    laneptr_vert_vec[k]->coeff[0],
                                    laneptr_vert_vec[k]->coeff[1],
                                    laneptr_vert_vec[k]->coeff[2],
                                    &intersect_pnt)) continue;
            //intersect_pnt.y = getValAtX(intersect_pnt.x,
            //                            laneptr_vert_vec[k]->coeff);
            float phi_h =  atan(getDerivAtX(intersect_pnt.x,
                                        laneptr_horiz_vec[i]->coeff));
            float phi_v =  atan(getDerivAtX(intersect_pnt.x,
                                        laneptr_vert_vec[k]->coeff));
            // compute angle difference between horiz and vertical
            if (phi_h < 0) {phi_h += 2*PI;}
            if (phi_v < 0) {phi_v += 2*PI;}
            float diff = fabs(phi_h - phi_v);
            float min_diff;
            if (diff > PI) {
                min_diff = fabs(2 * PI - diff);
            } else {
                min_diff = diff;
            }
            // printf("Intersection point with x, y, phi: %f %f %f \n",
            //         intersect_pnt.x, intersect_pnt.y, min_diff * 180. / PI);
            // filter out points not in ROI
            // filter out points without gap
            // filter out intersection points with angle lower than 65deg
            if (intersect_pnt.y < 2*LANEWIDTH_ && // lane in ROI
                    intersect_pnt.y > -LANEWIDTH_ &&

                    min_diff > 65.*PI/180.) { // and angle perpendicular //70
                //printf("Endpoint of list with x: %f \n",
                //           laneptr_vert_vec[k]->end_pnt.x);

                if ( (fabs(intersect_pnt.x - laneptr_vert_vec[k]->end_pnt.x) < GAP_OVERLAP) &&
                    (intersect_pnt.x < max_view_x_ - MIN_VERT_LANE_LENGTH_)) {  // intersection point with gap -> corner
                    corner_pnts->push_back(intersect_pnt);
                    deriv_vec.push_back(getDerivAtX(intersect_pnt.x,
                                                laneptr_vert_vec[k]->coeff));
                    //printf("Added corner point with x, y %f %f \n",
                    //       intersect_pnt.x, intersect_pnt.y);
                // remember intersec points without gap for t-crossing
                } else if (intersect_pnt.x - laneptr_vert_vec[k]->end_pnt.x < 0 && // endpunkt muss hinter intersection punkt liegen
                           intersect_pnt.x - laneptr_vert_vec[k]->start_pnt.x > 0.1) {  // intersection point musn't be start of a line
                    corner_candidates.push_back(intersect_pnt);
                    deriv_candidates_vec.push_back(getDerivAtX(intersect_pnt.x,
                                                laneptr_vert_vec[k]->coeff));
                    //printf("Added corner candidate point with x, y %f %f \n",
                    //       intersect_pnt.x, intersect_pnt.y);
                }
            }
        }
    }

    // has never been called during testing
    // (candidate close to intersection point ->wrong ip)
    // Remove corner point with close by corner candidate
    std::vector<bool> deletion_idx_vec(corner_pnts->size(), false);
    for (int i = corner_pnts->size() - 1; i >= 0; --i) {
        tPoint& ip = (*corner_pnts)[i];
        for (int c = 0; c < corner_candidates.size(); ++c) {
            if (fabs(ip.x - corner_candidates[c].x) < 0.05
                    && fabs(ip.y - corner_candidates[c].y) < 1.5*STOPLINE_OFFSET_
                    && ip.y < 0) { // only check the above criteria for points to the right
                deletion_idx_vec[i] = true;
                //printf("Erase corner point: P(%f;%f).\n", ip.x, ip.y);
            }
        }
    }
    for (int i = deletion_idx_vec.size()-1; i >= 0; --i) {
        if (deletion_idx_vec[i] == true) {
            corner_pnts->erase(corner_pnts->begin() + i);
            // keep deriv lanes vec synced with corners
            deriv_candidates_vec.erase(deriv_candidates_vec.begin() + i);
        }
    }

    /* Now we're left with points that have a gap */

    // match intersection point with candidate
    switch (corner_pnts->size()) {
        case 2: {
            // are the two ip 2 lanewidth apart?
            if (IsAtSameDistAnd2LanesApart((*corner_pnts)[0],
                                           (*corner_pnts)[1]))
                break;  // x crossing, everything fine

            printf("Two corners, IsAtSameDistAnd2LanesApart = false: ");

            // check if one of them has a partner
            int index = 0;
            if (FindPartnerIP((*corner_pnts)[0], corner_candidates, index)) {
                corner_pnts->push_back(corner_candidates[index]);
                deriv_vec.push_back(deriv_candidates_vec[index]);
                //printf("partner for corner found 1 \n");
                break;
            } else if (FindPartnerIP((*corner_pnts)[1],
                                       corner_candidates, index)) {
                corner_pnts->push_back(corner_candidates[index]);
                deriv_vec.push_back(deriv_candidates_vec[index]);
                //printf("partner for corner found 2 \n");
                break;
            } else {
                //printf("No partner for corner found. \n");
                return false;
            }
        } case 1: {
            // printf("One corner. ");
            // probably t crossing: find partner intersection point
            const tPoint& ip = (*corner_pnts)[0];
            int index = 0;
            if (!FindPartnerIP(ip, corner_candidates, index)){ return false;}
            // printf("partner found P(%f, %f).\n", corner_candidates[index].x, corner_candidates[index].y);
            corner_pnts->push_back(corner_candidates[index]);
            deriv_vec.push_back(deriv_candidates_vec[index]);
            break;
        } case 0:
            // no crossing found, return
            return false;
        default:
            // unexpected
            // printf("LaneFinder found %li intersection points with a gap.\n", corner_pnts->size());
            return false;
    }

    float slope_mean = 0.;
    for (int i = 0; i < deriv_vec.size(); ++i) {
        slope_mean += deriv_vec[i];
    }
    slope_mean = slope_mean / corner_pnts->size();
    if (fabs(slope_mean) > 1.) return false;
    *direction_angle = atan(slope_mean) + PI;
    return true;
}

// Partner IP means an intersection point that doesn't have a gap
// and IsAtSameDistAnd2LanesApart
bool LaneFinder::FindPartnerIP(const tPoint &ip,
                                const tPath &corner_candidates,
                                int candidates_index) {
    for (int i = 0; i < corner_candidates.size(); ++i) {
        const tPoint &cc = corner_candidates[i];
        // if on the same horizontal and 2 lanewidth apart
        if (IsAtSameDistAnd2LanesApart(ip, cc)) {
            // I expect this to happen only once
            candidates_index = i;
            // printf("Matched corner candidate without gap."
            //       "  P(%f,%f).\n", cc.x, cc.y);
            return true;
        }
    }
    return false;
}

bool LaneFinder::IsAtSameDistAnd2LanesApart(const tPoint &p1,
                                            const tPoint &p2) {
    if (fabs(fabs(p1.y-p2.y) - 2 * LANEWIDTH_) < 0.2
            && fabs(p1.x - p2.x) < 0.2)
        return true;
    return false;
}

// takes the coefficients of 2 lines and computes the intersection point
// assuming a quadratic function
// returns the intersection point that lies within x view distance 
bool LaneFinder::ComputeIntersectionPoint(float coeff_a1, float coeff_b1,
                            float coeff_c1, float coeff_a2, float coeff_b2,
                            float coeff_c2, tPoint *ip) {
    float a = coeff_a1 - coeff_a2;
    float b = coeff_b1 - coeff_b2;
    float c = coeff_c1 - coeff_c2;
    float sqare_root = std::sqrt(std::pow(b, 2) - 4*a*c);

    float x1 = 0.5 * (-b + sqare_root) / c;
    float x2 = 0.5 * (-b - sqare_root) / c;

    if (x1 <= max_view_x_ && x1 > 0)
        ip->x = x1;
    else if (x2 <= max_view_x_ && x2 > 0)
        ip->x = x2;
    else
        return false;
    ip->y = coeff_c2 * std::pow(ip->x, 2) + coeff_b2 * ip->x + coeff_a2;
    return true;
}


void LaneFinder::SetMaxViewX(float view) {
    max_view_x_ = view;
}

void LaneFinder::MeanFuseAdjacentCorners(const tPath &temp_corners,
                                    tPath* corner_pnts) {
    for (int c1 = 0; c1 < temp_corners.size(); ++c1) {
        for (int c2 = c1 + 1; c2 < temp_corners.size(); ++c2) {
            float dist_squared = pow(temp_corners[c1].x-temp_corners[c2].x, 2) +
                         pow(temp_corners[c2].y-temp_corners[c1].y, 2);
            if (dist_squared < 0.2*0.2) {
                tPoint mean;
                mean.x = 0.5*(temp_corners[c2].x + temp_corners[c1].x);
                mean.y = 0.5*(temp_corners[c2].y + temp_corners[c1].y);
                corner_pnts->push_back(mean);
            }
        }
    }
}

// input: vert. lane vec without dashed center lane
int LaneFinder::getNumberOfGaps(std::vector<Lane*>& lane_vert_vec,
                                float gap_dist) const {

    float dist_between_points = sqrt(point_dist_squared_);
    int gap_cnt = 0;
    for (int i = 0; i < lane_vert_vec.size(); ++i) {
        Lane& curr_lane = *lane_vert_vec[i];
        float lane_length =
            pow(curr_lane.start_pnt.x-curr_lane.end_pnt.x, 2) +
            pow(curr_lane.start_pnt.y-curr_lane.end_pnt.y, 2);
        float approx_marker_length = lane_length -
            dist_between_points*curr_lane.no_of_samples;
        if (lane_length - approx_marker_length > gap_dist) {
            gap_cnt++;
        }
    }
    return gap_cnt;
}

void LaneFinder::getLaneDistFilteredPtr(std::vector<Lane>& lane_vec,
                                        std::vector<Lane*>* lane_vecptr,
                                        float dist_min) {
    if (lane_vec.size() == 1) { //orientation_type vertical horizontal
        lane_vecptr->push_back(&lane_vec[0]);
        return;
    } else {
        std::vector<bool> is_added_vec(lane_vec.size(), false);
        for (int i = 0; i < lane_vec.size(); ++i) {
            for (int k = i+1; k < lane_vec.size(); ++k) {
                float dist = fabs(lane_vec[i].dist -
                                  lane_vec[k].dist);
                if (dist > dist_min) {
                    if (!is_added_vec[i]) {
                       lane_vecptr->push_back(&lane_vec[i]);
                        is_added_vec[i] = true;
                    }
                    if (!is_added_vec[k]) {
                       lane_vecptr->push_back(&lane_vec[k]);
                        is_added_vec[k] = true;
                    }
                }
            }
        }
    }
}

/*bool LaneFinder::getGapFromPath(tPath &point_vec,
                                orientation_type lane_orientation,
                                std::vector<corner_proposal> *gaps_detected) {
     // A gap is either at the start or end index when we'd expect more points to
     //   be detected beyond,
     //   but don't. Other way: List of points, with a segment missing inbetween (2
     //   gap points)
    const float DETECTION_DISTANCE_VERTICAL = 1.;
    // find gaps inbetween
    bool point_added = false;

    for (int i = 1; i < point_vec.size(); ++i) {
        float dist_to_last_squared =
            pow(point_vec[i].x - point_vec[i - 1].x, 2) +
            pow(point_vec[i].y - point_vec[i - 1].y, 2);
        if ((dist_to_last_squared > 1.8 * LANEWIDTH_ * LANEWIDTH_) &&
            (dist_to_last_squared < 2.2 * LANEWIDTH_ * LANEWIDTH_)) {
            lane_gap_type_enum first_point_type;
            lane_gap_type_enum second_point_type;
            if (lane_orientation == vertical) {
                if (point_vec[i].x - point_vec[i - 1].x > 0) {
                    first_point_type = LG_TO_FRONT;
                    second_point_type = LG_TO_BACK;
                } else {
                    first_point_type = LG_TO_BACK;
                    second_point_type = LG_TO_FRONT;
                }
                point_added = true;
            } else if (lane_orientation == horizontal) {
                if (point_vec[i].y - point_vec[i - 1].y > 0) {
                    first_point_type = LG_TO_LEFT;
                    second_point_type = LG_TO_RIGHT;
                } else {
                    first_point_type = LG_TO_RIGHT;
                    second_point_type = LG_TO_LEFT;
                }
                point_added = true;
            }
            if (point_added) {
                corner_proposal corner1(point_vec[i - 1].x, point_vec[i - 1].y,
                                        LG_UNKNOWN, first_point_type,
                                        C_UNKNOWN);
                corner_proposal corner2(point_vec[i].x, point_vec[i].y,
                                        LG_UNKNOWN, second_point_type,
                                        C_UNKNOWN);
                gaps_detected->push_back(corner1);
                gaps_detected->push_back(corner2);
            }
        }
    }
    // find point at start or end index that are not of image border (thus, gap
    // expected thereafter)
    // this will detect the closer part of a crossing when not the full crossing
    // is visible
    if (lane_orientation == vertical) {
        if (point_vec.front().x < DETECTION_DISTANCE_VERTICAL &&
            point_vec.back().x < DETECTION_DISTANCE_VERTICAL) {
            point_added = true;
            if (point_vec.front().x > point_vec.back().x) {
                corner_proposal corner_tmp(point_vec.back().x,
                                           point_vec.back().y, LG_UNKNOWN,
                                           LG_TO_FRONT, C_UNKNOWN);
                gaps_detected->push_back(corner_tmp);
            }
        }
    }
    return point_added;
}*/

float LaneFinder::getValAtX(float x,
                            const std::vector<float> &coeff) const {
    float res = 0;
    for (int i = 0; i < coeff.size(); ++i) {
        res += (coeff)[i] * pow(x, i);
    }
    return res;
}

float LaneFinder::getDerivAtX(float x,
                              const std::vector<float> &coeff) const {
    float res = 0;
    for (int i = 1; i < coeff.size(); ++i) {
        res += coeff[i] * pow(x, i - 1);
    }
    return res;
}
