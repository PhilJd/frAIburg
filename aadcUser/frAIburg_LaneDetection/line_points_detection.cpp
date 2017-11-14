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

#include "line_points_detection.h"
#include "adtf_log_macros.h"
#include "nonpin_types.h"

// _____________________________________________________________________________
LinePointsDetection::LinePointsDetection() {
}

// _____________________________________________________________________________
LinePointsDetection::~LinePointsDetection() {}

// _____________________________________________________________________________
void LinePointsDetection::DetectPoints(
    const cv::Mat& image, std::vector<std::vector<tPoint> >* detected_lines_v,
    std::vector<std::vector<tPoint> >* detected_lines_h,
    std::vector<tInt>* detection_lines) {
    std::vector<int> detection_lines_vertical;

    // get horizontal lines for finding vertical lane marking
    GetHorizontalDetectionLines(detection_lines);
    // find vertical lane marking

    // printf("LPD, size: %i\n",detected_lines->size());
    FindVerticalLines(*detection_lines, image, detected_lines_v);

    // get vertical lines for finding horizontal lane marking
    GetVerticalDetectionLines(&detection_lines_vertical);
    // find horizontal lane marking
    FindHorizontalLines(detection_lines_vertical, image, detected_lines_h);
}

// _____________________________________________________________________________
void LinePointsDetection::GetHorizontalDetectionLines(
    std::vector<tInt>* detection_lines) {
    int pixel_distance = MeterToPixel(filter_properties_.detection_distance);
    for (int i = filter_properties_.roi_offset_y;
         i <=
         filter_properties_.roi_offset_y + filter_properties_.roi_height - 1;
         i += pixel_distance) {
        detection_lines->push_back(i);
    }
}

// _____________________________________________________________________________
void LinePointsDetection::GetVerticalDetectionLines(
    std::vector<int>* detection_lines) {
    int pixel_dist = MeterToPixel(filter_properties_.detection_distance);

    for (int i = 0; i <= filter_properties_.roi_width - 1; i += pixel_dist) {
        detection_lines->push_back(filter_properties_.roi_offset_x + i);
    }
}

// _____________________________________________________________________________
void LinePointsDetection::FindVerticalLines(
    const std::vector<tInt>& detection_lines, const cv::Mat& img_transformed,
    std::vector<std::vector<tPoint> >* detected_lines) {
    bool is_vertical_scan = false;
    for (std::vector<tInt>::const_iterator line = detection_lines.begin();
         line < detection_lines.end(); line++) {
        // create vector with line data
        cv::Mat row_data =
            img_transformed(cv::Range(*line, (*line) + 1),
                            cv::Range(filter_properties_.roi_offset_x,
                                      filter_properties_.roi_offset_x +
                                          filter_properties_.roi_width));

        filter_properties_.min_line_contrast = 15;
        // std::max(20., std::min(0.5*stdev, 30.));
        /* min. lane brightness; Bounded by 50 and 130, between adaptive to
           account for shadows/dark images: */

        float abs_marking_min = 100;
        // std::min(std::max(50.,0.5*row_mean), 130.); //100
        /* road brightness below 130, or for very bright images adaptive */
        float abs_road_max = 255;  // std::max(0.7*row_mean, 130.);

        float brightness_change_start = 0;

        float sum = 0;
        int cnt = 0;
        tBool detected_start_of_line = false;
        int index_start_of_line = 0;
        int skip_val = 1;
        for (int current_index = kernel_offset_;
             current_index < filter_properties_.roi_width - kernel_offset_ - 1;
             current_index += skip_val) {
            cv::Mat sliding_window = row_data(
                cv::Range(0, 1), cv::Range(current_index - kernel_offset_,
                                           current_index + kernel_offset_ + 1));
            float brightness_change = Convolve(sliding_window);
            skip_val = 1;

            uchar current_pixel_val =
                row_data.at<uchar>(current_index);

            // as long as detected_start_of_line is true, sum up pixel values
            if (detected_start_of_line == true) {
                sum += current_pixel_val;
                cnt += 1;
            }
            // look for transition from dark to bright -> start of line corner
            if (brightness_change > filter_properties_.min_line_contrast &&
                detected_start_of_line == false) {
                detected_start_of_line = true;
                index_start_of_line = current_index;
                sum = current_pixel_val;
                cnt = 1;
                brightness_change_start = brightness_change;
                // look for transition from bright to dark -> end of line
            } else if (brightness_change <
                           (-1 * filter_properties_.min_line_contrast) &&
                       detected_start_of_line) {
                // check if width of detected line is in correct range
                bool is_greater_min_width =
                    ((current_index - index_start_of_line) >
                     MeterToPixel(filter_properties_.min_line_width));
                bool is_smaller_max_width =
                    ((current_index - index_start_of_line) <
                     MeterToPixel(filter_properties_.max_line_width));
                bool is_bright_enough = (sum / cnt > abs_marking_min);
                bool road_not_too_bright =
                    (sum / cnt - brightness_change_start < abs_road_max);
                if (is_greater_min_width && is_smaller_max_width &&
                    is_bright_enough && road_not_too_bright) {
                    skip_val = MeterToPixel(0.03);

                    /// current point from image coordinates
                    tPoint image_point, street_point;
                    image_point.y = *line;
                    image_point.x =
                        current_index -
                        ((current_index - index_start_of_line) / 2) +
                        filter_properties_.roi_offset_x;
                    transform_helper_.StreetImageToStreet(image_point,
                                                        &street_point);
                    // printf("Current point: %f , %f ;", current_point.x,
                    AddPointToLineVector(detected_lines, street_point,
                                         is_vertical_scan);
                    detected_start_of_line = false;
                    index_start_of_line = 0;
                    sum = 0;
                    cnt = 0;
                }
            }
            // we reached maximum line width limit, stop looking for end of line
            if (detected_start_of_line &&
                (current_index - index_start_of_line) >
                    MeterToPixel(filter_properties_.max_line_width)) {
                detected_start_of_line = false;
                index_start_of_line = 0;
                sum = 0;
                cnt = 0;
            }
        }
    }
}

// _____________________________________________________________________________
void LinePointsDetection::FindHorizontalLines(
    const std::vector<tInt>& detection_lines_vertical,
    const cv::Mat& img_transformed,
    std::vector<std::vector<tPoint> >* detected_lines) {
    bool is_vertical_scan = true;
    // iterate through the calculated vertical lines
    for (std::vector<int>::const_iterator line =
             detection_lines_vertical.begin();
         line != detection_lines_vertical.end(); ++line) {
        /// This column extraction and iterating over pixel values works
        cv::Mat column_data = img_transformed(
            cv::Range(filter_properties_.roi_offset_y,
                      filter_properties_.roi_offset_y +
                          filter_properties_.roi_height),
            cv::Range(*line, (*line) + 1));  // range last elem is exclusive

        // std::min(std::max(50.,0.5*col_mean), 130.);
        float abs_marking_min = 100;
        filter_properties_.min_line_contrast = 10;  // cv::sum(cv_col_std)[0];
        float brightness_change_start = 0;
        float abs_road_max = 250;
        float sum = 0;
        int cnt = 0;
        bool detected_start_of_line = false;
        int index_start_of_line = 0;
        int skip_val = 1;
        for (int current_index = kernel_offset_;
             current_index < filter_properties_.roi_height - kernel_offset_ - 1;
             current_index += skip_val) {
            skip_val = 1;
            uchar current_pixel_val = column_data.at<uchar>(current_index);

            // select pixels for convolution
            cv::Mat sliding_window =
                column_data(cv::Range(current_index - kernel_offset_,
                                      current_index + kernel_offset_ + 1),
                            cv::Range(0, 1));
            float brightness_change = Convolve(sliding_window);

            // look for transition from dark to bright -> start of line corner
            if (detected_start_of_line == true) {
                sum += current_pixel_val;
                cnt += 1;
            }
            if (brightness_change > filter_properties_.min_line_contrast &&
                detected_start_of_line == false) {
                detected_start_of_line = true;
                index_start_of_line = current_index;
                sum = current_pixel_val;
                cnt = 1;
                brightness_change_start = brightness_change;
            } else if (brightness_change <
                           (-1 * filter_properties_.min_line_contrast) &&
                       detected_start_of_line) {
                // we already have the start corner of line
                // , so check the width of detected line
                if (((current_index - index_start_of_line) >
                     MeterToPixel(filter_properties_.min_line_width)) &&
                    ((current_index - index_start_of_line) <
                     MeterToPixel(filter_properties_.max_line_width)) &&
                    sum / cnt > abs_marking_min &&
                    sum / cnt - brightness_change_start < abs_road_max) {
                    skip_val = MeterToPixel(0.03);
                    /*printf("convolve value %f, line width %d,
                     min_line_contrast %d, linemean %f, rowthreshold %f; \n",
                     brightness_change, (current_index - index_start_of_line),
                     filter_properties_.min_line_contrast, sum/cnt,
                     abs_marking_min);*/
                    tPoint image_point, street_point;
                    image_point.x = *line;
                    image_point.y =
                        current_index -
                        ((current_index - index_start_of_line) / 2) +
                        filter_properties_.roi_offset_y;
                    /// current point in meter and y-range = [+- width/2]
                    transform_helper_.StreetImageToStreet(image_point,
                                                        &street_point);

                    AddPointToLineVector(detected_lines, street_point,
                                         is_vertical_scan);
                    detected_start_of_line = false;
                    index_start_of_line = 0;
                    sum = 0;
                    cnt = 0;
                }
            }
            // we reached maximum line width limit, stop looking for EOL
            if (detected_start_of_line &&
                (current_index - index_start_of_line) >
                    MeterToPixel(filter_properties_.max_line_width)) {
                detected_start_of_line = false;
                index_start_of_line = 0;
                sum = 0;
                cnt = 0;
            }
        }
    }
}

// _____________________________________________________________________________
void LinePointsDetection::AddPointToLineVector(
    std::vector<std::vector<tPoint> >* detected_lines,
    const tPoint& current_point, bool is_vertical_scan) {
    // filter points on engine hood
    if (current_point.x < 0.34 && fabs(current_point.y) < 0.46) {  // TODO(Jan) can be a bit lower with the cropped basler I guess
        return;
    }
    if (detected_lines->size() == 0) {
        std::vector<tPoint> temp_vector;
        temp_vector.push_back(current_point);
        detected_lines->push_back(temp_vector);
        return;
    }

    // filter out vertical points that are too far to the right
    // to avoid wrong lanes from roadsigns / qr-codes
    //if (!is_vertical_scan && current_point.y < -LANEWIDTH_) {
    //    return;
    //}

    // check against vectors in vector detected_lines
    // iterate over vectors and remember only the shortest distance of all
    // last points to the current point

    // v1
    // remember all goal lists with a point close enough
    //tFloat32 min_dist = 0.05;
    std::vector<std::vector<tPoint> >::iterator goal_list;
    std::vector<std::vector<std::vector<tPoint> >::iterator> goal_lists_ptr;
    //std::vector<std::vector<tPoint>*> goal_lists_ptr;



    for (std::vector<std::vector<tPoint> >::iterator line =
             detected_lines->begin();
         line != detected_lines->end(); ++line) {

        tPoint& end_point = (*line)[line->size() - 1];  // last point in list
        tFloat32 dist = sqrt(pow(current_point.x - end_point.x, 2) +
                             pow(current_point.y - end_point.y, 2));

        float scan_dist;
        if (is_vertical_scan)
            scan_dist = fabs(end_point.x - current_point.x);
        else
            scan_dist = fabs(end_point.y - current_point.y);

        // check distance
        if (dist > 2.3 * filter_properties_.detection_distance ||
            0.5*scan_dist > filter_properties_.detection_distance) continue;
        // check angle
        if (line->size() > 1) {
            tPoint& before_end_point = (*line)[line->size() - 2];
            float dx = end_point.x - before_end_point.x;
            float dy = end_point.y - before_end_point.y;
            float angle_to_end = atan2(dy, dx);
            dx = current_point.x - end_point.x;
            dy = current_point.y - end_point.y;
            float angle_to_current = atan2(dy, dx);
            if (fabs(angle_to_current - angle_to_end) < 25 * PI / 180) 
                goal_lists_ptr.push_back(line);
        } else {
            goal_lists_ptr.push_back(line);
        }
    }

    for (int i = 0; i < goal_lists_ptr.size(); ++i) {
        goal_lists_ptr[i]->push_back(current_point);
    }
    if(goal_lists_ptr.empty()) {
        std::vector<tPoint> new_vector;
        new_vector.push_back(current_point);
        detected_lines->push_back(new_vector);
    }

    // v2
//    tFloat32 shortest_distance = filter_properties_.roi_width;
//    std::vector<std::vector<tPoint> >::iterator goal_list;
//    for (std::vector<std::vector<tPoint> >::iterator line =
//             detected_lines->begin();
//         line != detected_lines->end(); ++line) {
//        tPoint list_point = (*line)[line->size() - 1];  // last point in list
//        tFloat32 dist = sqrt(pow(current_point.x - list_point.x, 2) +
//                             pow(current_point.y - list_point.y, 2));
//        if (dist < shortest_distance) {
//            shortest_distance = dist;
//            // remember list
//            goal_list = line;
//        }
//    }
//    // add point to vector with closest point
//    // or create new list
//    float scan_dist;
//    if (is_vertical_scan)
//        scan_dist = fabs(goal_list->back().x - current_point.x);
//    else
//        scan_dist = fabs(goal_list->back().y - current_point.y);
//
//    if (shortest_distance > 2.3 * filter_properties_.detection_distance // allows for a gap of one detection line
//        || 0.5 * scan_dist > filter_properties_.detection_distance) { // or more than 45deg
//        std::vector<tPoint> new_vector;
//        new_vector.push_back(current_point);
//        detected_lines->push_back(new_vector);
//    } else {
//        goal_list->push_back(current_point);
//    }
}

// ____________________________________________________________________________
void LinePointsDetection::SetRoiOffsetX(int roi_offset_x) {
    filter_properties_.roi_offset_x = roi_offset_x;
}

// ____________________________________________________________________________
void LinePointsDetection::SetRoiOffsetY(int roi_offset_y) {
    filter_properties_.roi_offset_y = roi_offset_y;
}

// ____________________________________________________________________________
void LinePointsDetection::SetRoiWidth(int roi_width) {
    filter_properties_.roi_width = roi_width;
}

// ____________________________________________________________________________
void LinePointsDetection::SetRoiHeight(int roi_height) {
    filter_properties_.roi_height = roi_height;
}

// ____________________________________________________________________________
void LinePointsDetection::SetDetectionDistance(float detection_distance) {
    filter_properties_.detection_distance = detection_distance;
}

// ____________________________________________________________________________
void LinePointsDetection::SetMinLineWidth(float min_line_width) {
    filter_properties_.min_line_width = min_line_width;
}

// ____________________________________________________________________________
void LinePointsDetection::SetMaxLineWidth(float max_line_width) {
    filter_properties_.max_line_width = max_line_width;
}

// ____________________________________________________________________________
void LinePointsDetection::SetMinLineContrast(int min_line_contrast) {
    filter_properties_.min_line_contrast = min_line_contrast;
}

// ____________________________________________________________________________
int LinePointsDetection::MeterToPixel(float meter) {
    if ((meter * transform_helper_.GetScale()) < 1.0 && meter > 0.0){
        LOG_WARNING_PRINTF("LinePointsDetection: meter * ipm_scale_ < 1; "
            "With current scale some parameters have sub-pixel size!");
    }
    return static_cast<int>(meter * transform_helper_.GetScale());
}

// ____________________________________________________________________________
float LinePointsDetection::Convolve(const cv::Mat& sliding_window) {
    float convolved = 0;

    for (int i = 0; i < kernel_size_; ++i) {
        // implement as dot product if possible
        // note pixels[pixelpos-i] for true convolution
        convolved += kernel_[i] * sliding_window.at<uchar>(i);
    }
    return convolved / kernel_norm_;
}

// ____________________________________________________________________________
void LinePointsDetection::SetCameraTransformationsObj(
                                    const CameraTransformations &transform) {
    transform_helper_ = transform;
}
