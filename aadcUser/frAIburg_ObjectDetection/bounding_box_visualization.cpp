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

#include "bounding_box_visualization.h"
#include <iomanip>              // setprecision
#include <iostream>             // std::cout
#include <sstream>              // stringstream
#include "opencv2/imgproc.hpp"  // functions to draw on image

// ____________________________________________________________________________
void BoundingBoxVisualization::DrawBoundingBoxes(
    cv::Mat* img, const std::vector<cv::Rect>& boxes,
    const std::vector<std::string>& labels, const cv::Scalar& color,
    int thickness, FontStyle style) {
    if (boxes.size() != labels.size()) {
        return;
    }
    for (size_t i = 0; i < boxes.size(); ++i) {
        cv::rectangle(*img, boxes[i], color, thickness);  // bounding box
        int baseline = 0;  // unused output parameter of getTestSize
        cv::Size text_size = cv::getTextSize(labels[i], style.face, style.scale,
                                             style.thickness, &baseline);
        cv::Rect text_rectangle(boxes[i].tl(), text_size + cv::Size(6, 6));
        cv::rectangle(*img, text_rectangle, color, CV_FILLED);  // text rect
        cv::putText(*img, labels[i],
                    boxes[i].tl() + cv::Point(2, 2 + text_size.height),
                    style.face, style.scale, style.color, style.thickness);
    }
}

// ____________________________________________________________________________
void BoundingBoxVisualization::DrawBoundingBoxes(
    cv::Mat* img, const tf::Tensor& boxes, const tf::Tensor& class_ids,
    float min_score, const tf::Tensor& scores,
    const std::map<int, std::string>& id_to_label, const cv::Scalar& color,
    int thickness, FontStyle style) {
    std::vector<cv::Rect> cvboxes;
    std::vector<std::string> labels;
    int num_detections = GetValidDetections(min_score, scores);
    int tmp_shape_array[] = {0, 0};
    std::vector<size_t> idx(tmp_shape_array, tmp_shape_array + 2);
    for (int i = 0; i < num_detections; ++i) {
        idx[1] = i;
        cvboxes.push_back(TfBoxesToCvRect(boxes, i, img->size()));
        int label_id = static_cast<int>(class_ids.get<float>(idx, false));
        float score = scores.get<float>(idx, false);
        labels.push_back(GetLabel(score, label_id, id_to_label));
    }
    DrawBoundingBoxes(img, cvboxes, labels, color, thickness, style);
}

// ____________________________________________________________________________
int BoundingBoxVisualization::GetValidDetections(float min_score,
                                                 const tf::Tensor& scores) {
    int tmp_shape_array[] = {0, 0};
    std::vector<size_t> idx(tmp_shape_array, tmp_shape_array + 2);
    for (int i = 0; i < scores.Shape()[1]; ++i) {
        idx[1] = i;
        if (scores.get<float>(idx, false) < min_score) {
            std::cout << "Num valid detections:" << i << std::endl;
            return i;
        }
    }
    return scores.Shape()[1];
}

// ____________________________________________________________________________
cv::Rect BoundingBoxVisualization::TfBoxesToCvRect(const tf::Tensor& boxes,
                                                   int box_idx,
                                                   const cv::Size& img_size) {
    float width = static_cast<float>(img_size.width);
    float height = static_cast<float>(img_size.height);
    float min_x = boxes.get<float>(0, box_idx, 1, false);
    float min_y = boxes.get<float>(0, box_idx, 0, false);
    float max_x = boxes.get<float>(0, box_idx, 3, false);
    float max_y = boxes.get<float>(0, box_idx, 2, false);
    return cv::Rect(cv::Point(static_cast<int>(width * min_x),
                              static_cast<int>(height * min_y)),
                    cv::Point(static_cast<int>(width * max_x),
                              static_cast<int>(height * max_y)));
}

// ____________________________________________________________________________
std::string BoundingBoxVisualization::GetLabel(
    float score, int label_id, const std::map<int, std::string>& id_to_label) {
    std::stringstream stream;
    try {
        stream << id_to_label.at(label_id);
    } catch (const std::out_of_range&) {
        std::cout << "No label defined for class id " << label_id << "."
                  << std::endl;
        stream << "Class id:" << label_id;
    }
    stream << " " << std::fixed << std::setprecision(2) << score;
    return stream.str();
}