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
#ifndef AADCUSER_FRAIBURG_OBJECTDETECTION__BOUNDING_BOX_VISUALIZATION_H_
#define AADCUSER_FRAIBURG_OBJECTDETECTION__BOUNDING_BOX_VISUALIZATION_H_

#include <map>
#include <string>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include "tensorflow_cpp_wrapper.h"

namespace tf = tensorflow;


struct FontStyle {
    cv::Scalar color;
    double scale;
    int thickness;
    int face;
    // constructors
    FontStyle(cv::Scalar c, double s) : color(c), scale(s),
        thickness(1), face(cv::FONT_HERSHEY_DUPLEX) {}
    FontStyle(cv::Scalar c, double s, int t) : color(c), scale(s),
        thickness(t), face(cv::FONT_HERSHEY_DUPLEX) {}
};


class BoundingBoxVisualization {
 public:
  /*! Draws the specified bounding boxes with labels on the image. */
  static void DrawBoundingBoxes(cv::Mat* img,
                                const std::vector<cv::Rect>& boxes,
                                const std::vector<std::string>& labels,
                                const cv::Scalar& color,
                                int thickness,
                                FontStyle style);

    /*! Draws the specified bounding boxes with labels on the image. */
    static void DrawBoundingBoxes(cv::Mat* img,
                                  const tf::Tensor& boxes,
                                  const tf::Tensor& class_ids,
                                  float min_score,
                                  const tf::Tensor& scores,
                                  const std::map<int, std::string>& id_to_label,
                                  const cv::Scalar& color,
                                  int thickness,
                                  FontStyle style);

    /*! Returns the number of detections with score > min_score */
    static int GetValidDetections(float min_score, const tf::Tensor& scores);

    /*! Converts float[4] array containing relative coordiantes to cv::Rect */
    static cv::Rect TfBoxesToCvRect(const tf::Tensor& boxes,
                                    int box_idx,
                                    const cv::Size& img_size);

    /*! Creates a label string from class label and score. */
    static std::string GetLabel(float score, int label_id,
                                const std::map<int, std::string>& id_to_label);


};

#endif  // AADCUSER_FRAIBURG_OBJECTDETECTION__BOUNDING_BOX_VISUALIZATION_H_