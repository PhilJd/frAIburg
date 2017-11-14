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
#ifndef AADCUSER_FRAIBURG_LANEDETECTIONFILTER_LINEPOINTSDETECTION_H_
#define AADCUSER_FRAIBURG_LANEDETECTIONFILTER_LINEPOINTSDETECTION_H_

#include "stdafx.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "customtypes.h"
//#include "xml_helper.hpp"
#include "camera_transformations.h"

/*// For working on laptop:
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <numeric>
struct tPoint{
     float x;
     float y;
};
typedef int tInt;
typedef bool tBool;
typedef float tFloat32;*/

/* kernel for horizontal line detections*/
// static const float kernel_[] = {-3, -1, 0, 1, 3};
// static const int kernel_size_ = 5;
// static const int kernel_offset_ = 2;
// static const int kernel_norm_ = 4;

/* kernel for both line detections*/
// maybe make kernel even one element shorter
// static const float kernel_[] = {-3, -3, -1, 0, 1, 3, 3};
// static const int kernel_size_ = 7;
// static const int kernel_offset_ = 3;
// static const int kernel_norm_ = 7;

static const float kernel_[] = {-3, -3, -1, 0, 1, 3, 3};
static const int kernel_size_ = 7;
static const int kernel_offset_ = 3;
static const int kernel_norm_ = 7;


class LinePointsDetection {
 public:
    /**
     * Constructor. Pass Filter Properties
     */
    LinePointsDetection();

    ~LinePointsDetection();

    /**
     * This method does all the necessary steps to detect
     * points on the lane marking.
     */
    void DetectPoints(const cv::Mat &image,
                      std::vector<std::vector<tPoint> >* detected_lines_v,
                      std::vector<std::vector<tPoint> >* detected_lines_h,
                      std::vector<tInt>* detection_lines);

    /*!
     * update the filter properties from lane_detection filter
     * Set methods
     */
    void SetRoiOffsetX(int roi_offset_x);
    void SetRoiOffsetY(int roi_offset_y);
    void SetRoiWidth(int roi_width);
    void SetRoiHeight(int roi_height);
    void SetDetectionDistance(float detection_distance);
    void SetMinLineWidth(float min_line_width);
    void SetMaxLineWidth(float max_line_width);
    void SetMinLineContrast(int min_line_contrast);

    /*!
     * Get the TransformationReferencePoints from the config file
     */
    void SetCameraTransformationsObj(
                            const CameraTransformations &transform);

    /*!
     * convert the given value in meter to the corresponding pixel value
     * using transformation parameters
     */
    int MeterToPixel(float meter);

 private:
    CameraTransformations transform_helper_;

    /*! the struct with all the properties*/
    struct FilterProperties {
        /*! Offset of the ROI in the Stream*/
        int roi_offset_x;
        /*! Offset of the ROI in the Stream*/
        int roi_offset_y;
        /*! Width of the ROI*/
        int roi_width;
        /*! Height of the ROI*/
        int roi_height;
        /*! number of detection lines searched in ROI */
        float detection_distance;
        /*! Minimum Line Width in Meter */
        float min_line_width;
        /*! Maximum Line Width in Meter */
        float max_line_width;
        /*! Mimimum line contrast in gray Values */
        int min_line_contrast;
    } filter_properties_;

    /*
     * split the ROI into horizontal lines of equal distance
     * adds the row index of the horizontal line to the vector
     */
    void GetHorizontalDetectionLines(std::vector<tInt>* detection_lines);

    /**
     * compute vertical detection lines
     * integer values represent vertical lines from left to right
     */
    void GetVerticalDetectionLines(std::vector<tInt>* detection_lines);

    /*
     * go horizontally over image and detect white line marking
     * points that belong to the same vertical line are in the same vector
     */
    void FindVerticalLines(const std::vector<tInt> &detection_lines,
                           const cv::Mat &img_transformed,
                           std::vector<std::vector<tPoint> >* detected_lines);

    /*
     * go vertically over the image and detect white line marking
     * points that belong to the same horizontal line are in the same vector
     */
    void FindHorizontalLines(const std::vector<tInt> &detection_lines_vertical,
                            const cv::Mat &img_transformed,
                            std::vector<std::vector<tPoint> >* detected_lines);

    /*!
     * add point to line vector
     */
    void AddPointToLineVector(std::vector<std::vector<tPoint> >* detected_lines,
                              const tPoint &current_point,
                              bool is_vertical_scan);

    /*!
     * Convolves the vector (image data) with the kernel
     * for now they should have the same size
     */
    float Convolve(const cv::Mat &sliding_window);
};

#endif  // AADCUSER_FRAIBURG_LANEDETECTION_LINE_POINTS_DETECTION_H_
