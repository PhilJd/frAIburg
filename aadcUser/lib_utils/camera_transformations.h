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

#ifndef AADCUSER_LIB_UTILS_CAMERA_TRANSFORMATIONS_H_
#define AADCUSER_LIB_UTILS_CAMERA_TRANSFORMATIONS_H_

#include "opencv2/core/core.hpp"
#include "customtypes.h"

class CameraTransformations {
 public:
    CameraTransformations() {}

    /*! Constructor, the XML file at xml_file_path must contain a section
     *  "sensor" where the camera_name is listed. */
    CameraTransformations(const std::string& xml_file_path,
                          const std::string& camera_name);

    /*! Overloaded function. Transforms the given points in floating point
     *  representation from pixel space to the street plane. */
    void PixelToStreet(const std::vector<cv::Point2f>& source,
                       std::vector<cv::Point2f>* transformed);

    /*! Overloaded function. Transforms the given points in integer point
     *  representation from pixel space to the street plane. */
    void PixelToStreet(const std::vector<cv::Point2i>& source,
                       std::vector<cv::Point2f>* transformed);

    /*! Overloaded function. 
     *  Computes the world coordinates from an image that was already projected
     *  to the street plane, i.e. to obtain the points in the cars coordinate
     *  system the pixels of this image are shifted and scaled to meters.
     *  this also does the flipping from I(w|h) to P_street(x|y) */
    void StreetImageToStreet(const std::vector<cv::Point2f>& source,
                             std::vector<cv::Point2f>* transformed);
    /*! Overloaded function.
     *  Computes the world coordinates from an image that was already projected
     *  to the street plane, i.e. to obtain the points in the cars coordinate
     *  system the pixels of this image are shifted and scaled to meters.
     *  this also does the flipping from I(w|h) to P_street(x|y)
     */
    void StreetImageToStreet(const tPoint& image_point, tPoint* street_point);

    /*! Overloaded function.
     *  Computes the world coordinates from an image that was already projected
     *  to the street plane, i.e. to obtain the points in the cars coordinate
     *  system the pixels of this image are shifted and scaled to meters. */
    void StreetImageToStreet(const cv::Point& image_point,
                            cv::Point2f* street_point);


    /*! Overloaded function. 
     *  Computes the image coordinates in pixels from a point in the cars
     *  coordinate system. i.e. the detected points are shifted and scaled back
     *  onto the image for debugging.
     *  this also does the flipping from P_street(x|y) to I(w|h) */
    void StreetToStreetImage(const tPoint& street_point, cv::Point* image_point);

    /*! Overloaded function. 
     *  Computes the image coordinates in pixels from a point in the cars
     *  coordinate system. i.e. the detected points are shifted and scaled back
     *  onto the image for debugging.
     *  this also does the flipping from P_street(x|y) to I(w|h) */
    void StreetToStreetImage(float x, float y, cv::Point* image_point);

    /*! Transforms an image to the street plane and shifts + scales the image
     *  to have only positive coordinates, so it can be displayed. */
    void PixelToStreetImage(const cv::Mat &input_image, cv::Mat* output_image);

    /*! Retruns the scale that is used for the IPM in [pixels per meter] */
    float GetScale();

    /*! Returns true if the camera matrix was successfully initialized. */
    bool Initialized();

 private:
    /*! Reads the transformation matrix from the xml file into the
     *  pixel_to_street_transformation_ member.*/
    void ReadTransformationMatrixFromXml(const std::string& xml_file_path);

    /*! Holds the transformation from the pixel space to the street plane. */
    cv::Mat pix_to_street_;

    /*! Holds the transformation from the pixel space to the shifted street
     *  plane, i.e. the street plane shifted on the y-axis to be only
     *  positive. */
    cv::Mat pix_to_streetimage_;

    /*! The sensor name, must match a sensor name in the xml file
     *  used to initialize. */
    std::string camera_name_;

    /*! The width of the inverse perspective mapping in meters.
     *  Is read from the xml file. */
    float ipm_width_;

    /*! The height of the inverse perspective mapping in meters.
     *  Is read from the xml file. */    
    float ipm_height_;

    /*! The scaling that is applied to the transform, influences the size of
     *  the resulting top down image. */
    float ipm_scale_;

    /*! The offset that is applied to the points before transform.
        Influences the shift of the resulting top down image. */
    float ipm_street_img_offset_;

    /*! The output image dimensions for the inverse perspective mapping. Is
     *  computed as
     *  cv::Size(ipm_width_ * ipm_scale_, ipm_height_ * ipm_scale_) */
    cv::Size ipm_image_dims_;
};

#endif  // AADCUSER_LIB_UTILS_CAMERA_TRANSFORMATIONS_H_
