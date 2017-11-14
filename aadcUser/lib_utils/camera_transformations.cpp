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

#include "camera_transformations.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "xml_helper.hpp"

// ____________________________________________________________________________
CameraTransformations::CameraTransformations(const std::string& xml_file_path,
                                             const std::string& camera_name) {
    camera_name_ = camera_name;
    ReadTransformationMatrixFromXml(xml_file_path);
}

// ____________________________________________________________________________
void CameraTransformations::ReadTransformationMatrixFromXml(
    const std::string& xml_file_path) {
    frAIburg::utils::XMLHelper xml_reader;
    std::cout << xml_file_path << " camera name:" << camera_name_ << std::endl;
    if (!xml_reader.ReadNameValue(xml_file_path.c_str(), "sensor",
                                  camera_name_.c_str())) {
        LOG_WARNING_PRINTF("CameraTransformations: Could not read XML file.");
        return;
    }
    ipm_width_ = *(xml_reader.GetValue<float>("ipm_width"));
    ipm_height_ = *(xml_reader.GetValue<float>("ipm_height"));
    ipm_scale_ = *(xml_reader.GetValue<float>("ipm_scale"));
    ipm_street_img_offset_ =
            *(xml_reader.GetValue<float>("ipm_street_img_offset"));
    ipm_image_dims_ = cv::Size(ipm_width_ * ipm_scale_,
                               ipm_height_ * ipm_scale_);
    cv::Point2f src0(*(xml_reader.GetValue<float>("src_1y")),
                     *(xml_reader.GetValue<float>("src_1x")));
    cv::Point2f src1(*(xml_reader.GetValue<float>("src_2y")),
                     *(xml_reader.GetValue<float>("src_2x")));
    cv::Point2f src2(*(xml_reader.GetValue<float>("src_3y")),
                     *(xml_reader.GetValue<float>("src_3x")));
    cv::Point2f src3(*(xml_reader.GetValue<float>("src_4y")),
                     *(xml_reader.GetValue<float>("src_4x")));
    cv::Point2f dst0(*(xml_reader.GetValue<float>("dst_1y")),
                     *(xml_reader.GetValue<float>("dst_1x")));
    cv::Point2f dst1(*(xml_reader.GetValue<float>("dst_2y")),
                     *(xml_reader.GetValue<float>("dst_2x")));
    cv::Point2f dst2(*(xml_reader.GetValue<float>("dst_3y")),
                     *(xml_reader.GetValue<float>("dst_3x")));
    cv::Point2f dst3(*(xml_reader.GetValue<float>("dst_4y")),
                     *(xml_reader.GetValue<float>("dst_4x")));
    cv::Point2f source[] = {src0, src1, src2, src3};
    cv::Point2f destination[] = {dst0, dst1, dst2, dst3};
    pix_to_street_ = cv::getPerspectiveTransform(source, destination);
    pix_to_street_.convertTo(pix_to_street_, CV_32F);
    cv::Point2f destination_shifted[4];
    cv::Point2f shift(ipm_street_img_offset_, 0);
    for (int i = 0; i < 4; ++i) {
        destination_shifted[i] = (destination[i] + shift) * ipm_scale_;
    }
    pix_to_streetimage_ = cv::getPerspectiveTransform(source,
                                                      destination_shifted);
    pix_to_streetimage_.convertTo(pix_to_streetimage_, CV_32F);
}

// ____________________________________________________________________________
bool CameraTransformations::Initialized() { return !pix_to_street_.empty(); }
// ____________________________________________________________________________
void CameraTransformations::PixelToStreet(
        const std::vector<cv::Point2f>& source,
        std::vector<cv::Point2f>* transformed) {
    cv::perspectiveTransform(source, *transformed, pix_to_street_);
}

// ____________________________________________________________________________
//void CameraTransformations::StreetImageToStreet(
//        const std::vector<cv::Point2f>& source,
//        std::vector<cv::Point2f>* transformed) {
//    transformed->clear();
//    transformed->reserve(source.size());
//    cv::Point2f shift(-ipm_street_img_offset_, 0);  // _-0.5 * ipm_width_
//    for (size_t i = 0; i < source.size(); ++i) {
//        transformed->push_back((source[i] / ipm_scale_) + shift);
//    }
//}

// ____________________________________________________________________________
void CameraTransformations::StreetImageToStreet(const tPoint& image_point,
                                            tPoint* street_point) {
    // this does also the flipping from I(w|h) to P_street(x|y)
    street_point->y = (image_point.x / ipm_scale_) - ipm_street_img_offset_;
    street_point->x = (image_point.y / ipm_scale_);
}

// ____________________________________________________________________________
void CameraTransformations::StreetImageToStreet(const cv::Point& image_point,
                                            cv::Point2f* street_point) {
    // this does also the flipping from I(w|h) to P_street(x|y)
    street_point->y = (image_point.x / ipm_scale_) - ipm_street_img_offset_;
    street_point->x = (image_point.y / ipm_scale_);
}

// ____________________________________________________________________________
void CameraTransformations::StreetToStreetImage(const tPoint& street_point,
                                                cv::Point* image_point) {
    image_point->x = (street_point.y + ipm_street_img_offset_) * ipm_scale_;
    image_point->y = (street_point.x * ipm_scale_);
}

// ____________________________________________________________________________
void CameraTransformations::StreetToStreetImage(float x, float y,
                                                cv::Point* image_point) {
    image_point->x = (y + ipm_street_img_offset_) * ipm_scale_;
    image_point->y = (x * ipm_scale_);
}

// ____________________________________________________________________________
void CameraTransformations::PixelToStreet(
        const std::vector<cv::Point2i>& source,
        std::vector<cv::Point2f>* transformed) {
    std::vector<cv::Point2f> source_float;
    source_float.reserve(source.size());
    for (size_t i = 0; i < source.size(); ++i) {
        source_float.push_back(cv::Point2f(source[i]));
    }
    PixelToStreet(source_float, transformed);
}

// _____________________________________________________________________________
void CameraTransformations::PixelToStreetImage(
        const cv::Mat& input_image, cv::Mat* output_image) {
    cv::warpPerspective(
        input_image, *output_image, pix_to_streetimage_, ipm_image_dims_,
        cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT);
}

// _____________________________________________________________________________
float CameraTransformations::GetScale() {
    return ipm_scale_;
}
