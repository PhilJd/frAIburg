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

#include "r200_transformations.h"
#include <iostream>
namespace R200Transformations {

/*! The depth intrinsics retrieved by librealsense. */
rs::intrinsics depth_intrinsics_;
rs::intrinsics color_intrinsics_;
rs::extrinsics extrinsics_;

// ____________________________________________________________________________
void InitializeIntelParameters() {
    rs::context rsContext;
    rs::device* device_ = rsContext.get_device(0);
    depth_intrinsics_ = device_->get_stream_intrinsics(rs::stream::depth);
    color_intrinsics_ = device_->get_stream_intrinsics(rs::stream::color);
    // extrinsics: color to depth
    extrinsics_ = device_->get_extrinsics(rs::stream::color, rs::stream::depth);
}

// ____________________________________________________________________________
cv::Point3f PixelToWorldCoordinate(const cv::Point3f& pixel) {
    // pixel depth is in mm
    rs::float2 rs_pixel = {pixel.x, pixel.y};
    rs::float3 rs_point_camera =
        depth_intrinsics_.deproject(rs_pixel, pixel.z / 1000.0);
    return CameraWorldToCarWorld(rs_point_camera);
}

// ____________________________________________________________________________
cv::Point2i CarWorldToPixelCoordinate(const cv::Point3f& point) {
    rs::float3 rs_point = CarWorldToCameraWorldRs(point);
    rs::float2 rs_pixel = depth_intrinsics_.project(rs_point);
    return cv::Point2i(rs_pixel.x, rs_pixel.y);
}

// ____________________________________________________________________________
cv::Point3f CameraWorldToCarWorld(const cv::Point3f& p) {
    return cv::Point3f(p.z, -p.x, -p.y) + CAMERA_POS_IN_CAR_WORLD;
}

// ____________________________________________________________________________
cv::Point3f CameraWorldToCarWorld(const rs::float3& p) {
    return cv::Point3f(p.z, -p.x, -p.y) + CAMERA_POS_IN_CAR_WORLD;
}

// ____________________________________________________________________________
cv::Point3f CarWorldToCameraWorld(const cv::Point3f& p) {
    cv::Point3f p_shifted = p - CAMERA_POS_IN_CAR_WORLD;
    return cv::Point3f(-p_shifted.y, -p_shifted.z, p_shifted.x);
}

// ____________________________________________________________________________
rs::float3 CarWorldToCameraWorldRs(const cv::Point3f& p) {
    cv::Point3f p_shifted = p - CAMERA_POS_IN_CAR_WORLD;
    return {-p_shifted.y, -p_shifted.z, p_shifted.x};
}

}  // end namespace R200Transformations
