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

#ifndef AADCUSER_UTILS_R200_TRANSFORMATIONS_H_
#define AADCUSER_UTILS_R200_TRANSFORMATIONS_H_

#include "librealsense/rs.hpp"  // rs::intrinsics
#include "opencv2/core/core.hpp"

const cv::Point3f CAMERA_POS_IN_CAR_WORLD = cv::Point3f(-0.2, 0.0, 0.21);


namespace R200Transformations {

    /*! Fetches the depth intrinsics. Must happen after the camera filter
     *  activated the depth stream. */
    void InitializeIntelParameters();

    /*! Transforms a point in the depth camera's pixel space to world space
     *   (x,y, z) coordinate. Note that pixel depth values are given in mm. */
    cv::Point3f PixelToWorldCoordinate(const cv::Point3f& pixel);

    /*! Transform a point from the car's world space to a pixel. Note that this
     *  is the car's coordinate system, so the camera coordinate system is
     *  shifted. Used to create the depth map of the road. */
    cv::Point2i CarWorldToPixelCoordinate(const cv::Point3f& point);

    /*! Transformation from the camera's coordinate system to the car's
     *  coordinate system. Overloaded function taking a cv::Point as input. */
    cv::Point3f CameraWorldToCarWorld(const cv::Point3f& p);

    /*! Transformation from the camera's coordinate system to the car's
     *  coordinate system. Overloaded function taking a rs::float3 as input. */
    cv::Point3f CameraWorldToCarWorld(const rs::float3& p);

    /*! Transformation from the car's coordinate system to the camera's
     *  coordinate system. */
    cv::Point3f CarWorldToCameraWorld(const cv::Point3f& p);

    /*! Transformation from the car's coordinate system to the camera's
     *  coordinate system. */
    rs::float3 CarWorldToCameraWorldRs(const cv::Point3f& p);

}  // namespace R200Transformations

#endif  // AADCUSER_UTILS_R200_TRANSFORMATIONS_H_
