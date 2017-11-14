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
#ifndef AADCUSER_FRAIBURG_OBSTACLEDETECTION_OBSTACLE_DETECTION_H_
#define AADCUSER_FRAIBURG_OBSTACLEDETECTION_OBSTACLE_DETECTION_H_

#include "stdafx.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ADTF_OpenCV_helper.h"
#include "slim_pins.h"
#include "adtf_tools.h"
#include "map_types.h"  // tMapPoint
#include "global_map.hpp"
#include "map_helper.hpp"
#include "map_element_types.h"

// unique filter name
#define OID_ADTF_FILTER_DEF "adtf.frAIburg_ObstacleDetection"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "Obstacle  Detection Filter"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "ObstacleDetectionFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "Filter to find obstacles in depth image. "

namespace fmap = frAIburg::map;

/* Intel world coordinate space:
* x-axis points to the right, the positive y-axis points down, and the positive z-axis points forward.
*    A
*     \
*   z  \_______> x
*      |
*      |
*      |
*      v
*     y
*
*
*   --> camera x = -car y
*   --> camera z = car x
*
*
*  Car coordinate space:
*
*
*        z
*        |   x
*        | /
*  y_____|/
*
*/

// tuning parameters
// The minimum number of pixel a connected component should have to be
// considered as an obstacle
const int AREA_THRESHOLD_PX = 200;
// The number of pixels to crop from the rectangular hull of a connected
// component. (The depth values on the edges are less accurate)
const int CC_CROP_PX = 0;
const int MAX_DISTANCE_MM = 2000;
// objects 100 mm apart are considered separated, scale to 8 bit for canny
const int PIXEL_GRAD_THRESHOLD = 100.0 * 255.0 / MAX_DISTANCE_MM;
const float SEGMENT_PIXEL_WIDTH = 10.0;
const float ROAD_Z_COORD = 0.04;  // in m in the world coordinate system
const float CAR_ROOF_Z = 0.29;  // in m, obstacles above this will be discarded

// switch clang format off, to retain the markdown table
// clang-format off
/*! @defgroup ObstacleDetectionFilter
*  @{
*    This filter grabs the depth channel of the RGBD camera, detects near
*    obstacles and adds them to the map.
*
* **Dependencies** \n
* This plugin needs the following libraries:
* - OpenCV  v.3.2.0
*
* **Filter Properties**
* | Property | Description | Default |
* | -------- | ----------- | ------- |
* |          |             |         |
*
*
* **Output Pins**
* | Pin   | Description  | MajorType  | SubType  |
* | ---- -| ------------ | ---------- | -------- |
* | None  |              |            |          |
*
*
* **Input Pins**
* | Pin                   | Description           | MajorType        | SubType                          |
* | --------------------- | --------------------- | -----------------|--------------------------------- |
* | depth_raw_input_pin_  | pin for raw depth     | MEDIA_TYPE_VIDEO | MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED |

*
*
* **Plugin Details**
* | | |
* |-|-|
* | Path    | src/aadcUser/frAIburg_ObstacleDetection |
* | Filename| frAIburg_ObstacleDetection.plb          |
* | Version | 1.0.0                                   |
*
*/
// clang-format on

// ! Filter to detect obstacles using the depth image.
/*!
* This class grabs the depth camera output and identifies
* regions the car could collide with and enters them into the map.
*
*/
class ObstacleDetection : public adtf::cFilter {
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary, ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major, ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build, ADTF_FILTER_VERSION_LABEL);

 protected:
    /*! Input for the raw depth. */
    cInputPin depth_raw_input_pin_;

    /*! Output pin for debugging, visualization of the connected components. */
    slim::VideoOutputPin cc_visualization_pin_;

 public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    explicit ObstacleDetection(const tChar* __info);

    /*! default destructor */
    virtual ~ObstacleDetection();

    /*! Init stage. */
    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Shutdown stage. */
    tResult Shutdown(tInitStage stage,
                     ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to. */
    tResult OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
                       IMediaSample* media_sample);

    /*! Start Stage. */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Stop Stage. */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

 private:
    /*! The pointer to the map singleton. */
    fmap::GlobalMap* map_;

    /*! Bitmap format of input pin, 16bit grayscale. */
    tBitmapFormat input_format_;

    /*! Bitmap format of the visualization output pin. 8bit grayscale. */
    tBitmapFormat output_format_;

    /*! The threshold matrix containing the distance to the road. */
    cv::Mat road_distance_;

    /*! Store the car position as soon as the image sample is received. */
    frAIburg::map::tMapCarPosition car_position_at_input_received_;

    /*! Store the time stamp of the image sample after receiving. */
    frAIburg::map::tTimeMapStamp timestamp_at_input_received_;

    /*! True if the visualization output pin is connected. */
    bool visualization_enabled_;

    // ------------ initialization methods
    //
    /*! Creates the road_distance_ matrix that contains the depth
     *  image of the road. */
    void CreateRoadDistanceMatrix();


    // ------------ main loop method + helpers
    //
    /*! Detects obstacles within 20 cm to 1.20 m range. */
    void CheckForObstacles(const cv::Mat& depth_image);

    /*! Computes the edges in the depth images (boundaries between separate
     *  objects). Widens these edges and sets the according mask pixels to 0,
     *  separating the components. */
    void DrawEdgesOnMask(const cv::Mat& depth_image, cv::Mat* mask);

    /*! Helper function to create a cv::Rect from stats returned by
     *  cv::connectedComponentsWithStats. */
    cv::Rect StatsToRect(const cv::Mat& stats, int stat_index);

    /*! Helper function that fills the polygon vector with the closest point
     *  in each segment of the component. */
    void GetShapeOutline(std::vector<cv::Point3f>* outline,
        const cv::Mat& depth_image, const cv::Mat& labeled_mask,
        const cv::Mat& stats, int component_index);

    /*! Returns the closest point in the depth image, masked with mask.*/
    cv::Point3f GetNearestPoint(const cv::Mat& depth_image,
                                const cv::Mat& mask);

    /*! Adds the detected obstacle to the map. */
    void AddObstacleToMap(const std::vector<cv::Point3f>& outline);

    /*! Scales the labeled mask to fit in 0, 255 and transmits the mask. */
    void VisualizeAndTransmit(cv::Mat* labeled_mask, const cv::Mat& stats,
                              int num_components);
};

/** @} */  // end of group

#endif  // AADCUSER_FRAIBURG_OBSTACLEDETECTION_OBSTACLE_DETECTION_H_
