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
#include "stdafx.h"
#include "obstacle_detection.h"
#include <math.h>  // round
#include <sys/stat.h>
#include <algorithm>  // std::min
#include <stdexcept>
#include "adtf_tools.h"
#include "log_tools.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv_tools.h"
#include "r200_transformations.h"
#include "xml_helper.hpp"

namespace fmap = frAIburg::map;

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, ObstacleDetection)

ObstacleDetection::ObstacleDetection(const tChar* __info) : cFilter(__info) {
    SetPropertyFloat("distance_threshold", 0.5);
    cString info =
        "The minimum distance at which an obstacle is entered into the map.";
    SetPropertyStr("distance_threshold" NSSUBPROP_DESCRIPTION, info);
    // The raw depth data is sent as normal media sample as the video pin
    // doesn't support this. Therefore we use the known, fixed input format
    input_format_.nWidth = 320;
    input_format_.nHeight = 240;
    input_format_.nBitsPerPixel = 16;
    input_format_.nPixelFormat = adtf_util::cImage::PF_GREYSCALE_16;
    input_format_.nBytesPerLine = 640;    // nWidth * nBitsPerPixel / 8
    input_format_.nSize = 320 * 240 * 2;  // in bytes
    input_format_.nPaletteSize = 0;
    // output of connected components will be scaled to 255
    output_format_ = input_format_;  // height, width, palette size are the same
    output_format_.nBitsPerPixel = 8;
    output_format_.nPixelFormat = adtf_util::IImage::PF_GREYSCALE_8;
    output_format_.nBytesPerLine = 320;
    output_format_.nSize = 320 * 240;
}

// ____________________________________________________________________________
ObstacleDetection::~ObstacleDetection() {}

// ____________________________________________________________________________
tResult ObstacleDetection::Start(__exception) {
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult ObstacleDetection::Stop(__exception) {
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult ObstacleDetection::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));
    if (stage == StageFirst) {
        // depth input
        IPinEventSink* event_sink = static_cast<IPinEventSink*>(this);
        RETURN_IF_FAILED(depth_raw_input_pin_.Create(
            "raw_depth_input",
            new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA,
                                 MEDIA_SUBTYPE_STRUCT_STRUCTURED),
            event_sink));
        RETURN_IF_FAILED(RegisterPin(&depth_raw_input_pin_));
        // video output of connected components visualization
        slim::register_pin_func func = &ObstacleDetection::RegisterPin;
        RETURN_IF_FAILED(
            cc_visualization_pin_.StageFirst(this, func, "output_viz"));
    } else if (stage == StageNormal) {
        cc_visualization_pin_.SetFormat(output_format_);
    } else if (stage == StageGraphReady) {
        map_ = fmap::getInstance();
        // Initialize parameters here, after the camera filter has activated
        // the stream with the parameters only the camera filter knows
        // (e.g. fps). only after that the DistanceMatrix can be created
        R200Transformations::InitializeIntelParameters();
        CreateRoadDistanceMatrix();  // This might take some time...
        visualization_enabled_ = cc_visualization_pin_.IsConnected();
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ObstacleDetection::Shutdown(tInitStage stage,
                                    ucom::IException** __exception_ptr) {
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult ObstacleDetection::OnPinEvent(IPin* source, tInt event_code,
                                      tInt param1, tInt param2,
                                      IMediaSample* media_sample) {
    RETURN_IF_POINTER_NULL(media_sample);
    RETURN_IF_POINTER_NULL(source);
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        if (source == &depth_raw_input_pin_) {
            timestamp_at_input_received_ = _clock->GetStreamTime();
            map_->GetGlobalCarPosition(&car_position_at_input_received_);
            cv::Mat img = slim::cvtools::AdtfMediaSampleToCvMat(media_sample,
                                                                input_format_);
            CheckForObstacles(img);
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ObstacleDetection::CheckForObstacles(const cv::Mat& depth_image) {
    struct timeval tv1, tv2;
    gettimeofday(&tv1, NULL);

    // take median to remove spikes due to wrong depth estimation.
    cv::medianBlur(depth_image, depth_image, 5);
    cv::Mat dilation =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(depth_image, depth_image, dilation);
    cv::Mat mask = (depth_image > 200) & (depth_image < road_distance_);
    DrawEdgesOnMask(depth_image, &mask);  // cleanly separate the objects
    cv::Mat labeled_mask, centroids, stats;
    cv::connectedComponentsWithStats(mask, labeled_mask, stats, centroids, 8,
                                     CV_32S);  // identify separate objects
    for (int i = 1; i < stats.rows; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) < AREA_THRESHOLD_PX) {
            continue;
        }
        // Create a line of pixels closest to the camera, retain their value!
        std::vector<cv::Point3f> outline;
        GetShapeOutline(&outline, depth_image, labeled_mask, stats, i);
        // Transforms the outline to car coordinates and forms a polygon
        AddObstacleToMap(outline);
    }

    gettimeofday(&tv2, NULL);
    // printf ("Total time to get connected components = %f seconds\n",
    // (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 +
    // (double) (tv2.tv_sec - tv1.tv_sec));

    if (visualization_enabled_) {
        //VisualizeAndTransmit(&labeled_mask, stats, stats.rows);
        cc_visualization_pin_.Transmit(mask, _clock->GetStreamTime());
        // cc_visualization_pin_.Transmit(depth_image, _clock->GetStreamTime());
    }
}

// ____________________________________________________________________________
void ObstacleDetection::GetShapeOutline(std::vector<cv::Point3f>* outline,
                                        const cv::Mat& depth_image,
                                        const cv::Mat& labeled_mask,
                                        const cv::Mat& stats,
                                        int component_index) {
    // skip component 0, as it's the background class.
    cv::Rect component_rect = StatsToRect(stats, component_index);
    cv::Mat roi_depth = depth_image(component_rect);
    cv::Mat roi_labeled_mask = labeled_mask(component_rect);
    // filter out possible overlapping, differently labeled regions
    // by creating another mask
    cv::Mat roi_mask = roi_labeled_mask == component_index;
    int num_segments = component_rect.width / SEGMENT_PIXEL_WIDTH;
    // add polygon points from left to right
    cv::Rect segment(0, 0, static_cast<int>(SEGMENT_PIXEL_WIDTH),
                     component_rect.height);
    for (int j = 0; j < num_segments; j++) {
        segment.x = static_cast<int>(j * SEGMENT_PIXEL_WIDTH);
        // catch the case where component_rect.width % SEGMENT_PIXEL_WIDTH == 0
        if (segment.br().x >= component_rect.width) {
            break;
        }
        cv::Point3f p = GetNearestPoint(roi_depth(segment), roi_mask(segment));
        p.x += component_rect.x + segment.x;
        p.y += component_rect.y;
        outline->push_back(p);
    }
}

// ____________________________________________________________________________
cv::Rect ObstacleDetection::StatsToRect(const cv::Mat& stats, int stat_index) {
    return cv::Rect(stats.at<int>(stat_index, cv::CC_STAT_LEFT) + CC_CROP_PX,
                    stats.at<int>(stat_index, cv::CC_STAT_TOP) + CC_CROP_PX,
                    stats.at<int>(stat_index, cv::CC_STAT_WIDTH) - CC_CROP_PX,
                    stats.at<int>(stat_index, cv::CC_STAT_HEIGHT) - CC_CROP_PX);
}

// ____________________________________________________________________________
cv::Point3f ObstacleDetection::GetNearestPoint(const cv::Mat& depth_image,
                                               const cv::Mat& mask) {
    double min_val;
    int min_index[2];
    cv::minMaxIdx(depth_image, &min_val, NULL, min_index, NULL, mask);
    return cv::Point3f(min_index[1], min_index[0], min_val);
}

// ____________________________________________________________________________
void ObstacleDetection::DrawEdgesOnMask(const cv::Mat& depth_image,
                                        cv::Mat* mask) {
    // Convert to 8 bit so canny can be used. convertTo applies saturate_cast,
    // so with scaling we implicitly clip the value range to (0, 1.2m)
    cv::Mat depth_image_8bit, edges, dilation;
    depth_image.convertTo(depth_image_8bit, CV_8U, 255.0 / MAX_DISTANCE_MM, 0);
    if (depth_image.rows == 0) {
        return;
    }
    // Canny seems to produce spikes. When computing the derivatives manually
    // they don't occur.
    // cv::Canny(depth_image_8bit, edges, PIXEL_GRAD_THRESHOLD,
    //           PIXEL_GRAD_THRESHOLD, 3, true);


    cv::Mat grad_x, grad_y;
    cv::Sobel(depth_image_8bit, grad_x,  CV_16S, 1, 0, CV_SCHARR);
    cv::Sobel(depth_image_8bit, grad_y,  CV_16S, 0, 1, CV_SCHARR);
    cv::convertScaleAbs(grad_x, grad_x);
    cv::convertScaleAbs(grad_y, grad_y);
    cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, edges);
    edges = edges > PIXEL_GRAD_THRESHOLD;


    // enlarge edge to three pixels
    dilation = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(edges, edges, dilation);
    mask->setTo(0, edges);  // set edges to 0 to separate objects.
}

// ____________________________________________________________________________
void ObstacleDetection::AddObstacleToMap(
    const std::vector<cv::Point3f>& outline) {
    std::vector<fmap::tMapPoint> map_points;
    if (outline.size() == 0) {
        return;
    }
    // iterate reversely as the outline is clockwise order
    float last_z = 10000;
    for (size_t i = outline.size(); i-- > 0;) {
        cv::Point3f p = R200Transformations::PixelToWorldCoordinate(outline[i]);
        if (p.z > CAR_ROOF_Z) {  // omit objects under which the car can drive
            continue;
        }
        // remove unnecessary points that sometimes cause
        // a "non-clockwise polygon warning"
        if (std::abs(p.y - last_z) < 0.007) {
            continue;
        }
        map_points.push_back(fmap::tMapPoint(p.x, p.y));
    }
    size_t num_points = map_points.size();
    if (num_points == 0) {  // return if height check omitted all points
        return;
    }
    // add the same shape with an offset of 1 cm, to close the polygon
    for (size_t i = num_points; i-- > 0;) {
        fmap::tMapPoint p = map_points[i];
        map_points.push_back(fmap::tMapPoint(p.x() + 0.05, p.y()));
    }

    // for (size_t i = 0; i < map_points.size(); ++i) {
    //     std::cout << "[" << map_points[i].x() << ", " << map_points[i].y() << "]" << std::endl;
    // }

    // MapElementDepth to allow fuse with differnt map el subtypes
    // Map type depth
    fmap::tSptrMapElement el(new fmap::MapElementDepth(map_points,
                              timestamp_at_input_received_));

    // fix multi fuse not fully implemented
    // el not fused wiht sign etc, type OBSTACLE
    // fmap::tSptrMapElement el(new fmap::MapElement(
        // fmap::OBSTACLE, map_points, timestamp_at_input_received_));

    // el->EnableTimeOfLife(0.5e6);// reset life time if fused
    // el->EnableTimeOfLife(10000);// reset life time if fused
    el->LocalToGlobal(car_position_at_input_received_);

    //el->EnableTimeOfLife(1e6);     // overwrite life time with fuse
    map_->AddFuseElement(el,
                         0.2,  // max distance to fuse
                         -1,       // max area to fuse, -1 to deactivate
                         _clock->GetStreamTime(),
                         fmap::MAP_CONVEX_HULL_SIMPLIFY_5);

    // set no life time if element was fused
    // and replaced with a diffrent type
    // use creation time as life time to get rid of peaks after time
    if (el->GetType() == fmap::DEPTH || el->GetType() == fmap::OBSTACLE)
      el->EnableTimeOfLife(1*1e6); // take creation time as life time

    // debug: last sensor data all always with short life time and diff color
    // fmap::tSptrMapElement el_debug(
    //     new fmap::MapElement(fmap::DEBUG_POLYGON, map_points,
    //                          timestamp_at_input_received_));
    // el_debug->EnableTimeOfLife(10000);//
    // map_->AddElement(el_debug, _clock->GetStreamTime());
    // // debug change color ever update
    // static unsigned int color_index = 0;
    // el_debug->user_color_ui_ =
    // fmap::MapHelper::GetChangingDebugColor(&color_index);
}

// ____________________________________________________________________________
void ObstacleDetection::VisualizeAndTransmit(cv::Mat* labeled_mask,
                                             const cv::Mat& stats,
                                             int num_components) {
    int num_relevant_components = 1;  // background
    // filter out the non relevant components
    for (int i = 1; i < num_components; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) < AREA_THRESHOLD_PX) {
            labeled_mask->setTo(0, *labeled_mask == i);
            continue;
        }
        num_relevant_components++;
        labeled_mask->setTo(num_relevant_components, *labeled_mask == i);
    }
    // scale the colors so they're visible
    int scale = 200 / num_components;
    if (scale < 1) {
        std::cout << "Too many components. Not visualizing.";
        return;
    }
    // *labeled_mask *= scale;
    cv::Mat out;
    labeled_mask->convertTo(out, CV_8U, scale);
    cc_visualization_pin_.Transmit(out, _clock->GetStreamTime());
}

// ____________________________________________________________________________
void ObstacleDetection::CreateRoadDistanceMatrix() {
    road_distance_ = cv::Mat(240, 320, CV_16U, cv::Scalar(MAX_DISTANCE_MM));
    cv::Rect img_boundaries(cv::Point(), road_distance_.size());
    for (int x = CAMERA_POS_IN_CAR_WORLD.x; x < MAX_DISTANCE_MM; ++x) {
        for (int y = -650; y <= 500; ++y) {
            cv::Point3f world_coord(x / 1000.0, y / 1000.0, ROAD_Z_COORD);
            cv::Point2i pixel =
                R200Transformations::CarWorldToPixelCoordinate(world_coord);
            if (!img_boundaries.contains(pixel)) {
                continue;
            }
            // distance is l2 norm, scaled to mm
            int l2_dist = cv::norm(
                R200Transformations::CarWorldToCameraWorld(world_coord) * 1000);
            road_distance_.at<unsigned short>(pixel) =
                std::min(l2_dist, MAX_DISTANCE_MM);
        }
        x += x / 200;  // increase x faster for points further away
    }
}
