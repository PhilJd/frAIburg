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
#include "zebra_crossing_detection.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF,
                   ZebraCrossingDetection)

using namespace frAIburg::map;

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (property_debug_enabled_) {   \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

// ____________________________________________________________________________
ZebraCrossingDetection::ZebraCrossingDetection(const tChar *__info)
    : cFilter(__info) {
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED,
                    ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_DEBUG_ENABLED NSSUBPROP_DESCRIPTION,
                   "Enable Debug Mode");
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED NSSUBPROP_ISCHANGEABLE, tTrue);

    // ROI: used to limit search space for zebra crossing
    // TODO(Jan) Adjust default ROI when
//    SetPropertyFloat("ROI::XOffset", 0.2); 
//    SetPropertyStr("ROI::XOffset" NSSUBPROP_DESCRIPTION,
//                   "X Offset for Region of Interest Rectangular in meter");
//    SetPropertyBool("ROI::XOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
//
//    SetPropertyFloat("ROI::YOffset", 0.4);
//    SetPropertyStr("ROI::YOffset" NSSUBPROP_DESCRIPTION,
//                   "Y Offset for Region of Interest Rectangular in meter");
//    SetPropertyBool("ROI::YOffset" NSSUBPROP_ISCHANGEABLE, tTrue);
//
//    SetPropertyFloat("ROI::Width", 1.2);
//    SetPropertyStr("ROI::Width" NSSUBPROP_DESCRIPTION,
//                   "Width of the Region of Interest Rectangular in meter");
//    SetPropertyBool("ROI::Width" NSSUBPROP_ISCHANGEABLE, tTrue);
//
//    SetPropertyFloat("ROI::Height", 1.0);
//    SetPropertyStr("ROI::Height" NSSUBPROP_DESCRIPTION,
//                   "Height of the Region of Interest Rectangular in meter");
//    SetPropertyBool("ROI::Height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // Thresholding Params for Zebra Elements
    // TODO implement; use these params for the area limits and the thresholding
    // distance
    SetPropertyFloat("Zebra_Crossing_Element_Width",
                     0.1);  // 9cm on our test track
    SetPropertyStr("Zebra_Crossing_Element_Width" NSSUBPROP_DESCRIPTION,
                   "Zebra_Crossing_Element_Width (9cm on our test track).");
    SetPropertyBool("Zebra_Crossing_Element_Width" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat("Zebra_Crossing_Element_Height", 0.455);
    SetPropertyStr("Zebra_Crossing_Element_Height" NSSUBPROP_DESCRIPTION,
                   "Zebra_Crossing_Element_Height (58cm on our test track)");
    SetPropertyBool("Zebra_Crossing_Element_Height" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat("Max_Distance_Between_Elts", 0.3);
    SetPropertyStr("Max_Distance_Between_Elts" NSSUBPROP_DESCRIPTION,
                   "Max_Distance_Between_Zebra Crossings Elements for the "
                   "crossing to be valid (30cm on our test track).");
    SetPropertyBool("Max_Distance_Between_Elts" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("area_tolerance", 0.2);
    SetPropertyStr("area_tolerance" NSSUBPROP_DESCRIPTION,
                   "tolerance for the area of a zebra element.");
    SetPropertyBool("area_tolerance" NSSUBPROP_ISCHANGEABLE, tTrue);

    // XML config file
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME,
                   ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_DESCRIPTION,
                   "Configuration File");
    SetPropertyBool(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_ISCHANGEABLE,
                    tTrue);
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME,
                   ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_DESCRIPTION,
                   "which sensor to use");
    SetPropertyBool(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_ISCHANGEABLE,
                    tTrue);
}

// ____________________________________________________________________________
ZebraCrossingDetection::~ZebraCrossingDetection() {}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::Start(__exception) {
    last_time_ = _clock->GetStreamTime();
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::Stop(__exception) {
    // destroyWindow("Debug");
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        // Video Input
        RETURN_IF_FAILED(video_input_pin_.Create(
            "Video_Input", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&video_input_pin_));

        slim::register_pin_func func = &ZebraCrossingDetection::RegisterPin;
        RETURN_IF_FAILED(
            video_output_pin_.StageFirst(this, func, "Video_Output_Debug"));

    } else if (eStage == StageNormal) {
        map_ = frAIburg::map::getInstance();
    } else if (eStage == StageGraphReady) {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(video_input_pin_.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(
            pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO,
                                reinterpret_cast<tVoid **>(&pTypeVideo)));

        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat()))) {
           LOG_ERROR_PRINTF("Invalid Input Format for this filter");
        }
        LoadConfigFile();
        // ReloadROIParams();

    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ZebraCrossingDetection::ReloadROIParams() {
    PropertyChanged("ROI::XOffset");
    PropertyChanged("ROI::YOffset");
    PropertyChanged("ROI::Width");
    PropertyChanged("ROI::Height");
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::Shutdown(tInitStage eStage,
                                         ucom::IException **__exception_ptr) {
    // uncomment if needed later
    // if (eStage == StageGraphReady) {
    // }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::OnPinEvent(IPin *pSource, tInt nEventCode,
                                           tInt nParam1, tInt nParam2,
                                           IMediaSample *pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (pSource == &video_input_pin_) {
            if (video_input_format_.nPixelFormat == IImage::PF_UNKNOWN) {
                // const tBitmapFormat *format = video_input_pin_.GetFormat();
                // CheckFilterProperties(format->nHeight, format->nWidth);
                RETURN_IF_FAILED(
                    UpdateInputImageFormat(video_input_pin_.GetFormat()));
            }
            ProcessVideo(pMediaSample);
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &video_input_pin_) {
            // const tBitmapFormat *format = video_input_pin_.GetFormat();
            // CheckFilterProperties(format->nHeight, format->nWidth);
            RETURN_IF_FAILED(
                UpdateInputImageFormat(video_input_pin_.GetFormat()));

        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::CheckFilterProperties(int rows, int cols) {
    // This is not tested, can fail e.g. in StageInit when params are loaded
    if ((filter_properties_.roi_offset_x + filter_properties_.roi_width)
            > cols ||
        (filter_properties_.roi_offset_y + filter_properties_.roi_height)
            > rows) {
        LOG_ERROR_PRINTF(
            "ZebraCrossingDetection: ROI is bigger than transformed image."
            " ROI is set to size of incoming image");
        filter_properties_.roi_offset_x = 0;
        filter_properties_.roi_offset_y = 0;
        filter_properties_.roi_width = cols-1;
        filter_properties_.roi_height = rows-1;
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::PropertyChanged(const tChar *str_name) {
    RETURN_IF_FAILED(cFilter::PropertyChanged(str_name));
    if (cString::IsEqual(str_name, ADTF_PROPERTY_DEBUG_ENABLED)) {
        property_debug_enabled_ = GetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED);
//    } else if (cString::IsEqual(str_name, "ROI::Width")) {
//        filter_properties_.roi_width =
//            MeterToPixel(GetPropertyFloat("ROI::Width"));
//    } else if (cString::IsEqual(str_name, "ROI::Height")) {
//        filter_properties_.roi_height =
//            MeterToPixel(GetPropertyFloat("ROI::Height"));
//    } else if (cString::IsEqual(str_name, "ROI::XOffset")) {
//        filter_properties_.roi_offset_x =
//            MeterToPixel(GetPropertyFloat("ROI::XOffset"));
//    } else if (cString::IsEqual(str_name, "ROI::YOffset")) {
//        filter_properties_.roi_offset_y =
//            MeterToPixel(GetPropertyFloat("ROI::YOffset"));
    } else if (cString::IsEqual(str_name, "Zebra_Crossing_Element_Width")) {
        filter_properties_.zebra_crossing_element_width =
            GetPropertyFloat("Zebra_Crossing_Element_Width");
    } else if (cString::IsEqual(str_name, "Zebra_Crossing_Element_Height")) {
        filter_properties_.zebra_crossing_element_height =
            GetPropertyFloat("Zebra_Crossing_Element_Height");
    } else if (cString::IsEqual(str_name, "Max_Distance_Between_Elts")) {
        filter_properties_.max_dist_between_elts =
            GetPropertyFloat("Max_Distance_Between_Elts");
    } else if (cString::IsEqual(str_name, "area_tolerance")) {
        filter_properties_.area_tolerance = GetPropertyFloat("area_tolerance");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::UpdateInputImageFormat(
    const tBitmapFormat *pFormat) {
    if (pFormat != NULL) {
        // update member variable
        video_input_format_ = (*pFormat);
        LOG_INFO(adtf_util::cString::Format(
            "Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat %d;",
            video_input_format_.nWidth, video_input_format_.nHeight,
            video_input_format_.nBytesPerLine, video_input_format_.nSize,
            video_input_format_.nPixelFormat));
//        CheckFilterProperties(video_input_format_.nHeight,
//                            video_input_format_.nWidth);

        // create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(video_input_format_, input_image_));
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ZebraCrossingDetection::LoadConfigFile() {
    std::string config_file_path(
        GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME));
     std::string name_target(
        GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME));
     transform_helper_ = CameraTransformations(config_file_path, name_target);
}

// ____________________________________________________________________________
tResult ZebraCrossingDetection::ProcessVideo(IMediaSample *pSample) {
    //tTimeStamp start_time = _clock->GetStreamTime();

    RETURN_IF_POINTER_NULL(pSample);

    const tVoid *l_pSrcBuffer;
    cv::Mat img_transformed;
    cv::Mat img_gray;

    // receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer))) {
        // convert to mat, be sure to select the right pixelformat
        if (tInt32(input_image_.total() * input_image_.elemSize()) ==
            video_input_format_.nSize) {
            input_image_.data = const_cast<uchar *>(
                reinterpret_cast<const uchar *>(l_pSrcBuffer));

            if (input_image_.empty()) {
                LOG_ERROR_PRINTF("ZebraCrossingDetection Image empty");
                pSample->Unlock(l_pSrcBuffer);
                return false;
            }

            input_image_.copyTo(img_transformed);
        }
        pSample->Unlock(l_pSrcBuffer);
    }
    if (!map_->IsCarPosKnown()) {
        //LOG_WARNING_PRINTF("Car position unknown. "
        //    "Zebra Detection not working. Add map or disable this filter.");
        return false;
    }
    if(_clock->GetStreamTime() - last_time_ < 0.1 * 1e6) return false;
    last_time_ = _clock->GetStreamTime();

    // Save the current car position
    tMapCarPosition car_pos_when_image_received;
    map_->GetGlobalCarPosition(&car_pos_when_image_received);
    tMapData global_grid_orientation;
    GetFixedGridOrientation(car_pos_when_image_received.heading,
                            &global_grid_orientation);

    /// Compute mean of image
    cv::Scalar temp = cv::mean(img_transformed);
    int mean = (int)temp.val[0];

    /// subtract mean
    img_transformed = img_transformed - mean;

    /// threshold image using otsu
    cv::Mat img_binary;
    cv::threshold(img_transformed, img_binary, 0, 255,
                  cv::THRESH_BINARY + cv::THRESH_OTSU);

    // Take ROI of image, useful e.g. if only necessary to know zebra is ahead
    // in X meter
//    cv::Rect roi_rect(filter_properties_.roi_offset_x,
//                      filter_properties_.roi_offset_y,
//                      filter_properties_.roi_width,
//                      filter_properties_.roi_height);
//
//    std::cout << filter_properties_.roi_offset_x << std::endl;
//    std::cout << filter_properties_.roi_offset_y << std::endl;    
//    std::cout << filter_properties_.roi_width    << std::endl;
//    std::cout << filter_properties_.roi_height   << std::endl;
//    cv::Mat img_roi = img_binary(roi_rect);
//    std::cout << img_roi.cols << " " << img_roi.rows   << std::endl;

    if (video_output_pin_.IsConnected()) {
            video_output_pin_.Transmit(img_binary, _clock->GetStreamTime());
    }

    cv::Point zebra_location_img;
    if (Detect(img_binary, &zebra_location_img)) { // img_roi
        // Add offsets from roi
//        zebra_location_img.x += filter_properties_.roi_offset_x;
//        zebra_location_img.y += filter_properties_.roi_offset_y;
        cv::Point2f zebra_location_car;
        transform_helper_.StreetImageToStreet(zebra_location_img, &zebra_location_car);
        zebra_location_car.y = 0.5*LANEWIDTH_;//(zebra_location_car.y+0.5*LANEWIDTH_);
        cv::Point2f zebra_location_global;
        TransformToGlobal(global_grid_orientation,
                          car_pos_when_image_received.x,
                          car_pos_when_image_received.y, zebra_location_car,
                          &zebra_location_global);
        AddToMap(zebra_location_global, global_grid_orientation); // 9.11: car_pos_when_image_received.heading
        IF_DEBUG_LOG_PRINTF("Zebra local center x%f y%f;\n",
                            zebra_location_car.x, zebra_location_car.y);
    } else {
         IF_DEBUG_LOG_PRINTF("No zebra crossing detected\n");
    }

    //IF_DEBUG_LOG_PRINTF("Crossing Detection needs %li  microseconds \n",
    //				_clock->GetStreamTime()-start_time);

    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ZebraCrossingDetection::GetFixedGridOrientation(
    const tMapData &orientation, tMapData *return_grid_orientation) {
    // TODO map the orientation to the four directions of the grid pi, +-pi/2, 0
    // divide by pi/2 and then round and then multiply with pi/2 again
    float ratio = orientation / M_PI_2;  // Please note exception: for negative
                                         // orientations a ratio of -1.5 is
                                         // rounded to -1
    ratio += 0.5;
    ratio = floor(ratio);
    *return_grid_orientation = M_PI_2 * ratio;
}

// ____________________________________________________________________________
void ZebraCrossingDetection::TransformToGlobal(
    const tMapData &rotation_angle, const tMapData &rotation_point_x,
    const tMapData &rotation_point_y, const cv::Point2f &local_point,
    cv::Point2f *zebra_location_global) {
    zebra_location_global->x = cos(rotation_angle) * local_point.x -
                               sin(rotation_angle) * local_point.y +
                               rotation_point_x;

    zebra_location_global->y = sin(rotation_angle) * local_point.x +
                               cos(rotation_angle) * local_point.y +
                               rotation_point_y;
}

// ____________________________________________________________________________
int ZebraCrossingDetection::MeterToPixel(float meter) {
    if ((meter * transform_helper_.GetScale()) < 1.0 && meter > 0.0)
        LOG_WARNING_PRINTF("ZebraCrossingDetection: meter * scale < 1.0");
    return static_cast<int>(meter * transform_helper_.GetScale());
}

// ____________________________________________________________________________
bool ZebraCrossingDetection::Detect(const cv::Mat &img_binary,
                                    cv::Point *return_crossing_loc) {
    /// Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(img_binary, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));

    std::vector<Contour> zebra_candidates;
    FindCandidates(contours, &zebra_candidates);
    IF_DEBUG_LOG_PRINTF("Number of possible Zebra-Elements %li \n",
                        zebra_candidates.size());

    // Compute the center of the contour
    float elem_size = static_cast<float>(zebra_candidates.size());
    Contour global;
    for (std::vector<Contour>::iterator c = zebra_candidates.begin();
         c < zebra_candidates.end(); ++c) {
        global.center += (c->center * 1 / elem_size);
    }

    // initialize distance array
    float shortest_dist[zebra_candidates.size()];
    for (int i = 0; i < zebra_candidates.size(); ++i) {
        shortest_dist[i] = img_binary.cols;
    }
    // Compute the closest neighbor for each element
    for (int i = 0; i < zebra_candidates.size(); ++i) {
        for (int j = i + 1; j < zebra_candidates.size(); ++j) {
            double dist = cv::norm(zebra_candidates[i].center -
                                   zebra_candidates[j].center);
            if (dist < shortest_dist[i]) shortest_dist[i] = dist;
            if (dist < shortest_dist[j]) shortest_dist[j] = dist;
        }
    }
    // accept elements smaller than distance threshold
    std::vector<Contour> zebra_elements;
    for (int i = 0; i < zebra_candidates.size(); ++i) {
        //        printf("dist %f \n", shortest_dist[i]);
        if (shortest_dist[i] <
            MeterToPixel(
                filter_properties_.max_dist_between_elts))  // 300mm works good
            zebra_elements.push_back(zebra_candidates[i]);
    }
    IF_DEBUG_LOG_PRINTF("Number of Zebra-Elements detected: %li \n",
                        zebra_elements.size());
    float elements_size = static_cast<float>(zebra_elements.size());
    if (elements_size > 1) {
        Contour zebra;
        for (std::vector<Contour>::iterator element = zebra_elements.begin();
             element != zebra_elements.end(); ++element) {
            zebra.center += (element->center * 1 / elements_size);
        }

        *return_crossing_loc = zebra.center;
        return true;
    }

    // Todo set thresholds with variable
    // Thresholds used:
    //      area of contour within range 18000 - 80000 -> replaced with area
    //      contour is convex
    //      the approximated polygon has:
    //             cv::arcLength(cv::Mat(contours[i]), true)*0.02
    //      approximation accuracy. This is the maximum distance between
    //      the original curve and its approximation. -> relax for lower scale

    return false;
}

// ____________________________________________________________________________
void ZebraCrossingDetection::FindCandidates(
    const std::vector<std::vector<cv::Point> > &contours,
    std::vector<Contour> *zebra_candidates) {
    for (size_t i = 0; i < contours.size(); ++i) {
        std::vector<cv::Point> approx;
        // approximate contour with accuracy proportional
        // to the contour perimeter
        cv::approxPolyDP(cv::Mat(contours[i]), approx,
                         cv::arcLength(cv::Mat(contours[i]), true) * 0.02,
                         true);
        float element_area = filter_properties_.zebra_crossing_element_width *
                             filter_properties_.zebra_crossing_element_height *
                             transform_helper_.GetScale() *
                             transform_helper_.GetScale();
        float tolerance = element_area * filter_properties_.area_tolerance;
        if (4 == approx.size()  // actual size on our track is 52200mm^2,
                                // Distortion and occlusions account for
                                // variations
            &&
            fabs(cv::contourArea(cv::Mat(approx))) >
                element_area - tolerance  // 18000
            &&
            fabs(cv::contourArea(cv::Mat(approx))) <
                element_area + tolerance  // 80000
            && cv::isContourConvex(cv::Mat(approx))) {
            // IF_DEBUG_LOG_PRINTF("4p area
            // %f",cv::contourArea(cv::Mat(approx)));
            Contour c;
            c.points = approx;
            c.center = (approx[0] + approx[1] + approx[2] + approx[3]) / 4.0;
            zebra_candidates->push_back(c);
        }

        if (5 == approx.size() &&
            fabs(cv::contourArea(cv::Mat(approx))) > element_area - tolerance &&
            fabs(cv::contourArea(cv::Mat(approx))) < element_area + tolerance &&
            cv::isContourConvex(cv::Mat(approx))) {
            // IF_DEBUG_LOG_PRINTF("5p area %f",
            // cv::contourArea(cv::Mat(approx)));
            Contour c;
            c.points = approx;
            c.center =
                (approx[0] + approx[1] + approx[2] + approx[3] + approx[4]) /
                5.0;
            zebra_candidates->push_back(c);
        }
    }
    return;
}

// ____________________________________________________________________________
bool ZebraCrossingDetection::AddToMap(const cv::Point2f &zebra_location,
                                      const tMapData orientation) {
    // assume that the ROI is only for the lane in front of the car
    // then set the y value to 0 and put the center of the zebra crossing to
    // the distance of the zebra_location to the car into the map
    //    tSptrMapElement zcrossing_poly =
    //    MapHelper::CreatBox(STREET_MARKER_ZEBRA,   // TODO maybe include the
    //    orientation
    //                                zebra_location.x,
    //   old                          zebra_location.y,
    //                                0.4,
    //                                0.5,
    //                                _clock->GetStreamTime());
    std::vector<tMapPoint> zebra_rectangle;
    MapHelper::CreatBoxPoints(zebra_location.x, zebra_location.y,
                              0.225,  // TODO since my coordinates are in global,
                                    // the width and height are wrong for some
                                    // car orientations
                              0.5, zebra_rectangle, &orientation);

    tSptrMapElement zcrossing_poly(
        new MapElement(STREET_MARKER_ZEBRA, zebra_rectangle,
                       _clock->GetStreamTime(), orientation + PI));

    zcrossing_poly->LocalToGlobal(MapHelper::GetCarPosLocalIsGlobal());

    map_->AddFuseElement(zcrossing_poly,
                         1,  // max distances to fuse
                         -1, _clock->GetStreamTime(),
                         MAP_FUSE_REPLACE);

    // Elements are automatically removed from map after two meter

    return true;
}
