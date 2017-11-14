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
#include "lane_detection.h"
#include "opencv_tools.h"  // AdtfMediaSampleToCvMat

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, LaneDetection)

using namespace frAIburg::map;

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (property_debug_enabled_) {   \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

// ____________________________________________________________________________
LaneDetection::LaneDetection(const tChar* __info) : cFilter(__info) {
    // random number generator for displaying detected lane points
    cv::RNG rng(12345);
    for (size_t i = 0; i < 100; ++i) {
        color_vec_.push_back(cv::Point3i(rng.uniform(0, 255),
                                        rng.uniform(0, 255),
                                        rng.uniform(0, 255)));
    }

    // object that has the functionality for detecting lane marking
    LinePointsDetection points_detector_;
    // fit polynom to the detected points
    LaneFinder lane_finder_;

    is_filter_enabled_ = true;

    SetPropertyBool("Debug_Mode", false);
    // XML config file
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME,
                   ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT);

    // Set which sensor to use
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME,
                   ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_DEFAULT);
    SetPropertyStr(
        ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME NSSUBPROP_DESCRIPTION,
        "Caution for rear cam: "
        "make sure rear cam property FlipHorizontal is set to true");

    SetPropertyFloat("Transform::debug_pnt_x", 0);
    SetPropertyStr("Transform::debug_pnt_x" NSSUBPROP_DESCRIPTION,
                   "debug_pnt_x");
    SetPropertyBool("Transform::debug_pnt_x" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat("Transform::debug_pnt_y", 0);
    SetPropertyStr("Transform::debug_pnt_y" NSSUBPROP_DESCRIPTION,
                   "debug_pnt_y");
    SetPropertyBool("Transform::debug_pnt_y" NSSUBPROP_ISCHANGEABLE, tTrue);

    // ROI
    SetPropertyFloat("ROI::XOffset", 0.2);
    SetPropertyStr("ROI::XOffset" NSSUBPROP_DESCRIPTION,
                   "X Offset for Region of Interest Rectangular in meter");
    SetPropertyBool("ROI::XOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("ROI::YOffset", 0.1);
    SetPropertyStr("ROI::YOffset" NSSUBPROP_DESCRIPTION,
                   "Y Offset for Region of Interest Rectangular in meter");
    SetPropertyBool("ROI::YOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("ROI::Width", 1.7);
    SetPropertyStr("ROI::Width" NSSUBPROP_DESCRIPTION,
                   "Width of the Region of Interest Rectangular in meter");
    SetPropertyBool("ROI::Width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("ROI::Height", 1.1);
    SetPropertyStr("ROI::Height" NSSUBPROP_DESCRIPTION,
                   "Height of the Region of Interest Rectangular in meter");
    SetPropertyBool("ROI::Height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // Algorithm
    SetPropertyFloat("Algorithm::Detection Distance", 0.04);
    SetPropertyStr("Algorithm::Detection Distance" NSSUBPROP_DESCRIPTION,
                   "Distance between detection lines searched in ROI");
    SetPropertyBool("Algorithm::Detection Distance" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat("Algorithm::Minimum Line Width", 0.02);
    SetPropertyStr("Algorithm::Minimum Line Width" NSSUBPROP_DESCRIPTION,
                   "Minimum Line Width in Pixel");
    SetPropertyBool("Algorithm::Minimum Line Width" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat("Algorithm::Maximum Line Width", 0.06);
    SetPropertyStr("Algorithm::Maximum Line Width" NSSUBPROP_DESCRIPTION,
                   "Maximum Line Width in Pixel");
    SetPropertyBool("Algorithm::Maximum Line Width" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyInt("Algorithm::Minimum Line Contrast", 50);
    SetPropertyStr("Algorithm::Minimum Line Contrast" NSSUBPROP_DESCRIPTION,
                   "Currently fixed in code! "
                   "Mimimum line contrast in gray Values");
    SetPropertyBool("Algorithm::Minimum Line Contrast" NSSUBPROP_ISCHANGEABLE,
                    tTrue);
    SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MIN, 1);
    SetPropertyInt("Algorithm::Minimum Line Contrast" NSSUBPROP_MAX, 255);
}

// ____________________________________________________________________________
LaneDetection::~LaneDetection() {}

// ____________________________________________________________________________
tResult LaneDetection::Start(__exception) {
    last_time_ = _clock->GetStreamTime();
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult LaneDetection::Stop(__exception) {
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult LaneDetection::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    tResult nResult = ERR_NOERROR;
    if (eStage == StageFirst) {
        nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
    } else if (eStage == StageNormal) {
        map_ = frAIburg::map::getInstance();
    } else if (eStage == StageGraphReady) {
        video_input_pin_.StageGraphReady();
        SetPinIDs();
        LoadConfigFile();
        // after the config file is loaded set the roi params again
        ReloadROIParams();
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
void LaneDetection::ReloadROIParams() {
    PropertyChanged("ROI::XOffset");
    PropertyChanged("ROI::YOffset");
    PropertyChanged("ROI::Width");
    PropertyChanged("ROI::Height");
}

// ____________________________________________________________________________
tResult LaneDetection::CreateInputPins(__exception) {
    slim::register_pin_func func = &LaneDetection::RegisterPin;
    RETURN_IF_FAILED(video_input_pin_.StageFirst(this, func, "Video_Input"));
    RETURN_IF_FAILED(enable_filter_pin_.FirstStageCreate(
        this, func, "enable_filter", "tBoolSignalValue"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LaneDetection::CreateOutputPins(__exception) {
    slim::register_pin_func func = &LaneDetection::RegisterPin;
    RETURN_IF_FAILED(
        video_output_pin_.StageFirst(this, func, "Video_Output_Debug"));
    // GCL Output
    gcl_output_pin_.Create(
        "GCL",
        new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL),
        static_cast<IPinEventSink*>(this));
    RegisterPin(&gcl_output_pin_);
    // create output pin: lane point output
    RETURN_IF_FAILED(tLaneElement_output_pin_.FirstStageCreate(
        this, func, "Lane", "tLaneElement"));
    RETURN_IF_FAILED(tCrossing_output_pin_.FirstStageCreate(
        this, func, "Crossing", "tCrossing"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LaneDetection::SetPinIDs() {
    // set ids for tLane
    vector<string> ids = boost::assign::list_of("count")("idx")(
        "no_of_samples")("dist")("coeff0")("coeff1")("coeff2");
    RETURN_IF_FAILED(tLaneElement_output_pin_.StageGraphReadySetIDOrder(ids));
    // set ids for tCrossing
    vector<string> id_crossing = boost::assign::list_of("x")("y")("heading")
        ("accuracy");
    RETURN_IF_FAILED(
        tCrossing_output_pin_.StageGraphReadySetIDOrder(id_crossing));
    // set id for tBoolSignalValue
    std::vector<string> id_enable = boost::assign::list_of("bValue");
    RETURN_IF_FAILED(enable_filter_pin_.StageGraphReadySetIDOrder(id_enable));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LaneDetection::Shutdown(tInitStage eStage,
                                ucom::IException** __exception_ptr) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ____________________________________________________________________________
tResult LaneDetection::OnPinEvent(IPin* source, tInt nEventCode, tInt nParam1,
                                  tInt nParam2, IMediaSample* media_sample) {
    RETURN_IF_POINTER_NULL(media_sample);
    RETURN_IF_POINTER_NULL(source);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (video_input_pin_.IsSource(source)) {
            if (!is_filter_enabled_) {
                RETURN_NOERROR;
            }
            if (video_input_pin_.PixelFormatIsUnknown()) {
                RETURN_IF_FAILED(video_input_pin_.UpdateInputFormat());
                tBitmapFormat format = video_input_pin_.GetFormat();
                CheckFilterProperties(format.nHeight, format.nWidth);
            }
            ProcessVideo(media_sample);
        } else if (enable_filter_pin_.isSource(source)) {
            // read the pin
            RETURN_IF_FAILED(SetEnableBool(media_sample));
        }
    } else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
        if (video_input_pin_.IsSource(source)) {
            RETURN_IF_FAILED(video_input_pin_.UpdateInputFormat());
            tBitmapFormat format = video_input_pin_.GetFormat();
            CheckFilterProperties(format.nHeight, format.nWidth);
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LaneDetection::SetEnableBool(IMediaSample* media_sample) {
    // enable or disable the filter
    const vector<tVoid*> rx_buff_ =
        boost::assign::list_of((tVoid*)&is_filter_enabled_);
    if (IS_OK(enable_filter_pin_.ReadIDcopy(media_sample, rx_buff_))) {
        IF_DEBUG_LOG_PRINTF("is_filter_enabled_ changed: %d",
                            is_filter_enabled_);
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LaneDetection::PropertyChanged(const tChar* str_name) {
    RETURN_IF_FAILED(cFilter::PropertyChanged(str_name));
    // associate the properties to the member
    if (cString::IsEqual(str_name, "ROI::Width")) {
        filter_properties_.roi_width =
            points_detector_.MeterToPixel(GetPropertyFloat("ROI::Width"));
        points_detector_.SetRoiWidth(filter_properties_.roi_width);
    } else if (cString::IsEqual(str_name, "ROI::Height")) {
        filter_properties_.roi_height =
            points_detector_.MeterToPixel(GetPropertyFloat("ROI::Height"));
        points_detector_.SetRoiHeight(filter_properties_.roi_height);
        // tell lane finder how far we can see
        lane_finder_.SetMaxViewX(GetPropertyFloat("ROI::Height")
                                + GetPropertyFloat("ROI::YOffset"));
    } else if (cString::IsEqual(str_name, "ROI::XOffset")) {
        filter_properties_.roi_offset_x =
            points_detector_.MeterToPixel(GetPropertyFloat("ROI::XOffset"));
        points_detector_.SetRoiOffsetX(filter_properties_.roi_offset_x);
    } else if (cString::IsEqual(str_name, "ROI::YOffset")) {
        filter_properties_.roi_offset_y =
            points_detector_.MeterToPixel(GetPropertyFloat("ROI::YOffset"));
        points_detector_.SetRoiOffsetY(filter_properties_.roi_offset_y);
        // tell lane finder how far we can see
        lane_finder_.SetMaxViewX(GetPropertyFloat("ROI::Height")
                                + GetPropertyFloat("ROI::YOffset"));
    } else if (cString::IsEqual(str_name, "Algorithm::Detection Distance")) {
        filter_properties_.detection_distance =
            GetPropertyFloat("Algorithm::Detection Distance");
        points_detector_.SetDetectionDistance(
                                filter_properties_.detection_distance);
    } else if (cString::IsEqual(str_name, "Algorithm::Maximum Line Width")) {
        filter_properties_.max_line_width =
            GetPropertyFloat("Algorithm::Maximum Line Width");
        points_detector_.SetMaxLineWidth(filter_properties_.max_line_width);
    } else if (cString::IsEqual(str_name, "Algorithm::Minimum Line Width")) {
        filter_properties_.min_line_width =
            GetPropertyFloat("Algorithm::Minimum Line Width");
        points_detector_.SetMinLineWidth(filter_properties_.min_line_width);
    } else if (cString::IsEqual(str_name, "Algorithm::Minimum Line Contrast")) {
        filter_properties_.min_line_contrast =
            GetPropertyInt("Algorithm::Minimum Line Contrast");
        points_detector_.SetMinLineContrast(
            filter_properties_.min_line_contrast);
    } else if (cString::IsEqual(str_name, "Transform::debug_pnt_x")) {
        filter_properties_.debug_pnt_x =
            GetPropertyFloat("Transform::debug_pnt_x");
    } else if (cString::IsEqual(str_name, "Transform::debug_pnt_y")) {
        filter_properties_.debug_pnt_y =
            GetPropertyFloat("Transform::debug_pnt_y");
    } else if (cString::IsEqual(str_name, "Debug_Mode")) {
        property_debug_enabled_ = GetPropertyBool("Debug_Mode");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LaneDetection::ProcessVideo(IMediaSample* pSample) {
    // if (_clock->GetStreamTime() - last_time_ > 0.1 * 1e6) {
        RETURN_IF_POINTER_NULL(pSample);
        // here we store the pixel lines in the image where we search for lanes
        std::vector<tInt> detection_lines;
        // here we have all the detected line points
        std::vector<std::vector<tPoint> > detected_lines_h;
        std::vector<std::vector<tPoint> > detected_lines_v;

        const tVoid* sample_buffer;
        if (!IS_OK(pSample->Lock(&sample_buffer))) {
            return false;
        }
        tBitmapFormat format = video_input_pin_.GetFormat();
        tVoid* buffer = const_cast<tVoid*>(sample_buffer);
        cv::Mat img = slim::cvtools::NoCopyAdtfMediaSampleToCvMat(buffer,
                                                                  format);
        if (img.empty()) {
            LOG_ERROR_PRINTF("LaneDetection Image empty");
            pSample->Unlock(sample_buffer);
            return false;
        }
        points_detector_.DetectPoints(img, &detected_lines_v, &detected_lines_h,
                                                            &detection_lines);

        // IF_DEBUG_LOG_PRINTF("Line Detection found %li lines.\n",
        //                     detected_lines.size());
        // transmit output image
        if (video_output_pin_.IsConnected()) {
            video_output_pin_.Transmit(img, _clock->GetStreamTime());
        }
        pSample->Unlock(buffer);
        if (detected_lines_v.empty()) return false;

        // vectors of lanes filled by lanefinder
        std::vector<Lane> vert_lanes;
        std::vector<Lane> vert_lanes_temp;
        std::vector<Lane> horiz_lanes;
        std::vector<Lane> horiz_lanes_temp;
        // call lanefinder twice to get horizontal and vertical lines separately
        if (!lane_finder_.getLanes(detected_lines_v, &vert_lanes,
                                                    &horiz_lanes_temp)) {
            return false;
        }
        TransmitLaneVec(vert_lanes, pSample->GetTime());

        if (lane_finder_.getLanes(detected_lines_h, &vert_lanes_temp,
                                                    &horiz_lanes)) {
            tCrossing crossing;
            tPath corner_points;
            if (lane_finder_.detectCrossing(vert_lanes, horiz_lanes,
                    &crossing, &corner_points)) {
                 AddCornersToMap(corner_points);
                 TransmitCrossing(crossing, pSample->GetTime());
            }
        }
        // IF_DEBUG_LOG_PRINTF(
        //    "LaneFinder found %li horizontal and %li vertical"
        //    "lines.\n",
        //    horiz_lanes.size(), vert_lanes.size());

        detected_lines_v.insert(detected_lines_v.begin(),
                                detected_lines_h.begin(),
                                detected_lines_h.end());

        DisplayDetections(detection_lines, detected_lines_v,
                            vert_lanes, horiz_lanes);
        // last_time_ = _clock->GetStreamTime();
    //}
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void LaneDetection::DisplayDetections(
        const std::vector<tInt>& detection_lines,
        const std::vector<std::vector<tPoint> >& detected_lines,
        const std::vector<Lane>& vert_lanes,
        const std::vector<Lane>& horiz_lanes) {

    // sample lanes for map and debug image display
    std::vector<std::vector<tMapPoint> > sampled_lanes;
    SampleLanes(vert_lanes, horiz_lanes, &sampled_lanes);

    // IF_DEBUG_LOG_PRINTF("%li lines written to map\n", sampled_lanes.size());

    if (_clock->GetStreamTime() - last_time_ < 0.2 * 1e6) return;
    last_time_ = _clock->GetStreamTime();

    if (property_debug_enabled_) {
        AddLanesToMap(sampled_lanes);
    }

    if (gcl_output_pin_.IsConnected() && !detected_lines.empty()) {
        if (!sampled_lanes.empty()) {
            TransmitGCL(detection_lines, detected_lines, sampled_lanes, true);
        } else {
            TransmitGCL(detection_lines, detected_lines, sampled_lanes, false);
        }
    }
}

// ____________________________________________________________________________
tResult LaneDetection::TransmitGCL(
    const vector<tInt>& detectionLines,
    const std::vector<std::vector<tPoint> >& detectedLinePoints,
    const std::vector<std::vector<tMapPoint> >& sampled_lanes, bool has_lane) {
    IDynamicMemoryBlock* pGCLCmdDebugInfo;

    cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);

    // set color
    cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
                             cColor::Red.GetRGBA());
    // show roi within whole image
    cGCLWriter::StoreCommand(
        pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, filter_properties_.roi_offset_x,
        filter_properties_.roi_offset_y,
        filter_properties_.roi_offset_x + filter_properties_.roi_width,
        filter_properties_.roi_offset_y + filter_properties_.roi_height);

    // show detection lines for whole image
    cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
                             cColor::Yellow.GetRGBA());
    for (vector<tInt>::const_iterator it = detectionLines.begin();
         it != detectionLines.end(); it++) {
        cGCLWriter::StoreCommand(
            pGCLCmdDebugInfo, GCL_CMD_DRAWLINE, filter_properties_.roi_offset_x,
            *it, filter_properties_.roi_offset_x + filter_properties_.roi_width,
            *it);
    }

    /// draw detected lane polygon (transform coords back to img coords)
    if (has_lane) {
        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
                                 cColor::Green.GetRGBA());
        for (std::vector<std::vector<tMapPoint> >::const_iterator lane =
                 sampled_lanes.begin();
                lane != sampled_lanes.end(); ++lane) {
            if (lane->size() < 1) continue;
            for (std::vector<tMapPoint>::const_iterator sp1 = lane->begin();
                        sp1 < lane->end() - 1; ++sp1) {
                std::vector<tMapPoint>::const_iterator sp2 = sp1 + 1;
                cv::Point p1, p2;
                transform_helper_.StreetToStreetImage(sp1->x(), sp1->y(), &p1);
                transform_helper_.StreetToStreetImage(sp2->x(), sp2->y(), &p2);
                cGCLWriter::StoreCommand(
                    pGCLCmdDebugInfo, GCL_CMD_DRAWLINE,
                    p1.x, p1.y, p2.x, p2.y);
            }
        }
    }

    /// show detected line points
    /// use different colors for different lists
    int i = 0;
    for (std::vector<std::vector<tPoint> >::const_iterator line =
             detectedLinePoints.begin();
         line != detectedLinePoints.end(); line++) {
        cv::Point3i color_point = color_vec_[i];
        cColor list_color(color_point.x, color_point.y,
                          color_point.z);
        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
                                 list_color.GetRGBA());

        // glc(x,y) fill with glc(it.y, it.x)
        // because different coord system orientation
        for (vector<tPoint>::const_iterator it = line->begin();
                it != line->end(); it++) {
            cv::Point street_img_point;
            transform_helper_.StreetToStreetImage(*it, &street_img_point);
            cGCLWriter::StoreCommand(
                pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE,
                street_img_point.x,
                street_img_point.y,
                points_detector_.MeterToPixel(0.015));  // radius of circle
        }
        if (i < detectedLinePoints.size()) ++i;
    }

    // this point is for testing if the transformation works correctly
    cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
                             cColor::Blue.GetRGBA());
    cv::Point street_img_point;
    transform_helper_.StreetToStreetImage(filter_properties_.debug_pnt_x,
                        filter_properties_.debug_pnt_y, &street_img_point);
    cGCLWriter::StoreCommand(
        pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE,
        street_img_point.x,
        street_img_point.y,
        points_detector_.MeterToPixel(0.015));

    cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
                             cColor::Blue.GetRGBA());
    // show minimum and maximum line width
    cGCLWriter::StoreCommand(
        pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, filter_properties_.roi_offset_x,
        filter_properties_.roi_offset_y,
        filter_properties_.roi_offset_x +
            points_detector_.MeterToPixel(filter_properties_.max_line_width),
        filter_properties_.roi_offset_y - 20);
    cGCLWriter::StoreCommand(
        pGCLCmdDebugInfo, GCL_CMD_DRAWRECT,
        filter_properties_.roi_offset_x + filter_properties_.roi_width -
            points_detector_.MeterToPixel(filter_properties_.max_line_width),
        filter_properties_.roi_offset_y,
        filter_properties_.roi_offset_x + filter_properties_.roi_width,
        filter_properties_.roi_offset_y - 20);

    // show minimum and maximum line width at left and right roi
    cGCLWriter::StoreCommand(
        pGCLCmdDebugInfo, GCL_CMD_DRAWRECT,
        filter_properties_.roi_offset_x + filter_properties_.roi_width -
            points_detector_.MeterToPixel(filter_properties_.min_line_width),
        filter_properties_.roi_offset_y - 20,
        filter_properties_.roi_offset_x + filter_properties_.roi_width,
        filter_properties_.roi_offset_y - 40);
    cGCLWriter::StoreCommand(
        pGCLCmdDebugInfo, GCL_CMD_DRAWRECT, filter_properties_.roi_offset_x,
        filter_properties_.roi_offset_y - 20,
        filter_properties_.roi_offset_x +
            points_detector_.MeterToPixel(filter_properties_.min_line_width),
        filter_properties_.roi_offset_y - 40);

    cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);

    // alloc media sample and transmit it over output pin
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));
    RETURN_IF_FAILED(pSample->Update(
        _clock->GetStreamTime(), pGCLCmdDebugInfo->GetPtr(),
        (tInt)pGCLCmdDebugInfo->GetSize(), IMediaSample::MSF_None));
    RETURN_IF_FAILED(gcl_output_pin_.Transmit(pSample));

    cGCLWriter::FreeDynamicMemoryBlock(pGCLCmdDebugInfo);
    RETURN_NOERROR;
}

// _____________________________________________________________________________
tResult LaneDetection::TransmitLaneVec(const std::vector<Lane>& lane_vec,
                                        tTimeStamp sample_time) {
    int vec_size = lane_vec.size();

    for (int i = 0; i < vec_size; ++i) {
        vector<const tVoid*> vals2 = boost::assign::list_of
            ((const tVoid*)&vec_size)((const tVoid*)&i)(
                (const tVoid*)&lane_vec[i].no_of_samples)(
                (const tVoid*)&lane_vec[i].dist)(
                (const tVoid*)&lane_vec[i].coeff[0])(
                (const tVoid*)&lane_vec[i].coeff[1])(
                (const tVoid*)&lane_vec[i].coeff[2]);

        if (IS_FAILED(tLaneElement_output_pin_.Transmit(
                vals2, sample_time))) {
            LOG_ERROR_PRINTF("failed sending Lane ");
        }
    }
    RETURN_NOERROR;
}

// _____________________________________________________________________________
tResult LaneDetection::TransmitCrossing(const tCrossing& crossing,
                                        tTimeStamp sample_time) {
    vector<const tVoid*> vals =
        boost::assign::list_of((const tVoid*)&crossing.x)
                              ((const tVoid*)&crossing.y)
                              ((const tVoid*)&crossing.heading)
                              ((const tVoid*)&crossing.accuracy);
    if (IS_FAILED(tCrossing_output_pin_.Transmit(vals, sample_time))) {
        LOG_ERROR("Failed sending Crossing.");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void LaneDetection::SampleLanes(
        const std::vector<Lane>& vert_lanes,
        const std::vector<Lane>& horiz_lanes,
        std::vector<std::vector<tMapPoint> >* sampled_lanes) {
    if (!sampled_lanes) {
        printf("LaneDetection::SampleLanes failed!\n");
        return;
    }
    const float line_width_half = 0.015;
    /// horizontal lines:
    for (std::vector<Lane>::const_iterator h_lane = horiz_lanes.begin();
            h_lane < horiz_lanes.end(); ++h_lane) {
        std::vector<tMapPoint> temp;
        temp.push_back(tMapPoint(
            h_lane->start_pnt.x + line_width_half, h_lane->start_pnt.y));
        temp.push_back(tMapPoint(
            h_lane->end_pnt.x + line_width_half, h_lane->end_pnt.y));
        temp.push_back(tMapPoint(
            h_lane->end_pnt.x - line_width_half, h_lane->end_pnt.y));
        temp.push_back(tMapPoint(
            h_lane->start_pnt.x - line_width_half, h_lane->start_pnt.y));
        sampled_lanes->push_back(temp);
    }

    // vertical lines:
    // create lane polygon
    for (std::vector<Lane>::const_iterator lane = vert_lanes.begin();
         lane != vert_lanes.end(); ++lane) {
        int x_near = static_cast<int>(lane->start_pnt.x * 10);
        int x_far = static_cast<int>(lane->end_pnt.x * 10);
        std::vector<tMapPoint> temp_left;
        std::vector<tMapPoint> temp_right;
        for (int ix = x_near; ix < x_far; ++ix) {
            float x = ix / 10.0;
            float y_at_ix = lane_finder_.getValAtX(x, lane->coeff);

            temp_left.push_back(tMapPoint(x, y_at_ix + line_width_half));
            temp_right.push_back(tMapPoint(x, y_at_ix - line_width_half));
        }
        // flip temp right by 180 (last to first) and add to temp left, then
        // push back temp
        std::reverse(temp_right.begin(), temp_right.end());
        temp_left.insert(temp_left.end(), temp_right.begin(), temp_right.end());
        if (temp_left.empty()) continue;
        sampled_lanes->push_back(temp_left);
    }
}

// ____________________________________________________________________________
bool LaneDetection::AddLanesToMap(
    std::vector<std::vector<tMapPoint> >& sampled_lanes) {
    // make vec const->requires markus to make the reference in MapElem() const

    tTimeMapStamp t = _clock->GetStreamTime();
    for (std::vector<std::vector<tMapPoint> >::iterator sampled_lane =
             sampled_lanes.begin();
         sampled_lane != sampled_lanes.end(); ++sampled_lane) {
        // always create new shared_pointer
        tSptrMapElement lane_poly(
            new MapElement(STREET_MARKER_LANE, *sampled_lane, t));

        map_->AddFuseElement(lane_poly,  // add the poly to the map
                             0.2,        // max distances to fuse
                             -1, _clock->GetStreamTime(), MAP_FUSE_REPLACE);
        lane_poly->EnableTimeOfLife(1. * 1e6, &t);

        lane_poly->user_color_ui_ = "green";
    }
    return true;
}

// ____________________________________________________________________________
void LaneDetection::LoadConfigFile() {
    std::string config_file_path(
        GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME));
    std::string sensor_name(
        GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME));

    transform_helper_ = CameraTransformations(config_file_path, sensor_name);
    points_detector_.SetCameraTransformationsObj(transform_helper_);
}

// ____________________________________________________________________________
void LaneDetection::CheckFilterProperties(int rows, int cols) {
    if ((filter_properties_.roi_offset_x + filter_properties_.roi_width)
            > cols ||
            (filter_properties_.roi_offset_y + filter_properties_.roi_height)
            > rows) {
        LOG_WARNING_PRINTF(
            "LinePointsDetection: ROI is bigger than transformed image."
            " ROI is set to incoming image size");
        filter_properties_.roi_offset_x = 0;
        filter_properties_.roi_offset_y = 0;
        filter_properties_.roi_height = rows;
        filter_properties_.roi_width = cols;
        points_detector_.SetRoiHeight(rows);
        points_detector_.SetRoiWidth(cols);
        points_detector_.SetRoiOffsetY(0);
        points_detector_.SetRoiOffsetX(0);
    }
}

void LaneDetection::AddCornersToMap(const tPath &corner_points) {
    tTimeMapStamp t = _clock->GetStreamTime();
    for (int i = 0; i < corner_points.size(); ++i) {
        MapHelper::AddFuseDebugPointToMap(
            map_, corner_points[i].x, corner_points[i].y,
            t,       // creation time
            0.0001,  // fuse distance
            -1,
            1e6,     // life_time_microseconds
            "green");
    }
}
