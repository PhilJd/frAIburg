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

#include "us_to_map_filter.h"

using namespace frAIburg::map;

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (property_debug_enabled_) {   \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_TEMPLATE_FILTER_NAME, OID_ADTF_TEMPLATE_FILTER,
                   USToMapFilter);

USToMapFilter::USToMapFilter(const tChar* __info)
    : cFilter(__info), frAIburg::utils::CircularBufferEventListener() {
    // using convenience method for configuring dynamic connection pins ...
    ConfigureConnectionPins(0);
    SetAllProperties();
    pin_status_enabled_us_to_map_ = false;
}

// ____________________________________________________________________________
USToMapFilter::~USToMapFilter() {}

// ____________________________________________________________________________
tResult USToMapFilter::Init(tInitStage stage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

    tResult nResult = ERR_NOERROR;

    if (stage == StageFirst) {
        GetAllStaticProperties();
        // in StageFirst you can create and register your static pins.
        nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");

    } else if (stage == StageNormal) {
        // In this stage you would do further initialisation and/or create your
        // dynamic pins.
        // TODO set calibration based on pin name

    } else if (stage == StageGraphReady) {
        // query your pins about their media types and additional meta data.
        InitCalibrationBuffers();
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult USToMapFilter::Shutdown(tInitStage stage, __exception) {
    // In each stage clean up everything that you initialized in the
    // corresponding stage during Init. Pins are an exception:
    // - The base class takes care of static pins that are members of this
    //   class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please
    //   see the demo_dynamicpin example for further reference.
    if (stage == StageGraphReady) {
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult USToMapFilter::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    // if pin is connected use the enable function,
    // if not enable all us inputs to bufffer
    property_use_us_pin_enabled_ = pin_in_enabled_us_to_map_.pin_.IsConnected();
    IF_DEBUG_LOG_PRINTF("USToMapFilter poperties: enable pin is used "
                        "and enabled set to: %d ",
                        property_use_us_pin_enabled_);
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult USToMapFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                  tInt param2, IMediaSample* media_sample) {
    // first check what kind of event it is
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(media_sample);

        ProcessDynamicdata(source, media_sample, dynamic_pins_input_us_);
    }
    RETURN_NOERROR;
}
// ____________________________________________________________________________
tResult USToMapFilter::CreateInputPins(__exception) {
    slim::register_pin_func func = &USToMapFilter::RegisterPin;
    RETURN_IF_FAILED(pin_in_enabled_us_to_map_.FirstStageCreate(
        this, func, "enable", "tBoolSignalValue"));
    // ultrasonic inputs
    RETURN_IF_FAILED(dynamic_pins_input_us_.StageFirstCreate(
        this, func, slim::FIXED_MEDIA_TYPE, "tSignalValue"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult USToMapFilter::CreateOutputPins(__exception) { RETURN_NOERROR; }

// ____________________________________________________________________________
tResult USToMapFilter::SetPinIDs() {
    vector<string> ids = boost::assign::list_of("bValue");
    RETURN_IF_FAILED(pin_in_enabled_us_to_map_.StageGraphReadySetIDOrder(ids));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void USToMapFilter::GetAllStaticProperties(void) {
    property_us_map_time_of_life_sec_ =
        GetPropertyFloat(ADTF_PROPERTY_US_MAP_TIME_OF_LIFE_NAME);
    property_us_map_fuse_max_distance_meter_ =
        GetPropertyFloat(ADTF_PROPERTY_US_MAP_FUSE_MAX_DISTANCE_NAME);
    property_us_map_fuse_area_diff_meter_ =
        GetPropertyFloat(ADTF_PROPERTY_US_MAP_FUSE_AREA_NAME);

    property_us_map_fuse_area_diff_meter_ =
        GetPropertyFloat(ADTF_PROPERTY_US_MAP_FUSE_AREA_NAME);

    IF_DEBUG_LOG_PRINTF(
        "USToMapFilter poperties: "
        "us_map_time_of_life_sec = %.1f "
        "us_map_fuse_max_distance_meter = %.1f "
        "us_map_fuse_area_diff_meterc = %.1f",
        property_us_map_time_of_life_sec_,
        property_us_map_fuse_max_distance_meter_,
        property_us_map_fuse_area_diff_meter_);
}

// ____________________________________________________________________________
void USToMapFilter::SetAllProperties(void) {
    // filter debug mode changeable
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME,
                    ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT);
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyInt(ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_NAME,
                   ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_DEFAULT);

    SetPropertyFloat(ADTF_PROPERTY_US_MAX_VAL_TO_NAME,
                     ADTF_PROPERTY_US_MAX_VAL_TO_BUFFER);

    SetPropertyFloat(ADTF_PROPERTY_US_MAP_TIME_OF_LIFE_NAME,
                     ADTF_PROPERTY_US_MAP_TIME_OF_LIFE_DEFAULT);

    SetPropertyFloat(ADTF_PROPERTY_US_MAP_FUSE_MAX_DISTANCE_NAME,
                     ADTF_PROPERTY_US_MAP_FUSE_MAX_DISTANCE_DEFAULT);

    SetPropertyFloat(ADTF_PROPERTY_US_MAP_FUSE_AREA_NAME,
                     ADTF_PROPERTY_US_MAP_FUSE_AREA_DEFAULT);

    // frAIburg xml configuration
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME,
                   ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT);
    SetPropertyBool(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_FILENAME,
                    tTrue);
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_FILENAME
                       NSSUBSUBPROP_EXTENSIONFILTER,
                   "XML Files (*.xml)");
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_DESCRIPTION,
                   "Configuration file for camera postion offset");
}

// ____________________________________________________________________________
tResult USToMapFilter::PropertyChanged(const tChar* str_name) {
    RETURN_IF_FAILED(cFilter::PropertyChanged(str_name));
    // all properties with NSSUBPROP_ISCHANGEABLE
    // property changed in filter mask
    if (cString::IsEqual(str_name, ADTF_PROPERTY_DEBUG_ENABLED_NAME)) {
        property_debug_enabled_ =
            GetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME);
    }
    IF_DEBUG_LOG_PRINTF("PropertyChanged %s", str_name);
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult USToMapFilter::Connect(IPin* source, const tChar* strDestName,
                               __exception) {
    THROW_IF_POINTER_NULL(source);
    // connect all dynamicpin witht there media type here
    // LOG_INFO(A_UTILS_NS::cString::Format("connect new pin: %s",strDestName));
    if (dynamic_pins_input_us_.IsEqualMediaType(source)) {
        if (IS_FAILED(
                dynamic_pins_input_us_.StageConnect(source, strDestName))) {
            LOG_ERROR("dyn pin failed connect");
        }
    }
    return (cFilter::Connect(source, strDestName, __exception_ptr));
}



// ____________________________________________________________________________
tResult USToMapFilter::ProcessDynamicdata(IPin* source,
                                          IMediaSample* media_sample,
                                          slim::DynamicInputPin& dpins) {
    // check all the dynamic pins for data and save it the the corres modning
    // vector
    uint32_t index = 0;
    if (pin_in_enabled_us_to_map_.isSource(source)) {
        // enable or disable the filter
        const vector<tVoid*> rx_buff_ =
            boost::assign::list_of((tVoid*)&pin_status_enabled_us_to_map_);

        if (IS_OK( pin_in_enabled_us_to_map_
            .ReadIDcopy(media_sample, rx_buff_)))
        {
            IF_DEBUG_LOG_PRINTF("us enable changed: %d",
                                pin_status_enabled_us_to_map_);

        } else {
            LOG_ERROR("faild reading us to map enabled");
        }

    } else if ((pin_status_enabled_us_to_map_ ||
                !property_use_us_pin_enabled_) &&
               dpins.IsSource(source, &index)) {
        // Ultrasonic input data received and enabled
        cObjectPtr<cDynamicInputPin> pin;
        dpins.getDynamicPin(index, pin);
        tSignalValue* data;
        if (IS_OK(dpins.OnPinEventRead_start(media_sample, (const tVoid**)&data,
                                             sizeof(tSignalValue)))) {
            std::string name;
            dpins.getDynamicPinName(index, name);
            // LOG_INFO(A_UTILS_NS::cString::Format("index: %d,name %s",
            //                index,
            //                //pin->GetName(),
            //                name.c_str()));
            if (index < us_calibrated_buffers_.size()) {
                // data add to buffer if in limits
                us_calibrated_buffers_[index].PushBackCalibrate(data->f32Value);

            } else {
                LOG_ERROR(A_UTILS_NS::cString::Format(
                    "USToMapFilter: index %d"
                    " out of size for buffer array size %d",
                    index, us_calibrated_buffers_.size()));
            }

            dpins.OnPinEventRead_end(media_sample, (const tVoid**)&data);

        } else {
            LOG_ERROR("read dynamic pin failed, check types");
            RETURN_ERROR(ERR_IO_INCOMPLETE);
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void USToMapFilter::InitCalibrationBuffers() {
    // buffer id, pin index, order in us_calibrated_buffers_ are the same
    // pin name is also the sensor target
    int bullfer_size = GetPropertyInt(ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_NAME);
    float min_limit = 0;
    float max_limit = GetPropertyFloat(ADTF_PROPERTY_US_MAX_VAL_TO_NAME);

    std::string config_file(GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME));
    for (uint32_t i = 0; i < dynamic_pins_input_us_.getDynamicPinCnt(); i++) {
        std::string name;
        if (IS_OK(dynamic_pins_input_us_.getDynamicPinName(i, name))) {
            IF_DEBUG_LOG_PRINTF(
                "USToMapFilter configuration "
                "for sensor target: %s",
                name.c_str());

            us_calibrated_buffers_.push_back(tUSFilterBuffer(
                config_file.c_str(), name.c_str(), bullfer_size, this,
                i,  // pin index is buffer id for callback
                &min_limit, &max_limit));
        } else {
            LOG_ERROR("failed reading dynamic pin name");
        }
    }
}

void USToMapFilter::EventBufferFilled(int id) {
    // float mean =us_calibrated_buffers_[id].Mean();
    // LOG_INFO(A_UTILS_NS::cString::Format("EventBufferFilled event index:
    // %d,mean %f",
    //              id,mean));
}
void USToMapFilter::EventCycleCompleat(int id) {
    if (id < static_cast<int>(us_calibrated_buffers_.size())) {
        float mean = us_calibrated_buffers_[id].Mean();

        tSptrMapElement el_us =
            AddUSToMAP(mean, us_calibrated_buffers_[id].trans_x_,
                       us_calibrated_buffers_[id].trans_y_,
                       us_calibrated_buffers_[id].rotation_rad_,
                       us_calibrated_buffers_[id].field_of_view_angle_rad_);

        el_us->user_tag_ui_ = us_calibrated_buffers_[id].us_name_;

        // IF_DEBUG_LOG_PRINTF("USToMapFilter buffer cycle "
        //           "compleate for %s, mean %.3f meter, rotation %.2f rad,"
        //           " max %.2f, min %.2f",
        //           us_calibrated_buffers_[id].us_name_.c_str(),mean,
        //           us_calibrated_buffers_[id].rotation_rad_,
        //           us_calibrated_buffers_[id].Max(),
        //           us_calibrated_buffers_[id].Min());

    } else {
        LOG_ERROR("error buffer id");
    }
}

tSptrMapElement USToMapFilter::AddUSToMAP(float mean_dist, float trans_x,
                                          float trans_y, float rot,
                                          float field_of_view_angle_rad) {
    const float center_sensor_frame_x = mean_dist;
    const tMapData box_angle = -rot - M_PI_2;
    const float center_sensor_frame_y = 0;
    // roatation
    const float center_sensor_frame_rot_x =
        center_sensor_frame_x * cos(rot) + center_sensor_frame_y * sin(rot);
    const float center_sensor_frame_rot_y =
        center_sensor_frame_y * cos(rot) + center_sensor_frame_x * sin(rot);
    // trans
    const float local_x = center_sensor_frame_rot_x + trans_x;
    const float local_y = center_sensor_frame_rot_y + trans_y;
    // box size x
    const float width_half_x = tan(field_of_view_angle_rad * 0.5) * mean_dist;
    std::vector<tMapPoint> v1 =
        boost::assign::list_of(tMapPoint(local_x, local_y));
    tSptrMapElement el_us = MapHelper::CreatBox(
        ULTRRASONIC, local_x, local_y,              // center pos
        width_half_x,                               // box size
        0.05, _clock->GetStreamTime(), &box_angle); /*box_local_orientation*/

    // TODO(markus) fuse option and EnableTimeOfLife
    frAIburg::map::getInstance()->AddFuseElement(
        el_us, property_us_map_fuse_max_distance_meter_,
        property_us_map_fuse_area_diff_meter_, _clock->GetStreamTime(),
        MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN);

    if (property_us_map_time_of_life_sec_ != -1)
        el_us->EnableTimeOfLife(property_us_map_time_of_life_sec_ * 1e6);
    if (property_debug_enabled_) {
        // change color with update in debug modus
        static const std::vector<const char*> color_change_debug =
            boost::assign::list_of("darkseagreen")("darkmagenta")(
                "palevioletred");
        static unsigned int color_index = 0;
        if (++color_index == color_change_debug.size()) color_index = 0;
        el_us->user_color_ui_ = color_change_debug[color_index];
    }
    return el_us;
}
