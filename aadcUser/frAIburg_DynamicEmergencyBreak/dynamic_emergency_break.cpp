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

#include "dynamic_emergency_break.h"

namespace fm = frAIburg::map;

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (poperty_debug_enabled_) {    \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_TEMPLATE_FILTER_NAME, OID_ADTF_TEMPLATE_FILTER,
                   DynamicEmergencyBreak);

DynamicEmergencyBreak::DynamicEmergencyBreak(const tChar* __info)
    : cFilter(__info), frAIburg::utils::CircularBufferEventListener() {
    // using convenience method for configuring dynamic connection pins ...
    ConfigureConnectionPins(0);
    SetAllProperties();
    current_car_pos_x_ = 0;
    current_car_pos_y_ = 0;
    current_car_curvature_ = 0.;
    current_car_speed_ = 0.;
    emergeny_state_triggerd_time_ = 0;
    last_jury_obstacle_trans_time_ = -1;
    emergeny_state_triggerd_ = false;
    last_object_ahead_id_ = MAP_DEFAULT_ID;
    pos_rx_buff_ = boost::assign::list_of((tVoid*)&current_car_pos_x_)(
        (tVoid*)&current_car_pos_y_);
}

// ____________________________________________________________________________
DynamicEmergencyBreak::~DynamicEmergencyBreak() {}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::Init(tInitStage stage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

    tResult nResult = ERR_NOERROR;

    if (stage == StageFirst) {
        GetAllProperties();
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
        map_ = frAIburg::map::getInstance();
        // query your pins about their media types and additional meta data.
        InitCalibrationBuffers();
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");

        InitTimer();
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::Shutdown(tInitStage stage, __exception) {
    // In each stage clean up everything that you initialized in the
    // corresponding stage during Init. Pins are an exception:
    // - The base class takes care of static pins that are members of this
    //   class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please
    //   see the demo_dynamicpin example for further reference.
    if (stage == StageGraphReady) {
        DeleteTimer();
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    running_ok_ = true;
    if (poperty_debug_enabled_ && car_pos_input_pin_.pin_.IsConnected()) {
        DebugShowDrivableAreaInMap();
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::Stop(__exception) {
    running_ok_ = false;
    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::OnPinEvent(IPin* source, tInt event_code,
                                           tInt param1, tInt param2,
                                           IMediaSample* media_sample) {
    // first check what kind of event it is
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(media_sample);

        ProcessDynamicdata(source, media_sample, dynamic_pins_input_us_);
        ProcessPosData(source, media_sample);
        ProcessCurvatureData(source, media_sample);
        ProcessSpeedData(source, media_sample);
    }
    RETURN_NOERROR;
}

void DynamicEmergencyBreak::ProcessPosData(IPin* source,
                                            IMediaSample* mediaSample) {
    if (car_pos_input_pin_.isSource(source)) {
        car_pos_input_pin_.ReadIDcopy(mediaSample, pos_rx_buff_);
        bool lim_reachted = false;
        if (current_car_pos_x_ < config_pos_limit_x_min_ ||
            current_car_pos_x_ > config_pos_limit_x_max_) {
            // x lim reached
            lim_reachted = true;
        } else if (current_car_pos_y_ < config_pos_limit_y_min_ ||
                   current_car_pos_y_ > config_pos_limit_y_max_) {
            // y limit reached
            lim_reachted = true;
        }

        if (lim_reachted) {
            TransmitEmergencySignals();
            // IF_DEBUG_LOG_PRINTF("lim reached pos x %0.2f, pos y %0.2f",
            //         current_car_pos_x_,current_car_pos_y_);
        }
    }
}

void DynamicEmergencyBreak::ProcessCurvatureData(IPin* source,
                                                  IMediaSample* mediaSample) {
    if (car_curvature_input_pin_.isSource(source)) {
        tSignalValue* sample_data = NULL;
        if (IS_OK(car_curvature_input_pin_.ReadNoID_start(
                mediaSample, (const tVoid**)&sample_data,
                sizeof(tSignalValue)))) {
            current_car_curvature_ = sample_data->f32Value;
            car_curvature_input_pin_.ReadNoID_end(mediaSample,
                                                  (const tVoid**)sample_data);
        }
    }
}
void DynamicEmergencyBreak::ProcessSpeedData(IPin* source,
                                              IMediaSample* mediaSample) {
    if (car_speed_input_pin_.isSource(source)) {
        tSignalValue* sample_data = NULL;
        if (IS_OK(car_speed_input_pin_.ReadNoID_start(
                mediaSample, (const tVoid**)&sample_data,
                sizeof(tSignalValue)))) {
            current_car_speed_ = sample_data->f32Value;
            car_speed_input_pin_.ReadNoID_end(mediaSample,
                                              (const tVoid**)sample_data);
        }
    }
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::CreateInputPins(__exception) {
    slim::register_pin_func func = &DynamicEmergencyBreak::RegisterPin;
    RETURN_IF_FAILED(dynamic_pins_input_us_.StageFirstCreate(
        this, func, slim::FIXED_MEDIA_TYPE, "tSignalValue"));
    RETURN_IF_FAILED(car_pos_input_pin_.FirstStageCreate(
        this, func, "car_Postion", "tPosition"));

    RETURN_IF_FAILED(car_curvature_input_pin_.FirstStageCreate(
        this, func, "curvature", "tSignalValue"));
    RETURN_IF_FAILED(car_speed_input_pin_.FirstStageCreate(
        this, func, "car_speed", "tSignalValue"));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::CreateOutputPins(__exception) {
    slim::register_pin_func func = &DynamicEmergencyBreak::RegisterPin;

    // create jury emergeny output pin
    RETURN_IF_FAILED(jury_bool_pinoutput_.FirstStageCreate(
        this, func, "EmergencyStopJuryBool", "tJuryEmergencyStop"));
    // create speed outout zero empergency
    RETURN_IF_FAILED(speed_pinoutput_.FirstStageCreate(
        this, func, "EmergencyStopspeed", "tSignalValue"));
    // create speed outout zero empergency
    RETURN_IF_FAILED(pin_out_behaviour_planner_.FirstStageCreate(
        this, func, "PlannerBehaviour", "tBehaviour"));

    RETURN_IF_FAILED(jury_obstacle_output_pin_.FirstStageCreate(
         this, func, "jury_obstacle", "tObstacle"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::SetPinIDs() {
    vector<string> ids =
        boost::assign::list_of("f32Value")("ui32ArduinoTimestamp");
    RETURN_IF_FAILED(speed_pinoutput_.StageGraphReadySetIDOrder(ids));

    vector<string> id2s = boost::assign::list_of("bEmergencyStop");
    RETURN_IF_FAILED(jury_bool_pinoutput_.StageGraphReadySetIDOrder(id2s));

    vector<string> names_subtypes_pos = boost::assign::list_of("f32x")("f32y");
    car_pos_input_pin_.StageGraphReadySetIDOrder(names_subtypes_pos);

    vector<string> id6s = boost::assign::list_of("f32x")("f32y");
    RETURN_IF_FAILED(jury_obstacle_output_pin_.StageGraphReadySetIDOrder(id6s));


    RETURN_NOERROR;
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::GetAllProperties(void) {
    poperty_debug_enabled_ = GetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME);
    poperty_collision_check_planner_path_enabled_ =
        GetPropertyBool(ADTF_PROPERTY_MAP_COLLISION_PLANNER_ENABLED);

    std::string filter_config_file(
        GetPropertyStr(ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_NAME));
    // get postion limits form xml form the filter config
    frAIburg::utils::XMLHelper filter_config_pos;
    if (filter_config_pos.ReadNameValue(filter_config_file.c_str(),
                                        "DynamicEmergencyFilter",
                                        "PositionLimits")) {
        if (filter_config_pos.GetValue<float>("threshold_meter_x_min")) {
            config_pos_limit_x_min_ =
                *filter_config_pos.GetValue<float>("threshold_meter_x_min");
            config_pos_limit_y_min_ =
                *filter_config_pos.GetValue<float>("threshold_meter_y_min");
            config_pos_limit_x_max_ =
                *filter_config_pos.GetValue<float>("threshold_meter_x_max");
            config_pos_limit_y_max_ =
                *filter_config_pos.GetValue<float>("threshold_meter_y_max");

            IF_DEBUG_LOG_PRINTF(
                "DynamicEmergency PositionLimits x_min: %.2f, "
                "x_max: %.2f, y_min: %.2f, y_max: %.2f",
                config_pos_limit_x_min_, config_pos_limit_x_max_,
                config_pos_limit_y_min_, config_pos_limit_y_max_);

            if (config_pos_limit_x_min_ > config_pos_limit_x_max_) {
                LOG_WARNING_PRINTF("Emergency F: wrong x min(%.2f) > max(%.2f)",
                                   config_pos_limit_x_min_,
                                   config_pos_limit_x_max_);
            }
            if (config_pos_limit_y_min_ > config_pos_limit_y_max_) {
                LOG_WARNING_PRINTF("Emergency F: wrong y min(%.2f) > max(%.2f)",
                                   config_pos_limit_y_min_,
                                   config_pos_limit_y_max_);
            }
        } else {
            LOG_ERROR_PRINTF(
                "Emergency failed us threshold form config:threshold_meter");
        }

    } else {
        LOG_ERROR_PRINTF(
            "Emergency F. failed reading PositionLimits form config");
    }
    // get map collision box
    frAIburg::utils::XMLHelper filter_config_box;
    if (filter_config_box.ReadNameValue(filter_config_file.c_str(),
                                        "DynamicEmergencyFilter",
                                        "MapCollisionCheckBox")) {
        namespace fr = frAIburg::map;
        if (filter_config_box.GetValue<float>("center_local_meter_x")) {
            config_collision_box_local_center_x_ =
                *filter_config_box.GetValue<fr::tMapData>(
                    "center_local_meter_x");
            config_collision_box_local_center_y_ =
                *filter_config_box.GetValue<fr::tMapData>(
                    "center_local_meter_y");
            config_collision_box_size_half_x_ =
                *filter_config_box.GetValue<fr::tMapData>("size_half_meter_x");
            config_collision_box_size_half_y_ =
                *filter_config_box.GetValue<fr::tMapData>("size_half_meter_y");
            config_collision_box_angle_rad_ =
                (M_PI / 180) *
                *filter_config_box.GetValue<fr::tMapData>(
                    "box_orientation_clockwise_degree");
            IF_DEBUG_LOG_PRINTF(
                "DynamicEmergency colltion box x_center: %.2f, "
                "y_center: %.2f",
                config_collision_box_local_center_x_,
                config_collision_box_local_center_y_);
        } else {
            LOG_ERROR_PRINTF("failed us threshold form config:threshold_meter");
        }

    } else {
        LOG_ERROR_PRINTF("failed reading PositionLimits form config");
    }
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::SetAllProperties(void) {
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME,
                    ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT);

    SetPropertyInt(ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_NAME,
                   ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_DEFAULT);

    SetPropertyFloat(ADTF_PROPERTY_US_MAX_VAL_TO_NAME,
                     ADTF_PROPERTY_US_MAX_VAL_TO_BUFFER);
    // frAIburg xml configuration
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME,
                   ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT);
    SetPropertyBool(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_FILENAME,
                    tTrue);
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_FILENAME
                       NSSUBSUBPROP_EXTENSIONFILTER,
                   "XML Files (*.xml)");
    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME NSSUBPROP_DESCRIPTION,
                   "Configuration file for us calibration");

    // filter xml configuration
    SetPropertyStr(ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_NAME,
                   ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_DEFAULT);
    SetPropertyBool(
        ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_NAME NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_NAME NSSUBPROP_FILENAME
                       NSSUBSUBPROP_EXTENSIONFILTER,
                   "XML Files (*.xml)");
    SetPropertyStr(
        ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_NAME NSSUBPROP_DESCRIPTION,
        "Configuration file for us thresholds and postion limits");

    // MAP box collition check
    SetPropertyBool(ADTF_PROPERTY_MAP_COLLISION_CHECK_ENABLED,
                    ADTF_PROPERTY_MAP_COLLISION_CHECK_ENABLED_DEFAULT);
    SetPropertyBool(ADTF_PROPERTY_MAP_COLLISION_PLANNER_ENABLED,
                    ADTF_PROPERTY_MAP_COLLISION_PLANNER_ENABLED_DEFAULT);
    SetPropertyFloat(ADTF_PROPERTY_MAP_COLLISION_CHECK_TIME_INTERVAL,
                     ADTF_PROPERTY_MAP_COLLISION_CHECK_TIME_INTERVAL_DEFAULT);
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::Connect(IPin* source, const tChar* strDestName,
                                        __exception) {
    THROW_IF_POINTER_NULL(source);
    // connect all dynamicpin witht there media type here
    // LOG_INFO(A_UTILS_NS::cString::Format("connect new pin: %s",strDestName));
    if (dynamic_pins_input_us_.IsEqualMediaType(source)) {
        if (IS_FAILED(
                dynamic_pins_input_us_.StageConnect(source, strDestName))) {
            LOG_ERROR_PRINTF("dyn pin failed connect");
        }
    }
    return (cFilter::Connect(source, strDestName, __exception_ptr));
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::ProcessDynamicdata(
    IPin* source, IMediaSample* media_sample, slim::DynamicInputPin& dpins) {
    // check all the dynamic pins for data and save it the the corres modning
    // vector
    uint32_t index;
    if (dpins.IsSource(source, &index)) {
        cObjectPtr<cDynamicInputPin> pin;
        dpins.getDynamicPin(index, pin);
        tSignalValue* data;
        if (IS_OK(dpins.OnPinEventRead_start(media_sample, (const tVoid**)&data,
                                             sizeof(tSignalValue)))) {
            std::string name;
            dpins.getDynamicPinName(index, name);

            if (index < us_calibrated_buffers_.size()) {
                // data add to buffer if in limits
                us_calibrated_buffers_[index].PushBackCalibrate(data->f32Value);

            } else {
                LOG_ERROR_PRINTF(
                    "DynamicEmergencyBreak: index %d"
                    " out of size for buffer array size %d",
                    index, us_calibrated_buffers_.size());
            }

            dpins.OnPinEventRead_end(media_sample, (const tVoid**)&data);

        } else {
            LOG_ERROR_PRINTF("read dynamic pin failed, check types");
            RETURN_ERROR(ERR_IO_INCOMPLETE);
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::InitCalibrationBuffers() {
    // init a ring buffer for each dynamic pin connection
    // buffer id, pin index, order in us_calibrated_buffers_ are the same
    // pin name is also the sensor target
    int bullfer_size = GetPropertyInt(ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_NAME);

    float max_limit = GetPropertyFloat(ADTF_PROPERTY_US_MAX_VAL_TO_NAME);
    float min_limit = 0;
    std::string filter_config_file(
        GetPropertyStr(ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_NAME));

    std::string config_file(GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME));
    for (uint32_t i = 0; i < dynamic_pins_input_us_.getDynamicPinCnt(); i++) {
        std::string name;
        if (IS_OK(dynamic_pins_input_us_.getDynamicPinName(i, name))) {
            // add ringbuffer for the pin with calibred factor, transformation
            // etc
            us_calibrated_buffers_.push_back(
                tUSFilterBuffer(config_file.c_str(), name.c_str(), bullfer_size,
                                this,  // calllbacks
                                i,     // pin index is buffer id for callback
                                &min_limit,  // TODO //no min limit to ensure
                                             // stop (minus vals if very close)
                                &max_limit));

            // get threshold form the filter config
            frAIburg::utils::XMLHelper filter_config;
            if (filter_config.ReadNameValue(filter_config_file.c_str(),
                                            "DynamicEmergencyBreak",
                                            name.c_str())) {
                if (filter_config.GetValue<float>("threshold_meter")) {
                    float threshold =
                        *filter_config.GetValue<float>("threshold_meter");
                    us_emergency_threshold_.push_back(threshold);

                    IF_DEBUG_LOG_PRINTF(
                        "DynamicEmergency threshold %.2f "
                        "for activ %s",
                        threshold, name.c_str());

                    if (max_limit <= threshold) {
                        LOG_WARNING_PRINTF(
                            "US threshold %f for %s is bigger than "
                            " filter max value %f",
                            threshold, name.c_str(), max_limit);
                    }
                } else {
                    us_emergency_threshold_.push_back(1);
                    LOG_ERROR_PRINTF(
                        "failed us threshold form config:threshold_meter");
                }

            } else {
                us_emergency_threshold_.push_back(1);
                LOG_ERROR_PRINTF("failed us threshold form config");
            }

        } else {
            us_emergency_threshold_.push_back(1);
            LOG_ERROR_PRINTF("failed reading dynamic pin name");
        }
    }
}

void DynamicEmergencyBreak::EventBufferFilled(int id) {
    // float mean =us_calibrated_buffers_[id].Mean();
    // LOG_INFO(A_UTILS_NS::cString::Format("EventBufferFilled event index:
    // %d,mean %f",
    //              id,mean));
}
void DynamicEmergencyBreak::EventCycleCompleat(int id) {
    if (id < static_cast<int>(us_calibrated_buffers_.size()) &&
        id < static_cast<int>(us_emergency_threshold_.size())) {
        float mean = us_calibrated_buffers_[id].Mean();
        float us_threshold = us_emergency_threshold_[id];

        if (mean <= us_threshold) {
            IF_DEBUG_LOG_PRINTF(
                "DynamicEmergencyBreak us threshold "
                "for %s reached, mean %.3f meter, threshold %.2f rad,"
                " max %.2f, min %.2f",
                us_calibrated_buffers_[id].us_name_.c_str(), mean, us_threshold,
                us_calibrated_buffers_[id].Max(),
                us_calibrated_buffers_[id].Min());
            TransmitEmergencySignals();
        }

    } else {
        LOG_ERROR_PRINTF("failed buffer id error");
    }
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::TransmitEmergencySignals(
    const frAIburg::map::tMapID* id /* = NULL*/) {
    emergeny_state_triggerd_time_ = _clock->GetStreamTime();
    if (!emergeny_state_triggerd_) {
        // semt
        if (IS_FAILED(jury_bool_pinoutput_.Transmit<tBool>(
                tTrue, _clock->GetStreamTime()))) {
            LOG_ERROR_PRINTF(
                "DynamicEmergencyBreak jury_bool_pinoutput_"
                " failed tansmit");
        } else {
            LOG_INFO_PRINTF("Emergency ON");
            emergeny_state_triggerd_ = true;
            // send planner Emergency signal
        }
        // send planner Emergency signal if collision with a map element

        TransmitPlannerSignal(EM_ENABLED, id);
    }
    // send the speed singal continuously to overwirte speed other speed signals
    tResult r = speed_pinoutput_.Transmit<tFloat32, tUInt32>(
        0., 0, _clock->GetStreamTime());
    // transmit speed zero
    if (IS_FAILED(r)) {
        LOG_ERROR_PRINTF(
            "DynamicEmergencyBreak speed_pinoutput_ "
            "failed tansmit");
    }
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::TransmitDisabledEmergencySignals() {
    if (emergeny_state_triggerd_) {
        TransmitPlannerSignal(EM_DISABLED);
        if (IS_FAILED(jury_bool_pinoutput_.Transmit<tBool>(
                tFalse, _clock->GetStreamTime()))) {
            LOG_ERROR_PRINTF(
                "DynamicEmergencyBreak jury_bool_pinoutput_"
                " failed tansmit");
        } else {
            LOG_INFO_PRINTF("Emergency OFF");
            emergeny_state_triggerd_ = false;
        }
    }
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::TransmitPlannerSignal(
    behaviour_type type, const frAIburg::map::tMapID* id) {
    tBehaviour planner_behaviour;
    planner_behaviour.behaviour_id = tInt32(type);
    planner_behaviour.behaviour_next_id = tInt32(BT_UNKNOWN);
    planner_behaviour.object_id = (id) ? tInt32(*id) : MAP_DEFAULT_ID;
    planner_behaviour.speed_id = tInt32(ST_STOP);
    planner_behaviour.timestamp = _clock->GetStreamTime();
    if (IS_FAILED(pin_out_behaviour_planner_.TransmitStruct<tBehaviour>(
            &planner_behaviour, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF(
            "DynamicEmergencyBreak planner_behaviour failed tansmit");
    }
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::TransmitObjectAhead(const fm::tMapID id) {
    if (last_object_ahead_id_ != id) {
        TransmitPlannerSignal(EM_OBSTACLE_AHEAD,
                              &id);  // TODO correct behaviour_id
        last_object_ahead_id_ = id;
    }
}

// ____________________________________________________________________________
fm::tSptrMapElement DynamicEmergencyBreak::GetFixedRectCollisionBox() {
    return fm::MapHelper::CreatBox(
        fm::DEBUG_POLYGON, config_collision_box_local_center_x_,
        config_collision_box_local_center_y_, config_collision_box_size_half_x_,
        config_collision_box_size_half_y_, _clock->GetStreamTime(),
        &config_collision_box_angle_rad_);
}

// ____________________________________________________________________________
fm::tSptrMapElement DynamicEmergencyBreak::GetAdaptiveCollisionBox() {
    /* Adaptive collision box: based on speed and curvature*/
    // worst case processing speed from depth image to speed controller
    const float sampling_time = 0.1;
    // 0.1m min. distance for collsion box + dist. between samples
    float x_dist = std::min(0.2 + 1. * fabs(current_car_speed_) * sampling_time,
                            1. * RADIUS_MIN_);
    float y_dist = 0.;
    float y_half_width = 0.17;
    std::vector<fm::tMapPoint> temp;
    // x_dist must always be smaller than radius, otherwise invalid sqrt()
    if (current_car_curvature_ > 0) {
        float radius = std::max(1. / current_car_curvature_, 1. * RADIUS_MIN_);
        // printf("DynamicEmergencyBreak: radius is %f \n", radius);
        y_dist = radius - sqrt(pow(radius, 2) - pow(x_dist, 2));
    } else if (current_car_curvature_ < 0) {
        float radius = std::min(1. / current_car_curvature_, -1. * RADIUS_MIN_);
        y_dist = radius + sqrt(pow(radius, 2) - pow(x_dist, 2));
    }  // todo: check that y is not too big!
    temp.push_back(fm::tMapPoint(0, y_half_width));
    temp.push_back(fm::tMapPoint(x_dist, y_dist + 0.8 * y_half_width));
    temp.push_back(fm::tMapPoint(x_dist, y_dist - 0.8 * y_half_width));
    temp.push_back(fm::tMapPoint(0, -y_half_width));

    return fm::tSptrMapElement(
        new fm::MapElement(fm::DEBUG_POLYGON, temp, _clock->GetStreamTime()));
}


// ____________________________________________________________________________
fm::tSptrMapElement DynamicEmergencyBreak::GetPlannerCollisionPath() {
    if (!planner_path_ || planner_path_->GetID() == MAP_DEFAULT_ID) {
        if (planner_path_)
          LOG_ERROR_PRINTF("EM planner path removed from map");
        std::vector<fm::tSptrMapElement> planner_paths;
        map_->GetAllElementsWithType(fm::PLANNER_PATH, planner_paths);
        if (planner_paths.size() == 1) {
            planner_path_ = planner_paths.front();
        } else if (planner_paths.size() > 1) {
            planner_path_ = planner_paths.back();//take newest
            LOG_WARNING_PRINTF("EM more then one planner path found");
        }else{
          planner_path_ = fm::tSptrMapElement();
          return fm::tSptrMapElement();  // smart null pointer
        }
    }
    // TODO check update time
    return planner_path_;
}

// ____________________________________________________________________________
fm::tSptrMapElement DynamicEmergencyBreak::MapBoxCollisionCheck(
    fm::tSptrMapElement map_collision_check_box) {
    if (!map_collision_check_box){
      LOG_ERROR_PRINTF("EM MapBoxCollisionCheck with null element");
      return fm::tSptrMapElement();
    }
    // create a map element and check for collision
    // tTimeStamp start_time_micro_s = _clock->GetStreamTime();
    fm::tSptrMapElement ret_el;
    if (poperty_debug_enabled_ &&
        map_collision_check_box->GetID() ==
            MAP_DEFAULT_ID)  // path was not added to map
    {
        // add box to map for debugging to show in the map ui
        map_->AddFuseElement(map_collision_check_box,
                             0.5,  // fuse optoion disatance
                             0,    // fuse option same area
                             _clock->GetStreamTime(), fm::MAP_FUSE_REPLACE);
        map_collision_check_box->user_tag_ui_ = "CollisionBox";
        map_collision_check_box->EnableTimeOfLife(0.5e6);
    }

    // list with collision checj exclude types
    // TODO exclude or include
    // for sign the depth is used for emergency break
    static const std::vector<fm::MapElementType> exclude_types =
        boost::assign::list_of(fm::DEBUG_POINT)
            (fm::DEBUG_POLYGON)(
            fm::STREET_MARKER_LANE)
            (fm::STREET_MARKER_LANE)(
            fm::STREET_MARKER_STOP_LINE)
            (fm::STREET_MARKER_CROSSING_T)(
            fm::STREET_MARKER_CROSSING_X)
            (fm::STREET_MARKER_ZEBRA)(
            fm::STREET_PARKING_SPOT)
            (fm::LM_STREET_LANE)
            (fm::UNKNOWN)
            (fm::UNDEFINED)
            (fm::PLANNER_PATH)
            (fm::LM_STREET_PARKING_SPOT)
            (fm::STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT)
            (fm::STREET_TRAFFIC_SIGN_STOP)
            (fm::STREET_TRAFFIC_SIGN_PARKING)
            (fm::STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC)
            (fm::STREET_TRAFFIC_SIGN_JUST_STRAIGHT)
            (fm::STREET_TRAFFIC_SIGN_GIVE_WAY)
            (fm::STREET_TRAFFIC_SIGN_CROSSWALK)
            (fm::STREET_TRAFFIC_SIGN_CIRCLE)
            (fm::STREET_TRAFFIC_SIGN_NO_TAKE_OVER)
            (fm::STREET_TRAFFIC_SIGN_NO_ENTRY)
            (fm::STREET_TRAFFIC_ONE_WAY_STREET)
            (fm::STREET_TRAFFIC_ROAD_WORKS)
            (fm::STREET_TRAFFIC_SIGN_SPEED_50)
            (fm::STREET_TRAFFIC_SIGN_SPEED_100)
            (fm::STREET_TRAFFIC_SIGN_POSITION_MARKER);

    std::vector<fm::tSptrMapElement> collision_els;
    // note has no assied id (-1)
    // if debug elements added from step before a collision is detected
    // ->fuse el to map before colltion check or exclude_types
    bool is_collision =
        map_->CheckElementsCollision(map_collision_check_box, collision_els,
                                     5,   // max range
                                     -1,  // return all
                                     &exclude_types);
    if (poperty_debug_enabled_) {
        // change color
        if (is_collision)
            map_collision_check_box->user_color_ui_ = "red";
        else
            map_collision_check_box->user_color_ui_ = "green";
    }
    // get the element with the minimal distane
    fm::tMapData distances_min = -1;
    BOOST_FOREACH (fm::tSptrMapElement& el, collision_els) {
        // set the return id to in element with the min distance to the car
        if (distances_min == -1 || el->GetDistanceToCar() < distances_min) {
            ret_el = el;
            distances_min = el->GetDistanceToCar();
        }
    }

    // IF_DEBUG_LOG_PRINTF("EmergencyFilte map collstion check time %.3f s with
    // %d "
    //                     "elements in map",
    //                     (end_time_micro_s - start_time_micro_s)/1e6,
    //                     map_->GetElementCnt());

    return ret_el;
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::InitTimer() {
    timer_ = NULL;
    if (GetPropertyBool(ADTF_PROPERTY_MAP_COLLISION_CHECK_ENABLED)) {
        tUInt32 nFlags = 0;
        // Create our cyclic timer.
        cString strTimerName = OIGetInstanceName();
        strTimerName += ".rttimerStateMachineTimer";
        tTimeStamp tmPeriod =
            1e6 *
            GetPropertyFloat(ADTF_PROPERTY_MAP_COLLISION_CHECK_TIME_INTERVAL);
        tTimeStamp tmstartdelay = 1e6 * 0.2;
        timer_ =
            _kernel->TimerCreate(tmPeriod,
                                 tmstartdelay,  // dely for the first start
                                 static_cast<IRunnable*>(this),  // call run
                                 NULL, NULL, 0, nFlags, strTimerName);
        if (!timer_) {
            LOG_ERROR_PRINTF("state machine filter unable to create timer");
        }
    }
}

// ____________________________________________________________________________
tResult DynamicEmergencyBreak::Run(tInt nActivationCode,
                                    const tVoid* pvUserData,
                                    tInt szUserDataSize,
                                    ucom::IException** __exception_ptr) {
    if (running_ok_) {
        // check the map for collision
        fm::tSptrMapElement map_el_collision_check;
        if (poperty_collision_check_planner_path_enabled_ &&
            GetPlannerCollisionPath()) {
            map_el_collision_check = GetPlannerCollisionPath();

        } else {
            map_el_collision_check = GetAdaptiveCollisionBox();
        }

        fm::tSptrMapElement collision_closest_el =
            MapBoxCollisionCheck(map_el_collision_check);

        if (collision_closest_el) {

            tTimeStamp t = _clock->GetStreamTime();
            if (!emergeny_state_triggerd_) {
                IF_DEBUG_LOG_PRINTF(
                    "emergency collision with map element %s,"
                    " added to map: %.4f s",
                    fm::MapHelper::TypeToString(
                        collision_closest_el->GetType()),
                    (t - collision_closest_el->GetCreateTime()) / 1e6);
            }

            fm::tMapData dist_emergency = 0.2 + 0.2*fabs(current_car_speed_);  // TODO with property
            if (dist_emergency > 0.45) {dist_emergency = 0.45;}

            const fm::tMapID id = collision_closest_el->GetID();
            if (collision_closest_el->GetDistanceToCar() < dist_emergency) {
                TransmitJuryObstacle(collision_closest_el);
                TransmitEmergencySignals(&id);
            } else {
                TransmitObjectAhead(id);
            }

        } else if (emergeny_state_triggerd_ &&
                   (_clock->GetStreamTime() - emergeny_state_triggerd_time_) /
                           1e6 >
                       1.) {
            // disable emergeny status if no emergeny was triggered
            // for 1s
            TransmitDisabledEmergencySignals();
        } else {
            // reset object ahead
            last_object_ahead_id_ = MAP_DEFAULT_ID;
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::DeleteTimer() {
    if (timer_) {
        running_ok_ = false;
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
    }
}

// ____________________________________________________________________________
void DynamicEmergencyBreak::DebugShowDrivableAreaInMap() {
    std::vector<fm::tMapPoint> drivable_box = boost::assign::list_of(
        fm::tMapPoint(config_pos_limit_x_min_, config_pos_limit_y_min_))(
        fm::tMapPoint(config_pos_limit_x_min_, config_pos_limit_y_max_))(
        fm::tMapPoint(config_pos_limit_x_max_, config_pos_limit_y_max_))(
        fm::tMapPoint(config_pos_limit_x_max_, config_pos_limit_y_min_));

    fm::tSptrMapElement el(new fm::MapElement(fm::DEBUG_POLYGON, drivable_box,
                                              _clock->GetStreamTime()));
    el->user_tag_ui_ = "drivable area";
    el->user_color_ui_ = "green";
    map_->AddElement(el, _clock->GetStreamTime());
    el->EnableTimeOfLife(4e6);               // 4 sec life time
    el->SetUpdateWithRepostionJumps(false);  // fixed globale area
}


void DynamicEmergencyBreak::TransmitJuryObstacle(const fm::tSptrMapElement & el_obstacle)
{
    float current_time_sec = _clock->GetStreamTime() / 1e6;
    if (!el_obstacle) return;
    if (last_jury_obstacle_trans_time_ == -1
        || (current_time_sec - last_jury_obstacle_trans_time_) > 10.)
      {
            last_jury_obstacle_trans_time_ = current_time_sec;
            fm::tMapPoint center;

            jury_obstacle_output_pin_.Transmit<float, float>(
            center.get<0>(), center.get<1>(), _clock->GetStreamTime());
            IF_DEBUG_LOG_PRINTF("DynamicEmergencyBreak transmitted obstacle "
                "to jury with x y (%f %f)", center.get<0>(), center.get<1>());

      }
}
