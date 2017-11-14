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

#include "map_filter.h"

using namespace frAIburg::map;

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_MAP_FILTER_NAME, OID_ADTF_MAP_FILTER, MapFilter);
// ____________________________________________________________________________
MapFilter::MapFilter(const tChar* __info) : cFilter(__info) {
    InitMAP();  // map init call before ever other filter
    // set up the buffer in the same order as rx ids as in SetPinIDs
    rx_buff_ = boost::assign::list_of((tVoid*)&current_car_pos_.x)(
        (tVoid*)&current_car_pos_.y)((tVoid*)&current_car_pos_.heading);
    timer_ = NULL;
    property_el_remove_dist_ =
        ADTF_PROPERTY_MAP_ELEMENT_REMOVE_DISTANCE_DEFAULT_METER;
    property_repostion_jump_dist_ =
        ADTF_PROPERTY_MAP_REPOSITION_CAR_POS_DISTANCE_DEFAULT_METER;
    property_repostion_jump_time_to_update_ =
        ADTF_PROPERTY_MAP_REPOSITION_PAST_TIME_TO_UPDATE_DISTANCE_DEFAULT_VAL;

    SetAllProperties();
}

// ____________________________________________________________________________
MapFilter::~MapFilter() {}

// ____________________________________________________________________________
void MapFilter::InitMAP() {
    map_ = frAIburg::map::getInstance();
    map_->Reset();
    // inital car postion at 0 0 0
    map_->Update(MapHelper::GetCarPosLocalIsGlobal());
    map_->SetCarPosKnown(false);
}

// ____________________________________________________________________________
void MapFilter::InitTimer() {
    tUInt32 nFlags = 0;
    // Create our cyclic timer.
    cString strTimerName = OIGetInstanceName();
    strTimerName += ".rttimerMapTupdateTimer";
    tTimeStamp tmPeriod = 1e6 * property_map_car_pos_update_interval_in_s_;
    tTimeStamp tmstartdelay = 1e6 * 0.5;
    timer_ = _kernel->TimerCreate(tmPeriod,
                                  tmstartdelay,  // dely for the first start
                                  static_cast<IRunnable*>(this),  // call run
                                  NULL, NULL, 0, nFlags, strTimerName);
    if (!timer_) {
        LOG_ERROR("map filter unable to create timer");
    }
}

// ____________________________________________________________________________
tResult MapFilter::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

    tResult nResult = ERR_NOERROR;

    if (stage == StageFirst) {
        first_car_pos_received_ = false;
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
    } else if (stage == StageGraphReady) {
        // query your pins about their media types and additional meta data.
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");

        InitTimer();
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapFilter::Shutdown(tInitStage stage, __exception) {
    // Destroy the timer
    if (timer_ && stage == StageGraphReady) {
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
        map_->ClearElements();  // clean the map with all smart ptr
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult MapFilter::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    running_ok_ = true;
    current_car_pos_.update_time_microseconds = 0;
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapFilter::Stop(__exception) {
    running_ok_ = false;

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                              tInt param2, IMediaSample* media_sample) {
    // first check what kind of event it is
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(media_sample);
        // by comparing it to our member pin variable we can find out which
        // pin received the sample
        if (car_pos_input_pin_.isSource(source)) {
            // update current_car_pos_
            tTimeMapStamp timestamp =
                _clock->GetStreamTime();  // todo(use sample t)
            tMapData distance_new_car_pos = 0;
            current_car_pos_.update_time_microseconds = timestamp;
            if (!first_car_pos_received_ || !map_->IsCarPosKnown()) {
                map_->SetCarPosKnown(true);
                first_car_pos_received_ = true;
            }

            // mutex fro timer (Run func)
            __synchronized_obj(update_mutex_);

            car_pos_input_pin_.ReadIDcopy(media_sample, rx_buff_);
            map_->DistanceToCar(current_car_pos_.x, current_car_pos_.y,
                                &distance_new_car_pos);
            if (distance_new_car_pos > property_repostion_jump_dist_) {
                LOG_INFO_PRINTF("MAP reposition jump detected");
                map_->UpdateRepositionCar(
                    current_car_pos_,
                    // past el to update in micro s:
                    property_repostion_jump_time_to_update_ * 1e6, timestamp);
                // GetRepositionCarExcludeTypes());
            } else {
                map_->UpdateCarPos(current_car_pos_, timestamp);
            }
        }
    }
    RETURN_NOERROR;
}
// ____________________________________________________________________________
tResult MapFilter::CreateInputPins(__exception) {
    slim::register_pin_func func = &MapFilter::RegisterPin;
    RETURN_IF_FAILED(car_pos_input_pin_.FirstStageCreate(
        this, func, "car_Postion", "tPosition"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapFilter::CreateOutputPins(__exception) { RETURN_NOERROR; }

// ____________________________________________________________________________
tResult MapFilter::SetPinIDs() {
    vector<string> names_subtypes =
        boost::assign::list_of("f32x")("f32y")("f32heading");
    // not interested in ("Radius")("Speed")
    car_pos_input_pin_.StageGraphReadySetIDOrder(names_subtypes);
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapFilter::Run(tInt nActivationCode, const tVoid* pvUserData,
                       tInt szUserDataSize,
                       ucom::IException** __exception_ptr) {
    if (running_ok_) {
        // LOG_INFO("Timer running");
        // check if smaple time is ok
        tTimeMapStamp current_time_micro_s = _clock->GetStreamTime();
        __synchronized_obj(update_mutex_);
        // set the car pos at the befinning to true

        // TODO CheckForLostCarPos is disabled
        // if (first_car_pos_received_)
        //   CheckForLostCarPos(current_time_micro_s);
        map_->RemoveAllElementsOverLifeTime(current_time_micro_s);

        // TODO disabeld for test event  if (map_->IsCarPosKnown())
        map_->UpdateElements();

        // or remove_if or list
        // elements with creation time zero will not be removed
        map_->RemoveAllElementsUnderProjecteDistanceToCar(
            property_el_remove_dist_,
            NULL);
            //GetExcludeTypesProjecteDistanceRemove());
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void MapFilter::CheckForLostCarPos(tTimeMapStamp current_time_micro_s) {
    // check if timout for pos lost
    tTimeMapStamp time_since_pos_update =
        current_time_micro_s - current_car_pos_.update_time_microseconds;
    if (time_since_pos_update >= property_pos_lost_timeout_time_micro_s) {
        if (map_->IsCarPosKnown()) {
            LOG_WARNING_PRINTF("map car postion lost, last update %f s ago",
                               time_since_pos_update * 1e-6);
            map_->SetCarPosKnown(false);
        }
    } else if (!map_->IsCarPosKnown()) {
        LOG_WARNING_PRINTF("map car postion know again: after %.2f",
                           time_since_pos_update * 1e-6);
        map_->SetCarPosKnown(true);
    }
}

// ____________________________________________________________________________
void MapFilter::GetAllStaticProperties(void) {
    // sample time
    property_map_car_pos_update_interval_in_s_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL_NAME);
    // LOG_DUMP(A_UTILS_NS::cString::Format("map car postion update intervall in
    // s: ",
    //                                      property_map_update_interval_in_s_));
    property_repostion_jump_dist_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_REPOSITION_CAR_POS_DISTANCE_NAME);

    property_repostion_jump_time_to_update_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_REPOSITION_PAST_TIME_TO_UPDATE_NAME);

    property_el_remove_dist_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_ELEMENT_REMOVE_DISTANCE_NAME);

    property_pos_lost_timeout_time_micro_s = static_cast<tTimeMapStamp>(
        GetPropertyFloat(ADTF_PROPERTY_MAP_POS_UNKNOWN_TIMEOUT) * 1e6);
}

// ____________________________________________________________________________
void MapFilter::SetAllProperties(void) {
    // sample time
    SetPropertyFloat(ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL_NAME,
                     ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL_DEFAULT_VAL);

    SetPropertyFloat(
        ADTF_PROPERTY_MAP_REPOSITION_CAR_POS_DISTANCE_NAME,
        ADTF_PROPERTY_MAP_REPOSITION_CAR_POS_DISTANCE_DEFAULT_METER);

    SetPropertyFloat(
        ADTF_PROPERTY_MAP_REPOSITION_PAST_TIME_TO_UPDATE_NAME,
        ADTF_PROPERTY_MAP_REPOSITION_PAST_TIME_TO_UPDATE_DISTANCE_DEFAULT_VAL);

    SetPropertyFloat(ADTF_PROPERTY_MAP_ELEMENT_REMOVE_DISTANCE_NAME,
                     ADTF_PROPERTY_MAP_ELEMENT_REMOVE_DISTANCE_DEFAULT_METER);

    SetPropertyFloat(ADTF_PROPERTY_MAP_POS_UNKNOWN_TIMEOUT,
                     ADTF_PROPERTY_MAP_POS_UNKNOWN_TIMEOUT_DEFAULT_SEC);
    SetPropertyStr(ADTF_PROPERTY_MAP_POS_UNKNOWN_TIMEOUT NSSUBPROP_DESCRIPTION,
                   "timeout for lost postion at pos input pin");
}

// ____________________________________________________________________________
const std::vector<MapElementType>* MapFilter::GetExcludeTypesProjecteDistanceRemove() {
    // exuled all sigin s and parking spaces
    static const std::vector<MapElementType> exclude_types =
        boost::assign::list_of(DEBUG_POINT)(STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT)(
            STREET_TRAFFIC_SIGN_STOP)(STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC)(
            STREET_TRAFFIC_SIGN_JUST_STRAIGHT)(STREET_TRAFFIC_SIGN_GIVE_WAY)(
            STREET_TRAFFIC_SIGN_CROSSWALK)(STREET_TRAFFIC_SIGN_PARKING)(
            STREET_TRAFFIC_SIGN_CIRCLE)(STREET_TRAFFIC_SIGN_NO_TAKE_OVER)(
            STREET_TRAFFIC_SIGN_POSITION_MARKER)(STREET_TRAFFIC_ONE_WAY_STREET)(
            STREET_TRAFFIC_ROAD_WORKS)(STREET_TRAFFIC_SIGN_SPEED_50)(
            STREET_TRAFFIC_SIGN_NO_ENTRY)(STREET_TRAFFIC_SIGN_SPEED_100)(
            STREET_PARKING_SPOT)(LM_STREET_PARKING_SPOT);
    return &exclude_types;
}
