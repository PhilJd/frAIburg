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
#include "roadsign_filter.h"

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (property_debug_enabled_) {   \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_FILTER_ROADSIGN_FILTER_NAME, OID_ADTF_ROADSIGN_FILTER,
                   RoadSignFilter);

using namespace frAIburg::map;

// ____________________________________________________________________________
RoadSignFilter::RoadSignFilter(const tChar* __info)
    : cFilter(__info), MapEventListener() {
    last_update_time_map_sign_s_ = 0;
    config_camera_offse_x_ = 0;
    config_camera_offse_y_ = 0;
    property_debug_enabled_ = ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT;
    first_sensor_sign_after_init_position_jump_ = false;
    SetAllProperties();
}

// ____________________________________________________________________________
RoadSignFilter::~RoadSignFilter() {}

// ____________________________________________________________________________
tResult RoadSignFilter::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))
    tResult nResult = ERR_NOERROR;
    if (stage == StageFirst) {
        GetAllStaticProperties();
        nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");

    } else if (stage == StageNormal) {
        InitMAP();  // after  GetAllStaticProperties
    } else if (stage == StageGraphReady) {
        nResult = SetPinIDs();

        if (GetPropertyBool(ADTF_PROPERTY_XML_LOAD_ENABLED_NAME))
            LoadKnownLandmarkConfiguration();
        else
            LOG_WARNING("loading known landmarks sign form xml disabled");
        InitTimer();
    }
    return nResult;
}

// ____________________________________________________________________________
void RoadSignFilter::InitMAP() {
    map_ = frAIburg::map::getInstance();

    map_->RegisterEventListener(this);
}

// ____________________________________________________________________________
tResult RoadSignFilter::Shutdown(tInitStage stage, __exception) {
    // In each stage clean up everything that you initialized in the
    // corresponding stage during Init. Pins are an exception:
    // - The base class takes care of static pins that are members of this
    //   class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please
    //   see the demo_dynamicpin example for further reference.
    if (stage == StageGraphReady) {
        DeleteTimer();
    } else if (stage == StageNormal) {
        map_->DeregisterEventListener(this);
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult RoadSignFilter::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                   tInt param2, IMediaSample* pMediaSample) {
    // first check what kind of event it is
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(source);
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        if (pin_in_road_sign_ext_.isSource(source)) {
            ProcessRoadSignInputExt(pMediaSample);
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult RoadSignFilter::Start(__exception) {
    running_ok_ = true;
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult RoadSignFilter::Stop(__exception) {
    running_ok_ = false;
    return cFilter::Stop(__exception_ptr);
}
// ____________________________________________________________________________
void RoadSignFilter::GetAllStaticProperties(void) {
    // map   add properties
    property_sign_fuse_dist_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_FUSE_DIST_NAME);
    property_sign_fuse_area_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_FUSE_AREA_NAME);
    property_sign_update_time_s_map_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_SIGIN_ADD_TIME_NAME);
    property_range_scan_for_corresponding_sign_ =
        GetPropertyFloat(ADTF_PROPERTY_SCAN_RANGE_NAME);
    property_projected_dist_inform_jury_missing_sign_ =
        GetPropertyFloat(ADTF_PROPERTY_PROJECTED_DIST_TO_INFORM_JURY_NAME);

    if (property_projected_dist_inform_jury_missing_sign_
        > property_range_scan_for_corresponding_sign_ ){
          LOG_ERROR_PRINTF("sign projected distjury inform must be smaller"
                           " than scan range")
    }
    //  std::string call to ensure c_str exists
    std::string file_name(GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME));
    std::string name_target(
        GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME));
    frAIburg::utils::XMLHelper config;
    if (config.ReadNameValue(file_name.c_str(), "sensor",
                             name_target.c_str())) {
        config_camera_offse_x_ = *(config.GetValue<float>("offset_meter_x"));
        config_camera_offse_y_ = *(config.GetValue<float>("offset_meter_y"));

        IF_DEBUG_LOG_PRINTF("RoadSignFilter car target: %s ",
                            config.car_target_.c_str());
        IF_DEBUG_LOG_PRINTF(
            "RoadSignFilter camera offset "
            "x: %.2f, y: %.2f in meter",
            config_camera_offse_x_, config_camera_offse_y_);
    } else {
        LOG_ERROR_PRINTF(
            "RoadSignFilter frAIburg xml calibration file or"
            " sensor traget wrong property");
    }

    std::string file_dim_name(
        GetPropertyStr(ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME));
    if (config.ReadNameValue(file_dim_name.c_str(), "dimension", "RoadSign")) {
        config_sign_box_size_half_x_ =
            *(config.GetValue<float>("size_half_meter_x"));
        config_sign_box_size_half_y_ =
            *(config.GetValue<float>("size_half_meter_y"));

        IF_DEBUG_LOG_PRINTF(
            "RoadSignFilter sign size half "
            "x: %.2f, y: %.2f in meter",
            config_sign_box_size_half_x_, config_sign_box_size_half_y_);

    } else {
        LOG_ERROR_PRINTF("RoadSignFilter xml dimensions not found");
    }
}

// ____________________________________________________________________________
void RoadSignFilter::SetAllProperties(void) {
    // filter debug mode changeable
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME,
                    ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT);
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat(ADTF_PROPERTY_JURY_CHECK_INTERVAL_NAME,
                     ADTF_PROPERTY_JURY_CHECK_INTERVAL_DEFAULT);
    // map options
    SetPropertyFloat(ADTF_PROPERTY_MAP_FUSE_DIST_NAME,
                     ADTF_PROPERTY_ROADSGIN_FILTER_SIGN_MAX_FUSE_DIST_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_MAP_FUSE_DIST_NAME NSSUBPROP_DESCRIPTION,
                   "set the maximum distance to fuse sigin el wit same type, "
                   "-1 to disable");
    SetPropertyFloat(ADTF_PROPERTY_MAP_FUSE_AREA_NAME,
                     ADTF_PROPERTY_ROADSGIN_FILTER_SIGN_MAX_FUSE_AREA_DEFAULT);
    SetPropertyStr(
        ADTF_PROPERTY_MAP_FUSE_AREA_NAME NSSUBPROP_DESCRIPTION,
        "set the maximum area to fuse sigin el wit same type, -1 to disable");

    SetPropertyFloat(ADTF_PROPERTY_MAP_SIGIN_ADD_TIME_NAME,
                     ADTF_PROPERTY_MAP_SING_UPDATE_TIME_S_DEFAULT);
    // roadsign xml
    SetPropertyStr(ADTF_PROPERTY_XML_ROAD_SIGN_FILE_NAME,
                   ADTF_PROPERTY_XML_ROAD_SIGN_FILE_DEFAULT);
    SetPropertyBool(ADTF_PROPERTY_XML_ROAD_SIGN_FILE_NAME NSSUBPROP_FILENAME,
                    tTrue);
    SetPropertyStr(ADTF_PROPERTY_XML_ROAD_SIGN_FILE_NAME NSSUBPROP_FILENAME
                       NSSUBSUBPROP_EXTENSIONFILTER,
                   "XML Files (*.xml)");
    SetPropertyStr(ADTF_PROPERTY_XML_ROAD_SIGN_FILE_NAME NSSUBPROP_DESCRIPTION,
                   "Configuration file for the roadsign coordinates");

    // disablel load known roadSigns
    SetPropertyBool(ADTF_PROPERTY_XML_LOAD_ENABLED_NAME,
                    ADTF_PROPERTY_XML_LOAD_ENABLED_DEFAULT);

    // jury inform range
    SetPropertyFloat(ADTF_PROPERTY_SCAN_RANGE_NAME,
                     ADTF_PROPERTY_SCAN_RANGE_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_SCAN_RANGE_NAME NSSUBPROP_DESCRIPTION,
                   "In range threadhold to to scan for corresponding signs "
                   "in the debug mode corresponding signs are moar[m]");
    SetPropertyFloat(ADTF_PROPERTY_PROJECTED_DIST_TO_INFORM_JURY_NAME,
                    ADTF_PROPERTY_PROJECTED_DIST_TO_INFORM_JURY_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_PROJECTED_DIST_TO_INFORM_JURY_NAME
                    NSSUBPROP_DESCRIPTION,
                  "Projected distance of a sign to inform the jury"
                  ", of wrong sigin must be small than scan range[m]");

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

    // xml configuration dimension
    SetPropertyStr(ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME,
                   ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_DEFAULT);
    SetPropertyBool(
        ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME NSSUBPROP_FILENAME,
        tTrue);
    SetPropertyStr(
        ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME NSSUBPROP_FILENAME
            NSSUBSUBPROP_EXTENSIONFILTER,
        "XML Files (*.xml)");
    SetPropertyStr(
        ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME NSSUBPROP_DESCRIPTION,
        "Configuration file for camera postion offset");

    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME,
                   ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_XML_ROAD_SIGN_FILE_NAME NSSUBPROP_DESCRIPTION,
                   "set the xml sensor target in <sensor target=select");
}

// ____________________________________________________________________________
tResult RoadSignFilter::PropertyChanged(const tChar* str_name) {
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
tResult RoadSignFilter::CreateInputPins(__exception) {
    slim::register_pin_func func = &RoadSignFilter::RegisterPin;
    RETURN_IF_FAILED(pin_in_road_sign_ext_.FirstStageCreate(
        this, func, "RoadSignExt", "tRoadSignExt"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult RoadSignFilter::CreateOutputPins(__exception) {
    slim::register_pin_func func = &RoadSignFilter::RegisterPin;
    RETURN_IF_FAILED(pin_out_jury_sign_.FirstStageCreate(
        this, func, "JuryTrafficSign", "tTrafficSign"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult RoadSignFilter::SetPinIDs() {
    vector<string> id_jury =
        boost::assign::list_of("i16Identifier")("f32x")("f32y")("f32angle");
    RETURN_IF_FAILED(pin_out_jury_sign_.StageGraphReadySetIDOrder(id_jury));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
bool RoadSignFilter::TransmitJurySign(tInt16 jury_id, tFloat32 x, tFloat32 y,
                                      tFloat32 angle_) {
    IF_DEBUG_LOG_PRINTF(
        "Transmit parking to jury id %d, x %.2f, y %.2f, "
        "agnle jury degree %f",
        jury_id, x, y, angle_);
    const vector<const tVoid*> vals =
        boost::assign::list_of((const tVoid*)&jury_id)((const tVoid*)&x)(
            (const tVoid*)&y)((const tVoid*)&angle_);
    return IS_OK(pin_out_jury_sign_.Transmit(vals, _clock->GetStreamTime()));
}

// ____________________________________________________________________________
tResult RoadSignFilter::ProcessRoadSignInputExt(IMediaSample* pMediaSample) {
    tRoadSignExt* pSampleData = NULL;
    if (IS_OK(pin_in_road_sign_ext_.ReadNoID_start(
            pMediaSample, (const tVoid**)&pSampleData, sizeof(tRoadSignExt)))) {
        tFloat32 rvecFl32array[3];
        tFloat32 tvecFl32array[3];

        tInt16 i16ID = pSampleData->i16Identifier;
        //        tFloat32 f32Area = pSampleData->f32Imagesize;
        rvecFl32array[0] = pSampleData->af32RVec[0];
        rvecFl32array[1] = pSampleData->af32RVec[1];
        rvecFl32array[2] = pSampleData->af32RVec[2];

        tvecFl32array[0] = pSampleData->af32TVec[0];
        tvecFl32array[1] = pSampleData->af32TVec[1];
        tvecFl32array[2] = pSampleData->af32TVec[2];
        // float abs_sign_angle = 180 -
        // abs(AxisAnglesToYawAngle(rvecFl32array));

        pin_in_road_sign_ext_.ReadNoID_end(pMediaSample,
                                           (const tVoid**)pSampleData);

        const tMapData local_sign_center_x =
            tvecFl32array[2] + config_camera_offse_x_;
        const tMapData local_sign_center_y =
            -tvecFl32array[0] + config_camera_offse_y_;
        const tMapData local_sign_angle_rad =
            AxisAnglesToYawAngle(rvecFl32array) * (M_PI / 180.);
        // IF_DEBUG_LOG_PRINTF("dt last sign %.3f ",dt_last_update);
        const tMapData distance_to_car_front_pos =
            sqrt(pow(local_sign_center_x, 2) + pow(local_sign_center_y, 2));
        // IF_DEBUG_LOG_PRINTF("sign distance %.3f ",distance_to_car_front_pos);

        if (property_debug_enabled_) {
            AddDebugPointToMap(local_sign_center_x, local_sign_center_y,
                               local_sign_angle_rad);
        }

        // min distance to add the sign needed because the car postioning system
        if (map_->IsCarPosKnown() &&
            distance_to_car_front_pos >
                0.2)  // TODO markus change to lower value in test event
        {
            if (IsUpdateTimeOkToAddNewSignToMap()) {
                AddSignToMap(
                    i16ID,
                    local_sign_center_x,  // sign center x in local frame
                    local_sign_center_y,  // sign center y
                    local_sign_angle_rad, _clock->GetStreamTime(), SIGN_SENSOR);
            } else {
                IF_DEBUG_LOG_PRINTF(
                    "sign detected but not added to map"
                    "because user min time to add signs");
            }
        }
    } else
        LOG_ERROR_PRINTF("error read road sign failed!");

    RETURN_NOERROR;
}

// ____________________________________________________________________________
bool RoadSignFilter::IsUpdateTimeOkToAddNewSignToMap() {
    const float currenttime_s = _clock->GetStreamTime() * 1e-6;
    const float dt_last_update = currenttime_s - last_update_time_map_sign_s_;

    bool ret = ((dt_last_update >= property_sign_update_time_s_map_) ||
                dt_last_update <= 0.01);  // multiple singn detected
    last_update_time_map_sign_s_ = currenttime_s;
    return ret;
}

// ____________________________________________________________________________
void RoadSignFilter::AddDebugPointToMap(tMapData local_sign_center_x,
                                        tMapData local_sign_center_y,
                                        tMapData local_sign_angle_rad) {
    static unsigned int color_index = 0;
    MapHelper::AddFuseDebugPointToMap(
        map_, local_sign_center_x, local_sign_center_y,
        (tTimeMapStamp)_clock->GetStreamTime(),  // creation time
        0.3,                                     // fuse distance
        local_sign_angle_rad,
        1e6,  // life_time_microseconds
        MapHelper::GetChangingDebugColor(&color_index));
}

// ____________________________________________________________________________
void RoadSignFilter::AddSignToMap(tInt16 id_aadc, tMapData x, tMapData y,
                                  tMapData angle,  // rad
                                  tTimeMapStamp timestamp,
                                  MapRoadSignSubType type) {
    tMapData local_box_angle = -angle;

    // add sign with fuse mean to map
    // get a box with sign orientation
    std::vector<tMapPoint> points;
    MapHelper::CreatBoxPoints(x, y,  // center pos of the sign
                              config_sign_box_size_half_x_,
                              config_sign_box_size_half_y_, points,
                              &local_box_angle);

    tSptrMapElement el(
        new MapElementRoadSign(id_aadc, type, points, timestamp, angle));

    if (type == SIGN_LANDMARK) {
        // do no trans if in global frame
        tMapCarPosition zero_pos = MapHelper::GetCarPosLocalIsGlobal();
        el->LocalToGlobal(zero_pos);
        el->user_color_ui_ = MAP_ELEMENT_COLOR_LANDMARK;
        // fixed globale pos, disable postion jumps
        el->SetUpdateWithRepostionJumps(false);
        map_->AddElement(el, timestamp);
    }else{
        map_->AddFuseElement(el,
                             property_sign_fuse_dist_,
                             property_sign_fuse_area_,
                             timestamp,
                             MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_007);
    }


    // live time at the beginning for
    // sign sensor data sensor if low fuse count
    if (type == SIGN_SENSOR) {
        if (el->GetFuseCount() < 2) {
            // enableld life time form the creation time when the element was
            // first
            // created
            el->EnableTimeOfLife(1e6);  // 1s to get 5 samples
        } else if (el->GetFuseCount() == 2) {
            // disable jump after repostioning
            IF_DEBUG_LOG_PRINTF("sign disabled jump: %s ",
                                el->ToString().c_str());
            //el->SetUpdateWithRepostionJumps(false);
        }
    }

    if (property_debug_enabled_ && type == SIGN_LANDMARK)
        IF_DEBUG_LOG_PRINTF("added xml sign: %s ", el->ToString().c_str());
}

// ____________________________________________________________________________
float RoadSignFilter::AxisAnglesToYawAngle(
    tFloat32 rvecFl32array[3]) {  // this function trnslate the rodrigues vector
                                  // to yaw angle in deg.
    cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
    cv::Mat rvecR(3, 1, cv::DataType<double>::type);
    rvecR.at<double>(0) = rvecFl32array[2];
    rvecR.at<double>(1) = -rvecFl32array[0];
    rvecR.at<double>(2) = -rvecFl32array[1];
    cv::Rodrigues(rvecR, rotationMatrix);

    float gamma =
        atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));

    return (gamma * 180 / M_PI);
}

/*!
 * loads road-sign configuration from a file, and stores into memory
 * */
// ____________________________________________________________________________
void RoadSignFilter::LoadKnownLandmarkConfiguration() {  // func from aadc demo
                                                         // proj

    cFilename fileConfig =
        GetPropertyStr(ADTF_PROPERTY_XML_ROAD_SIGN_FILE_NAME);

    // create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");

    if (fileConfig.IsEmpty()) {
        LOG_ERROR_PRINTF("roadSign xml file INVALID");
    }

    tInt i = 0;
    if (cFileSystem::Exists(fileConfig)) {
        cDOM oDOM;
        oDOM.Load(fileConfig);
        cDOMElementRefList oElems;

        if (IS_OK(oDOM.FindNodes("configuration/roadSign", oElems))) {
            for (cDOMElementRefList::iterator itElem = oElems.begin();
                 itElem != oElems.end(); ++itElem) {
                tInt16 id =
                    tUInt16((*itElem)->GetAttribute("id", "0").AsInt32());
                tFloat32 global_x =
                    tFloat32((*itElem)->GetAttribute("x", "0").AsFloat64());
                tFloat32 global_y =
                    tFloat32((*itElem)->GetAttribute("y", "0").AsFloat64());

                tFloat32 global_dir = tFloat32(
                    (*itElem)->GetAttribute("direction", "0").AsFloat64());
                global_dir += 180;
                global_dir *= (M_PI / 180.0);  // convert to radians
                // add sign to map
                i++;

                AddSignToMap(
                    id, global_x, global_y,  // sign center y
                    global_dir,
                    0,  // time zero so not deleted in map remove distance el
                    SIGN_LANDMARK);  // set form global frame
            }

            IF_DEBUG_LOG_PRINTF("RoadSignFilter xml loaded %d signs ", i);

        } else {
            LOG_ERROR_PRINTF("roadSign xml onfiguration/roadSign not found");
        }
    } else {
        LOG_ERROR_PRINTF("roadSign xml file does not exist");
    }
}

// ____________________________________________________________________________
void RoadSignFilter::MapOnEventDistanceReached(tSptrMapElement el,
                                               const tMapData threshold,
                                               bool distance_lower_threshold) {}

// ____________________________________________________________________________
void RoadSignFilter::MapOnEventAddedNew(tSptrMapElement el) {
    __synchronized_obj(update_mutex_);
    ProcessMapEvent(el);
}

// ____________________________________________________________________________
void RoadSignFilter::MapOnEventRemoved(frAIburg::map::tSptrMapElement el) {
    __synchronized_obj(update_mutex_);
    if (RemoveRelevantElement(el->GetID())) {
        LOG_ERROR_PRINTF(
            "RoadSignFilter::MapOnEventRemoved"
            " unchecked sign removed");
    }
}

// ____________________________________________________________________________
bool RoadSignFilter::RemoveRelevantElement(tMapID id) {
    std::vector<tSptrMapElement>::iterator it = relevant_signs_.begin();
    for (; it != relevant_signs_.end(); ++it) {
        if ((*it)->GetID() == id) {
            relevant_signs_.erase(it);
            return true;
        }
    }
    return false;
}

// ____________________________________________________________________________
void RoadSignFilter::ProcessMapEvent(tSptrMapElement& el) {
    __synchronized_obj(update_mutex_);
    if (MapElementRoadSign::IsRoadSign(el->GetType())  // is sign
        && el->GetType() != STREET_TRAFFIC_SIGN_POSITION_MARKER) {
        relevant_signs_.push_back(el);
    }
}

// ____________________________________________________________________________
void RoadSignFilter::UpdateSignWithCorrespondingSign(MapElementRoadSign* el_r,
  MapElementRoadSign* el_corresponding)
{
    IF_DEBUG_LOG_PRINTF("FindCorrespondingSign : %s \n to %s",
                        el_corresponding->ToString().c_str(),
                        el_r->ToString().c_str());

    if (property_debug_enabled_){
      el_r->user_color_ui_ = "green";
      el_corresponding->user_color_ui_ = "darkgreen";
    }

    if (el_corresponding->GetCorrespondingSign()){
      LOG_ERROR_PRINTF("CorrespondingSign : %s \n to %s "
                       "already has a corresponding sign",
                       el_corresponding->ToString().c_str(),
                       el_r->ToString().c_str());
    }
    //set corresponding sigins
    el_r->SetCorrespondingSign(map_->GetElement(el_corresponding->GetID()));
    el_corresponding->SetCorrespondingSign(map_->GetElement(el_r->GetID()));
}

// ____________________________________________________________________________
void RoadSignFilter::UpdateJurySign(MapElementRoadSign* el_r)
{
    // car is behind the sign
    // if no corresponding sign before
    // the sign is missing or a new sign was added
    // inform the jruy if there is a missing or unexpected sign
    const tMapPoint sign_center = el_r->GetGlobalPolyCenter();
    tInt16 jury_id;

    el_r->JuryInformed(_clock->GetStreamTime());

    if (property_debug_enabled_) el_r->user_color_ui_ = "red";

    if (el_r->GetSignSubType() == SIGN_LANDMARK) {
        // missing sign, no Detection
        IF_DEBUG_LOG_PRINTF("missing sign detected: %s",
                            el_r->ToString().c_str());
        // the jury expets id -1 for a missing sign
        jury_id = -1;
    } else {
        // unexpected sigin
        IF_DEBUG_LOG_PRINTF("missing sign detected: %s",
                            el_r->ToString().c_str());
        jury_id = el_r->GetJuryID();
    }

    TransmitJurySign(jury_id,
        sign_center.get<0>(), sign_center.get<1>(), //center
        el_r->GetGlobalOrientation() * (180 / M_PI)); // TODO angle wrong ?
}

// ____________________________________________________________________________
void RoadSignFilter::CheckSign(tSptrMapElement& el) {
    // elements in range and parking tpye
    // also check again also if out of range: IsJuryInformed should be set
    // if no error
    // note: at the car starte there is no postion signal, so the car pos
    //      is not updated in the map filter ->no distance reached called
    // the first sign in used for repositioning the car and a repositioning
    // jump is detected, a flag is set to avoid the missing sensor informmation

    if (el->GetDistanceToCar() <= property_range_scan_for_corresponding_sign_
        && MapHelper::IsElementOrientatedToCar(map_, *el)
        && el->GetFuseCount())
    {
        // check signs in range and fancing the car
        // TODO check fuse count
        MapElementRoadSign* el_r = dynamic_cast<MapElementRoadSign*>(el.get());
        if (el_r) {
            if (!first_sensor_sign_after_init_position_jump_ &&
                el_r->GetSignSubType() == SIGN_SENSOR) {
                first_sensor_sign_after_init_position_jump_ = true;

            } else if (first_sensor_sign_after_init_position_jump_
                && !el_r->IsJuryInformed()
                && !el_r->GetCorrespondingSign())
            {
                // check sign if in scan range,
                // not checked before:
                // - no corresponding sign was added
                // - jury war not informed before
                MapElementRoadSign* el_corresponding =
                    FindCorrespondingSign(el_r);

                //  debug mode: orange to signal waiting for data
                if (property_debug_enabled_) el_r->user_color_ui_ = "orange";

                if (el_corresponding) {
                  UpdateSignWithCorrespondingSign(el_r, el_corresponding);
                  // remove form checklist if corresponding sign found
                  RemoveRelevantElement(el_corresponding->GetID());
                  RemoveRelevantElement(el_r->GetID());

                } else if (el->ProjectedDistanceToCar()
                  <= property_projected_dist_inform_jury_missing_sign_)
                {
                  // car is behind the sign
                  // if no corresponding sign before
                  // the sign is missing or a new sign was added
                  UpdateJurySign(el_r);
                  RemoveRelevantElement(el_r->GetID());
                }
            } else {
                LOG_WARNING_PRINTF(
                    "RoadSignFilter jury informed sign in checklist");
            }
        } else {
            LOG_ERROR_PRINTF(
                "RoadSignFilter distance event cast to sign failed ");
        }
    }
}

// ____________________________________________________________________________
MapElementRoadSign* RoadSignFilter::FindCorrespondingSign(
    MapElementRoadSign* el_r) {
    MapElementRoadSign* ret_el = NULL;
    if (el_r) {
        // find similar elements in the map
        std::vector<tSptrMapElement> similar_elements;
        map_->GetSimilarElements(
            el_r,
            1,  // range aroud el to find check for sign
            -1, similar_elements,
            false,  // ignore types -> sign subtypes can be diffrent
            NULL,   // TODO exclude_types
            -1);
        // check all similar elements for SIG_NSENSOR or SIGN_LANDMARK subtypes
        MapElementRoadSign* similar_el_r = NULL;
        IF_DEBUG_LOG_PRINTF("RoadSignFilter %d similar_elements",
                            similar_elements.size())
        BOOST_FOREACH (tSptrMapElement& el, similar_elements) {
            // check if the same street sign is found
            if (el_r->GetType() == el->GetType()) {
                similar_el_r = dynamic_cast<MapElementRoadSign*>(el.get());
                if (similar_el_r) {
                    // corresponding sign is found if subtypes
                    // SIGN_SENSOR and SIGN_LANDMARK are found
                    if (el_r->GetSignSubType() !=
                        similar_el_r->GetSignSubType()) {
                        if (similar_el_r->IsJuryInformed()) {
                            // error but continue
                            LOG_ERROR_PRINTF(
                                "RoadSignFilter found similar sign "
                                " but jury infomed");
                        }
                        ret_el = similar_el_r;
                    } else {
                        LOG_ERROR_PRINTF(
                            "RoadSignFilter similar sign with "
                            "same subtypes found");
                    }

                    return ret_el;
                }
            }
        }
    }
    return ret_el;
}

// ____________________________________________________________________________
void RoadSignFilter::InitTimer() {
    float check_time_intervall_second =
        GetPropertyFloat(ADTF_PROPERTY_JURY_CHECK_INTERVAL_NAME);
    tUInt32 nFlags = 0;
    // Create our cyclic timer.
    cString strTimerName = OIGetInstanceName();
    strTimerName += ".rttimerSignTimer";
    tTimeStamp tmPeriod = 1e6 * check_time_intervall_second;
    tTimeStamp tmstartdelay = 5e6;
    timer_ = _kernel->TimerCreate(tmPeriod,
                                  tmstartdelay,  // dely for the first start
                                  static_cast<IRunnable*>(this),  // call run
                                  NULL, NULL, 0, nFlags, strTimerName);
    if (!timer_) {
        LOG_ERROR_PRINTF("RoadSignFilter filter unable to create timer");
    }
}

// ____________________________________________________________________________
void RoadSignFilter::DeleteTimer() {
    if (timer_) {
        running_ok_ = false;
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
    }
}

// ____________________________________________________________________________
tResult RoadSignFilter::Run(tInt nActivationCode, const tVoid* pvUserData,
                            tInt szUserDataSize, __exception) {
    // timer check all signs and langmarks relevant elements in range
    if (running_ok_) {

        __synchronized_obj(update_mutex_);
        // copy because elments are removved in CheckSign
        std::vector<frAIburg::map::tSptrMapElement> relevant_signs
          = relevant_signs_;

        BOOST_FOREACH (tSptrMapElement& el, relevant_signs) {
          CheckSign(el);
        }
    }
    RETURN_NOERROR;
}
