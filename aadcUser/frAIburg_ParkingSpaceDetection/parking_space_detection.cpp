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

#include "parking_space_detection.h"

using namespace frAIburg::map;

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (property_debug_enabled_) {   \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_MAP_FILTER_NAME, OID_ADTF_MAP_FILTER,
                   ParkingSpaceDetection);
// ____________________________________________________________________________
ParkingSpaceDetection::ParkingSpaceDetection(const tChar* __info)
    : cFilter(__info),
      frAIburg::map::MapEventListener(),
      map_(NULL),
      timer_(NULL) {
    SetAllProperties();
    property_map_parking_space_size_half_x_ = 0;
    property_map_parking_space_size_half_y_ = 0;
}

// ____________________________________________________________________________
ParkingSpaceDetection::~ParkingSpaceDetection() {}

// ____________________________________________________________________________
void ParkingSpaceDetection::InitMAP() {
    map_ = frAIburg::map::getInstance();

    map_->RegisterEventListener(this);
}

// ____________________________________________________________________________
void ParkingSpaceDetection::InitTimer() {
    float check_free_time_intervall_second =
        GetPropertyFloat(ADTF_PROPERTY_TIME_CHECK_INTERVAL_NAME);
    tUInt32 nFlags = 0;
    // Create our cyclic timer.
    cString strTimerName = OIGetInstanceName();
    strTimerName += ".rttimerParkingSpaceDetectionTimer";
    tTimeStamp tmPeriod = 1e6 * check_free_time_intervall_second;
    tTimeStamp tmstartdelay = 0;
    timer_ = _kernel->TimerCreate(tmPeriod,
                                  tmstartdelay,  // dely for the first start
                                  static_cast<IRunnable*>(this),  // call run
                                  NULL, NULL, 0, nFlags, strTimerName);
    if (!timer_) {
        LOG_ERROR_PRINTF("map filter unable to create timer");
    }
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

    tResult nResult = ERR_NOERROR;

    if (stage == StageFirst) {
        GetAllStaticProperties();

        // in StageFirst you can create and register your static pins.
        nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

    } else if (stage == StageNormal) {
        // In this stage you would do further initialisation and/or create your
        // dynamic pins.

        InitMAP();  // after  GetAllStaticProperties
    } else if (stage == StageGraphReady) {
        // query your pins about their media types and additional meta data.
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");

        InitTimer();  // after  GetAllStaticProperties
        LoadKnownLandmarkConfiguration();
        InitUltrasonic();
    }
    return nResult;
}

//
void ParkingSpaceDetection::InitUltrasonic() {
    us_enabled_ = false;
    pin_out_us_enable_.Transmit<tBool>(tFalse, _clock->GetStreamTime());
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::Shutdown(tInitStage stage, __exception) {
    // Destroy the timer
    if (stage == StageGraphReady) {
        DeleteTimer();
        map_->DeregisterEventListener(this);
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    running_ok_ = true;

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::Stop(__exception) {
    running_ok_ = false;

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::CreateOutputPins(__exception) {
    slim::register_pin_func func = &ParkingSpaceDetection::RegisterPin;
    pin_out_jury_parking_.FirstStageCreate(this, func, "JuryParking",
                                           "tParkingSpace");

    pin_out_us_enable_.FirstStageCreate(this, func, "Ultrasonic_enable",
                                        "tBoolSignalValue");
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::SetPinIDs() {
    vector<string> id_jury =
        boost::assign::list_of("i16Identifier")("f32x")("f32y")("ui16Status");
    RETURN_IF_FAILED(pin_out_jury_parking_.StageGraphReadySetIDOrder(id_jury));

    vector<string> id_us_en = boost::assign::list_of("bValue");
    RETURN_IF_FAILED(pin_out_us_enable_.StageGraphReadySetIDOrder(id_us_en));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
bool ParkingSpaceDetection::TransmitJuryParking(tInt16 jury_id, tFloat32 x,
                                                    tFloat32 y,
                                                    tUInt16 is_occupied) {
    IF_DEBUG_LOG_PRINTF(
        "Transmit parking to jury id %d, x %.2f, y %.2f, "
        "occupied %d",
        jury_id, x, y, is_occupied);
    vector<const tVoid*> vals = boost::assign::list_of((const tVoid*)&jury_id)(
        (const tVoid*)&x)((const tVoid*)&y)((const tVoid*)&is_occupied);
    return IS_OK(pin_out_jury_parking_.Transmit(vals, _clock->GetStreamTime()));
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::Run(
    tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize,
    __exception) {  // timer check all parking spaces in range

    if (running_ok_ && !parking_el_to_check_.empty()) {
        SetUSPin();  // enable or disable us
        CheckParkingSpacesInRange();
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ParkingSpaceDetection::GetAllStaticProperties(void) {
    float margin_x =
        GetPropertyFloat(ADTF_PROPERTY_PARKING_SPACE_MARGIN_X_NAME);
    float margin_y =
        GetPropertyFloat(ADTF_PROPERTY_PARKING_SPACE_MARGIN_Y_NAME);

    property_range_check_parking_ =
        GetPropertyFloat(ADTF_PROPERTY_PARKING_SPACE_TO_CHECK_RANGE_NAME);
    property_range_send_jury_ =
        GetPropertyFloat(ADTF_PROPERTY_PARKING_JURY_RANGE_NAME);

    property_parking_space_us_hits_occupied_ =
        GetPropertyInt(ADTF_PROPERTY_PARKING_SPACE_US_HTIS_OCCUPIED_NAME);
    property_parking_space_hepth_hits_occupied_ =
        GetPropertyInt(ADTF_PROPERTY_PARKING_SPACE_DEPTH_HTIS_OCCUPIED_NAME);

    // get size of the parking space form a .xml
    std::string config_file(
        GetPropertyStr(ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME));

    frAIburg::utils::XMLHelper config;
    if (config.ReadNameValue(config_file.c_str(), "dimension",
                             "ParallelParkingSpace")) {
        if (config.GetValue<tMapData>("size_half_meter_x")) {
            property_map_parking_space_size_half_x_ =
                *(config.GetValue<tMapData>("size_half_meter_x"));
            property_map_parking_space_size_half_y_ =
                *(config.GetValue<tMapData>("size_half_meter_y"));
            // set margin for collision check
            property_map_parking_space_size_half_x_ += margin_x;
            property_map_parking_space_size_half_y_ += margin_y;

            IF_DEBUG_LOG_PRINTF(
                "ParkingSpace: with margin size half_x: %f,"
                " size half_y: %f",
                property_map_parking_space_size_half_x_,
                property_map_parking_space_size_half_y_);
        } else {
            LOG_ERROR_PRINTF(
                "ParkingSpaceDetection faild GetValue from config");
        }
    } else {
        LOG_ERROR_PRINTF("ParkingSpaceDetection faild reading xml file");
    }
}

// ____________________________________________________________________________
void ParkingSpaceDetection::SetAllProperties(void) {
    // filter debug mode changeable
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME,
                    ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT);
    SetPropertyBool(ADTF_PROPERTY_DEBUG_ENABLED_NAME NSSUBPROP_ISCHANGEABLE,
                    tTrue);
    // check sample time
    SetPropertyFloat(ADTF_PROPERTY_TIME_CHECK_INTERVAL_NAME,
                     ADTF_PROPERTY_TIME_CHECK_INTERVAL_NAME_DEFAULT);
    // range to check
    SetPropertyFloat(ADTF_PROPERTY_PARKING_SPACE_TO_CHECK_RANGE_NAME,
                     ADTF_PROPERTY_PARKING_SPACE_TO_CHECK_RANGE_DEFAULT);

    // range to send info to jury if left
    SetPropertyFloat(ADTF_PROPERTY_PARKING_JURY_RANGE_NAME,
                     ADTF_PROPERTY_PARKING_JURY_RANGE_DEFAULT);
    // collision margin x
    SetPropertyFloat(ADTF_PROPERTY_PARKING_SPACE_MARGIN_X_NAME,
                     ADTF_PROPERTY_PARKING_SPACE_MARGIN_X_DEFAULT);
    // colltion margin y
    SetPropertyFloat(ADTF_PROPERTY_PARKING_SPACE_MARGIN_Y_NAME,
                     ADTF_PROPERTY_PARKING_SPACE_MARGIN_Y_DEFAULT);
    //hits to be occupied
    SetPropertyInt(ADTF_PROPERTY_PARKING_SPACE_US_HTIS_OCCUPIED_NAME,
                   ADTF_PROPERTY_PARKING_SPACE_US_HTIS_OCCUPIED_DEFAULT);

    SetPropertyInt(ADTF_PROPERTY_PARKING_SPACE_DEPTH_HTIS_OCCUPIED_NAME,
                   ADTF_PROPERTY_PARKING_SPACE_DEPTH_HTIS_OCCUPIED_DEFAULT);

    // dimension filter xml configuration
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
        "Configuration file wiht parking space sizes");

    // parking space xml configuration
    SetPropertyStr(ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_NAME,
                   ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_DEFAULT);
    SetPropertyBool(
        ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_NAME NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(
        ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_NAME NSSUBPROP_FILENAME
            NSSUBSUBPROP_EXTENSIONFILTER,
        "XML Files (*.xml)");
    SetPropertyStr(
        ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_NAME NSSUBPROP_DESCRIPTION,
        "Configuration file wiht parking space sizes");
}

// ____________________________________________________________________________
tResult ParkingSpaceDetection::PropertyChanged(const tChar* str_name) {
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
void ParkingSpaceDetection::DeleteTimer() {
    if (timer_) {
        running_ok_ = false;
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
    }
}

// ____________________________________________________________________________
void ParkingSpaceDetection::AddParkingSpaceToMap(
    tMapData global_fornt_x, tMapData global_fornt_y,
    tMapData angle,  // rad
    bool is_not_free, tInt16 jury_id, tTimeMapStamp timestamp,
    bool global_frame /*=ture*/) {
    MapParkingStatus status = is_not_free ? PARKING_OCCUPIED : PARKING_FREE;
    // get to the parking center point based on the orientation
    tMapData box_angle = -angle - M_PI_2;
    tMapData d_x = -(property_map_parking_space_size_half_y_)*sin(-box_angle);
    tMapData d_y = (property_map_parking_space_size_half_y_)*cos(-box_angle);

    std::vector<tMapPoint> points;
    MapHelper::CreatBoxPoints(global_fornt_x + d_x, global_fornt_y + d_y,
                              property_map_parking_space_size_half_x_,
                              property_map_parking_space_size_half_y_, points,
                              &box_angle);

    tSptrMapElement el(
        new MapElementParking(jury_id, status, points, timestamp, angle));

    if (global_frame) {
        // do no trans if in global frame
        tMapCarPosition zero_pos;
        zero_pos.x = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_X;
        zero_pos.y = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_Y;
        zero_pos.heading = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_HEADING;
        el->LocalToGlobal(zero_pos);
        el->user_color_ui_ = MAP_ELEMENT_COLOR_LANDMARK;
    }
    map_->AddElement(el, timestamp);
    el->SetUpdateWithRepostionJumps(false);

    // add xml point in  debug mode
    if (property_debug_enabled_){

      el->user_color_ui_ = is_not_free ? "red" : "green";

      std::vector<tMapPoint> v1 =
          boost::assign::list_of(tMapPoint(global_fornt_x, global_fornt_y));
      tSptrMapElement el_xml_point_debug(
          new MapElement(DEBUG_POINT, v1, timestamp, angle));

      map_->AddElement(el_xml_point_debug, timestamp);
      el_xml_point_debug->SetUpdateWithRepostionJumps(false);
      el_xml_point_debug->user_color_ui_ = "red";
      el_xml_point_debug->user_tag_ui_ = "xml";

      IF_DEBUG_LOG_PRINTF("added xml parking Space: %s ",
                           el->ToString().c_str());
    }
}

/*!
 * loads road-sign configuration from a file, and stores into memory
 * */
// ____________________________________________________________________________
void ParkingSpaceDetection::LoadKnownLandmarkConfiguration() {
   // func from aadc demo proj

    cFilename fileConfig =
        GetPropertyStr(ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_NAME);

    // create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");

    if (fileConfig.IsEmpty()) {
        LOG_ERROR_PRINTF("parkingSpace xml file INVALID");
    }

    tInt i = 0;
    if (cFileSystem::Exists(fileConfig)) {
        cDOM oDOM;
        oDOM.Load(fileConfig);
        cDOMElementRefList oElems;

        if (IS_OK(oDOM.FindNodes("configuration/parkingSpace", oElems))) {
            for (cDOMElementRefList::iterator itElem = oElems.begin();
                 itElem != oElems.end(); ++itElem) {
                tInt16 jury_id =
                    tUInt16((*itElem)->GetAttribute("id", "0").AsInt32());
                tFloat32 global_x_front =
                    tFloat32((*itElem)->GetAttribute("x", "0").AsFloat64());
                tFloat32 global_y_front =
                    tFloat32((*itElem)->GetAttribute("y", "0").AsFloat64());
                bool is_not_free =
                    bool((*itElem)->GetAttribute("status", "0").AsFloat64());

                tFloat32 global_dir_rad = tFloat32(
                    (*itElem)->GetAttribute("direction", "0").AsFloat64());
                // global_dir_rad += 180;
                global_dir_rad *= (M_PI / 180.0);
                // add sign to map
                i++;
                AddParkingSpaceToMap(
                    global_x_front, global_y_front,  // font center to road
                    global_dir_rad, is_not_free, jury_id,
                    0,  // time zero so not deleted in map remove distance el
                    true);  // set form global frame
            }

            IF_DEBUG_LOG_PRINTF(
                "parkingSpace xml loaded %d spaces from %s", i,
                GetPropertyStr(ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_NAME));
        }
    } else {
        LOG_ERROR_PRINTF("parkingSpace xml file does not exist");
    }
}

bool ParkingSpaceDetection::RemoveElementCheck(tMapID id) {
    std::vector<tSptrMapElement>::iterator it = parking_el_to_check_.begin();
    for (; it != parking_el_to_check_.end(); ++it) {
        if ((*it)->GetID() == id) {
            parking_el_to_check_.erase(it);
            return true;
        }
    }
    return false;
}

// ____________________________________________________________________________
void ParkingSpaceDetection::MapOnEventAddedNew(tSptrMapElement el) {
    __synchronized_obj(update_mutex_);

    AddParkingSpaceToCheckList(el);
}

// ____________________________________________________________________________
void ParkingSpaceDetection::MapOnEventRemoved(tSptrMapElement el) {
    __synchronized_obj(update_mutex_);
    if (RemoveElementCheck(el->GetID())) {
        LOG_ERROR_PRINTF("Parking space removed form map");
        SetUSPin();
    }
}

// ____________________________________________________________________________
void ParkingSpaceDetection::AddParkingSpaceToCheckList(
    tSptrMapElement& el) {
    // elements in range and parking tpye
    if (MapElementParking::IsParkingType(el->GetType())) {
        bool el_already_in_added = false;
        BOOST_FOREACH (const tSptrMapElement& parking_el,
                       parking_el_to_check_)
        {
            if (parking_el->GetID() == el->GetID()) {
                el_already_in_added = true;
                LOG_ERROR_PRINTF(
                    "AddParkingSpaceToCheckList el already in "
                    " vector: %d",
                    el->GetID());
                break;
            }
        }
        if (!el_already_in_added) {
            parking_el_to_check_.push_back(el);
            // enable us if no el to check
            SetUSPin();
        }
    }
}

// ____________________________________________________________________________
void ParkingSpaceDetection::MapOnEventDistanceReached(
    tSptrMapElement el, const tMapData threshold,
    bool distance_lower_threshold) {
    // NOT USED
}

// ____________________________________________________________________________
void ParkingSpaceDetection::UpdateInformJuryParkingSpaceStatus(
    MapElementParking* p_space, bool space_free) {
    if (p_space){
      if (property_debug_enabled_)
          p_space->user_color_ui_ = space_free ? "green" : "red";

      p_space->SetParkingStatus(space_free ? PARKING_FREE : PARKING_OCCUPIED);

      const tMapPoint center = p_space->GetGlobalPolyCenter();
      if (TransmitJuryParking(p_space->GetJuryID(),
                  center.get<0>(), center.get<1>(), // center parking space
                  space_free ? 0U : 1U))  // 1 is occupied
      {
          p_space->JuryInformed(_clock->GetStreamTime());
          //remove_ids.push_back(p_space->GetID());
          //todo remove form check list
          //check of locks if rm
      } else {
          LOG_ERROR_PRINTF("faild sending parking info to jury");
      }
    }
}

void ParkingSpaceDetection::CheckParkingSpacesInRange() {
    // check all parking spaces in range; if free after + under projected distance
    // the parking space status is set.
    // If the jury wasn't informed of the state the status will be send once
    // parking spots in range are added to parking_el_to_check_ in the map
    // events
    __synchronized_obj(update_mutex_);

    BOOST_FOREACH (tSptrMapElement& parking_el, parking_el_to_check_) {
        // check if free if ProjectedDistanceToCar under range
        if (parking_el->GetDistanceToCar() <= property_range_check_parking_)
        {
            // get and set jruy infomration
            MapElementParking* el_p =
                dynamic_cast<MapElementParking*>(parking_el.get());

            // inform the jury if not informed before
            if (el_p && !el_p->IsJuryInformed()) {
              bool space_free = IsFree(parking_el);

              if (!space_free) {
                // send info to jury if parking was detected as occupied
                UpdateInformJuryParkingSpaceStatus(el_p, space_free);
              }else if (parking_el->ProjectedDistanceToCar()
                        <= property_range_send_jury_)
              {
                //inform jury if parking space is out of range
                // the parking space was not marked as occupied before
                // -> cars US passed the car
                if (!space_free){
                  LOG_WARNING_PRINTF("parking out of range but is occupied");
                }
                UpdateInformJuryParkingSpaceStatus(el_p, space_free);
              }
            } else if (!el_p) {
                LOG_ERROR_PRINTF("CheckParkingSpacesInRange cast faild");
            }else{
              LOG_WARNING_PRINTF("checked parking space still in check list");
            }
        }
    }
}

bool ParkingSpaceDetection::IsFree(tSptrMapElement& parking_el) {
    // map collition to check if parking_el is free
    // list with tpyes to exclude form collision check
    static const std::vector<MapElementType> exclude_types =
        boost::assign::list_of(DEBUG_POINT)(DEBUG_POLYGON)(STREET_MARKER_LANE)(
            STREET_MARKER_STOP_LINE)(STREET_MARKER_CROSSING_T)(
            STREET_MARKER_CROSSING_X)(STREET_MARKER_ZEBRA)(LM_STREET_LANE)(
            PLANNER_PATH);//TODO add signs
    std::vector<tSptrMapElement> collision_els;

    if (map_->CheckElementsCollision(
            parking_el, collision_els,
            5,                  // max range TODO(markus) range parking check
            -1,
            &exclude_types))   // cnt the Ultrasonic hits for the parking space
    {
        unsigned int us_hits = 0;
        unsigned int depth_hits = 0;
        bool car_center_in_parking_space = false;
        BOOST_FOREACH (const tSptrMapElement& el, collision_els) {
            if (el->GetType() == ULTRRASONIC) {
                us_hits += std::max(el->GetFuseCount(), 1U);
            }
            else if (el->GetType() == DEPTH) {
                depth_hits += std::max(el->GetFuseCount(), 1U);
            }if (el->GetType() == CAR && el->GetFuseCount() > 2) {
              //check if gloabal center point of the car is in the parking space
              if (!parking_el->GlobalPolyDisjoint(el->GetGlobalPolyCenter())){
                // note depth data is fused with the car
                car_center_in_parking_space = true;
                //break;
              }
            }
        }

        if (us_hits >= property_parking_space_us_hits_occupied_) {
          IF_DEBUG_LOG_PRINTF("occupied parking space detected "
                                "id %d us hits: %d",
                              parking_el->GetID(), us_hits);
          return false;
        }else if (depth_hits >= property_parking_space_hepth_hits_occupied_){
          IF_DEBUG_LOG_PRINTF("occupied  parking space detected"
                              "map id %d depth hits: %d",
                              parking_el->GetID(), depth_hits);
          return false;
        }else if (car_center_in_parking_space){
          IF_DEBUG_LOG_PRINTF("occupied  parking space detected"
                              " car center point in parking",
                              parking_el->GetID());
          return false;
        }
    }

    return true;
}

void ParkingSpaceDetection::SetUSPin() {
    bool parking_in_scan_range = false;
    BOOST_FOREACH (const tSptrMapElement& parking_el, parking_el_to_check_) {
        if (property_debug_enabled_) {
            // change color if in range
            // orange scanning
            // magenta removed from scanning and no different color was set
            MapElementParking* el_p =
                dynamic_cast<MapElementParking*>(parking_el.get());
            if (el_p) {
                if (parking_el->GetDistanceToCar() <= property_range_check_parking_)
                {
                    // in the debug mode set the clolor to orange if waiting for
                    // data
                    if (!el_p->IsJuryInformed()) {
                        // mark in range elememnt with color
                        parking_el->user_color_ui_ = "orange";
                    }
                } else if (parking_el->user_color_ui_ == "orange") {
                    // parking was in range an moved out of range before jury
                    // checked
                    // if the parking is checked in the debug mode the color is
                    // changed
                    IF_DEBUG_LOG_PRINTF(
                        "Parking was in range but moved out of "
                        "range before checked if free");
                        parking_el->user_color_ui_ = "magenta";
                }
            }
        }
        //if one space is in range enable scanning
        if (parking_el->GetDistanceToCar() <= property_range_check_parking_) {
            parking_in_scan_range = true;
            if (!property_debug_enabled_) {
                break;//keep mark all the parking in range orange
            }
        }
    }

    // enable us if parking space in range check
    if (!parking_in_scan_range && us_enabled_) {
        IF_DEBUG_LOG_PRINTF("parking sending us signal disable signal");
        pin_out_us_enable_.Transmit<tBool>(tFalse, _clock->GetStreamTime());
        us_enabled_ = false;
    } else if (parking_in_scan_range && !us_enabled_) {
        IF_DEBUG_LOG_PRINTF(" parking  sending us signal enable signal");
        pin_out_us_enable_.Transmit<tBool>(tTrue, _clock->GetStreamTime());
        us_enabled_ = true;
    }
}
