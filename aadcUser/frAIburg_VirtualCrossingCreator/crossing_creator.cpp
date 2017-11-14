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
#include "crossing_creator.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF,
                   CrossingCreator)

using namespace frAIburg::map;

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (property_debug_enabled_) {   \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

// ____________________________________________________________________________
CrossingCreator::CrossingCreator(const tChar* __info)
    : cFilter(__info),
      frAIburg::map::MapEventListener(),
      map_(NULL),
      timer_(NULL) {
    SetPropertyBool("Use fix xml crossings", false);
    SetPropertyBool("Debug_mode", false);
    SetPropertyStr("Debug_mode" NSSUBPROP_DESCRIPTION, "Enable Debug Mode");
    SetPropertyBool("Debug_mode" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyBool("Only add crossings if corresponding crossing sign is in map", tTrue);

    SetPropertyFloat("zero_sign_angle_offset_x", -0.355);
    SetPropertyStr("zero_sign_angle_offset_x" NSSUBPROP_DESCRIPTION,
                   "for sign with orientation zero (aligned with global xy): "
                   "sign-offset to crossing-bottom-right-corner in x "
                   "direction. Testevent value");
    SetPropertyBool("zero_sign_angle_offset_x" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("zero_sign_angle_offset_y", -0.135);
    SetPropertyStr("zero_sign_angle_offset_y" NSSUBPROP_DESCRIPTION,
                   "for sign with orientation zero (aligned with global xy): "
                   "sign-offset to crossing-bottom-right-corner in y "
                   "direction. Testevent value");
    SetPropertyBool("zero_sign_angle_offset_y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("zero_sign_angle_offset_x_stopline", -0.2);
    SetPropertyStr("zero_sign_angle_offset_x_stopline" NSSUBPROP_DESCRIPTION,
                   "for sign with orientation zero (aligned with global xy): "
                   "sign-offset to crossing-bottom-right-corner in x "
                   "direction");
    SetPropertyBool("zero_sign_angle_offset_x_stopline" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat("zero_sign_angle_offset_y_stopline", -0.15);
    SetPropertyStr("zero_sign_angle_offset_y_stopline" NSSUBPROP_DESCRIPTION,
                   "for sign with orientation zero (aligned with global xy): "
                   "sign-offset to crossing-bottom-right-corner in y "
                   "direction");
    SetPropertyBool("zero_sign_angle_offset_y_stopline" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat("half_crossing_width", 0.5);
    SetPropertyStr(
        "half_crossing_width" NSSUBPROP_DESCRIPTION,
        "from car perspective: crossing-bottom-right-corner in y direction");
    SetPropertyBool("half_crossing_width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("half_crossing_height", 0.5);
    SetPropertyStr(
        "half_crossing_height" NSSUBPROP_DESCRIPTION,
        "from car perspective: crossing-bottom-right-corner in x direction");
    SetPropertyBool("half_crossing_height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // Map range check
    SetPropertyFloat("property_map_range_check_sign", 1.0);
    SetPropertyStr("property_map_range_check_sign" NSSUBPROP_DESCRIPTION,
                   "The range when the map notifies me for new signs");
    SetPropertyBool("property_map_range_check_sign" NSSUBPROP_ISCHANGEABLE,
                    tTrue);
    SetPropertyFloat("property_map_range_check_marker", 1.0);
    SetPropertyStr("property_map_range_check_marker" NSSUBPROP_DESCRIPTION,
                   "The range when the map notifies me for new markers");
    SetPropertyBool("property_map_range_check_marker" NSSUBPROP_ISCHANGEABLE,
                    tTrue);

    SetPropertyFloat("check_time_intervall_second", 0.1);
    SetPropertyStr("check_time_intervall_second" NSSUBPROP_DESCRIPTION,
                   "timer starts run every x [s]");

    SetPropertyFloat("Placeholder_2", 0.0);
    SetPropertyStr("Placeholder_2" NSSUBPROP_DESCRIPTION, "...");
    SetPropertyBool("Placeholder_2" NSSUBPROP_ISCHANGEABLE, tTrue);
}

// ____________________________________________________________________________
CrossingCreator::~CrossingCreator() {}

// ____________________________________________________________________________
tResult CrossingCreator::Start(__exception) {
    running_ok_ = true;
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult CrossingCreator::Stop(__exception) {
    running_ok_ = false;
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult CrossingCreator::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    slim::register_pin_func func = &CrossingCreator::RegisterPin;


    if (eStage == StageFirst) {
        RETURN_IF_FAILED(crossing_input_pin_.FirstStageCreate(
        this, func, "crossing_in", "tCrossing"));
    } else if (eStage == StageNormal) {
        InitMAP();
    } else if (eStage == StageGraphReady) {
        if (!filter_properties_.use_fix_xml_crossings_)
            InitTimer();  // this must be after GetAllStaticProperties
        // set input pin id
        vector<string> id = boost::assign::list_of("x")("y")
            ("heading")("accuracy");
        RETURN_IF_FAILED(crossing_input_pin_.StageGraphReadySetIDOrder(id));
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult CrossingCreator::Shutdown(tInitStage eStage,
                                         ucom::IException** __exception_ptr) {
    if (eStage == StageGraphReady) {
        if (!filter_properties_.use_fix_xml_crossings_) DeleteTimer();
        map_->DeregisterEventListener(this);
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ____________________________________________________________________________
tResult CrossingCreator::OnPinEvent(IPin* source, tInt nEventCode,
                                           tInt nParam1, tInt nParam2,
                                           IMediaSample* media_sample) {
    RETURN_IF_POINTER_NULL(media_sample);
    RETURN_IF_POINTER_NULL(source);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if (crossing_input_pin_.isSource(source)) {
            AddCrossingToVector(media_sample);
        }
    }
    RETURN_NOERROR;
}

tResult CrossingCreator::AddCrossingToVector(IMediaSample* sample) {
    RETURN_IF_POINTER_NULL(sample);

    // TODO(jan) use time! maybe save within crossing object
    tTimeStamp time_image_taken = sample->GetTime();
    tCrossing* crossing = NULL;
    if (IS_OK(crossing_input_pin_.ReadNoID_start(sample,
                                        (const tVoid**)&crossing,
                                        sizeof(tCrossing)))){
        AddLaneCrossingToMap(*crossing);
        crossing_input_pin_.ReadNoID_end(sample, (const tVoid**)&crossing);
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult CrossingCreator::PropertyChanged(const tChar* str_name) {
    RETURN_IF_FAILED(cFilter::PropertyChanged(str_name));
    // associate the properties to the member
    if (cString::IsEqual(str_name, "Debug_mode")) {
        property_debug_enabled_ = GetPropertyBool("Debug_mode");
    } else if (cString::IsEqual(str_name, "zero_sign_angle_offset_y")) {
        filter_properties_.sign_offset_y =
            GetPropertyFloat("zero_sign_angle_offset_y");
    } else if (cString::IsEqual(str_name, "zero_sign_angle_offset_x")) {
        filter_properties_.sign_offset_x =
            GetPropertyFloat("zero_sign_angle_offset_x");
    } else if (cString::IsEqual(str_name, "half_crossing_width")) {
        filter_properties_.half_crossing_width =
            GetPropertyFloat("half_crossing_width");
    } else if (cString::IsEqual(str_name, "half_crossing_height")) {
        filter_properties_.half_crossing_height =
            GetPropertyFloat("half_crossing_height");
    } else if (cString::IsEqual(str_name, "property_map_range_check_sign")) {
        filter_properties_.property_map_range_check_sign =
            GetPropertyFloat("property_map_range_check_sign");
    } else if (cString::IsEqual(str_name, "property_map_range_check_marker")) {
        filter_properties_.property_map_range_check_marker =
            GetPropertyFloat("property_map_range_check_marker");
    } else if (cString::IsEqual(str_name,
                                "zero_sign_angle_offset_x_stopline")) {
        filter_properties_.zero_sign_angle_offset_x_stopline =
            GetPropertyFloat("zero_sign_angle_offset_x_stopline");
    } else if (cString::IsEqual(str_name,
                                "zero_sign_angle_offset_y_stopline")) {
        filter_properties_.zero_sign_angle_offset_y_stopline =
            GetPropertyFloat("zero_sign_angle_offset_y_stopline");
    } else if (cString::IsEqual(str_name, "Placeholder_2")) {
        filter_properties_.pl2 = GetPropertyInt("Placeholder_2");

    } else if (cString::IsEqual(str_name, "Use fix xml crossings")) {
        filter_properties_.use_fix_xml_crossings_ =
            GetPropertyBool("Use fix xml crossings");
    } else if (cString::IsEqual(str_name, "only add crossings if corresponding crossings sigin in map")) {
        filter_properties_.validate_crossings_with_map_ =
            GetPropertyBool("only add crossings if corresponding crossings sigin in map");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void CrossingCreator::InitMAP() {
    map_ = frAIburg::map::getInstance();
    // range to check signs
    // AddDistanceThreshold(filter_properties_.property_map_range_check_sign);
    // //range to check marker
    // AddDistanceThreshold(filter_properties_.property_map_range_check_marker);
    map_->RegisterEventListener(this);
}

// ____________________________________________________________________________
void CrossingCreator::InitTimer() {
    float check_time_intervall_second =
        GetPropertyFloat("check_time_intervall_second");
    tUInt32 nFlags = 0;
    // Create our cyclic timer.
    cString strTimerName = OIGetInstanceName();
    strTimerName += ".rttimerFreeParkingSpaceDetectionTimer";
    tTimeStamp tmPeriod = 1e6 * check_time_intervall_second;
    tTimeStamp tmstartdelay = 0;
    timer_ = _kernel->TimerCreate(tmPeriod,
                                  tmstartdelay,  // dely for the first start
                                  static_cast<IRunnable*>(this),  // call run
                                  NULL, NULL, 0, nFlags, strTimerName);
    if (!timer_) {
        LOG_ERROR_PRINTF(
            "CrossingCreator filter unable to create timer.");
    }
}

// ____________________________________________________________________________
void CrossingCreator::DeleteTimer() {
    if (timer_) {
        running_ok_ = false;
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
    }
}

bool CrossingCreator::RemoveElementCheck(
    std::vector<tSptrMapElement>& map_vec, tMapID id) {
    std::vector<tSptrMapElement>::iterator it = map_vec.begin();
    for (; it != map_vec.end(); ++it) {
        if ((*it)->GetID() == id) {
            map_vec.erase(it);
            return true;
        }
    }
    return false;
}

// ____________________________________________________________________________
void CrossingCreator::MapOnEventRemoved(tSptrMapElement el) {
    __synchronized_obj(update_mutex_);
    if (RemoveElementCheck(crossing_relevant_signs_, el->GetID())) {
        LOG_ERROR_PRINTF(
            "In class CrossingCreator:"
            " Sign was removed from Map id %d",
            el->GetID());
    }
    // crossings are removed with distance
    if (RemoveElementCheck(crossing_relevant_markings_, el->GetID())) {
        LOG_INFO_PRINTF(
            "In class CrossingCreator:"
            " marker was removed from Map!");
    }
}

// ____________________________________________________________________________
bool CrossingCreator::isValid(const tSptrMapElement &new_crossing,
  tMapData max_distance_to_crossing_sign_center)
{

  std::vector<tSptrMapElement> relevant_crossing_signs;
  const std::vector<MapElementType> gettypes = boost::assign::list_of(
      STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT)(STREET_TRAFFIC_SIGN_STOP)(
      STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC)(
      STREET_TRAFFIC_SIGN_GIVE_WAY);
  map_->GetAllElementsWithTypes(gettypes, relevant_crossing_signs,
          1.5/*max range from car*/);
  if (relevant_crossing_signs.empty()) {
      return false;
  }
  BOOST_FOREACH (const tSptrMapElement& s, relevant_crossing_signs) {

        //there is a relevant sign in front of the car
        tSptrMapElement el_crossing = CreateCrossingElementFromSign(
                                      s,
                                      new_crossing->GetType(),
                                      (tTimeMapStamp) _clock->GetStreamTime());
        if (new_crossing->IsSimilar(*el_crossing.get(),
                                        max_distance_to_crossing_sign_center,
                                        -1/*not use area*/))
        {
          IF_DEBUG_LOG_PRINTF("CrossingCreator vaild check ok");
          return true;
        }
  }
  IF_DEBUG_LOG_PRINTF("CrossingCreator vaild vaild check was not ok");
  return false;
}

// ____________________________________________________________________________
void CrossingCreator::MapOnEventAddedNew(
    frAIburg::map::tSptrMapElement el) {
    __synchronized_obj(update_mutex_);

    UpdateRelevantElementToSignVector(el);
}

// ____________________________________________________________________________
void CrossingCreator::MapOnEventDistanceReached(
    tSptrMapElement el, const tMapData threshold,
    bool distance_lower_threshold) {
    // NOT USEED
}

// ____________________________________________________________________________
void CrossingCreator::UpdateRelevantElementToSignVector(
    frAIburg::map::tSptrMapElement& el)
{
    switch (el->GetType())
    {
        case STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT:
        case STREET_TRAFFIC_SIGN_STOP:
        case STREET_TRAFFIC_SIGN_GIVE_WAY:
        case STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC:{
            // only consider signs from the sensor (ignore signs from xml)
            MapElementRoadSign* el_sign =
                dynamic_cast<MapElementRoadSign*>(el.get());
            if (!el_sign) {
                LOG_ERROR_PRINTF(
                    "CrossingCreator: Cast from MapElement to "
                    "MapElementRoadSign failed.");
                return;
            }

            if (!filter_properties_.use_fix_xml_crossings_ &&
                el_sign->GetSignSubType() == SIGN_SENSOR) {
                crossing_relevant_signs_.push_back(el);
                IF_DEBUG_LOG_PRINTF("CrossingCreator sign added %s.",
                                    el->ToString().c_str());
            } else if (filter_properties_.use_fix_xml_crossings_
                       && el_sign->GetSignSubType() == SIGN_LANDMARK)
             {
               //addd a crossing from xml
               //disable rm over dist and repostioning jumps
                IF_DEBUG_LOG_PRINTF("CrossingCreator xml sign added %s.",
                                    el->ToString().c_str());
                tSptrMapElement el_crossing = CreateCrossingElementFromSign(
                                              el,
                                              STREET_MARKER_CROSSING_X,
                                              0);//no remove over distances

                map_->AddFuseElement(el_crossing,
                   1.5,  // max distances to fuse (last merge was 1.5)
                   -1,
                   _clock->GetStreamTime(),
                   MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_007);  // check orientation
               el_crossing->SetUpdateWithRepostionJumps(false);
          }
          break;
        }
        default:
            break;
      }
}


// ____________________________________________________________________________
tResult CrossingCreator::Run(tInt nActivationCode,
                                    const tVoid* pvUserData,
                                    tInt szUserDataSize, __exception) {
    // timer check all crossing relevant elements in range
    if (running_ok_) {
        // handle first all marker and fuse replace if a sign is detected later
        // TODO disabled for test event HandleMapMarkerElementsWithinDistance();
        HandleMapSignElementsWithinDistance();
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void CrossingCreator::HandleMapSignElementsWithinDistance() {
    // adding crossing if fuse cnt high
    // stop adding crossing if  ProjectedDistanceToCar < -0.1
    std::vector<tMapID> remove_ids;
    __synchronized_obj(update_mutex_);
    const tMapData fuse_cnt_to_add_sign = 2;  // TODO adjust fuse cnt number
    // copy and update list to remove element
    std::vector<frAIburg::map::tSptrMapElement> current_sign =
        crossing_relevant_signs_;
    BOOST_FOREACH (tSptrMapElement crossing_relevant_el, current_sign) {
        switch (crossing_relevant_el->GetType()) {
            case STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT:
            case STREET_TRAFFIC_SIGN_STOP:
            case STREET_TRAFFIC_SIGN_GIVE_WAY:
            case STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC: {
                // TODO fix in test event use road sign xml if Corresponding
                // found
                bool landmark_found = false;
                // MapElementRoadSign *el_r = dynamic_cast<MapElementRoadSign*>(
                //                               crossing_relevant_el.get());
                // if (el_r && el_r->GetSignSubType() == SIGN_SENSOR){
                //   tSptrMapElement el_xml =  el_r->GetCorrespondingSign();
                //   if (el_xml){
                //     if (!MapHelper::IsElementOrientatedToCar(map_,
                //             *(crossing_relevant_el.get()), 0.7 * M_PI )){
                //               //TODO DO nicer
                //               //Don t use the xml if the sensor sign is not
                //               facing to the car
                //               //LOG_ERROR_PRINTF("Roadsign element with
                //               corresponding "
                //               //          "sigin facing away form car");
                //         LOG_ERROR_PRINTF("Roadsign element with corresponding
                //         "
                //                   "sigin facing away form car");
                //     }else if ( el_xml->GetID() != MAP_DEFAULT_ID){
                //       //change color for the sensror
                //       if (property_debug_enabled_)
                //         crossing_relevant_el->user_color_ui_ = "hotpink";
                //       crossing_relevant_el = el_xml;
                //       landmark_found = true;
                //     }
                //     else if ( el_xml->GetID() == MAP_DEFAULT_ID)
                //       LOG_ERROR_PRINTF("Roadsign element with corresponding "
                //                 "sigin with default id, removed form map?!");
                //   }
                // }else if (el_r && el_r->GetSignSubType() == SIGN_LANDMARK){
                //   break;//TODO (markus)fix for test event
                // }

                // check if sign is oriented towards car
                if (!MapHelper::IsElementOrientatedToCar(
                        map_, *(crossing_relevant_el.get()), 0.7 * M_PI) &&
                    !landmark_found) {  // wait until sign origined so that sign
                                        // can be added if angle is rounded to
                                        // grid
                    if (property_debug_enabled_)
                        crossing_relevant_el->user_color_ui_ = "yellowgreen";
                } else if (crossing_relevant_el->GetDistanceToCar() > 1.5) {
                    // over max range to add crossing
                    // also not take the landmark if taken
                    if (property_debug_enabled_)
                        crossing_relevant_el->user_color_ui_ = "purple";
                } else if (crossing_relevant_el->GetFuseCount() <
                               fuse_cnt_to_add_sign &&
                           !landmark_found) {
                    if (property_debug_enabled_)
                        crossing_relevant_el->user_color_ui_ = "orange";
                } else {
                    tSptrMapElement el = CreateCrossingElementFromSign(
                                      crossing_relevant_el,
                                      STREET_MARKER_CROSSING_X,
                                      (tTimeMapStamp) _clock->GetStreamTime());

                    map_->AddFuseElement(el,
                       1.5,  // max distances to fuse (last merge was 1.5)
                       -1,
                       _clock->GetStreamTime(),
                       MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_007);  // check orientation
                    if (property_debug_enabled_) {
                        el->user_tag_ui_ = "CrossingFuseCnt: " +
                                           boost::lexical_cast<std::string>(
                                               el->GetFuseCount()) +
                                           " id:";

                        crossing_relevant_el->user_color_ui_ = "green";
                        el->user_color_ui_ = "dodgerblue";
                    }
                }
                break;
            }
            default:
                LOG_WARNING_PRINTF(
                    "HandleMapSignElementsWithinDistance: "
                    "Element Type not handled. %s",
                    crossing_relevant_el->ToString().c_str());
                break;
        }
        // TODO markus changed at test event event
        // add crossing again if drive by again
        // signs are not removed from the map
        // remove if over projected distance
        // if (crossing_relevant_el->ProjectedDistanceToCar() < -0.1){ //TODO
        // set dist
        //   if (property_debug_enabled_){
        //
        //     if (crossing_relevant_el->GetFuseCount() < fuse_cnt_to_add_sign){
        //       LOG_WARNING_PRINTF("HandleMapSignElements "
        //           "fuse cnt low no crossing added: %s",
        //           crossing_relevant_el->ToString().c_str());
        //     }
        //     if(!MapHelper::IsElementOrientatedToCar(map_,
        //         *(crossing_relevant_el.get()), M_PI_2)) { //3/4*M_PI is too
        //         restrictive for corners   // Not the same orientation
        //         threshold angle as above
        //         LOG_WARNING_PRINTF("Sign is not oriented towards car "
        //                             " no crossing added: %s",
        //                             crossing_relevant_el->ToString().c_str());
        //     }
        //     crossing_relevant_el->user_color_ui_ = "red";
        //   }
        //
        //   if (!RemoveElementCheck(crossing_relevant_signs_,
        //                           crossing_relevant_el->GetID()))
        //    {
        //      LOG_WARNING_PRINTF("CrossingCreator: "
        //                         "sign el not removed after "
        //                         "over rm distance id: %d",
        //                         crossing_relevant_el->GetID());
        //    }
        //
        // }
    }
}

// ____________________________________________________________________________
tSptrMapElement CrossingCreator::CreateCrossingElementFromSign(
    const tSptrMapElement &sign_detected,
    MapElementType crossing_type,
    tTimeMapStamp t)
{
    // add crossing to map
    tMapData rotation_angle = 0;
    GetFixedGridOrientation(sign_detected->GetGlobalOrientation(),
                              &rotation_angle);
    // rotation clockwise
    // offset at angle 0 is negative -> substract height and width

    tMapPoint center_of_sign = sign_detected->GetGlobalPolyCenter();
    float center_of_crossing_x =
      cos(rotation_angle) *
          (filter_properties_.sign_offset_x - filter_properties_.half_crossing_height) -
      sin(rotation_angle) *
          (filter_properties_.sign_offset_y - filter_properties_.half_crossing_width) +
      center_of_sign.get<0>();

    float center_of_crossing_y =
        sin(rotation_angle) *
            (filter_properties_.sign_offset_x - filter_properties_.half_crossing_height) +
        cos(rotation_angle) *
            (filter_properties_.sign_offset_y - filter_properties_.half_crossing_width) +
        center_of_sign.get<1>();

    tMapPoint center_of_crossing(center_of_crossing_x, center_of_crossing_y);
    return AddCrossingBoxToMap(center_of_crossing, crossing_type,
                               rotation_angle, t);  // TODO fuse type
}

// ____________________________________________________________________________
tSptrMapElement CrossingCreator::AddCrossingBoxToMap(
    const tMapPoint& crossing_center_global,
    MapElementType crossing_type,
    tMapData grid_orientation,
    tTimeMapStamp t)
{
    std::vector<tMapPoint> crossing_rectangle;
    MapHelper::CreatBoxPoints(
        crossing_center_global.get<0>(), crossing_center_global.get<1>(),
        filter_properties_.half_crossing_width,
        filter_properties_.half_crossing_height, crossing_rectangle);

    // always create new shared_pointer
    tSptrMapElement crossing_poly(
        new MapElement(crossing_type, crossing_rectangle, t, grid_orientation));
    crossing_poly->LocalToGlobal(MapHelper::GetCarPosLocalIsGlobal());
    return crossing_poly;
}

// ____________________________________________________________________________
void CrossingCreator::GetFixedGridOrientation(
    const tMapData& sign_orientation, tMapData* return_grid_orientation) {
    // Maps the orientation to the four directions of the grid
    float ratio = sign_orientation / M_PI_2;  // Please note exception: for
                                              // negative orientations a ratio
                                              // of -1.5 is rounded to -1
    ratio += 0.5;
    ratio = floor(ratio);
    *return_grid_orientation = M_PI_2 * ratio;
}

// ____________________________________________________________________________
bool CrossingCreator::AddLaneCrossingToMap(const tCrossing &crossing) {
    tTimeMapStamp t = _clock->GetStreamTime();
    std::vector<tMapPoint> box_points;
    tMapData fixed_heading;  // = crossing.heading;
    GetFixedGridOrientation(crossing.heading, &fixed_heading);
    MapHelper::CreatBoxPoints(crossing.x, crossing.y, 0.5, 0.5,
                    box_points, &fixed_heading);

    tSptrMapElement crossing_poly(new MapElement(STREET_MARKER_CROSSING_X,
                                                box_points, t,
                                                fixed_heading));
    crossing_poly->user_color_ui_ = "purple";
    tMapCarPosition pos;
    if (map_->GetGlobalCarPosition(&pos)) {
      // transform to global frame to compare the crossings
      crossing_poly->LocalToGlobal(pos);
      if (filter_properties_.validate_crossings_with_map_
          && isValid(crossing_poly, 0.2/*range for similar crossing in map*/))
      {
        map_->AddFuseElement(crossing_poly,  // add the poly to the map
                             0.3,        // max distances to fuse
                             -1, _clock->GetStreamTime(),
                            MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_007);
        // crossing_poly->EnableTimeOfLife(1. * 1e6, &t);
      }
      // crossing_poly->user_color_ui_ = "green";

    }




    return true;
}
