
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
#include "map_element_types.h"

namespace frAIburg {
namespace map {

// ----------------------------------------------------------------------------
//                                    AADCJuryStatus
// ----------------------------------------------------------------------------

AADCJuryStatus::AADCJuryStatus() : jury_id_(-1), jury_informed_(false) {}

AADCJuryStatus::AADCJuryStatus(int jury_id)
    : jury_id_(jury_id), jury_informed_(false) {}

AADCJuryStatus::~AADCJuryStatus() {}

bool AADCJuryStatus::IsJuryInformed() const { return jury_informed_; }
void AADCJuryStatus::JuryInformed(tTimeMapStamp t) {
    jury_informed_ = true;
    jury_informed_time_ = t;
}
bool AADCJuryStatus::GetJuryInformedTime(tTimeMapStamp *t) {
    *t = jury_informed_time_;
    return jury_informed_;
}
int AADCJuryStatus::GetJuryID() const { return jury_id_; }

bool AADCJuryStatus::JuryUpdateFuse(const AADCJuryStatus *element_fuse) {
    if (!element_fuse || jury_id_ != element_fuse->GetJuryID()) {
        // check if id are same, but continue if not
        LOG_WARNING_PRINTF("jury fuse, null or status fuse id not same");
        return false;
    }

    if (element_fuse->IsJuryInformed() && !jury_informed_) {
        GetJuryInformedTime(&jury_informed_time_);
        jury_informed_ = true;
    }
    return true;
}

// ----------------------------------------------------------------------------
//                            MapElementParking
// ----------------------------------------------------------------------------

MapElementParking::MapElementParking(short jury_id, MapParkingStatus status,
                                     const std::vector<tMapPoint> &local_poly,
                                     tTimeMapStamp timestamp,
                                     tMapData local_orientation_angle)
    : MapElement(LM_STREET_PARKING_SPOT, local_poly, timestamp,
                 local_orientation_angle),
      AADCJuryStatus(jury_id),
      status_(status),
      status_is_set_(false) {}

// ____________________________________________________________________________
MapFuseReturn MapElementParking::UpdateFuse(
    const tMapCarPosition &global_car_pos, MapElement &element_fuse,
    tTimeMapStamp timestamp /*= 0*/, MapFuseType fuse_type) {
    // fuse of a parking space should not happen
    LOG_ERROR_PRINTF("MapElementParking is fused");
    if (GetType() == element_fuse.GetType()) {
        const MapElementParking *el_f =
            dynamic_cast<const MapElementParking *>(&element_fuse);
        if (el_f) {
            // safely casted to MapElementParking
            MapParkingStatus new_status;
            if (el_f->GetStatus(&new_status)) {
                status_ = new_status;
            }
            // fuse the jury info
            JuryUpdateFuse(static_cast<const AADCJuryStatus *>(el_f));
        } else {
            LOG_ERROR_PRINTF("MapElementParking fuse error: cast failed");
        }
    }
    return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
                                  fuse_type);
}

// ____________________________________________________________________________
void MapElementParking::SetParkingStatus(MapParkingStatus new_status) {
    if (!status_is_set_) status_is_set_ = true;
    status_ = new_status;
}

// ____________________________________________________________________________
bool MapElementParking::GetStatus(MapParkingStatus *ret_status) const {
    *ret_status = status_;
    return status_is_set_;
}

// ____________________________________________________________________________
std::string MapElementParking::ToString() const {
    std::string status("| parking status: ");
    if (status_ == PARKING_FREE) {
        status += "free";
    } else {
        status += "occupied";
    }
    return MapElement::ToString() + status;
}

// ____________________________________________________________________________
bool MapElementParking::IsSimilar(const MapElement &el,
                                  tMapData tolerance_distance /*= 0.1*/,
                                  tMapData tolerance_area /*= -1*/,
                                  bool similar_fuse /*= true*/) {
    return MapElement::IsSimilar(el, tolerance_distance, tolerance_area,
                                 similar_fuse);
}

// ____________________________________________________________________________
bool MapElementParking::IsParkingType(MapElementType type) {
    switch (type) {
        case LM_STREET_PARKING_SPOT:
        case STREET_TRAFFIC_SIGN_PARKING:
            return true;
        default:
            return false;
    }
}

// ----------------------------------------------------------------------------
//                              MapElementRoadSign
// ----------------------------------------------------------------------------

MapElementRoadSign::MapElementRoadSign(short jury_id_sign,
                                       MapRoadSignSubType type,
                                       const std::vector<tMapPoint> &local_poly,
                                       tTimeMapStamp timestamp,
                                       tMapData local_orientation_angle)
    : MapElement(AADCSignTypeToMapType(jury_id_sign), local_poly, timestamp,
                 local_orientation_angle),
      AADCJuryStatus(jury_id_sign),
      sign_type_(type) {}

// ____________________________________________________________________________
MapFuseReturn MapElementRoadSign::UpdateFuse(
    const tMapCarPosition &global_car_pos, MapElement &element_fuse,
    tTimeMapStamp timestamp /*= 0*/, MapFuseType fuse_type) {
    const MapElementType ftype = element_fuse.GetType();
    if (ftype == GetType()) {
        if (!JuryUpdateFuse(
                dynamic_cast<const MapElementRoadSign *>(&element_fuse))) {
            LOG_ERROR_PRINTF("MapElementRoadSign fuse error: no jury parent");
        }
        return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
                                      fuse_type);
    } else if (ftype == DEPTH) {
        // fuse with depth: don t use the data, the depth data in the map will
        // be
        // replaced in the map if the depth data was befor the sign
        // because the depth fuse return ensure a replacement of the sign
        return MAP_UPDATE_FUSE_KEEP;
    } else {
        LOG_ERROR_PRINTF("MapElementRoadSign fuse error: %s with %s",
                         MapHelper::TypeToString(GetType()),
                         MapHelper::TypeToString(ftype));
        return MAP_UPDATE_FUSE_FAILD;
    }
    return MAP_UPDATE_FUSE_FAILD;
}

// ____________________________________________________________________________
std::string MapElementRoadSign::ToString() const {
    std::string sign_type = "| sign sub type: ";
    if (sign_type_ == SIGN_SENSOR) {
        sign_type += "sensor";
    } else {
        sign_type += "landmark";
    }
    return MapElement::ToString() + sign_type + "| send to jury: " +
           (IsJuryInformed() ? "true" : "false");
    // + boost::lexical_cast<std::string>(IsJuryInformed());
}

// ____________________________________________________________________________
bool MapElementRoadSign::IsSimilar(const MapElement &element_fuse,
                                   tMapData tolerance_distance /*= 0.1*/,
                                   tMapData tolerance_area /*= -1*/,
                                   bool similar_fuse /*= true*/) {
    bool ret = false;
    bool is_similar_type_ignored = MapElement::IsSimilar(
        element_fuse, tolerance_distance, tolerance_area, false);
    if (similar_fuse) {
        // return true if the elements can be fused
        const MapElementType ftype = element_fuse.GetType();
        if (GetType() == ftype) {
            // fuse road sign if subtype is same and same disatance and area
            // diff
            // dynamic_cast only for base similar types
            const MapElementRoadSign *el_f =
                dynamic_cast<const MapElementRoadSign *>(&element_fuse);
            if (el_f) {
                ret = (sign_type_ == el_f->GetSignSubType()) &&
                      is_similar_type_ignored;
            } else {
                LOG_ERROR_PRINTF(
                    "MapElementRoadSign IsSimilar error, cased failed");
            }
        }
        // fuse with depth only if depth new depth is added to the map
        // not with existing depth in the map
        // because the fuse distance for sign elments is lanrge
        // and many diffrent types would be combinded in a multifuse
        // else if (ftype == DEPTH && sign_type_ != SIGN_LANDMARK) {
        //     ret = is_similar_type_ignored;
        // }
        else {
            ret = false;
        }

    } else {
        ret = is_similar_type_ignored;
    }
    return ret;
}

// ____________________________________________________________________________

void MapElementRoadSign::SetCorrespondingSign(tSptrMapElement el) {
    if (!IsRoadSign(el->GetType())) {
        LOG_ERROR_PRINTF("SetCorrespondingSign for not a road sign");
        return;
    }
    corresponding_sigin_ = el;
}
// ____________________________________________________________________________

tSptrMapElement MapElementRoadSign::GetCorrespondingSign() {
    return corresponding_sigin_;
}

// ____________________________________________________________________________
MapRoadSignSubType MapElementRoadSign::GetSignSubType() const {
    return sign_type_;
}

// ____________________________________________________________________________
MapElementType MapElementRoadSign::AADCSignTypeToMapType(short id) {
    switch (id) {
        case 0:
            return STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT;
        case 1:
            return STREET_TRAFFIC_SIGN_STOP;
        case 2:
            return STREET_TRAFFIC_SIGN_PARKING;
        case 3:
            return STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC;
        case 4:
            return STREET_TRAFFIC_SIGN_JUST_STRAIGHT;
        case 5:
            return STREET_TRAFFIC_SIGN_GIVE_WAY;
        case 6:
            return STREET_TRAFFIC_SIGN_CROSSWALK;
        case 7:
            return STREET_TRAFFIC_SIGN_CIRCLE;
        case 8:
            return STREET_TRAFFIC_SIGN_NO_TAKE_OVER;
        case 9:
            return STREET_TRAFFIC_SIGN_NO_ENTRY;
        case 10:
            return STREET_TRAFFIC_SIGN_POSITION_MARKER;
        case 11:
            return STREET_TRAFFIC_ONE_WAY_STREET;
        case 12:
            return STREET_TRAFFIC_ROAD_WORKS;
        case 13:
            return STREET_TRAFFIC_SIGN_SPEED_50;
        case 14:
            return STREET_TRAFFIC_SIGN_SPEED_100;
        default:
            LOG_ERROR_PRINTF("unknown sign type: %d", id);
            return UNKNOWN;
    }
}

// ____________________________________________________________________________
bool MapElementRoadSign::IsRoadSign(MapElementType type) {
    switch (type) {
        case STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT:
        case STREET_TRAFFIC_SIGN_STOP:
        case STREET_TRAFFIC_SIGN_PARKING:
        case STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC:
        case STREET_TRAFFIC_SIGN_JUST_STRAIGHT:
        case STREET_TRAFFIC_SIGN_GIVE_WAY:
        case STREET_TRAFFIC_SIGN_CROSSWALK:
        case STREET_TRAFFIC_SIGN_CIRCLE:
        case STREET_TRAFFIC_SIGN_NO_TAKE_OVER:
        case STREET_TRAFFIC_SIGN_NO_ENTRY:
        case STREET_TRAFFIC_ONE_WAY_STREET:
        case STREET_TRAFFIC_ROAD_WORKS:
        case STREET_TRAFFIC_SIGN_SPEED_50:
        case STREET_TRAFFIC_SIGN_SPEED_100:
        case STREET_TRAFFIC_SIGN_POSITION_MARKER:
            return true;
        default:
            return false;
    }
}

// ----------------------------------------------------------------------------
//                         MapElementBasePedestrian
// ----------------------------------------------------------------------------

MapElementPedestrian::MapElementPedestrian(
    MapPedestrianType type, MapElementOrientation orientation,
    const std::vector<tMapPoint> &local_poly, tTimeMapStamp timestamp)
    : MapElement(PedestrianTypeToMapType(type), local_poly, timestamp),
      pedestrian_type_(type),
      orientation_(orientation) {}

// ____________________________________________________________________________
MapFuseReturn MapElementPedestrian::UpdateFuse(
    const tMapCarPosition &global_car_pos, MapElement &element_fuse,
    tTimeMapStamp timestamp /*= 0*/, MapFuseType fuse_type) {

    const MapElementType ftype = element_fuse.GetType();
    if (GetType() == ftype) {
        // fuse with Pedestrian
        const MapElementPedestrian *el_f =
            dynamic_cast<const MapElementPedestrian *>(&element_fuse);
        if (el_f) {
            // safely casted to MapElementBasePedestrian
            orientation_ = el_f->GetPedestrianOrientation();
        } else {
            LOG_ERROR_PRINTF("MapElementPedestrian fuse error: cast failed");
        }
        return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
                                      fuse_type);
    } else if (ftype == DEPTH) {
        // fuse with depth
        // fuse with depth
         // TODO life time bug if MapElement::UpdateFuse is called
        // return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
        //                               MAP_CONVEX_HULL_SIMPLIFY_5);
        return MAP_UPDATE_FUSE_KEEP;// dont use depth data
    } else {
        LOG_ERROR_PRINTF("MapElementPedestrian fuse error: %s with %s",
                         MapHelper::TypeToString(GetType()),
                         MapHelper::TypeToString(ftype));
        return MAP_UPDATE_FUSE_FAILD;
    }
}

// ____________________________________________________________________________
bool MapElementPedestrian::IsSimilar(const MapElement &element_fuse,
                                     tMapData tolerance_distance /*= 0.1*/,
                                     tMapData tolerance_area /*= -1*/,
                                     bool similar_fuse /*= true*/) {
    bool ret = false;
    bool is_similar_type_ignored = MapElement::IsSimilar(
        element_fuse, tolerance_distance, tolerance_area, false);
    if (similar_fuse) {
        // return true if the elements can be fused and similar
        const MapElementType ftype = element_fuse.GetType();
        if (GetType() == ftype) {
            // fuse with same type
            // no fuse with depth because large fuse distance in pedestrian
            // is added
            ret = is_similar_type_ignored;
        } else {
            ret = false;
        }
    } else {
        ret = is_similar_type_ignored;
    }
    return ret;
}

// ____________________________________________________________________________
MapElementOrientation MapElementPedestrian::GetPedestrianOrientation() const {
    return orientation_;
}

// ____________________________________________________________________________
MapPedestrianType MapElementPedestrian::GetPedestrianType() const {
    return pedestrian_type_;
}

// ____________________________________________________________________________
std::string MapElementPedestrian::ToString() const {
    std::string orientation_str("| Pedestrian orientation: ");
    if (orientation_ == LEFT) {
        orientation_str += "LEFT";
    } else if (orientation_ == RIGHT) {
        orientation_str += "RIGHT";
    } else if (orientation_ == TOWARDS) {
        orientation_str += "TOWARDS";
    } else if (orientation_ == AWAY) {
        orientation_str += "AWAY";
    }
    return MapElement::ToString() + orientation_str;
}

// ____________________________________________________________________________
MapElementType MapElementPedestrian::PedestrianTypeToMapType(
    MapPedestrianType type) {
    switch (type) {
        case ADULT:
            return PEDESTRIAN_ADULT;
        case CHILD:
            return PEDESTRIAN_CHILD;
        case REAL_PERSON:
            return PEDESTRIAN_REAL;
        default:
            LOG_ERROR_PRINTF("PedestrianTypeToMapType type error");
            return UNKNOWN;
    }
}
// ____________________________________________________________________________
bool MapElementPedestrian::IsPedestrian(MapElementType type) {
    switch (type) {
        case PEDESTRIAN_CHILD:
        case PEDESTRIAN_ADULT:
            return true;
        default:
            return false;
    }
}

// ----------------------------------------------------------------------------
//                              MapElementDepth
// ----------------------------------------------------------------------------

MapElementDepth::MapElementDepth(const std::vector<tMapPoint> &local_poly,
                                 tTimeMapStamp timestamp)
    : MapElement(DEPTH, local_poly, timestamp) {}

// ____________________________________________________________________________
MapFuseReturn MapElementDepth::UpdateFuse(const tMapCarPosition &global_car_pos,
                                          MapElement &element_fuse,
                                          tTimeMapStamp timestamp /*= 0*/,
                                          MapFuseType fuse_type) {
    // combine depth data with MapElement::UpdateFuse
    // with the return value the map is notified to use the
    if (DEPTH == element_fuse.GetType()) {

      // fuse with same depth type
      return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
                                  fuse_type);
    } else if (IsFuseTypeWithDepth(element_fuse)) {
        // the depth data was defore in the map
        // fuse with different map types
        // the differnt types handle the fuse and the depth data in the map
        // is replaced
        element_fuse.UpdateFuse(global_car_pos, *this, timestamp, fuse_type);
        return MAP_UPDATE_FUSE_REPLACE;  // replace the depth in map
    } else {
        LOG_ERROR_PRINTF("MapElementDepth fuse type error");
        return MAP_UPDATE_FUSE_FAILD;
    }
}

// ____________________________________________________________________________
std::string MapElementDepth::ToString() const { return MapElement::ToString(); }

// ____________________________________________________________________________
bool MapElementDepth::IsSimilar(const MapElement &el,
                                tMapData tolerance_distance /*= 0.1*/,
                                tMapData tolerance_area /*= -1*/,
                                bool similar_fuse /*= true*/) {
    bool is_similar_type_ignored =
        MapElement::IsSimilar(el, tolerance_distance, tolerance_area, false);
    if (similar_fuse) {
        // the depth information was in the map before:
        // allow fuse with differnt map types: depth, pedestrian, sigins
        // the child classes handle the fuse with depth
        // and the depth in replaced with a new element in the map after the
        // fuse
        // when MapElementDepth::UpdateFuse is called

        return is_similar_type_ignored && IsFuseTypeWithDepth(el);

    } else {
        return is_similar_type_ignored;
    }
}

// ____________________________________________________________________________
bool MapElementDepth::IsFuseTypeWithDepth(const MapElement &el) {
    const MapElementType ftype = el.GetType();
    if (DEPTH == ftype) {
        return true;  // depth
    } else if (MapElementPedestrian::IsPedestrian(ftype)) {
        return true;
    }
    // fuse with signs disabled: the depth is used for emergency break
    // else if (MapElementRoadSign::IsRoadSign(ftype)) {
    //   //only fuse depth with sensor data
    //   const MapElementRoadSign *el_f =
    //       dynamic_cast<const MapElementRoadSign *>(&el);
    //
    //   return el_f && (SIGN_SENSOR == el_f->GetSignSubType());
    //
    // }
    else if (ftype == OBSTACLE) {
        return true;
    } else if (ftype == CAR) {
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
//                              MapElementObstacle
// ----------------------------------------------------------------------------

MapElementObstacle::MapElementObstacle(const std::vector<tMapPoint> &local_poly,
                                       tTimeMapStamp timestamp)
    : MapElement(OBSTACLE, local_poly, timestamp), AADCJuryStatus() {}

// ____________________________________________________________________________
MapFuseReturn MapElementObstacle::UpdateFuse(
    const tMapCarPosition &global_car_pos, MapElement &element_fuse,
    tTimeMapStamp timestamp /*= 0*/, MapFuseType fuse_type) {
    const MapElementType ftype = element_fuse.GetType();
    if (GetType() == ftype) {
        // fuse jury info
        if (!JuryUpdateFuse(
                dynamic_cast<const MapElementObstacle *>(&element_fuse))) {
            LOG_ERROR_PRINTF("MapElementObstacle fuse error: no jury parent");
        }
        return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
                                      fuse_type);
    } else if (ftype == DEPTH) {
        // fuse with depth: keep the fuse options
        // TODO live time !
        // return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
                                      // fuse_type);
        return MAP_UPDATE_FUSE_KEEP;
    } else {
        LOG_ERROR_PRINTF("MapElementObstacle fuse error: %s with %s",
                         MapHelper::TypeToString(GetType()),
                         MapHelper::TypeToString(ftype));
        return MAP_UPDATE_FUSE_FAILD;
    }
}

// ____________________________________________________________________________
std::string MapElementObstacle::ToString() const {
    return MapElement::ToString() + (IsJuryInformed() ? "true" : "false");
}

// ____________________________________________________________________________
bool MapElementObstacle::IsSimilar(const MapElement &element_fuse,
                                   tMapData tolerance_distance /*= 0.1*/,
                                   tMapData tolerance_area /*= -1*/,
                                   bool similar_fuse /*= true*/) {
    bool ret = false;
    bool is_similar_type_ignored = MapElement::IsSimilar(
        element_fuse, tolerance_distance, tolerance_area, false);
    if (similar_fuse) {
        // return true if the elements can be fused and similar
        const MapElementType ftype = element_fuse.GetType();
        if (GetType() == ftype || ftype == DEPTH) {
            // fuse with same type or with depth
            ret = is_similar_type_ignored;
        } else {
            return false;
        }
    } else {
        ret = is_similar_type_ignored;
    }
    return ret;
}

// ----------------------------------------------------------------------------
//                              MapElementCar
// ----------------------------------------------------------------------------


MapElementCar::MapElementCar(MapElementOrientation orientation,
                             const std::vector<tMapPoint> &local_poly,
                             tTimeMapStamp timestamp)
    : MapElement(CAR, local_poly, timestamp),
      orientation_(orientation),
      last_fuse_time_object_detection_(timestamp),
      buf_speed_rel_x_object_detection_(5),// capacity
      last_fuse_time_depth_(timestamp),
      buf_speed_rel_x_depth_(30)  // capacity
{}

// ____________________________________________________________________________
MapFuseReturn MapElementCar::UpdateFuse(const tMapCarPosition &global_car_pos,
                                        MapElement &element_fuse,
                                        tTimeMapStamp timestamp,
                                        MapFuseType fuse_type) {

    const MapElementType ftype = element_fuse.GetType();
    if (GetType() == ftype) {
        MapElementCar *updated_car =
            dynamic_cast<MapElementCar *>(&element_fuse);

        if (updated_car){
          orientation_ = updated_car->GetCarOrientation();
        }

        // abs proj. diff to car pos if in front or back?
        // FIR filter needed?

        //update relative speed estimate in x direction
        UpdateSpeedBuffer(element_fuse.ProjectedDistanceToCar(),
                          &last_projected_distance_object_detection_,
                          timestamp,
                          &last_fuse_time_object_detection_,
                          &buf_speed_rel_x_object_detection_);
        static int  debug_print_cnt = 0;
        // if (++debug_print_cnt %2 ==0)
        //   LOG_WARNING_PRINTF(ToString().c_str());
          //  user_tag_ui_ = "o:: " +
          //  boost::lexical_cast<std::string>(GetRelativeLocalXSpeedObjectDetection())
          //  + "d: " +
          //  boost::lexical_cast<std::string>(GetRelativeLocalXSpeedObjectDepth());
        return MapElement::UpdateFuse(global_car_pos, element_fuse, timestamp,
                                      fuse_type);
    } else if (ftype == DEPTH) {
        // fuse with depth: keep the fuse options
        // TODO live time !
        // TODO speed update?
        //update relative speed estimate in x direction
        UpdateSpeedBuffer(element_fuse.ProjectedDistanceToCar(),
                          &last_projected_distance_depth_,
                          timestamp,
                          &last_fuse_time_depth_,
                          &buf_speed_rel_x_depth_);
         user_tag_ui_ = "o:: " +
         boost::lexical_cast<std::string>(GetRelativeLocalXSpeedObjectDetection())
         + "d: " +
         boost::lexical_cast<std::string>(GetRelativeLocalXSpeedObjectDepth());
        return MAP_UPDATE_FUSE_KEEP;
    } else {
        LOG_ERROR_PRINTF("MapElementObstacle fuse error: %s with %s",
                         MapHelper::TypeToString(GetType()),
                         MapHelper::TypeToString(ftype));
        return MAP_UPDATE_FUSE_FAILD;
    }
}

// ____________________________________________________________________________
std::string MapElementCar::ToString() const {
    return MapElement::ToString() + "rel. x speed obj. detec m/s: " +
     boost::lexical_cast<std::string>(GetRelativeLocalXSpeedObjectDetection())
     + "rel. x speed depth m/s: " +
     boost::lexical_cast<std::string>(GetRelativeLocalXSpeedObjectDepth());
}

// ____________________________________________________________________________
bool MapElementCar::IsSimilar(const MapElement &element_fuse,
                              tMapData tolerance_distance /*= 0.1*/,
                              tMapData tolerance_area /*= -1*/,
                              bool similar_fuse /*= true*/) {
    bool ret = false;
    bool is_similar_type_ignored = MapElement::IsSimilar(
        element_fuse, tolerance_distance, tolerance_area, false);
    if (similar_fuse) {
        // return true if the elements can be fused and similar
        const MapElementType ftype = element_fuse.GetType();
        if (GetType() == ftype) {
            // fuse with same type or with depth
            // no fuse with depth if car in added: larger fuse distance
            ret = is_similar_type_ignored;
        } else {
            ret = false;
        }
    } else {
        ret = is_similar_type_ignored;
    }
    return ret;
}



// ____________________________________________________________________________
MapElementOrientation MapElementCar::GetCarOrientation() const {
    return orientation_;
}

// ____________________________________________________________________________
void MapElementCar::UpdateSpeedBuffer(tMapData current_dist,
                      tMapData *last_dist,
                      tTimeMapStamp current_time_micor_s,
                      tTimeMapStamp *last_time_micor_s,
                      utils::CircularBuffer<tMapData> *buf_speed )
{

  const tMapData speed = (current_dist - *last_dist)
                  / ((current_time_micor_s - *last_time_micor_s) / 1e6);

  if (*last_time_micor_s != GetCreateTime()){
    //not first update if
    if (std::fabs(speed) <= MAP_EL_CAR_MAX_SPEED){
          buf_speed->PushBack(speed);
    }
    // else{
    //   LOG_WARNING_PRINTF("car speed to large to add to buf: %f", speed);
    // }
  }
  *last_dist = current_dist; //update also in first step
  *last_time_micor_s = current_time_micor_s; //update also in first step
}


// ____________________________________________________________________________
tMapData MapElementCar::GetSpeed(
  const utils::CircularBuffer<tMapData> &buf_speed) const {
  if (buf_speed.Capacity() == buf_speed.Size()){
    return buf_speed.Mean();
  }else{
    return 0;
  }
}

// ____________________________________________________________________________
tMapData MapElementCar::GetRelativeLocalXSpeedObjectDetection() const {
  return GetSpeed(buf_speed_rel_x_object_detection_);
}

// ____________________________________________________________________________
tMapData MapElementCar::GetRelativeLocalXSpeedObjectDepth() const {
  return GetSpeed(buf_speed_rel_x_depth_);
}


}
} /*NAME SPACE frAIburg,map*/
