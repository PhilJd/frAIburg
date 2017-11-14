
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
//
//  map_element.cpp
//  map
//
//  Created by Markus on 05.07.17.
//

#include "map_element.hpp"
#include "map_helper.hpp"
namespace bg = boost::geometry;

namespace frAIburg {
namespace map {

typedef bg::model::multi_polygon<tMapPolygon> tMapMultiPolygon;

// ____________________________________________________________________________
MapElement::MapElement(MapElementType type,
                       std::vector<tMapPoint> const &local_poly,
                       tTimeMapStamp timestamp)
    : type_(type),
      element_angle_used_(false),
      local_element_orientation_angle_(0),
      timestamp_create_microseconds_(timestamp) {
    Init(local_poly);
}

// ____________________________________________________________________________
MapElement::MapElement(MapElementType type,
                       std::vector<tMapPoint> const &local_poly,
                       tTimeMapStamp timestamp,
                       tMapData local_orientation_angle)
    : type_(type),
      element_angle_used_(true),
      local_element_orientation_angle_(
          MapHelper::NormalizeAngle(local_orientation_angle)),
      timestamp_create_microseconds_(timestamp)

{
    Init(local_poly);
}

// ____________________________________________________________________________
void MapElement::Init(std::vector<tMapPoint> const &local_poly) {
    // call default init (constructor just in c++11)
    // mutex not needed only called in constructor
    car_distance_ = 0.;
    car_distance_previous_ = 99999.;  // huge distance to set in range callback
    fused_cnt_ = 0;
    fuse_center_mean_decay_weight_factor_ = 0.;
    timestamp_of_life_microseconds_ = 0;
    update_with_repostion_jump_ = true;
    time_of_life_is_used_ = false;
    orientation_angle_to_car_ = -1;
    id_ = MAP_DEFAULT_ID;

    if (local_poly.empty()) {
        LOG_ERROR_PRINTF("MapElement %s local polygon is empty",
                         MapHelper::TypeToString(type_));
        return;
    }

    if (local_poly.size() > MAP_ELEMENT_RESOURCES_MAX_NUMER_OF_POINTS) {
        LOG_WARNING_PRINTF("MapElement %s added with too many points: %lu",
                           MapHelper::TypeToString(type_), local_poly.size());
        // no return for testing //TODO(markus)
    }
    // add the vetor the the local polygon
    bg::assign_points(local_element_polygon_, local_poly);

#ifdef MAP_ENABLE_DEBUG_MODE  // TODO(markus) check if disabled in final proj
    if (!bg::is_valid(local_element_polygon_) && local_poly.size() > 2) {
        LOG_WARNING_PRINTF(
            "MapElement %s poly points not vaild, "
            "needed to correct: clockwise order, NOT Closed",
            MapHelper::TypeToString(type_));
        //  bg::correct(local_element_polygon_);
    }
#endif  // MAP_ENABLE_DEBUG_MODE
    // init area
    polygon_area_ = bg::area(local_element_polygon_);
}

// ____________________________________________________________________________
void MapElement::LocalToGlobal(tMapCarPosition const &global_car_pos) {
    timestamp_update_microseconds_ = global_car_pos.update_time_microseconds;

    if (local_element_polygon_.outer().empty()) {
        LOG_WARNING_PRINTF("MapElement %s id:%d local_to_global but empty",
                           MapHelper::TypeToString(type_), id_);
        return;
    }

    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        global_element_polygon_.clear();
        Transformation(local_element_polygon_, global_element_polygon_,
                       global_car_pos);
        UpdateGlobalPolyCenter();
        if (element_angle_used_) {
            // rotate local angel to global
            global_element_orientation_angle_ = MapHelper::NormalizeAngle(
                local_element_orientation_angle_ + global_car_pos.heading);
        }
        // update info to car
        UpdateDistanceToCar(global_car_pos);
        UpdateOrientationToCar(global_car_pos);
        // area is set in init
    }
}

// ____________________________________________________________________________
void MapElement::TransfromGlobal(tMapCarPosition const &global_car_pos) {
    if (global_element_polygon_.outer().empty()) {
        LOG_WARNING_PRINTF("MapElement %s id:%d transfrom_global but empty",
                           MapHelper::TypeToString(type_), id_);
        return;
    }
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    timestamp_update_microseconds_ = global_car_pos.update_time_microseconds;
    tMapPolygon tmp = global_element_polygon_;
    Transformation(tmp, global_element_polygon_, global_car_pos);
    UpdateGlobalPolyCenter();
    if (element_angle_used_) {
        // rotate local angel to global
        global_element_orientation_angle_ = MapHelper::NormalizeAngle(
            global_element_orientation_angle_ + global_car_pos.heading);
    }
}

// ____________________________________________________________________________
void MapElement::RotateAroundCenterGlobal(const tMapData angle_clockwise,
                                          tTimeMapStamp t) {
    if (global_element_polygon_.outer().empty()) {
        LOG_WARNING_PRINTF("MapElement %s id:%d transfrom_global but empty",
                           MapHelper::TypeToString(type_), id_);
        return;
    }

    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    timestamp_update_microseconds_ = t;
    tMapPolygon tmp = global_element_polygon_;
    MapHelper::PolyRotateAroundPoint(
        tmp, global_element_polygon_,
        bg::get<0>(global_element_polygon_center_),  // x
        bg::get<1>(global_element_polygon_center_),  // y
        angle_clockwise);

    if (element_angle_used_) {
        // rotate local angel to global clockwise
        DoAngleRotation(angle_clockwise);
    }
}

// ____________________________________________________________________________
void MapElement::RotateAroundCenterThenTranslateGlobal(const tMapData angle_rad,
                                                       const tMapData trans_x,
                                                       const tMapData trans_y,
                                                       tTimeMapStamp t) {
    if (global_element_polygon_.outer().empty()) {
        LOG_WARNING_PRINTF("MapElement %s id:%d transfrom_global but empty",
                           MapHelper::TypeToString(type_), id_);
        return;
    }
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    timestamp_update_microseconds_ = t;
    tMapPolygon tmp = global_element_polygon_;
    tMapData center_x = bg::get<0>(global_element_polygon_center_);
    tMapData center_y = bg::get<1>(global_element_polygon_center_);
    MapHelper::PolyRotateCenterThenTranslate(
        tmp, global_element_polygon_, center_x, center_y,  // rotation center
        angle_rad,                                         // clockwise
        trans_x, trans_y);
    // update center pos
    bg::set<0>(global_element_polygon_center_, center_x + trans_x);  // x
    bg::set<1>(global_element_polygon_center_, center_y + trans_y);  // y
    if (element_angle_used_) {
        // rotate local angel to global clockwise
        DoAngleRotation(angle_rad);
    }
}

// ____________________________________________________________________________
void MapElement::UpdateArea() {
    // no mutex private
    polygon_area_ = bg::area(global_element_polygon_);
}

// ____________________________________________________________________________
void MapElement::UpdateDistanceToCar(tMapCarPosition const &global_car_pos) {
    // no mutex private, lock in GlobalDistance
    car_distance_previous_ = car_distance_;
    // GlobalDistance not used becaue lock
    car_distance_ = bg::distance(global_element_polygon_center_,
                                 tMapPoint(global_car_pos.x, global_car_pos.y));
}

// ____________________________________________________________________________
void MapElement::Update(tMapCarPosition const &global_car_pos) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    // update distance and angle to the car

    // transformation in gloablal frame
    // updata distance
    UpdateDistanceToCar(global_car_pos);
    UpdateOrientationToCar(global_car_pos);
}

// ____________________________________________________________________________
MapFuseReturn MapElement::UpdateFuse(
    tMapCarPosition const &global_car_pos, MapElement &p,
    tTimeMapStamp timestamp, MapFuseType fuse_type /*=MAP_FUSE_APPEND*/) {
    // mutex is used for fuse in subfuncitons
    timestamp_update_microseconds_ = timestamp;
    // check if fused with empty poly
    if (p.GetGlobalPoly().outer().empty()) {
        LOG_ERROR_PRINTF("MapElement %s id:%d fuse with empty element",
                         MapHelper::TypeToString(type_), id_);
        return MAP_UPDATE_FUSE_FAILD;
    }
    // check for max poly point size
    if (p.GetGlobalPoly().outer().size() +
            global_element_polygon_.outer().size() >
        MAP_ELEMENT_RESOURCES_MAX_NUMER_OF_POINTS) {
        // point resource limit
        LOG_ERROR_PRINTF(
            "MapElement %s id:%d with %lu s point cant be fused,\
                            points limit of %d is reached",
            MapHelper::TypeToString(type_), id_,
            global_element_polygon_.outer().size(),
            MAP_ELEMENT_RESOURCES_MAX_NUMER_OF_POINTS);
        return MAP_UPDATE_FUSE_FAILD;
    }

#ifdef MAP_ENABLE_DEBUG_MODE
    // if (timestamp_create_microseconds_ > p.timestamp_create_microseconds_ ){
    //   LOG_WARNING_PRINTF("MapElement %s id:%d fused "
    //                      "with an older element",
    //                       MapHelper::TypeToString(type_),id_);
    // }
    if (p.GetID() == id_) {
        LOG_ERROR_PRINTF("MapElement fuse with same id:%d ", id_);
        return MAP_UPDATE_FUSE_FAILD;
    }
    // warning if angle is not used
    if (element_angle_used_ != p.IsOrientationUsed()) {
        LOG_WARNING_PRINTF(
            "MapElement %s id:%d fused error, \
                              angle not used for both elements ",
            MapHelper::TypeToString(type_), id_);
    }

    tMapData pre_area = polygon_area_;
    // change color for the ui in the debug mode
    if (!p.user_tag_ui_.empty()) user_tag_ui_ = p.user_tag_ui_;
    if (!p.user_color_ui_.empty()) user_color_ui_ = p.user_color_ui_;
// warning msg if area is getting smaller after fuse
#endif

    // replace the user tage for the new element
    if (!p.user_tag_.empty()) user_tag_ = p.user_tag_;

    DoTimeOfLifeFuse(p);

    // update the element with the new informmation
    // combine the global polygons
    switch (fuse_type) {
        case MAP_FUSE_REPLACE:
            DoFuseReplace(p);
            break;
        case MAP_FUSE_APPEND:
            DoFuseAppend(p);
            break;
        case MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN:
            DoFuseCenterMean(p);
            break;
        case MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN_DECAY_9:
            DoFuseCenterMean(p, 0.9);
            break;
        case MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN_DECAY_5:
            DoFuseCenterMean(p, 0.5);
            break;
        case MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN_DECAY_8:
            DoFuseCenterMean(p, 0.8);
            break;
        case MAP_CONVEX_HULL:
            DoFuseConvecHull(p);
            break;
        case MAP_CONVEX_HULL_SIMPLIFY_5:
            DoFuseConvecHull(p, 0.05);
            break;
        case MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_007:
            DoFuseExpDecay(p, 0.07);
            break;
        case MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_02:
            DoFuseExpDecay(p, 0.2);
            break;
        default:
            LOG_ERROR_PRINTF("MapElement Fuse Type error");
            return MAP_UPDATE_FUSE_FAILD;
    }

    // update angle and distance to car
    // with lock
    Update(global_car_pos);

#ifdef MAP_ENABLE_DEBUG_MODE
    // print fuse info msg
    if (fused_cnt_ % MAP_ELEMENT_FUSE_CNT_INFO_MSG == 0) {
        LOG_INFO_PRINTF("MapElement %s id:%d fused cnt: %d",
                        MapHelper::TypeToString(type_), id_, fused_cnt_);
    }
#endif

    return MAP_UPDATE_FUSE_KEEP;
}

// ____________________________________________________________________________
void MapElement::DoFuseReplace(MapElement const &p) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    ++fused_cnt_;
    // clear then append
    if (element_angle_used_)
        global_element_orientation_angle_ = p.GetGlobalOrientation();
    bg::clear(global_element_polygon_);
    bg::append(global_element_polygon_, p.GetGlobalPoly().outer());
    polygon_area_ = p.GetArea();
    global_element_polygon_center_ = p.GetGlobalPolyCenter();
}

// ____________________________________________________________________________
void MapElement::DoFuseAppend(MapElement const &p) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    ++fused_cnt_;
    bg::append(global_element_polygon_, p.GetGlobalPoly().outer());
    bg::correct(global_element_polygon_);
    UpdateArea();
    UpdateGlobalPolyCenter();
}

// ____________________________________________________________________________
void MapElement::DoFuseConvecHull(const MapElement &p,
                                  const tMapData simplify_distance /*= -1.*/) {
    if (p.element_angle_used_ || element_angle_used_) {
        LOG_WARNING_PRINTF(
            "MapElement %s id: %d DoFuseConvecHull not support"
            "angle fuse",
            MapHelper::TypeToString(type_), id_);
    }

    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    ++fused_cnt_;
    tMapMultiPolygon multi_poly;
    multi_poly.resize(2);
    // bg::assign_points(multi_poly[0].outer(),
    // global_element_polygon_.outer());
    // bg::assign_points(multi_poly[1].outer(),
    // p.global_element_polygon_.outer());
    multi_poly[0] = global_element_polygon_;
    multi_poly[1] = p.global_element_polygon_;
    if (simplify_distance != -1) {
        tMapPolygon hull;
        bg::convex_hull(multi_poly, hull);
        // simplify adds unwanted close point
        bg::simplify(hull, global_element_polygon_, simplify_distance);
    } else {
        bg::convex_hull(multi_poly, global_element_polygon_);
    }
// TODO close point is added!
#ifdef MAP_ENABLE_DEBUG_MODE
    if (!bg::is_valid(global_element_polygon_) &&
        global_element_polygon_.outer().size() > 2) {
        LOG_WARNING_PRINTF(
            "DoFuseConvecHull %s poly points not vaild, "
            "after fuse",
            MapHelper::TypeToString(type_));
        ;
    }
#endif  // MAP_ENABLE_DEBUG_MODE
    UpdateArea();
    UpdateGlobalPolyCenter();
}

// ____________________________________________________________________________
void MapElement::DoFuseCenterMean(MapElement const &p,
                                  const tMapData decay_factor) {
    // calculate a new center mean and mean angle
    // translation and rotation to new center and angle
    if (GetArea() != p.GetArea() &&
        p.GetGlobalPoly().outer().size() !=
            global_element_polygon_.outer().size()) {
        // error if area and points not the same
        LOG_ERROR_PRINTF(
            "MapElement %s id:%d fixed area"
            "mean fuse error, areas are not the same or not the same"
            "number of points",
            MapHelper::TypeToString(type_), id_);
    }
    // continue also after error
    // fuse center point with a mean
    tMapData rotation = 0;  // in rad clockwise
    tMapData trans_x = 0;
    tMapData trans_y = 0;
    tMapData new_x, old_x_mean, new_x_mean, new_y, old_y_mean, new_y_mean;
    tMapData factor_mean;
    // set the factor if the decay is used to compute the new eneter
    if (!fused_cnt_) {
        // before  ++fused_cnt_;
        fuse_sin_global_mean_ = sin(GetGlobalOrientation());
        fuse_cos_global_mean_ = cos(GetGlobalOrientation());
    }
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        ++fused_cnt_;
        ++fuse_center_mean_decay_weight_factor_;
        if (decay_factor != -1. && decay_factor > 0 && decay_factor < 1.) {
            factor_mean = decay_factor * fuse_center_mean_decay_weight_factor_;
            fuse_center_mean_decay_weight_factor_ = factor_mean;
        } else {
            factor_mean = fused_cnt_;
        }

        tMapPoint new_center = p.GetGlobalPolyCenter();
        new_x = bg::get<0>(new_center);
        old_x_mean = bg::get<0>(global_element_polygon_center_);
        new_x_mean = (factor_mean * old_x_mean + new_x) / (factor_mean + 1);
        // y
        new_y = bg::get<1>(new_center);
        old_y_mean = bg::get<1>(global_element_polygon_center_);
        new_y_mean = (factor_mean * old_y_mean + new_y) / (factor_mean + 1);
        // set new center
        trans_x = new_x_mean - old_x_mean;
        trans_y = new_y_mean - old_y_mean;
        // transforamtion the current global poly to the now center

        if (element_angle_used_) {
            tMapData angle_factor = decay_factor;
            // TODO add decay for normal mean
            if (decay_factor == -1) {
                angle_factor = 0.5;
            }
            fuse_cos_global_mean_ =
                (1 - angle_factor) * cos(p.GetGlobalOrientation()) +
                angle_factor * fuse_cos_global_mean_;
            fuse_sin_global_mean_ =
                (1 - angle_factor) * sin(p.GetGlobalOrientation()) +
                angle_factor * fuse_sin_global_mean_;
            tMapData new_angle_mean =
                atan2(fuse_sin_global_mean_, fuse_cos_global_mean_);

            rotation = global_element_orientation_angle_ - new_angle_mean;
        }
    }

    // rotation trans has lock agian
    // update center pos, angle and poly points via roation and translation
    RotateAroundCenterThenTranslateGlobal(rotation, trans_x, trans_y);
}

// ____________________________________________________________________________
void MapElement::DoFuseExpDecay(MapElement const &p,
                                const tMapData decay_factor) {
    // calculate a new center mean and mean angle
    // translation and rotation to new center and angle
    if (GetArea() != p.GetArea() &&
        p.GetGlobalPoly().outer().size() !=
            global_element_polygon_.outer().size()) {
        // error if area and points not the same
        LOG_ERROR_PRINTF(
            "MapElement %s id:%d fixed area"
            "mean fuse error, areas are not the same or not the same"
            "number of points",
            MapHelper::TypeToString(type_), id_);
    }

    if (!fused_cnt_) {
        // before  ++fused_cnt_;
        fuse_sin_global_mean_ = sin(GetGlobalOrientation());
        fuse_cos_global_mean_ = cos(GetGlobalOrientation());
    }
    // continue also after error
    // fuse center point with a mean
    tMapData rotation = 0;  // in rad clockwise
    tMapData trans_x = 0;
    tMapData trans_y = 0;
    tMapData new_x, old_x_mean, new_x_mean, new_y, old_y_mean, new_y_mean;

    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        ++fused_cnt_;

        tMapPoint new_center = p.GetGlobalPolyCenter();
        new_x = bg::get<0>(new_center);
        old_x_mean = bg::get<0>(global_element_polygon_center_);
        new_x_mean = decay_factor * new_x + (1 - decay_factor) * old_x_mean;

        new_y = bg::get<1>(new_center);
        old_y_mean = bg::get<1>(global_element_polygon_center_);
        new_y_mean = decay_factor * new_y + (1 - decay_factor) * old_y_mean;
        // set new center
        trans_x = new_x_mean - old_x_mean;
        trans_y = new_y_mean - old_y_mean;
        // transform the current global poly to the new center
        if (element_angle_used_) {
            fuse_cos_global_mean_ =
                decay_factor * cos(p.GetGlobalOrientation()) +
                (1 - decay_factor) * fuse_cos_global_mean_;
            fuse_sin_global_mean_ =
                decay_factor * sin(p.GetGlobalOrientation()) +
                (1 - decay_factor) * fuse_sin_global_mean_;
            tMapData new_angle_mean =
                atan2(fuse_sin_global_mean_, fuse_cos_global_mean_);

            rotation = global_element_orientation_angle_ - new_angle_mean;
        }
    }
    // rotation trans has lock again
    // update center pos, angle and poly points via rotation and translation
    RotateAroundCenterThenTranslateGlobal(rotation, trans_x, trans_y);
}

// ____________________________________________________________________________
void MapElement::UpdateOrientationToCar(const tMapCarPosition &global_car_pos) {

    orientation_angle_to_car_ = MapHelper::OrientationToCar(global_car_pos,
      global_element_polygon_center_);

}

// ____________________________________________________________________________
std::string MapElement::ToString() const {
    // no mutex, no risk if data is not consistent, can used for debug even if
    // mutex is locked

    // static const char * format="%3.2f";
    std::stringstream s;
    s << "MapElement | id: " << id_
      << " | type: " << MapHelper::TypeToString(type_)
      // << " | angle to car front rad: " << GetOrientationAngleToCar()
      // << " | angle to car front degree: "
      // << boost::format(format) % (GetOrientationAngleToCar() * (180. / M_PI))
      // << " | car distance: " << boost::format(format) % car_distance_
      // << " | proj. dist: " << ProjectedDistanceToCar()
      // print the polygons
      // << "| num points: " << global_element_polygon_.outer().size()
      // << "| local pos: "<< bg::dsv(local_element_polygon_)
      // << "| global poly: "<< bg::dsv(GetGlobalPoly())
      << " | center pos: " << bg::dsv(global_element_polygon_center_);

    // if(GetArea() != 0)
    //     s << " | area : " << boost::format(format) % GetArea();
    if (!user_tag_.empty()) s << " | tag: " << user_tag_;
    if (!user_tag_ui_.empty()) s << " | ui tag: " << user_tag_ui_;
    // if (element_angle_used_)
    //     s << " | global angel rad : " << boost::format(format) %
    //     GetGlobalOrientation();
    if (fused_cnt_) s << " | fused cnt: " << fused_cnt_;
    return s.str();
}

// ____________________________________________________________________________
void MapElement::UpdateGlobalPolyCenter() {
    // no mutex, private, call form func with lock
    // update the center point
    // check if closed for 3 points (in bg::simplify a colose point is added)
    // TODO not clear why close points are added, multpoly also with not closed
    if (global_element_polygon_.outer().size() == 2 ||
        (global_element_polygon_.outer().size() == 3 &&
         bg::equals(global_element_polygon_.outer().front(),
                    global_element_polygon_.outer().back()))) {
        // in case of two points take the mean as center
        // x mean
        global_element_polygon_center_.set<0>(
            (global_element_polygon_.outer()[0].get<0>() +
             global_element_polygon_.outer()[1].get<0>()) /
            2);
        // y mean
        global_element_polygon_center_.set<1>(
            (global_element_polygon_.outer()[0].get<1>() +
             global_element_polygon_.outer()[1].get<1>()) /
            2);
    } else {
        bg::centroid(global_element_polygon_, global_element_polygon_center_);
    }
}

// ____________________________________________________________________________
void MapElement::Transformation(const tMapPolygon &poly_from,
                                tMapPolygon &poly_to,
                                tMapCarPosition const &pos) {
    // no mutex, private, call form func with lock
    MapHelper::PolyTranslateRotate(poly_from, poly_to, pos.x, pos.y,
                                   -pos.heading);
}

// ____________________________________________________________________________
float MapElement::SimilarFactor(const MapElement &el) {
    // option with union_ or gloabal oriantation
    if (type_ != el.type_) return 0.;  // TODO check if needed

    float similar_area = 0;
    float similar_dist = 0;

    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        if (polygon_area_ != 0)
            similar_area = el.polygon_area_ / polygon_area_;
        else if (el.polygon_area_ == polygon_area_)
            similar_area = 1;

        if (car_distance_ != 0)
            similar_dist = el.car_distance_ / car_distance_;
        else if (el.car_distance_ == car_distance_)
            similar_dist = 1;
    }
    return similar_area * similar_dist;
}

// ____________________________________________________________________________
bool MapElement::IsType(const std::vector<MapElementType> &types) const {
    BOOST_FOREACH (const MapElementType t, types) {
        if (t == type_) return true;
    }
    return false;
}

// ____________________________________________________________________________
bool MapElement::IsSimilar(MapElement const &el, tMapData tolerance_distance,
                           tMapData tolerance_area,
                           bool similar_fuse /*true*/) {
    // not mutex here, GlobalDistance locks

    const float delta_area = std::fabs(el.polygon_area_ - polygon_area_);
    const float delta_dist = GlobalDistance(el);  // distance to center
    if (global_element_polygon_.outer().empty()
      || el.global_element_polygon_.outer().empty())
    {
      LOG_ERROR_PRINTF("IsSimilar with empty global poly");
      return false;
    }

    if ((delta_area == 0 && polygon_area_ == 0 && delta_dist == 0 &&
         car_distance_ == 0) ||
        (tolerance_area == -1 && tolerance_distance == -1)) {
        // all zero or both tolerance are -1
        LOG_ERROR_PRINTF("MapElement similar check, values wrong");
        return false;
    } else if ((type_ == el.type_ ||
                !similar_fuse) &&  // if fuse types must be same
               (delta_area <= tolerance_area ||
                tolerance_area == -1) &&  // check area
               (delta_dist <= tolerance_distance ||
                tolerance_distance == -1))  // check distance
    {
        return true;  // sim element found
    } else
        return false;
}

// ____________________________________________________________________________
bool MapElement::GlobalPolyWithin(MapElement const &p) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    return bg::within(global_element_polygon_, p.global_element_polygon_);
}

// ____________________________________________________________________________
bool MapElement::GlobalPolyTouches(MapElement const &p) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    return bg::touches(global_element_polygon_, p.global_element_polygon_);
}

// ____________________________________________________________________________
bool MapElement::GlobalPolyOverlaps(const MapElement &p) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    return bg::overlaps(global_element_polygon_, p.global_element_polygon_);
}

// ____________________________________________________________________________
bool MapElement::GlobalPolyDisjoint(const MapElement &p) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    return bg::disjoint(global_element_polygon_, p.global_element_polygon_);
}

// ____________________________________________________________________________
bool MapElement::GlobalPolyDisjoint(const tMapPoint &p) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    return bg::disjoint(global_element_polygon_, p);
}

// ____________________________________________________________________________
tMapData MapElement::GlobalDistance(MapElement const &el, bool to_center) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    if (to_center) {
        return bg::distance(global_element_polygon_center_,
                            el.global_element_polygon_center_);
    } else {
        return bg::distance(global_element_polygon_,
                            el.global_element_polygon_);
        // return bg::comparable_distance(global_element_polygon_,
        //                     el.global_element_polygon_);
    }
}

// ____________________________________________________________________________
tMapData MapElement::GlobalDistance(tMapPoint const &p, bool to_center) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    if (to_center) {
        return bg::distance(global_element_polygon_center_, p);
    } else
        return bg::distance(global_element_polygon_, p);
}

// ____________________________________________________________________________
void MapElement::EnableTimeOfLife(
    tTimeMapStamp life_time_microseconds,
    tTimeMapStamp *start_time_microseconds /*=NULL*/) {
    if (start_time_microseconds) {
        timestamp_start_life_microseconds_ = *start_time_microseconds;
    } else {
        if (timestamp_create_microseconds_ <= 0) {
            LOG_ERROR_PRINTF(
                "MapElement %s id:%d EnableTimeOfLife failed, "
                "creation time: %d INVALID  ",
                MapHelper::TypeToString(type_), id_,
                timestamp_create_microseconds_);
        }

        timestamp_start_life_microseconds_ = timestamp_create_microseconds_;
    }
    time_of_life_is_used_ = true;
    timestamp_of_life_microseconds_ = life_time_microseconds;
}

// ____________________________________________________________________________
void MapElement::DoTimeOfLifeFuse(const MapElement &p) {
    // get the fuse option from the new element (in case the )
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    // if use life time if it is enabled for one element
    time_of_life_is_used_ = p.time_of_life_is_used_;
    if (p.time_of_life_is_used_) {
        // use the life time of the new element
        timestamp_of_life_microseconds_ = p.timestamp_of_life_microseconds_;
        timestamp_start_life_microseconds_ =
            p.timestamp_start_life_microseconds_;
    }
}

// ____________________________________________________________________________
bool MapElement::IsOverTimeOfLive(tTimeMapStamp time_microseconds) {
    // lock needed to prevent error if timestamp_start_life_microseconds_
    // changed if fused
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    if (!time_of_life_is_used_) {
        return false;
    } else {
        return (time_microseconds - timestamp_start_life_microseconds_) >
               timestamp_of_life_microseconds_;
    }
}

// ____________________________________________________________________________
void MapElement::DoAngleRotation(const tMapData angle_rad) {
    global_element_orientation_angle_ = MapHelper::NormalizeAngle(
        global_element_orientation_angle_ - angle_rad);
}

tMapData MapElement::ProjectedDistanceToCar() const {
    return cos(orientation_angle_to_car_) * car_distance_;
}

/// GETTER and SETTER
// ____________________________________________________________________________
tMapData MapElement::GetGlobalOrientation() const {
    if (!element_angle_used_) {
        LOG_WARNING_PRINTF( "MapElement: GetGlobalOrientation id %d, type %s "
                            "orientation was not set in constructor",
                            id_, MapHelper::TypeToString(type_));
    }
    return global_element_orientation_angle_;
}

void MapElement::SetUpdateWithRepostionJumps(bool set) {
    update_with_repostion_jump_ = set;
}

bool MapElement::IsUpdateWithRepostionJumpEnabled() const {
    return update_with_repostion_jump_;
}

bool MapElement::IsOrientationUsed() const { return element_angle_used_; }

tMapData MapElement::GetArea() const { return polygon_area_; }

tMapPolygon MapElement::GetGlobalPoly() const {
    // copy because can be cahnged in fuse or in transformation
    return global_element_polygon_;
}
tMapPoint MapElement::GetGlobalPolyCenter() const {
    return global_element_polygon_center_;
}

tMapData MapElement::GetOrientationAngleToCar() const {
    return orientation_angle_to_car_;  // zero if the element is in the front:
}

/*ret the distance to the center of the poly form the lust update car pos*/
tMapData MapElement::GetDistanceToCar() const { return car_distance_; }

tMapData MapElement::GetPreviousDistanceCar() const {
    return car_distance_previous_;
}

void MapElement::SetID(tMapID id) { id_ = id; }

tMapID MapElement::GetID() const { return id_; }

MapElementType MapElement::GetType() const { return type_; }

tTimeMapStamp MapElement::GetUpdateTime() const {
    return timestamp_update_microseconds_;
}

tTimeMapStamp MapElement::GetCreateTime() const {
    return timestamp_create_microseconds_;
}

unsigned int MapElement::GetFuseCount() const { return fused_cnt_; }

// ____________________________________________________________________________
}
} /*NAME SPACE frAIburg,map*/
