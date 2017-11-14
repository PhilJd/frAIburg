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
//  map_helper.cpp
//  map tpye to string for the gui
//
//  Created by Markus on 05.07.17.
//

#include "map_helper.hpp"

namespace bg = boost::geometry;
namespace trans = boost::geometry::strategy::transform;

namespace frAIburg {
namespace map {

// ____________________________________________________________________________
const char *MapHelper::TypeToString(MapElementType type) {
    // map from enum type to string for the map gui and print
    static const boost::unordered_map<MapElementType, const char *>
        map_types_string = boost::assign::map_list_of
        (UNDEFINED, "UNDEFINED")
        (UNKNOWN, "UNKNOWN")
        // street
        (STREET_MARKER_LANE, "Lane")
        (STREET_MARKER_STOP_LINE, "StopLine")
        (STREET_MARKER_CROSSING_T, "CrossingT")
        (STREET_MARKER_CROSSING_X, "CrossingX")
         (STREET_MARKER_ZEBRA, "Zebra")
        (STREET_PARKING_SPOT, "ParkingSpot")
        (LM_STREET_PARKING_SPOT, "LMParkingSpot")
        // signs
        (STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT,"GiveAwayRight")
        (STREET_TRAFFIC_SIGN_STOP, "Stop")
        (STREET_TRAFFIC_SIGN_PARKING,"Parking")(
        STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC, "Priority")
        (STREET_TRAFFIC_SIGN_JUST_STRAIGHT, "RightOnly")
        (STREET_TRAFFIC_SIGN_GIVE_WAY, "GiveAway")
        (STREET_TRAFFIC_SIGN_CIRCLE, "Circle")
        (STREET_TRAFFIC_SIGN_NO_TAKE_OVER, "NoTakeOver")
        (STREET_TRAFFIC_SIGN_NO_ENTRY, "NoEntry")
        (STREET_TRAFFIC_ONE_WAY_STREET, "OneWay")
        (STREET_TRAFFIC_ROAD_WORKS, "RoadWork")
        (STREET_TRAFFIC_SIGN_SPEED_50,"Speed50")
        (STREET_TRAFFIC_SIGN_SPEED_100, "Speed100")
        (STREET_TRAFFIC_SIGN_CROSSWALK, "Crosswalk")
        (STREET_TRAFFIC_SIGN_POSITION_MARKER, "PositionMarker")
        // PEDESTRIAN
        (PEDESTRIAN_ADULT, "Adult")
        (CAR, "CAR")
        (PEDESTRIAN_CHILD, "Child")
        (OBSTACLE, "OBSTACLE")
        (DEPTH, "Depth")
        (ULTRRASONIC, "Ultrasonic")
        (LM_STREET_LANE, "RefLane")
        (DEBUG_POINT, "DebugPoint")
        (PLANNER_PATH, "Planner")
        (DEBUG_POLYGON, "DebugPoly");

    const char *ret_type_name = map_types_string.at(UNDEFINED);
    try {
        ret_type_name = map_types_string.at(type);
    } catch (std::out_of_range &e) {
        std::stringstream ss;
        ss << "map helper TypeToString unknown type " << type << " "
           << e.what();
        LOG_ERROR_PRINTF("%s", ss.str().c_str());
    }
    return ret_type_name;
}

// ____________________________________________________________________________
tMapCarPosition MapHelper::GetCarPosLocalIsGlobal() {
    tMapCarPosition zero_pos;
    zero_pos.x = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_X;
    zero_pos.y = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_Y;
    zero_pos.heading = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_HEADING;
    zero_pos.update_time_microseconds = 0;
    return zero_pos;
}

// ____________________________________________________________________________
void MapHelper::PolyTranslateRotate(const tMapPolygon &poly_from,
                                    tMapPolygon &poly_to, const tMapData x,
                                    const tMapData y, const tMapData angle) {
    // translates, then rotates
    // Rotate and Translate with one boost call
    const trans::rotate_transformer<boost::geometry::radian, tMapCoords, 2, 2>
        rotate(angle);
    const trans::translate_transformer<tMapCoords, 2, 2> translate(x, y);

#if BOOST_VERSION >= 106000

    const trans::matrix_transformer<tMapCoords, 2, 2> rotateTranslate(
        translate.matrix() * rotate.matrix());
#else
    const trans::ublas_transformer<tMapCoords, 2, 2> rotateTranslate(
        prod(translate.matrix(), rotate.matrix()));
#endif

    bg::transform(poly_from, poly_to, rotateTranslate);
}

// ____________________________________________________________________________
void MapHelper::PolyRotateAroundPoint(const tMapPolygon &poly_from,
                                      tMapPolygon &poly_to, const tMapData x,
                                      const tMapData y, const tMapData angle) {
    // rotation around point: translate*rotation*-translate
    // Rotate and Translate with one boost call
    const trans::rotate_transformer<boost::geometry::radian, tMapCoords, 2, 2>
        rotate(angle);
    const trans::translate_transformer<tMapCoords, 2, 2> translate(x, y);
    const trans::translate_transformer<tMapCoords, 2, 2> translate_back(-x, -y);

#if BOOST_VERSION >= 106000
    // Boost 1.64
    const trans::matrix_transformer<tMapCoords, 2, 2> rotateTranslate(
        translate.matrix() * rotate.matrix() * translate_back.matrix());

    bg::transform(poly_from, poly_to, rotateTranslate);
#else
    // Boost v.1.57.0
    // TODO(markus) find solution with one trans, nested?
    // const trans::ublas_transformer<tMapCoords, 2, 2>
    //   translate_rot_trans_back(prod(prod(translate.matrix(),rotate.matrix()),
    //                           translate_back.matrix()));
    const trans::ublas_transformer<tMapCoords, 2, 2> rotateTranslate(
        prod(translate.matrix(), rotate.matrix()));
    tMapPolygon tmp, tmp2;
    bg::transform(poly_from, tmp, translate_back);
    bg::transform(tmp, tmp2, rotate);
    bg::transform(tmp2, poly_to, translate);
#endif
}

// ____________________________________________________________________________
void MapHelper::PolyTranslate(const tMapPolygon &poly_from,
                              tMapPolygon &poly_to, const tMapData x,
                              const tMapData y) {
    const trans::translate_transformer<tMapCoords, 2, 2> translate(x, y);
    bg::transform(poly_from, poly_to, translate);
}

// ____________________________________________________________________________
void MapHelper::PolyRotate(const tMapPolygon &poly_from, tMapPolygon &poly_to,
                           const tMapData angle) {
    const trans::rotate_transformer<boost::geometry::radian, tMapCoords, 2, 2>
        rotate(angle);
    bg::transform(poly_from, poly_to, rotate);
}

// ____________________________________________________________________________
void MapHelper::PolyRotateCenterThenTranslate(
    const tMapPolygon &poly_from, tMapPolygon &poly_to, const tMapData center_x,
    const tMapData center_y, const tMapData angle_rad_clockwise,
    const tMapData trans_x, const tMapData trans_y) {
    tMapPolygon tmp;
    PolyRotateAroundPoint(poly_from, tmp, center_x, center_y,
                          angle_rad_clockwise);
    PolyTranslate(tmp, poly_to, trans_x, trans_y);
}

// ____________________________________________________________________________
void MapHelper::CreatBoxPoints(
    const tMapData center_local_x, const tMapData center_local_y,
    const tMapData lenght_half_x, const tMapData lenght_half_y,
    std::vector<tMapPoint> &ret_points,
    const tMapData *box_local_orientation /*= NULL*/) {
    const std::vector<tMapPoint> p_box = boost::assign::list_of(tMapPoint(
        center_local_x - lenght_half_x, center_local_y - lenght_half_y))(
        tMapPoint(center_local_x - lenght_half_x,
                  center_local_y + lenght_half_y))(
        tMapPoint(center_local_x + lenght_half_x,
                  center_local_y + lenght_half_y))(
        tMapPoint(center_local_x + lenght_half_x,
                  center_local_y - lenght_half_y));
    // TODO(markus) no copy
    if (box_local_orientation) {
        tMapPolygon trans_poly, tmp;
        bg::assign_points(trans_poly, p_box);
        // rotate poly to ensure same point order in globale frame
        MapHelper::PolyRotateAroundPoint(trans_poly, tmp, center_local_x,
                                         center_local_y,
                                         *box_local_orientation);
        ret_points = tmp.outer();
    } else {
        ret_points = p_box;
    }
}

// ____________________________________________________________________________
tSptrMapElement MapHelper::CreatBox(
    MapElementType type, const tMapData center_local_x,
    const tMapData center_local_y, const tMapData lenght_half_x,
    const tMapData lenght_half_y, const tTimeMapStamp timestamp /* =0*/,
    const tMapData *box_local_orientation /*=NULL*/,
    const tMapData *el_local_angle /*=NULL*/) {
    // create a tSptrMapElement map box elment polygon
    std::vector<tMapPoint> el_points;
    CreatBoxPoints(center_local_x, center_local_y, lenght_half_x, lenght_half_y,
                   el_points, box_local_orientation);
    if (el_local_angle)
        return tSptrMapElement(
            new MapElement(type, el_points, timestamp, *el_local_angle));
    else
        return tSptrMapElement(new MapElement(type, el_points, timestamp));
}

#ifdef MAP_BUILD_ADTF  // TODO(markus)
// ____________________________________________________________________________
tMapPoint MapHelper::CvPointToMapPoint(const cv::Point2f &p) {
    return tMapPoint(p.x, p.y);
}
#endif  // LOG_MACRO_BUILD_ADTF

// ____________________________________________________________________________
tMapData MapHelper::MinDiffAngles(tMapData angle_a, tMapData angle_b) {
    const tMapData angle_delta = std::fabs(angle_b - angle_a);
    if (angle_delta < M_PI)
        return (angle_delta);
    else
        return ((M_PI * 2) - angle_delta);  // here negative values can appear
                                            // for angle values > 2PI
}

// ____________________________________________________________________________
bool MapHelper::IsElementOrientatedToCar(
    const GlobalMap *map_, const MapElement &el,
    const tMapData threshold_max /*= M_PI_2*/) {
    if (!map_) {
        LOG_ERROR_PRINTF("IsRoadSignOrientatedToCar null pointer");
    }
    if (!el.IsOrientationUsed()) {
        LOG_ERROR_PRINTF(
            "IsRoadSignOrientatedToCar failed, "
            "Orientation not used, el: %s",
            el.ToString().c_str());
    }

    tMapCarPosition pos;
    if (!map_->GetGlobalCarPosition(&pos)) {
        return false;  // car pos is not known
    }

    const tMapData angle_diff =
        MapHelper::MinDiffAngles(pos.heading, el.GetGlobalOrientation());
    // printf("Angle diff %f \n", angle_diff);
    // car is direcly facing to the element if heading =-GetGlobalOrientation
    return (angle_diff >= threshold_max) ? true : false;
}

// ____________________________________________________________________________
tMapData MapHelper::NormalizeAngle(tMapData angle) {
    tMapData normalized_angle = std::fmod(angle, 2 * M_PI);
    if (normalized_angle < 0) {
        normalized_angle += 2 * M_PI;
    }
    return normalized_angle;
}

// ____________________________________________________________________________
const char *MapHelper::GetChangingDebugColor(unsigned int *index) {
    static const std::vector<const char *> color_change_debug =
        boost::assign::list_of("darkseagreen")("darkmagenta")("palevioletred")(
            "hotpink")("lightgreen")("purple")("yellowgreen")("dodgerblue")(
            "crimson")("magenta")("darkslategrey")("skyblue");
    if (++(*index) >= color_change_debug.size()) *index = 0;
    return color_change_debug[*index];
}

// ____________________________________________________________________________
tTimeMapStamp MapHelper::GetTimestamp() {
    typedef boost::posix_time::microsec_clock clock;
    return clock::local_time().time_of_day().total_microseconds();
}

// ____________________________________________________________________________
tSptrMapElement MapHelper::AddFuseDebugPointToMap(
    GlobalMap *map_, tMapData x, tMapData y, tTimeMapStamp timestamp,
    tMapData fuse_distance,
    tMapData angle,                  // rad -1 to disable
    tMapData life_time_microseconds, /* -1 to disable*/
    const char *ui_color /*=NULL*/, const char *ui_name /*=NULL*/,
    bool global /*=false*/) {
    const std::vector<tMapPoint> point =
        boost::assign::list_of(tMapPoint(x, y));

    tSptrMapElement el_point;
    if (angle != -1) {
        el_point = tSptrMapElement(
            new MapElement(DEBUG_POINT, point, timestamp, angle));
    } else {
        el_point =
            tSptrMapElement(new MapElement(DEBUG_POINT, point, timestamp));
    }

    if (life_time_microseconds != -1)
        el_point->EnableTimeOfLife(life_time_microseconds);  // reset if fused
    if (ui_color) el_point->user_color_ui_ = ui_color;
    if (ui_name) el_point->user_tag_ui_ = ui_name;

    if (global) {
        // trans  in global frame
        el_point->LocalToGlobal(MapHelper::GetCarPosLocalIsGlobal());
    }
    map_->AddFuseElement(el_point, fuse_distance,
                         0,  // must be a point
                         timestamp, MAP_FUSE_REPLACE);
    return el_point;
}

// ____________________________________________________________________________
tMapData MapHelper::DistancePoint(const tMapPoint &p1, const tMapPoint &p2) {

    return bg::distance(p1, p2);
}

// ____________________________________________________________________________
tMapData MapHelper::OrientationToCar(const tMapCarPosition &global_car_pos,
    const tMapPoint &global_point) {

    // gloabal directed  ange of the centerpoints
    float dx = bg::get<0>(global_point) - global_car_pos.x;
    float dy = bg::get<1>(global_point) - global_car_pos.y;

    // dx, dy in old frame, rotate shifted current frame cw by
    // pose_global_t0_.heading
    float x = (dx)*cos(-global_car_pos.heading)
              - (dy)*sin(-global_car_pos.heading);
    float y = (dx)*sin(-global_car_pos.heading)
              + (dy)*cos(-global_car_pos.heading);
    return atan2(y, x);

    // orientation_angle_to_car_ = angel_rad_points - global_car_pos.heading;
    // if (orientation_angle_to_car_ > M_PI)
    //     orientation_angle_to_car_ -= M_PI* 2;
    // else if (orientation_angle_to_car_ <= - M_PI)
    //     orientation_angle_to_car_ += M_PI * 2;
    // return angel_rad_points -  global_car_pos.heading;
}

// ____________________________________________________________________________
tMapData MapHelper::ProjectedDistanceToCar(const tMapCarPosition &global_car_pos,
    const tMapPoint &global_point) {

    return cos(OrientationToCar(global_car_pos, global_point)) *
      DistancePoint(global_point, tMapPoint(global_car_pos.x, global_car_pos.y));

}



}
} /*NAME SPACE frAIburg,map*/
