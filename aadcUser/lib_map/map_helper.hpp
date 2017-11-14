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
//  map_helper.hpp
//  - get the string for the map type
//
//  Created by Markus on 05.07.17.

#ifndef AADCUSER_FRAIBURG_MAP_HELPER_LIB_H_
#define AADCUSER_FRAIBURG_MAP_HELPER_LIB_H_

#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>
#include "adtf_log_macros.h"
#include "global_map.hpp"
#include "map_element.hpp"
#include "map_types.h"

#ifdef MAP_BUILD_CV
#include <opencv/cv.h>
#endif

#include <boost/version.hpp>
#if BOOST_VERSION >= 106000
#include <boost/qvm/mat_operations.hpp>
#endif
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace frAIburg {
namespace map {

class __attribute__((visibility("default"))) MapHelper {
 public:
    /*! normalize angle between zero and two PI*/
    static tMapData NormalizeAngle(tMapData angle);

    /*! maps the type id to a string, only use in the debug modus*/
    static const char *TypeToString(MapElementType type);

    static tMapCarPosition GetCarPosLocalIsGlobal();

    // translates, then rotates
    static void PolyTranslateRotate(const tMapPolygon &poly_from,
                                    tMapPolygon &poly_to, const tMapData x,
                                    const tMapData y,
                                    const tMapData angle_rad_clockwise);

    static void PolyRotateAroundPoint(const tMapPolygon &poly_from,
                                      tMapPolygon &poly_to,
                                      const tMapData x,  // rotation center
                                      const tMapData y,  // rotation center
                                      const tMapData angle_clockwise);

    static void PolyTranslate(const tMapPolygon &poly_from,
                              tMapPolygon &poly_to, const tMapData x,
                              const tMapData y);

    // Rotate with respect to the origin (0,0) in rad (clockwise)
    static void PolyRotate(const tMapPolygon &poly_from, tMapPolygon &poly_to,
                           const tMapData angle_rad_clockwise);

    static void PolyRotateCenterThenTranslate(
        const tMapPolygon &poly_from, tMapPolygon &poly_to,
        const tMapData rot_center_x, const tMapData rot_center_y,
        const tMapData angle_rad_clockwise, const tMapData trans_x,
        const tMapData trans_y);

    /*!  ret_points as return with points with rotation around the center */
    static void CreatBoxPoints(const tMapData center_local_x,
                               const tMapData center_local_y,
                               const tMapData lenght_half_x,
                               const tMapData lenght_half_y,
                               std::vector<tMapPoint> &ret_points,
                               const tMapData *box_local_orientation = NULL);

    /*! creats a box map element with a fixed point order based on the current
    *     car position to simplify the fuse operation with
    *        MAP_FUSE_FIXED_POINT_ORDER_MEAN
    *   EXAMPLE:
        tSptrMapElement el=MapHelper::creat_fixed_order_box(DEBUG_POLYGON,
                               1.5,0,0.5,1,,get_timestamp(),&box_local_angle);
        map->add_fuse_element(el,1,1,get_timestamp(),
                                MAP_FUSE_FIXED_POINT_ORDER_MEAN);
    *   \param center_local_x center point of the box in m
    *   \param center_local_x center point of the box in m
    *   \param lenght_half_x half lenght in x direction in m
    *   \param lenght_half_y half lenght in y direction in m
    *   \param type map element type
    *   \param box_local_angle_clockwise which can be diffrent for the el
    oritation
    *   \param el_local_angle angle in rad of the element in rad
    *   \return smart pointer to the created element
    */
    static tSptrMapElement CreatBox(
        MapElementType type, const tMapData center_local_x,
        const tMapData center_local_y, const tMapData lenght_half_x,
        const tMapData lenght_half_y, const tTimeMapStamp timestamp = 0,
        const tMapData *box_local_orientation = NULL,
        const tMapData *el_local_angle = NULL);

#ifdef MAP_BUILD_CV
    /*! Creates a tMapPoint from a cv::Point. */
    static tMapPoint CvPointToMapPoint(const cv::Point2f &p);
#endif  // MAP_BUILD_CV

    /*! smallest difference between two angles in radians in rang 0-2pi*/
    static tMapData MinDiffAngles(tMapData angle_a, tMapData angle_b);

    /*!
     *  param threshold_max: should be in range from PI-half to PI/2. Where PI
     *  means the elements are facing each other and PI-half means they are
     *  orthogonal.
     *  returns true if MapElement::IsOrientationUsed and the above holds.
     *  By default (threshold_max = M_PI_2) an orthogonal element is considered
     *  to be oriented towards the car
     */
    static bool IsElementOrientatedToCar(const GlobalMap *map_,
                                         const MapElement &el,
                                         const tMapData threshold_max = M_PI_2);

    /*increments the index and return a color */
    static const char *GetChangingDebugColor(unsigned int *index);

    /*! Returns a boost timestamp. */
    static tTimeMapStamp GetTimestamp();

    /*! addfuse a map debug point to the map*/
    static tSptrMapElement AddFuseDebugPointToMap(
        GlobalMap *map_, tMapData x, tMapData y, tTimeMapStamp timestamp,
        tMapData fuse_distance,
        tMapData angle,                   // rad0-2pi, -1 to disable
        tMapData life_time_microseconds,  // -1 to disable, rest if fused*/
        const char *ui_color = NULL, const char *ui_name = NULL,
        bool global = false);

    /*! reutns the distance between tow map points*/
    static tMapData DistancePoint(const tMapPoint &p1, const tMapPoint &p2);

    /*! zero if the element is in the front, negative value to the right*/
    static tMapData OrientationToCar(const tMapCarPosition &global_car_pos,
        const tMapPoint &global_point);

    static tMapData ProjectedDistanceToCar(const tMapCarPosition &global_car_pos,
            const tMapPoint &global_point);
};
}
} /*NAME SPACE frAIburg,map*/

#endif /* AADCUSER_FRAIBURG_MAP_HELPER_LIB_H_ */
