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
//  map_element.hpp
//  map elemnts used in the GlobalMap
//      - id,types, timestamps
//      - a polygons (boost geometry) in a local and global frame
//        to describe the potions
//      - updates distances, angle measurments based on the car position
//      - simple similar check to new elements
//      -
//  Created by Markus on 05.07.17.

#ifndef AADCUSER_FRAIBURG_MAP_ELEMENT_LIB_H_
#define AADCUSER_FRAIBURG_MAP_ELEMENT_LIB_H_

// set the maximum of points a map element can have when its fused
#define MAP_ELEMENT_RESOURCES_MAX_NUMER_OF_POINTS 500U
#define MAP_ELEMENT_FUSE_CNT_INFO_MSG 1000

#include <math.h>
#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "adtf_log_macros.h"
#include "map_types.h"

#include <boost/version.hpp>
#if BOOST_VERSION >= 106000
#include <boost/qvm/mat_operations.hpp>
#endif
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/thread/mutex.hpp>

namespace frAIburg {
namespace map {

class __attribute__((visibility("default"))) MapElement {
 public:
    /*! created am map element defined by a polygon and type
     *   \param type enum type of the element
     *   \param local_poly define the outline of an map element, in [m]
     *           local car frame: to the front x pos, to the left y pos,
     *           points in clockwise order
     *   \param timestamp creation time
     *   \note -transform to a gloabal frame by calling local_to_global.
     *         the map calls this method if not called before
     *         -if timestamp creation time is 0, this element will be ignored
     *          by the map in remove_elements_over_distance
     *          and update_reposition_car()
     *    \note the id will bet set by the map if the element is added
     */
    MapElement(MapElementType type, const std::vector<tMapPoint> &local_poly,
               tTimeMapStamp timestamp = 0);

    /*! see above MapElement(...)
     *   \param local_orientation_angle set and enable a angle angle, in [rad],
     *            angle is define zero at front and positive vals to the left
     */
    MapElement(MapElementType type, const std::vector<tMapPoint> &local_poly,
               tTimeMapStamp timestamp, tMapData local_orientation_angle);

    virtual ~MapElement(){};  // polymorphic class to enable dynamic_cast child
                              // types

 public:
    /* user_tag_ which can be set by the user to add addtion information      */
    std::string user_tag_;

    /* user_tag_ which will be shown instelead of the type name in the map ui */
    std::string user_tag_ui_;  // use only in debug mode
    /* user color for the ui, poly fill color and outer line
     * valid names: https://www.w3.org/TR/SVG/types.html#ColorKeywords */
    std::string user_color_ui_;  // use only in debug mode

    /*! check if the elemnt is one from the given types
    *   \param types types to check
    *   \return true if the elemnt is one of the types in the vector
    */
    bool IsType(const std::vector<MapElementType> &types) const;

    /*! calculate distance to MapElement
    *   \param MapElement el to comapre with
    *   \param to_center true distance form the center, else closest distance
    *   \note update_mutex_ is used
    *   \note if the elements are Disjoint and to_center = false, the distatnce
    *         is zero
    *   \return distance
    */
    tMapData GlobalDistance(const MapElement &el, bool to_center = true);

    /*! calculate distance to Point
    *   \param MapElement el to comapre with
    *   \param to_center true distance form the center, else closest distance
    *   \note update_mutex_ is used
    *   \return distance
    */
    tMapData GlobalDistance(const tMapPoint &p, bool to_center = true);

    /* check if elements are disjoint in the global frame
     * \note check with if(!el->GlobalPolyDisjoint(el2)) for collisions
     */
    bool GlobalPolyDisjoint(const MapElement &p);
    /* check if a map point is disjoint in the global polyon frame
     */
    bool GlobalPolyDisjoint(const tMapPoint &p);

    /* check if other element is (complitly) within the global polygon*/
    bool GlobalPolyWithin(const MapElement &p);
    /* check if other element is touches the outer line of the global polygon*/
    bool GlobalPolyTouches(const MapElement &p);
    bool GlobalPolyOverlaps(const MapElement &p);

    /* \return true if local_orientation_angle was used in the constructor*/
    bool IsOrientationUsed() const;
    /* return the angle of the element in the global fram in [rad]*/
    tMapData GetGlobalOrientation() const;

    tMapData GetArea() const;
    /* \return an copy of the polygon in the global frame */
    tMapPolygon GetGlobalPoly() const;

    /* \return polygon center in the global frame if transfromed from local to
     *    global before
     *  EXAMPLE: tMapPoint center = el->GetGlobalPolyCenter();
     *           tMapData x = center.get<0>();
    //  *           tMapData y = center.get<1>();
     */
    tMapPoint GetGlobalPolyCenter() const;

    /*! orientation angle based on the poly center point from -pi to pi
     * returns zero if the element is in the front, negative value to the right
     */
    tMapData GetOrientationAngleToCar() const;

    /* ret the distance to the center of the poly form the last update call
       with car pos
      \note distance and angle updated by the map with the call of Update
    */
    tMapData GetDistanceToCar() const;
    tMapData GetPreviousDistanceCar() const;

    /*! return prejedted distance based on the OrientationAngleToCar of the last
     *  Update() call
     */
    tMapData ProjectedDistanceToCar() const;

    tTimeMapStamp GetUpdateTime() const;
    tTimeMapStamp GetCreateTime() const;

    MapElementType GetType() const;

    /*! return how often the element was fused */
    unsigned int GetFuseCount() const;

    /*! element string for the element with different information*/
    virtual std::string ToString() const;

    /*! set a live time for the map element, after this time the map deletes
    *    the element
    *   \param life_time_microseconds set a live time of the element
    *   \param start_time_microseconds optinal start time of the life time,
    *          use if AFTER the element is GlobalMap::AddFuseElement to
    * overwrite
    *          the life time if fused
    *   \note call BEFORE the element is GlobalMap::AddFuseElement map if
    *         life time reset wanted with fuse start_time_microseconds
    *   \note call AFTER the element is GlobalMap::AddFuseElement map if
    *         life time reset form creation time of the first element added to
    *          the map
    *   \note creation timestamp must no be zero if used or negative
    *   \note map delets then the element with RemoveAllElementsOverLifeTime
    *   \note if elements if fused the life time setting of the new elment will
    *         be taken
    *   \note once enableld the life time can not disabled
    */
    void EnableTimeOfLife(tTimeMapStamp life_time_microseconds,
                          tTimeMapStamp *start_time_microseconds = NULL);
    bool IsOverTimeOfLive(tTimeMapStamp time_microseconds);

    /*! get the ID of the alement
    *   \note call after element was added to the map
    *   \note the id will bet set by the map
    */
    tMapID GetID() const;

    /*! set if the elment is updated in a map repostion jump,
       call after the element is added to the map, in case the of a fusion,
       the default is ture
    */
    void SetUpdateWithRepostionJumps(bool set);

    // FUNCTION CALLED BY THE MAP:

    /*! set the ID of the alement
    *   \note the id will bet set by the map if the element is added
    */
    void SetID(tMapID id);

    /*! transfrom the local polygon and angle if set a glabale frame
    *   \param tMapCarPosition position of the car to transformation
    *   \note only use this there is a long time needed for data porcessing:
    *         save the car postion with the sensor information and
    *         perform the tranfromation after the computation
    *   \note this method is called by the map, unless it was called before
    *   \note element update time stamp is set to car pos time_pos_update
    *   \note update_mutex_ is used
    */
    void LocalToGlobal(const tMapCarPosition &global_car_pos);

    /*! transfrom the globale polygon, trans and roation based on car pos
     *  \note distance and angle to car is not updated
     *  \note element update time stamp is set to car pos time_pos_update
     *  \note update_mutex_ is used
     */
    void TransfromGlobal(const tMapCarPosition &global_car_pos);

    /*! rotate the globale polygon, angle in rad
     *  \note distance and angle to car is not updated
     *  \note update_mutex_ is used
     */
    void RotateAroundCenterGlobal(const tMapData angle_clockwise,
                                  tTimeMapStamp t = 0);

    /*! rotate the globale polygon, angle in rad
     *  \note distance and angle to car is not updated
     *  \note update_mutex_ is used
     */
    void RotateAroundCenterThenTranslateGlobal(
        const tMapData angle_rad_clockwise, const tMapData trans_x,
        const tMapData trans_y, tTimeMapStamp t = 0);

    /*! update global position distance and angle to the car
     *   \param tMapCarPosition position update of the car
     *   \note timestamp is set to the global_car_pos time_pos_update
     *   \note this method is called by the map
     *   \note update_mutex_ is used
     */
    void Update(const tMapCarPosition &global_car_pos);

    /*! combines the GLOBAL polygon with a diffrent element,
     *  user_tag_ and user_tag_ui_ will be used from the element to fuse with
     *   \param type enum type of the element
     *   \param element_fuse to fuse with
     *   \param local_poly define the, ob
     *           local car frame: to the front x pos, to the left y pos,
     *           points in clockwise order
     *   \param timestamp creation time
     *   \param fuse_type MAP_FUSE_APPEND, polygon points will be appended.
     *                    MAP_FUSE_REPLACE, replace the polygon
     *                    MAP_FUSE_FIXED_POINT_MEAN, number of points stays
     *                     same by computing the mean,
     *                     point number and order must be the same
     *   \return MapFuseReturn to inform the map if which element to keep in
     *           the map
     *   \note  this method is called by the map
     *   \note user_tag_, user_tag_ui_, user_color_ui_ will be change to
     *        the one of element_fuse if not empty
     *   \note update_mutex_ is used
     */
    virtual MapFuseReturn UpdateFuse(const tMapCarPosition &global_car_pos,
                                     MapElement &element_fuse,
                                     tTimeMapStamp timestamp = 0,
                                     MapFuseType fuse_type = MAP_FUSE_APPEND);

    /*! check if element are similar based of area, type and center distance
    *   \note  the map calls IsSimilar to check if elements should be fused
    *          child classes can change this func to fuse with differnt types
    *   \param MapElement element to compare with
    *   \param tolerance_distance max distance between the two elements,
    *            -1 to ignore
    *   \param tolerance_area,-1 to ignore
    *            note area might change if element s were fuesed before
    *   \param similar_fuse true to check if the elements should
    *          be fused: the type must be same,
    *          if fuse the type of the elment is ignored
    *   \return true if the element are similar based on the args
    */
    virtual bool IsSimilar(const MapElement &el,
                           tMapData tolerance_distance = 0.1,
                           tMapData tolerance_area = -1,
                           bool similar_fuse = true);//TODO as const

    /*! simialr factor based on distance, type and area
    *   \param MapElement element to compare with
    *   \return simialr factor based on distance, type and area
    *   \note update_mutex_ is used
    */
    float SimilarFactor(const MapElement &el);

    /*! return ture if the element should be updated in a repostion jump*/
    bool IsUpdateWithRepostionJumpEnabled() const;

 private:
    /* mutx car pos update and vector updated */
    boost::mutex update_mutex_;
    /* type of element */
    const MapElementType type_;
    /* id of the element wich can be set by the map*/
    tMapID id_;
    /* keep track if the angle for the map element is set in a
     construtctor*/
    const bool element_angle_used_;

    /*counter with the indicates how often the element was fused*/
    unsigned int fused_cnt_;
    tMapData fuse_center_mean_decay_weight_factor_;
    bool update_with_repostion_jump_;
    tMapData fuse_sin_global_mean_;
    tMapData fuse_cos_global_mean_;

    /* local positions and oriantation of the element*/
    tMapPolygon local_element_polygon_;
    tMapData local_element_orientation_angle_;

    /* global position and oriantation of the element*/
    tMapPolygon global_element_polygon_;
    tMapPoint global_element_polygon_center_;
    tMapData global_element_orientation_angle_;  // in rad
    // zero if the element is in the front, to the right negative, updated
    tMapData orientation_angle_to_car_;  // in rad

    /* distance to the last update car position of the center of the poly */
    tMapData car_distance_;
    tMapData car_distance_previous_;
    // tMapData hight_;//TODO(markus) needed?

    /* save the last calculated area*/
    tMapData polygon_area_;

    /// TIMESTAMPS
    /* creation timestamp set in the construtctor */
    const tTimeMapStamp timestamp_create_microseconds_;
    /* creation timestamp for the last update for the element */
    tTimeMapStamp timestamp_update_microseconds_;
    tTimeMapStamp timestamp_of_life_microseconds_;
    tTimeMapStamp timestamp_start_life_microseconds_;

    bool time_of_life_is_used_;
    bool time_of_life_reset_with_fuse_;

    /*! init func, for multiple contributors, c11 is not used */
    void Init(const std::vector<tMapPoint> &local_poly);

    /*! calculate the areo of the polygon*/
    void UpdateArea();

    /*! set the decrete orientation to the last update position
     */
    void UpdateOrientationToCar(const tMapCarPosition &global_car_pos);

    /*calculate the inear distance to the car pos and saved center of the global
     * polygon */
    void UpdateDistanceToCar(const tMapCarPosition &global_car_pos);
    /*calculate the center of the gobal polygon*/
    void UpdateGlobalPolyCenter();

    /*! change the global angle of the map el in clockwise in rad*/
    void DoAngleRotation(const tMapData angle_diff_rad_clockwise);

    /*! transforamtion of the local to the gloabal car coord based on the
     *  current car pos */
    void Transformation(const tMapPolygon &poly_from, tMapPolygon &poly_to,
                        const tMapCarPosition &global_car_pos);

    /// FUSE FUNCTIONS all are using update_mutex_
    void DoFuseReplace(const MapElement &p);
    void DoFuseAppend(const MapElement &p);

    /*fuse with fix number of points and same area
      decay_factor to -1 to disable, if not and between 1 and zero
      the new center poly is computed:
       new_center=(old_mean * decay_factor + new)
                    / (old_mean * decay_factor + 1 )
    */
    void DoFuseCenterMean(const MapElement &p,
                          const tMapData decay_factor = -1.);

    void DoFuseExpDecay(MapElement const &p, const tMapData decay_factor);
    /*! creats a convex hull of two poly, -1 to disable simplify.
    simplify_distance (in units of input coordinates)
    of a vertex to other segments to be removed
    */
    void DoFuseConvecHull(const MapElement &p,
                          const tMapData simplify_distance = -1.);
    /*enable time of life if set for one element and
      use the time of life and creation time form p (the new element)*/
    void DoTimeOfLifeFuse(const MapElement &p);
};
}
} /*NAME SPACE frAIburg,map*/

#endif /* AADCUSER_FRAIBURG_MAP_ELEMENT_LIB_H_ */
