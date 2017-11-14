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
//  map.hpp
//  GlobalMap
//      - mapping with known poses
//      - Singleton and observer pattern (for adtf integration)
//      - vector with MapElement with local and global polygons as a map
//      - tranformation for local to global frame is based on the current
//          car position in the map
//      - adds elements can be fused
//      - save map to svg or txt file, to plot in python
//      - mutex to avoid a car position change while adding elements
//
//  Coordinate system:  the local frame of the car is
//                      defined at the center front
//
//  y_global[m]
//  ^
//  |               ^ y_local[m]
//  |               |
//  |               |
//  |           (car)>–––>x_local[m]    // car looks to the right
//  |
//  ––––––––––––>x_global[m]
//
//  requirements:
//      -boost >=v.1.58.0: filesystem, thread, geometry
//
//  Created by Markus on 05.07.17.
//

#ifndef AADCUSER_FRAIBURG_MAP_LIB_H_
#define AADCUSER_FRAIBURG_MAP_LIB_H_

#include <stdio.h>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/timer/timer.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include "adtf_log_macros.h"
#include "map_element.hpp"
#include "map_event_listener.hpp"
#include "map_types.h"

namespace frAIburg {
namespace map {

class GlobalMap;  // forward declaration
// \note  call getInstance in cFtiler init StageNormal
__attribute__((visibility("default"))) GlobalMap *getInstance();

class __attribute__((visibility("default"))) GlobalMap {
 public:
    /*! add an new element, if the local frame was transformed
    * to the global frame it will be tranformed
    *   EXAMPLE:frAIburg::map::getInstance()->add_element(
    *            tSptrMapElement(new MapElement(STREET_TRAFFIC_SIGN_PARKING,
    *                                 polygon_local,get_timestamp())));
    *   \param el boost smart pointer to the map element
    *   \param timestamp  update time
    *   \note 1.the local polygon of the map is getting transform to the globale
    *           frame based on the car position in the map
    *         2. el with creation timestamp=0 will be irgored in
    *            remove_elements_over_distance and update_reposition_car(jumps)
    *   \note uses update_mutex_
    */
    MapEventCode AddElement(tSptrMapElement &el, tTimeMapStamp timestamp = 0);

    /*! - fuses a similar element based on type, distance and area
    *     by appending all points to one polygon
    *   - add a new element to the map if no similar element was found
    *   EXAMPLE:frAIburg::map::getInstance()->AddFuseElement(
    *            tSptrMapElement(new MapElement(STREET_TRAFFIC_SIGN_PARKING,
    *                                 polygon_local,get_timestamp())));
    *   \param el boost smart pointer to the map element
    *   \param tolerance_distance max distance between two elements to be fused
    *   \param tolerance_area max area difference to other element to be fused
    *   \param timestamp update time
    *   \param fuse_type MAP_FUSE_APPEND, polygon points will be appended.
    *                    MAP_FUSE_REPLACE, replace the polygon
    *                    MAP_FUSE_FIXED_POINT_MEAN, number of points stays
    *                     same by computing the mean,
    *                     point number and order must be the same
    *   \note uses update_mutex_
    *   \note the el added to the map decides in the method with which element
    *         to fuse the element with, in child classes can overwrite this method
    *         to emable multifuse
    *   \note if the element is fuesed the smart point will point to the element
    *         fused elment that was already in the map
    *   \note see note for add_element
    */
    MapEventCode AddFuseElement(tSptrMapElement &el,
                                tMapData tolerance_distance = 0.05,
                                tMapData tolerance_area = -1,
                                tTimeMapStamp timestamp = 0,
                                MapFuseType fuse_type = MAP_FUSE_REPLACE);

    /*! find similar object in map
    *     by appending all points to one polygon
    *   - add a new element to the map if no similar element was found
    *   \param el pointer to the map element
    *   \param tolerance_distance max distance between two elements to be fused
    *   \param tolerance_area max area difference to other element to be fused
    *   \param timestamp update time
    *   \param fuse_type MAP_FUSE_APPEND, polygon points will be appended.
    *                    MAP_FUSE_REPLACE, replace the polygon
    *                    MAP_FUSE_FIXED_POINT_MEAN, number of points stays
    *                     same by computing the mean,
    *                     point number and order must be the same
    *   \note see note for add_element
    */
    void GetSimilarElements(
        MapElement *el, const tMapData tolerance_distance,
        const tMapData tolerance_area, std::vector<tSptrMapElement> &ret_el,
        bool must_be_same_type = true,
        const std::vector<MapElementType> *exclude_types = NULL,
        const int max_return_cnt = -1);

    /*! get the map element with the id
    *   EXAMPLE:
              tSptrMapElement el =frAIburg::map::getInstance()->get_element(id);
              if (el){//el is in map}
    *   \param id of the tSptrMapElement
    *   \return smart_pointer_with_id
    *   \note check with if(smart_pointer_with_id){...} if element was found
    *   \note uses update_mutex_
    */
    tSptrMapElement GetElement(tMapID id);

    /*update the current car pos in the map*/
    void UpdateCarPos(tMapCarPosition const &pos, tTimeMapStamp t = 0);
    /*update all the distance, and angles of the world elemtns to the car*/
    void UpdateElements(
        const std::vector<MapElementType> *exclude_types = NULL);
    /*updatecar pos and map elements*/
    void Update(tMapCarPosition const &pos, tTimeMapStamp t = 0,
                const std::vector<MapElementType> *exclude_types = NULL);

    /*reposition of the car in in map, "jump":
     transfrom the globale frame of map el to fit the new pos of the car for
     which were updated time_diff_el_to_update ago,
     elements with creat timestamp =0 will be ignored
     */
    void UpdateRepositionCar(
        tMapCarPosition const &pos, tTimeMapStamp max_time_diff_el_to_update,
        tTimeMapStamp timestamp,
        const std::vector<MapElementType> *exclude_types = NULL);

    /*  \note is call by the map filter*/
    void Reset();

    /*remove all map elements*/
    void ClearElements();

    /*remove one element in the world with the id, return true iff deleted
    *   \note uses update_mutex_
    */
    MapEventCode RemoveElement(tMapID id);

    /*remove elemtnts the world over the max distance
    *   if the are not exluded or create timestamp is zero
    *   \note uses update_mutex_
    */
    unsigned int RemoveAllElementsUnderProjecteDistanceToCar(
        tMapData distance,
        const std::vector<MapElementType> *exclude_types = NULL);

    /* remove all elements  where live time was enableld
     * \note  uses update_mutex_
     */
    unsigned int RemoveAllElementsOverLifeTime(tTimeMapStamp time_microseconds);

    /* map event listener wiht func that will be called when
     *   elements are change, added, fused
     * \note  call in cFtiler init StageGraphReady
     */
    void RegisterEventListener(MapEventListener *);

    /** \note  call in cFtiler Shutdown StageGraphReady*/
    void DeregisterEventListener(MapEventListener *);

    /*! check map elments if element the global polygons are disjoint
    *   \param element_checked boost smart pointer to the map element,
    *          transformed to globale frame if not set before
    *   \param ret_el all elelments that were not disjoint
    *   \param max_center_distance_to_car max center distance
    *                                       to the car to check, -1 to disable
    *   \param max_return_cnt limit the size of ret_el ,-1 to disable
    *   \param exclude_types set types only to exclude_types for colltion,if
    *                         NULL to check all
    *   \return true if not failed and elements found that are not disjoint
    *   \note ensure that the gobale poly of el is not changed during the call
    *   \note update_mutex_ is used
    *   \note collstion only for el with a diffrent id to el
    *   \note the el if the local polygon is not transform to the global frame
    *         it will be transform to the global frame based on the current
    *         car pos
    */
    bool CheckElementsCollision(
        tSptrMapElement &element_checked, std::vector<tSptrMapElement> &ret_el,
        tMapData max_center_distance_to_car = -1, int max_return_cnt = -1,
        const std::vector<MapElementType> *exclude_types = NULL);

    /*save all global polyons of all current map element
     and the current car pos to .txt file */
    void SaveGlobaltxt(const char *file_name = NULL,
                       std::vector<MapElementType> *exclude_types = NULL) const;

    /*save all global polyons of all current map element
     and the current car pos to .svg  */
    void SaveGlobalsvg(const char *fsvg_name = NULL,
                       std::vector<MapElementType> *exclude_types = NULL) const;

    /*print out the car position and all polygons:
     *   \param title title shwon above the print, if not NULL
     *   \param include_types if not NULL, only this types will be in the print
     */
    void Print(const char *title = NULL,
               std::vector<MapElementType> *include_types = NULL);

    /* get the global car position*/
    bool GetGlobalCarPosition(tMapCarPosition *p) const;
    bool IsCarPosKnown() const;
    void SetCarPosKnown(bool set);

    /*ret_dist will be changed to distance s to the last known car pos and x y
    in the global frame, return true if the car pos in known */
    bool DistanceToCar(tMapData gobal_x, tMapData gobal_y,
                       tMapData *ret_dist) const;

    bool DistanceToCar(const tMapPoint &point, tMapData *ret_dist) const;

    bool ElementsExist(const tMapID id);

    unsigned int GetElementCnt() const;

    /* get all element with the type in a range of the car,
     * if range is 0 all elements will be returned,
     * \note  uses update_mutex_
     * \note the distance to the car is cahnged with the update call */
    void GetAllElementsWithType(MapElementType type,
                                std::vector<tSptrMapElement> &ret_el,
                                float range = 0);

    /* get all element with the typeS in a range of the car,
     * if range is 0 all elements will be returned,
     * \note  uses update_mutex_
     * \note the distance to the car is cahnged with the update call */
    void GetAllElementsWithTypes(const std::vector<MapElementType> &types,
                                 std::vector<tSptrMapElement> &ret_el,
                                 float range = 0);

    std::vector<tSptrMapElement> GetAllElements();

 private:
    /* private for Singleton design pattern
     Singleton is used to integrate the map in adtf*/
    GlobalMap(GlobalMap const &copy);             // Do not implement!
    GlobalMap &operator=(GlobalMap const &copy);  // Prevents copy of singleton

    // transform el local to global with last car pos, if to global poly for el
    bool ElementLocalToGlobal(tSptrMapElement &el);

    void CallEventsListener(tSptrMapElement &el, MapEventCode e);
    void CallEventsListener(MapEventCode e);

    /*! fuses two map element and based on the return value of the fuse the
        map is updated, the el_new will be changed and is pointing to the
        element in the map
        returns the new element in the map
    */
    tSptrMapElement DoFuseElement(tSptrMapElement &el_map,
                                  tSptrMapElement &el_new, tTimeMapStamp t,
                                  MapFuseType fuse_type);

 private:
    friend GlobalMap *getInstance();
    GlobalMap();

    unsigned int id_cnt_;
    tMapCarPosition car_pos_;
    bool car_pos_known_;

    // mutx car pos update and vector updated
    boost::mutex update_mutex_;

    /*vector with the elements in the world*/
    std::vector<tSptrMapElement>
        elements_;  // optinal use boost::geometry::index::rtree
    /*vector with all event listener*/
    std::vector<MapEventListener *> evlistener_;
};
}
} /*NAME SPACE frAIburg,map*/

#endif /* AADCUSER_FRAIBURG_MAP_LIB_H_ */
