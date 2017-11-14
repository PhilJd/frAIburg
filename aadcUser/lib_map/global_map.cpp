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
//  map.cpp
//  GlobalMap
//
//  Created by Markus on 05.07.17.
//  Copyright © 2017 me. All rights reserved.
//

#include "global_map.hpp"
#include "map_helper.hpp"

#define MAP_DEFAULT_SAVE_NAME_TXT "global_poly_map.txt"
#define MAP_DEFAULT_SAVE_NAME_SVG "global_poly_map.svg"

namespace bg = boost::geometry;

namespace frAIburg {
namespace map {

// ____________________________________________________________________________
GlobalMap *getInstance() {
    static GlobalMap instance;  // Guaranteed to be destroyed.

    return &instance;
}

// ____________________________________________________________________________
GlobalMap::GlobalMap() {
    LOG_INFO_PRINTF("GlobalMap: singelton constructor called");
    Reset();  // reset the map used in a shared lib
}
// ____________________________________________________________________________
void GlobalMap::Reset() {
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        id_cnt_ = 0;
        car_pos_ = MapHelper::GetCarPosLocalIsGlobal();
        // clear all element and evlistener_
        // because in adtf the map is used as static singleton in a shared lib
        if (!evlistener_.empty()) {
            LOG_WARNING_PRINTF(
                "Map MapEventListener registered before map init call\
        (call in cFilter init StageGraphReady) OR DeregisterEventListener "
                "was not called ")
        }
        // clear all evlistener_ because there might be not deregistered ones
        // (used with a shared lib with singelton in the adtf proj)
        evlistener_.clear();
    }
    ClearElements();
}
// ____________________________________________________________________________
void GlobalMap::UpdateCarPos(tMapCarPosition const &pos, tTimeMapStamp t) {
    // mutex to ensure that no a element is fused at the same time
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        // update the car pos in the map
        car_pos_ = pos;
        car_pos_.update_time_microseconds = t;
        car_pos_.heading = MapHelper::NormalizeAngle(car_pos_.heading);
    }
    if (!car_pos_known_) car_pos_known_ = true;
}

// ____________________________________________________________________________
void GlobalMap::UpdateElements(
    const std::vector<MapElementType> *exclude_types /*=NULL*/) {
    // if (!car_pos_known_){
    //     LOG_WARNING_PRINTF("GlobalMap: car pos not know but elements
    //     updated");
    // }

    // mutex to ensure that no a element is fused or added at the same time
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        // update all the distance and angle for the car for the elements
        BOOST_FOREACH (const tSptrMapElement &s, elements_) {
            if (exclude_types == NULL || !s->IsType(*exclude_types))
                s->Update(car_pos_);
        }
    }
}

void GlobalMap::UpdateRepositionCar(
    tMapCarPosition const &pos, tTimeMapStamp max_time_diff_el_to_update,
    tTimeMapStamp timestamp,
    const std::vector<MapElementType> *exclude_types /*=NULL*/) {
    int time_diff;  // change if no fp time stamp
    tMapCarPosition trans_pos;
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);

        BOOST_FOREACH (const tSptrMapElement &s, elements_) {
            time_diff = timestamp - s->GetUpdateTime();
            if (s->IsUpdateWithRepostionJumpEnabled()) {  // don t tranform if
                                                          // creation time=0
                if (time_diff < 0) {
                    LOG_WARNING_PRINTF(
                        "MAP reposition: el %s id:%d\
                                  wrong time_stamp: %d",
                        MapHelper::TypeToString(s->GetType()), s->GetID(),
                        s->GetUpdateTime());
                } else if (time_diff < max_time_diff_el_to_update) {
                    if (exclude_types == NULL || !s->IsType(*exclude_types)) {
                        // TODO(markus) boost geo with one call
                        trans_pos.x = -car_pos_.x;
                        trans_pos.y = -car_pos_.y;
                        trans_pos.update_time_microseconds = timestamp;
                        trans_pos.heading = 0.;
                        s->TransfromGlobal(trans_pos);
                        trans_pos.x = 0;
                        trans_pos.y = 0;
                        trans_pos.heading = -(car_pos_.heading - pos.heading);
                        s->TransfromGlobal(trans_pos);
                        trans_pos.x = pos.x;
                        trans_pos.y = pos.y;
                        trans_pos.heading = 0;
                        s->TransfromGlobal(trans_pos);

                        // s->RotateAroundCenterThenTranslateGlobal(
                        //                             -(car_pos_.heading-pos.heading),
                        //                             (pos.x-car_pos_.x),
                        //                             pos.y-car_pos_.y);//toodo
                        //                             add offset to car
                    }
                }
            }
        }
    }
    // UpdateCarPos and UpdateElements out of scoped_lock, have the same mutex
    UpdateCarPos(pos);
    // call changed event wiht UpdateElements
    UpdateElements();  // update ALL angle and dist to the car
}

// ____________________________________________________________________________
void GlobalMap::Update(
    tMapCarPosition const &pos, tTimeMapStamp timestamp /*= 0*/,
    const std::vector<MapElementType> *exclude_types /*=NULL*/) {
    UpdateCarPos(pos, timestamp);
    UpdateElements(exclude_types);
}

// ____________________________________________________________________________
bool GlobalMap::ElementLocalToGlobal(tSptrMapElement &el) {
    // transform element local frame to global if not set
    if (el->GetGlobalPoly().outer().empty()) {
        // if (!car_pos_known_){
        //   LOG_WARNING_PRINTF("GlobalMap: car pos not know but element
        //   added");
        // }
        // transform poly and updte distance and angle to car
        el->LocalToGlobal(car_pos_);
        // update distances
        return true;
    } else {
        // update car to current car postion if global was set by USER
        el->Update(car_pos_);
    }

    return false;
}

// ____________________________________________________________________________
MapEventCode GlobalMap::AddElement(tSptrMapElement &el,
                                   tTimeMapStamp timestamp) {
    if (!el){
      LOG_ERROR_PRINTF("MAP AddElement NULL element");
      return MAP_ERROR;
    }

    // transfrom the local poly to a global based on the current pos
    ElementLocalToGlobal(el);
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);

        // add new element to list set new id
        el->SetID(id_cnt_);
        ++id_cnt_;
        elements_.push_back(el);
    }
    // call event listener with no mutex, so that lock possible
    CallEventsListener(el, MAP_ADDED_NEW_ELEMENT);
    return MAP_ADDED_NEW_ELEMENT;
}

// ____________________________________________________________________________
MapEventCode GlobalMap::AddFuseElement(
    tSptrMapElement &el, tMapData tolerance_distance, tMapData tolerance_area,
    tTimeMapStamp t, MapFuseType fuse_type /*=MAP_FUSE_APPEND*/) {
    MapEventCode ret = MAP_ERROR;
    if (!el){
      LOG_ERROR_PRINTF("MAP AddFuseElement NULL element");
      return MAP_ERROR;
    }
    // update_mutex_ is used in GetSimilarElements, DoFuseElement and AddElement

    ElementLocalToGlobal(el);  // transform if the glabal frame is not set

    if (el->GetID() != MAP_DEFAULT_ID) {
        LOG_WARNING_PRINTF(
            "GlobalMap element (type %s)\
          is fused with element which haas an assied id already",
            MapHelper::TypeToString(el->GetType()));
    }

    // find similar elements in the map
    std::vector<tSptrMapElement> similar_els;
    GetSimilarElements(el.get(), tolerance_distance, tolerance_area,
                       similar_els,
                       /*must_be_same_type=*/true, /*exclude_types*/ NULL,
                       /*max_return_cnt*/ 5);

    if (similar_els.size() == 1) {
        // one similar element was found
        tSptrMapElement fuesed =
            DoFuseElement(similar_els.front(), el, t, fuse_type);
        if (fuesed) {
            ret = MAP_FUSED_ELEMENT;
            // add element to map if the map elment was replaced
            if (fuesed->GetID() == MAP_DEFAULT_ID) AddElement(el);
            el = fuesed;
        }

    } else if (similar_els.size() > 1) {
        // more then one similar element was found

        // TODO(markus) with simlar factor
        // ret = DoFuseElement(similar_els.back(),el,t,fuse_type);
        // we update the last element for now
        // fuse with all the similar elements
        tSptrMapElement next_fuse = el;
        //  LOG_INFO_PRINTF("multi start ");
        BOOST_FOREACH (tSptrMapElement &sim_el, similar_els) {
            tMapID id_in_map = sim_el->GetID();
            tMapID id_new = sim_el->GetID();
            //  LOG_INFO_PRINTF("-- multi fuse with element in map %s\n "
            //      "--with new %s", sim_el->ToString().c_str(),
            //                     next_fuse->ToString().c_str());
            if (next_fuse && sim_el->GetID() != next_fuse->GetID()) {
                next_fuse = DoFuseElement(sim_el, next_fuse, t, fuse_type);
                // debug printf
                //   tSptrMapElement next_fuse_tmp_debug = DoFuseElement(sim_el,
                //   next_fuse, t, fuse_type);
                //   if (sim_el->GetType() == DEPTH
                //       && next_fuse->GetType() == STREET_TRAFFIC_SIGN_GIVE_WAY
                //      && next_fuse_tmp_debug->GetType() == DEPTH  ){
                //     LOG_ERROR_PRINTF("***************depth fuse with sign to
                //     depth");
                //   }
                //   if (sim_el->GetType() == STREET_TRAFFIC_SIGN_GIVE_WAY
                //       && next_fuse->GetType() == DEPTH
                //      && next_fuse_tmp_debug->GetType() == DEPTH  ){
                //     LOG_ERROR_PRINTF("depth fuse with sign to depth");
                //   }
                //   next_fuse = next_fuse_tmp_debug;
                //   LOG_INFO_PRINTF("--to %s\n ",
                //   next_fuse->ToString().c_str());
            }
            if (id_in_map != MAP_DEFAULT_ID && id_new != MAP_DEFAULT_ID
                 && next_fuse && next_fuse->GetID() == MAP_DEFAULT_ID) {
                LOG_ERROR_PRINTF(
                    "map fuse error: default id at the end"
                    " of fuse, but all fuse types with no default id ");
            }
        }

        if (next_fuse) {
            ret = MAP_FUSED_ELEMENT;
            // add element to the map if fused with multiple
            // and all were replaced
            if (next_fuse->GetID() == MAP_DEFAULT_ID) {
                AddElement(next_fuse);
                //  LOG_INFO_PRINTF("add to map %s\n ",
                //                  next_fuse->ToString().c_str());
            }
            el = next_fuse;
        } else {
            LOG_ERROR_PRINTF(
                "map multi fuse error, null element after fuse"
                " with all similar elements");
        }
        // LOG_INFO_PRINTF("multi end");

    } else {
        // add new element to list set new id
        ret = AddElement(el, t);  // AppendElement without update_mutex_
    }
    return ret;
}

// ____________________________________________________________________________
tSptrMapElement GlobalMap::DoFuseElement(tSptrMapElement &el_map,
                                         tSptrMapElement &new_el,
                                         tTimeMapStamp t,
                                         MapFuseType fuse_type) {
    tSptrMapElement ret_e_in_map;

    // updated the map based on the return value
    // in the child classes the diffrent option are used
    MapFuseReturn update = el_map->UpdateFuse(car_pos_, *new_el, t, fuse_type);
    if (update == MAP_UPDATE_FUSE_KEEP) {
        ret_e_in_map = el_map;           // change to the element in the map
        RemoveElement(new_el->GetID());  // remove if el was in map
    } else if (update == MAP_UPDATE_FUSE_REPLACE) {
        ret_e_in_map = new_el;
        RemoveElement(el_map->GetID());  // locks

    } else {
        LOG_ERROR_PRINTF(
            "fuse failed with element in map %s\n "
            "with new %s",
            el_map->ToString().c_str(), new_el->ToString().c_str());
    }

    return ret_e_in_map;
}

// ____________________________________________________________________________
void GlobalMap::GetSimilarElements(
    MapElement *el, const tMapData tolerance_distance,
    const tMapData tolerance_area, std::vector<tSptrMapElement> &ret_el,
    bool must_be_same_type /*=true*/,
    const std::vector<MapElementType> *exclude_types /*=NULL*/,
    int max_return_cnt /*=-1*/) {
    // optinal: use  k-nearest neighbour search
    // bgi::query(spatial_index, bgi::nearest(pt, 5),
    // std::back_inserter(result));
    if (!ret_el.empty()) {
        LOG_WARNING_PRINTF(
            "GlobalMap get similar elements,\
                          return vector is not empty");
    }
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    BOOST_FOREACH (const tSptrMapElement &s, elements_) {
        if (el->GetID() != s->GetID()) {
            if (static_cast<int>(ret_el.size()) <= max_return_cnt ||
                max_return_cnt == -1) {
                // get more similar elements
                if (exclude_types == NULL ||
                    !s->IsType(*exclude_types)) {  // element not exluded
                    // the element currently added to the map (el)
                    // decides in IsSimilar if the
                    // element should be fused or not
                    // sub classes can overwrite the method IsSimilar
                    // so that fuse with a differnt types is possible
                    if (el->IsSimilar(*s, tolerance_distance, tolerance_area,
                                     must_be_same_type)) {
                        ret_el.push_back(s);
                    }
                }
            } else {
                return;
            }
        }
    }
}

// ____________________________________________________________________________
void GlobalMap::GetAllElementsWithType(MapElementType type,
                                       std::vector<tSptrMapElement> &ret_el,
                                       float distance /*=0*/) {
    const std::vector<MapElementType> types = boost::assign::list_of(type);
    GetAllElementsWithTypes(types, ret_el, distance);
}

// ____________________________________________________________________________
void GlobalMap::GetAllElementsWithTypes(
    const std::vector<MapElementType> &types,
    std::vector<tSptrMapElement> &ret_el, float distance /*=0*/) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    BOOST_FOREACH (const tSptrMapElement &s, elements_) {
        if (s->GetDistanceToCar() <= distance || distance == 0) {
            if (s->IsType(types)) {
                ret_el.push_back(s);  // copy smart pointer
            }
        }
    }
}

// ____________________________________________________________________________
std::vector<tSptrMapElement> GlobalMap::GetAllElements() { return elements_; }

// ____________________________________________________________________________
bool GlobalMap::ElementsExist(const tMapID id) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    BOOST_FOREACH (const tSptrMapElement &s, elements_) {
        if (s->GetID() == id) {
            return true;
        }
    }
    return false;
}

// ____________________________________________________________________________
MapEventCode GlobalMap::RemoveElement(tMapID id) {
    tSptrMapElement el_erased;
    if (id == MAP_DEFAULT_ID) return MAP_ERROR;
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);

        std::vector<tSptrMapElement>::iterator it = elements_.begin();
        for (; it != elements_.end(); ++it) {
            if ((*it)->GetID() == id) {
                el_erased = *it;  // copy smart prt
                elements_.erase(it);
                break;
            }
        }
    }
    // eventlist with no lock!
    if (el_erased) {
        CallEventsListener(el_erased, MAP_REMOVED_ELEMENT);
        el_erased->SetID(MAP_DEFAULT_ID);  // set id after eventlist
        return MAP_NO_ERROR;
    } else
        return MAP_ERROR;
}

// ____________________________________________________________________________
unsigned int GlobalMap::RemoveAllElementsUnderProjecteDistanceToCar(
    tMapData dist, const std::vector<MapElementType> *exclude_types /*=NULL*/) {
    std::vector<tSptrMapElement> erased_els;
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        std::vector<tSptrMapElement>::iterator it = elements_.begin();
        // TODO(markus)  remove_if
        for (; it != elements_.end();) {
            // erese if over distance and not excludeded and creation timestamp
            // is 0
            if ((*it)->GetCreateTime() != 0 &&
                (exclude_types == NULL || !(*it)->IsType(*exclude_types)) &&
                (*it)->ProjectedDistanceToCar() < dist) {
                erased_els.push_back(*it);  // copy smart pointer
                it = elements_.erase(it);

            } else
                ++it;
        }
    }

    BOOST_FOREACH (tSptrMapElement &el, erased_els) {
        CallEventsListener(el, MAP_REMOVED_ELEMENT);
        el->SetID(MAP_DEFAULT_ID);  // set id after event list
    }

    return erased_els.size();
}

// ____________________________________________________________________________
unsigned int GlobalMap::RemoveAllElementsOverLifeTime(
    tTimeMapStamp time_microseconds) {
    std::vector<tSptrMapElement> erased_els;
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        std::vector<tSptrMapElement>::iterator it = elements_.begin();
        // TODO(markus)  remove_if
        for (; it != elements_.end();) {
            // erese if over time of live if enabled
            if ((*it)->IsOverTimeOfLive(time_microseconds)) {
                erased_els.push_back(*it);  // copy smart pointer
                it = elements_.erase(it);
            } else
                ++it;
        }
    }

    BOOST_FOREACH (tSptrMapElement &el, erased_els) {
        CallEventsListener(el, MAP_REMOVED_ELEMENT);
        el->SetID(MAP_DEFAULT_ID);  // set id after event list
    }
    return erased_els.size();
}

// ____________________________________________________________________________
void GlobalMap::ClearElements() {
    std::vector<tSptrMapElement> erased_els;


    { boost::mutex::scoped_lock scoped_lock(update_mutex_);
      // inform all event list
      erased_els = elements_;
      elements_.clear();
    }
    BOOST_FOREACH (tSptrMapElement &el, erased_els) {
        CallEventsListener(el, MAP_REMOVED_ELEMENT);
        el->SetID(MAP_DEFAULT_ID);  // set id after event list
    }


    // LOG_INFO_PRINTF("GlobalMap all elements cleared");
    // we keep the id counter
}

// ____________________________________________________________________________
void GlobalMap::RegisterEventListener(MapEventListener *new_list) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    // check if already registered
    std::vector<MapEventListener *>::iterator it =
        std::find(evlistener_.begin(), evlistener_.end(), new_list);
    if (it == evlistener_.end()) {
        // list not in vector
        evlistener_.push_back(new_list);
    } else
        LOG_WARNING_PRINTF("reset not yet implemented");
}

// ____________________________________________________________________________
void GlobalMap::DeregisterEventListener(MapEventListener *list) {
    boost::mutex::scoped_lock scoped_lock(update_mutex_);
    evlistener_.erase(std::remove(evlistener_.begin(), evlistener_.end(), list),
                      evlistener_.end());
}

// ____________________________________________________________________________
void GlobalMap::CallEventsListener(tSptrMapElement &el, MapEventCode e) {
    // call the event list. method for all registerd listener
    tMapData dis;
    tMapData dis_prev;
    tMapData list_rang_max;

    try {
        BOOST_FOREACH (MapEventListener *l, evlistener_) {
            // position and angle to the car were updated
            dis = el->GetDistanceToCar();
            dis_prev = el->GetPreviousDistanceCar();
            list_rang_max = l->GetMaxRange();
            // check if the car is in range if so call map_on_event_changed
            // max range, if zero disabled
            if (dis >= l->GetMinRange() &&
                (list_rang_max == 0 || dis <= list_rang_max)) {
                // call the event listener
                if (e == MAP_ADDED_NEW_ELEMENT) {
                    l->MapOnEventAddedNew(el);
                } else if (e == MAP_REMOVED_ELEMENT) {
                    l->MapOnEventRemoved(el);
                } else if (e == MAP_CHANGED_ELEMENT) {
                    // element in in range for listener
                    // position of the car changed
                    // check if a distance threshold is reached
                    if (dis_prev != dis) {
                        BOOST_FOREACH (const tMapData &threshold,
                                       l->GetAllDistanceThresholds()) {
                            // check in the distance threshold was reached
                            // aftter
                            // last update

                            if (dis > threshold && dis_prev < threshold) {
                                // moved over threshold
                                l->MapOnEventDistanceReached(el, threshold,
                                                             false);
                            } else if (dis <= threshold &&
                                       threshold <= dis_prev) {
                                // moved under distance threshold
                                l->MapOnEventDistanceReached(el, threshold,
                                                             true);
                            }
                        }
                    }
                } else {
                    LOG_WARNING_PRINTF("map unknown event callback");
                }
            }
        }
    } catch (std::exception const &e) {
        LOG_ERROR_PRINTF("Map MapEventListener method call error");
        std::cout << e.what() << std::endl;
    }
}

// ____________________________________________________________________________
void GlobalMap::Print(const char *title /*=NULL*/,
                      std::vector<MapElementType> *include_types /*=NULL*/) {
    // func can t be const because update_mutex_
    // print the car position and all map elements to cout with the log macro
    if (title == NULL) title = "";
    LOG_INFO_PRINTF("MAP - Start - %s", title);
    LOG_INFO_PRINTF("   Car position x: %0.2f y: %0.2f heading degree: %0.2f",
                    car_pos_.x, car_pos_.y, (car_pos_.heading * (180. / M_PI)));
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        BOOST_FOREACH (const tSptrMapElement &s, elements_) {
            if (include_types == NULL || s->IsType(*include_types))
                LOG_INFO_PRINTF("   %s", s->ToString().c_str());
        }
    }
    LOG_INFO_PRINTF("MAP - End - %s", title);
}

// ____________________________________________________________________________
void GlobalMap::SaveGlobaltxt(
    const char *file_name,
    std::vector<MapElementType> *exclude_types /*=NULL*/) const {
    if (file_name == NULL) file_name = MAP_DEFAULT_SAVE_NAME_TXT;

    std::ofstream mapfile(file_name);
    // write header
    mapfile << "# first line current car pos[x,y,theta], \
    then each line is a polygon in the format:\
    type,bool_is_global_agngel_used,global_agngel_used,,x1,y1,x2,y2,...,"
            << std::endl;
    mapfile << car_pos_.x << "," << car_pos_.y << "," << car_pos_.heading << ","
            << std::endl;
    {  // boost::mutex::scoped_lock scoped_lock(update_mutex_);
        BOOST_FOREACH (const tSptrMapElement &s, elements_) {
            const tMapPolygon &poly = s->GetGlobalPoly();
            mapfile << s->GetType() << ",";
            mapfile << s->IsOrientationUsed() << ",";
            if (s->IsOrientationUsed())
                mapfile << s->GetGlobalOrientation() << ",";
            else
                mapfile << -1 << ",";
            BOOST_FOREACH (tMapPoint const &p, bg::exterior_ring(poly)) {
                if (exclude_types == NULL || !s->IsType(*exclude_types))
                    mapfile << p.x() << "," << p.y() << ",";
            }
            mapfile << std::endl;  // flush
        }
    }
    mapfile.close();
    boost::filesystem::path full_path(boost::filesystem::current_path());
    LOG_INFO_PRINTF("GlobalMap: saved map to txt: %s/%s",
                    full_path.string().c_str(), file_name);
}

// ____________________________________________________________________________
void GlobalMap::SaveGlobalsvg(
    const char *file_name /*=NULL*/,
    std::vector<MapElementType> *exclude_types /*=NULL*/) const {
    // example to add string to svg
    // https://github.com/alexanderkjeldaas/boost-mirror/blob/master/libs/geometry/example/06_b_transformation_example.cpp
    // boost::mutex::scoped_lock scoped_lock(update_mutex_);//TODO
    if (file_name == NULL) file_name = MAP_DEFAULT_SAVE_NAME_SVG;

    // Declare a stream and an SVG mapper
    std::ofstream svg(file_name);
    boost::geometry::svg_mapper<tMapPoint> mapper(svg, 500, 500);
    tMapPoint car_pos(car_pos_.x, car_pos_.y);

    // Add geometries such that all these geometries fit on the map
    mapper.add(car_pos);  // add car pos

    BOOST_FOREACH (const tSptrMapElement &s, elements_) {
        if (exclude_types == NULL || !s->IsType(*exclude_types))
            mapper.add(s->GetGlobalPoly());
    }

    // Draw the geometries on the SVG map, using a specific SVG style
    mapper.map(car_pos,
               "fill-opacity:0.5;fill:rgb(153,204,0);\
               stroke:rgb(153,204,0);\
               stroke-width:2",
               30);
    BOOST_FOREACH (const tSptrMapElement &s, elements_) {
        // add polygon
        if (exclude_types == NULL || !s->IsType(*exclude_types)) {
            if (s->GetGlobalPoly().outer().size() > 1) {
                mapper.map(s->GetGlobalPoly(),
                           "fill-opacity:0.3;fill:rgb(211,211,211);\
                   stroke:rgb(169,169,169);\
                   stroke-width:2");
            } else if (s->GetGlobalPoly().outer().size() == 1) {
                // add point
                mapper.map(s->GetGlobalPoly().outer().at(0),
                           "fill-opacity:0.5;\
                       fill:rgb(211,211,211);\
                       stroke:rgb(169,169,169);\
                       stroke-width:2",
                           10);
            }
        }
    }
    boost::filesystem::path full_path(boost::filesystem::current_path());
    LOG_INFO_PRINTF("GlobalMap: saved map to svg: %s/%s",
                    full_path.string().c_str(), file_name);
    // Destructor of map will be called - adding </svg>
    // Destructor of stream will be called, closing the file
    // calling  svg.close(); leads to an error in the svg file
}

// ____________________________________________________________________________
bool GlobalMap::CheckElementsCollision(
    tSptrMapElement &el, std::vector<tSptrMapElement> &ret_el,
    tMapData max_center_distance_to_car /* = -1*/, int max_return_cnt /*= -1*/,
    const std::vector<MapElementType> *exclude_types /*=NULL*/) {
    // optional use:
    // bgi::query(spatial_index, bgi::disjoint(box),
    // std::back_inserter(result));
    bool ret = false;

    if (max_return_cnt != -1 && !ret_el.empty()) {
        LOG_WARNING_PRINTF(
            "GlobalMap: CheckElementsCollision "
            "return vector not empty and max ret cnt is used");
    }
    // tranform the local poly to the globale if not done before
    ElementLocalToGlobal(el);  // locks

    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);

        BOOST_FOREACH (const tSptrMapElement &s, elements_) {
            // check if in range (center)
            if (max_center_distance_to_car == -1 ||
                s->GetDistanceToCar() <= max_center_distance_to_car) {
                // check if type is excluded if used
                if (exclude_types == NULL || !s->IsType(*exclude_types)) {
                    // if (!s->GlobalPolyDisjoint(*el)){
                    if (!el->GlobalPolyDisjoint(*s)) {
                        // collision detected
                        if (s->GetID() != el->GetID()) {
                            ret_el.push_back(s);  // copy smart pointer

                            if (!ret) ret = true;
                            // check if max return cnt is reached
                            if (max_return_cnt != -1 &&
                                max_return_cnt ==
                                    static_cast<int>(ret_el.size())) {
                                return ret;
                            }
                        }
                    }
                }
            }
        }
    }

    return ret;
}

// ____________________________________________________________________________
bool GlobalMap::GetGlobalCarPosition(tMapCarPosition *p) const {
    /* get the global car position*/
    if (!p){
      LOG_ERROR_PRINTF("GetGlobalCarPosition NULL POINTER");
    }
    *p = car_pos_;  // last know pos
    return car_pos_known_;
}

// ____________________________________________________________________________
bool GlobalMap::IsCarPosKnown() const { return car_pos_known_; }
// ____________________________________________________________________________
void GlobalMap::SetCarPosKnown(bool set) { car_pos_known_ = set; }

// ____________________________________________________________________________
bool GlobalMap::DistanceToCar(tMapData x, tMapData y,
                              tMapData *ret_dist) const {
    return DistanceToCar(tMapPoint(x, y), ret_dist);
}
// ____________________________________________________________________________
bool GlobalMap::DistanceToCar(const tMapPoint &point,
                              tMapData *ret_dist) const {
    *ret_dist = bg::distance(point, tMapPoint(car_pos_.x, car_pos_.y));
    return car_pos_known_;
}

// ____________________________________________________________________________
unsigned int GlobalMap::GetElementCnt() const {
    return (unsigned int)elements_.size();
}

// ____________________________________________________________________________
tSptrMapElement GlobalMap::GetElement(tMapID id) {
    if (id == MAP_DEFAULT_ID) {
        LOG_ERROR_PRINTF("GlobalMap::GetElement with default id called");
    }
    {
        boost::mutex::scoped_lock scoped_lock(update_mutex_);
        BOOST_FOREACH (const tSptrMapElement &el, elements_) {
            if (el->GetID() == id) {
                return el;  // copy
            }
        }
    }
    return tSptrMapElement();  // return null smart ptr
}
}
} /*NAME SPACE frAIburg,map*/
