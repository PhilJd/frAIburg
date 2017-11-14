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
//  displaywidget.h
//  qt wight that plots the map if registered as a event listener in the map
//  Created by Markus on 05.07.17.
//

#ifndef AADCUSER_FRAIBURG_MAP_TYPES_LIB_H_
#define AADCUSER_FRAIBURG_MAP_TYPES_LIB_H_

//  MAP_VERSION % 100 is the patch level
//  MAP_VERSION / 100 % 1000 is the minor version
//  MAP_VERSION / 100000 is the major version
#define MAP_VERSION 100300

//#define MAP_ENABLE_DEBUG_MODE
#define MAP_BUILD_ADTF
#define MAP_BUILD_CV

#define MAP_ELEMENT_COLOR_DEBUG "plum"
#define MAP_ELEMENT_COLOR_LANDMARK "steelblue"

#define MAP_LOCAL_IS_GLOBAL_CAR_POSITION_X 0
#define MAP_LOCAL_IS_GLOBAL_CAR_POSITION_Y 0
#define MAP_LOCAL_IS_GLOBAL_CAR_POSITION_HEADING 0
#define MAP_DEFAULT_ID -1

#include <boost/version.hpp>
#if BOOST_VERSION >= 106000
#include <boost/qvm/mat_operations.hpp>
#endif
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace frAIburg {
namespace map {

class MapElement;/*forward declaration for typedef*/

typedef int64_t tMapID;
typedef int64_t tTimeMapStamp; //microsecond adtf: _clock->GetStreamTime()
typedef float tMapData;
typedef tMapData tMapCoords;
/*!smart pointer to for the map elements */
typedef boost::shared_ptr<MapElement> tSptrMapElement;
/*!boost geometry polygon to describe the location of a map element*/
typedef boost::geometry::model::d2::point_xy<tMapCoords> tMapPoint;
typedef boost::geometry::model::polygon<tMapPoint,
                      true/*ClockWise*/,false/*Closed*/> tMapPolygon;

/* adtf input position of the car*/
typedef struct {
    float x;
    float y;
    float heading;
    //time to check for data consistency
    tTimeMapStamp update_time_microseconds;
} tMapCarPosition;

/*! diffrent fuse option for MapElement::UpdateFuse*/
enum MapFuseType {
    MAP_FUSE_APPEND,
    MAP_FUSE_REPLACE,
    MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN,
    MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_007, // alpha = (sample time)/(time constant), low = new val less important
    MAP_FUSE_FIXED_AREA_POINTS_CENTER_EXP_DECAY_02,
    MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN_DECAY_9, //deacy of old mean by 0.9
    MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN_DECAY_8, //deacy of old mean by 0.8
    MAP_FUSE_FIXED_AREA_POINTS_CENTER_MEAN_DECAY_5, //deacy of old mean by 0.5
    MAP_CONVEX_HULL, //use simplify option if elements overlaps or can be within
    MAP_CONVEX_HULL_SIMPLIFY_5 //simplify 5 cm of a vertex to other segments to be removed
};

enum MapEventCode {
    MAP_ERROR,
    MAP_NO_ERROR,
    /*MAP changes*/
    MAP_ADDED_NEW_ELEMENT,
    MAP_CHANGED_ELEMENT,
    MAP_REMOVED_ELEMENT,
    // MAP_REMOVED_ELEMENT_FUSE, //TODO CHANGE
    // MAP_REMOVED_ELEMENT_LIFE_TIME,
    // MAP_REMOVED_ELEMENT_DISTANCE,
    // MAP_REMOVED_ELEMENT_USER,
    MAP_FUSED_ELEMENT,
    MAP_CAR_POS_TO_ELEMENT_CHANGED,
};

/*! return values after fuse to inform the map which element to keep
 */
enum MapFuseReturn {
    MAP_UPDATE_FUSE_FAILD,
    MAP_UPDATE_FUSE_KEEP,/* keep the element in the map*/
    MAP_UPDATE_FUSE_REPLACE/* use the map new element in the map, with the old id*/
};

/*! orientation for map_element_types */
enum MapElementOrientation{UNKNOWN_ORIENTATION,
                          LEFT,
                          RIGHT,
                          TOWARDS,
                          AWAY};

/* diffrent map element types */
enum MapElementType {
    UNDEFINED,
    UNKNOWN,
    /* STREET MARKER */
    STREET_MARKER_LANE,
    STREET_MARKER_STOP_LINE,
    STREET_MARKER_CROSSING_T,
    STREET_MARKER_CROSSING_X,
    STREET_MARKER_ZEBRA,
    STREET_PARKING_SPOT,
    /* PEDESTRIAN */
    PEDESTRIAN_ADULT,
    PEDESTRIAN_CHILD,
    PEDESTRIAN_REAL,
    /* Car */
    CAR,
    /* OBSTACLE */
    OBSTACLE,
    DEPTH,
    ULTRRASONIC,
    /* SIGNS detected by sensor*/
    STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT,
    STREET_TRAFFIC_SIGN_STOP,
    STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC,
    STREET_TRAFFIC_SIGN_JUST_STRAIGHT,
    STREET_TRAFFIC_SIGN_GIVE_WAY,  // yield sign
    STREET_TRAFFIC_SIGN_CROSSWALK,
    STREET_TRAFFIC_SIGN_PARKING,
    STREET_TRAFFIC_SIGN_CIRCLE,
    STREET_TRAFFIC_SIGN_NO_TAKE_OVER,
    STREET_TRAFFIC_SIGN_NO_ENTRY,
    STREET_TRAFFIC_SIGN_POSITION_MARKER,
    STREET_TRAFFIC_ONE_WAY_STREET,
    STREET_TRAFFIC_ROAD_WORKS,
    STREET_TRAFFIC_SIGN_SPEED_50,
    STREET_TRAFFIC_SIGN_SPEED_100,
    /* KONWN LANDMARKS: PARKING SPOT  */
    LM_STREET_PARKING_SPOT,
    /* KONWN LANDMARKS: STREELANE FOR TEST TRACK  */
    LM_STREET_LANE,
    /* DEBUG */
    DEBUG_POINT = 200,
    DEBUG_POLYGON,
    PLANNER_PATH
};

#ifdef __GNUC__
//set as cont MapElementType var for deprecated types
__attribute__ ((deprecated("use class MapElementRoadSign"))) const MapElementType LM_STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT = MapElementType(-13);
__attribute__ ((deprecated)) const MapElementType STREET_TRAFFIC_TEST_TRACK = MapElementType(-14);
//__attribute__ ((deprecated("use subtypes"))) const MapEventCode MAP_REMOVED_ELEMENT = MapEventCode(-1);
#endif

}
} /*NAME SPACE frAIburg,map*/
#endif /* AADCUSER_FRAIBURG_MAP_TYPES_LIB_H_ */
