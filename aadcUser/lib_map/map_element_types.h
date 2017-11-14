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

#ifndef AADCUSER_FRAIBURG_MAP_LIB_ELEMENT_TYPE_H_
#define AADCUSER_FRAIBURG_MAP_LIB_ELEMENT_TYPE_H_

#include <boost/lexical_cast.hpp>
#include "map_element.hpp"
#include "map_helper.hpp"
#include "adtf_log_macros.h"
#include "circular_buffer.hpp"
#include <string>
#include <cmath>

namespace frAIburg {
namespace map {

/*! @defgroup map_element_types
*  @{
*  MapElement subtypes, with addition information for jury communication and
*   specific information for diffrent AADC tasks and specific fuse options
*  multifuse for child classes of MapElement
*  - in the method IsSimilar the element can decide with which othere elements it
*  should be fused
*  - in the method UpdateFuse it can be set how the fuse with diffrent types is
*     handled
*/

#ifdef __GNUC__
class __attribute__((visibility("default"))) AADCJuryStatus;
class __attribute__((visibility("default"))) MapElementParking;
class __attribute__((visibility("default"))) MapElementRoadSign;
class __attribute__((visibility("default"))) MapElementPedestrian;
class __attribute__((visibility("default"))) MapElementCar;
class __attribute__((visibility("default"))) MapElementObstacle;
class __attribute__((visibility("default"))) MapElementDepth;
#endif

// ----------------------------------------------------------------------------
//                                    AADCJuryStatus
// ----------------------------------------------------------------------------
/*! Basic information for the jruy communication: id, sendstatus, fuse update
 */
class AADCJuryStatus  {

public:
  AADCJuryStatus();
  AADCJuryStatus(int jury_id);
  virtual ~AADCJuryStatus();

  bool IsJuryInformed() const;
  void JuryInformed(tTimeMapStamp t);
  bool GetJuryInformedTime(tTimeMapStamp *t);
  int GetJuryID() const;

  bool JuryUpdateFuse(const AADCJuryStatus  *element_fuse);

private:
  const int jury_id_;
  bool jury_informed_;
  tTimeMapStamp jury_informed_time_;
};

// ----------------------------------------------------------------------------
//                                    MapElementParking
// ----------------------------------------------------------------------------
/*! Map ELement with addistion information for parking: space status, jruy id
 *  and send information
 */
enum MapParkingStatus{PARKING_FREE, PARKING_OCCUPIED};

class MapElementParking : public MapElement, public AADCJuryStatus {

public:

   MapElementParking(short jury_id_parking,
                     MapParkingStatus status, //set in the xml file
                     const std::vector<tMapPoint> &local_poly,
                     tTimeMapStamp timestamp,
                     tMapData local_orientation_angle);

   /*! fuse with and changing the parking status, and jruy send status*/
   virtual MapFuseReturn UpdateFuse(const tMapCarPosition  &global_car_pos,
                           MapElement  &element_fuse,
                           tTimeMapStamp timestamp = 0,
                           MapFuseType fuse_type = MAP_FUSE_APPEND);

   /*! return MapElement ToString with parking space occupied status*/
   virtual std::string ToString() const;

   /*! MapElement::IsSimilar: check if elements shoul be fused */
   virtual bool IsSimilar(const MapElement &el,
                         tMapData tolerance_distance = 0.1,
                         tMapData tolerance_area = -1,
                         bool similar_fuse = true);

   void SetParkingStatus(MapParkingStatus new_status);
    /*! returns true once SetStatus was called for the element*/
   bool GetStatus(MapParkingStatus * ret_status) const;

   static bool IsParkingType(MapElementType type);

private:
  MapParkingStatus status_;
  bool status_is_set_;

};

// ----------------------------------------------------------------------------
//                              MapElementRoadSign
// ----------------------------------------------------------------------------
/*! Map Sign ELement with information if jury was informed, send info stored
 *   with the map element because there is no jruy unique jury id for a sign.
 *  fuse option with depth
 */
enum MapRoadSignSubType{SIGN_SENSOR, SIGN_LANDMARK};

class MapElementRoadSign : public MapElement, public AADCJuryStatus {

public:

  MapElementRoadSign(short jury_id_sign,
                     MapRoadSignSubType type,
                     const std::vector<tMapPoint> &local_poly,
                     tTimeMapStamp timestamp,
                     tMapData local_orientation_angle);

  /*! fuse same sign and handle depth data*/
  virtual MapFuseReturn UpdateFuse(const tMapCarPosition  &global_car_pos,
                          MapElement  &element_fuse,
                          tTimeMapStamp timestamp = 0,
                          MapFuseType fuse_type = MAP_FUSE_APPEND);

  /*! return MapElement with jury send info*/
  virtual std::string ToString() const;

  /*! Allow only fuse with same sign sub type and allow fuse with depth data*/
  virtual bool IsSimilar(const MapElement  &el,
                        tMapData tolerance_distance = 0.1,
                        tMapData tolerance_area = -1,
                        bool similar_fuse = true);

  MapRoadSignSubType GetSignSubType() const;

  void SetCorrespondingSign(tSptrMapElement el);
  /*! check with if if set*/
  tSptrMapElement GetCorrespondingSign();

  static MapElementType AADCSignTypeToMapType(short id);

  static bool IsRoadSign(MapElementType type);

private:
  const MapRoadSignSubType sign_type_;
  tSptrMapElement corresponding_sigin_;
};

// ----------------------------------------------------------------------------
//                          MapElementBasePedestrian
// ----------------------------------------------------------------------------
/*! Pedestrian class with extra info for child/ adult, and orientation
 */

enum MapPedestrianType{CHILD, ADULT, REAL_PERSON};


class MapElementPedestrian : public MapElement {

public:

   MapElementPedestrian(MapPedestrianType type,
                            MapElementOrientation orientation,
                            const std::vector<tMapPoint> &local_poly,
                            tTimeMapStamp timestamp);

   /*! fuse with and changing the orientation and handle depth data.
      not virtual for child and adult*/
   MapFuseReturn UpdateFuse(const tMapCarPosition  &global_car_pos,
                  MapElement  &element_fuse,
                   tTimeMapStamp timestamp = 0,
                   MapFuseType fuse_type = MAP_FUSE_APPEND);

   virtual std::string ToString() const;

   /*! allow fuse only with same MapPedestrianType and allow fuse with
       depth data*/
   virtual bool IsSimilar(const MapElement  &el,
                         tMapData tolerance_distance = 0.1,
                         tMapData tolerance_area = -1,
                         bool similar_fuse = true);

   MapElementOrientation GetPedestrianOrientation() const;
   MapPedestrianType GetPedestrianType() const;

   static MapElementType PedestrianTypeToMapType(MapPedestrianType type);
   static bool IsPedestrian(MapElementType type);

private:
  const MapPedestrianType pedestrian_type_;
  MapElementOrientation orientation_;

};


// ----------------------------------------------------------------------------
//                              MapElementDepth
// ----------------------------------------------------------------------------
/*! depth info in the map, child calss to enable fuse with dirrent classes */

class MapElementDepth: public MapElement {

public:

   MapElementDepth(const std::vector<tMapPoint> &local_poly,
                      tTimeMapStamp timestamp);

  /*! MapElement with changing tie jury communication info */
  virtual MapFuseReturn UpdateFuse(const tMapCarPosition  &global_car_pos,
                                   MapElement  &element_fuse,
                                   tTimeMapStamp timestamp = 0,
                                   MapFuseType fuse_type = MAP_FUSE_APPEND);

   virtual std::string ToString() const;

   /*! enable fuse with differnt map types*/
   virtual bool IsSimilar(const MapElement  &el,
                         tMapData tolerance_distance = 0.1,
                         tMapData tolerance_area = -1,
                         bool similar_fuse = true);

   /*! return ture if the map type can be fused with depth */
   static bool IsFuseTypeWithDepth(const MapElement &el);

};

// ----------------------------------------------------------------------------
//                              MapElementObstacle
// ----------------------------------------------------------------------------
/*! Obstacle with jury send information */

class MapElementObstacle : public MapElement, public AADCJuryStatus {

public:

   MapElementObstacle(const std::vector<tMapPoint> &local_poly,
                      tTimeMapStamp timestamp);
  /*! MapElement with changing tie jury communication info */
  virtual MapFuseReturn UpdateFuse(const tMapCarPosition  &global_car_pos,
                          MapElement  &element_fuse,
                          tTimeMapStamp timestamp = 0,
                          MapFuseType fuse_type = MAP_FUSE_APPEND);

   virtual std::string ToString() const;

   /*! calls base class IsSimilar*/
   virtual bool IsSimilar(const MapElement  &el,
                         tMapData tolerance_distance = 0.1,
                         tMapData tolerance_area = -1,
                         bool similar_fuse = true);

};

// ----------------------------------------------------------------------------
//                              MapElementCar
// ----------------------------------------------------------------------------
/*! Car with speed information based on fuse update */
#define MAP_CAR_MAX_SPEED_TO_ADD_TO_BUFFER_MS 1

#define MAP_EL_CAR_MAX_SPEED 1.5


class MapElementCar : public MapElement {

public:

   MapElementCar(MapElementOrientation orientation,
                 const std::vector<tMapPoint>& local_poly,
                 tTimeMapStamp timestamp);
  /*! MapElement with changing tie jury communication info */
  virtual MapFuseReturn UpdateFuse(const tMapCarPosition  &global_car_pos,
                          MapElement  &element_fuse,
                          tTimeMapStamp timestamp = 0,
                          MapFuseType fuse_type = MAP_FUSE_APPEND);

   virtual std::string ToString() const;

   /*! calls base class IsSimilar*/
   virtual bool IsSimilar(const MapElement  &el,
                         tMapData tolerance_distance = 0.1,
                         tMapData tolerance_area = -1,
                         bool similar_fuse = true);

    MapElementOrientation GetCarOrientation() const;

    tMapData GetRelativeLocalXSpeedObjectDetection() const;
    tMapData GetRelativeLocalXSpeedObjectDepth() const;

 private:

   void UpdateSpeedBuffer(tMapData current_dist,
                           tMapData *last_dist,
                           tTimeMapStamp current_time_micor_s,
                           tTimeMapStamp *last_time_micor_s,
                           utils::CircularBuffer<tMapData> *buf);
   tMapData GetSpeed( const utils::CircularBuffer<tMapData> &buf) const;

   MapElementOrientation orientation_;

   tTimeMapStamp last_fuse_time_object_detection_;
   tMapData last_projected_distance_object_detection_;
   utils::CircularBuffer<tMapData> buf_speed_rel_x_object_detection_;

   tTimeMapStamp last_fuse_time_depth_;
   tMapData last_projected_distance_depth_;
   utils::CircularBuffer<tMapData> buf_speed_rel_x_depth_;


};


}
} /*NAME SPACE frAIburg,map*/

#endif //AADCUSER_FRAIBURG_MAP_LIB_ELEMENT_TYPE_H_
