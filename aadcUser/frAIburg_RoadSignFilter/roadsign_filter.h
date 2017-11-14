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
#ifndef AADCUSER_FRAIBURG_ROADSIGN_FILTER_H_
#define AADCUSER_FRAIBURG_ROADSIGN_FILTER_H_



#define OID_ADTF_ROADSIGN_FILTER "adtf.example.roadsign_filter" //TODO
#define ADTF_FILTER_ROADSIGN_FILTER_NAME "frAIburg Road Sign To Map Filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "frAIburg Road Sign To Map Filter"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "frAIburgRoadSignToMapFilter"

#define ADTF_PROPERTY_MAP_FUSE_DIST_NAME "Sign fuse distance [m]"
#define ADTF_PROPERTY_MAP_FUSE_AREA_NAME "Sign fuse area [m2]"
#define ADTF_PROPERTY_MAP_SIGIN_ADD_TIME_NAME "Min update time between sign map [s]"
#define ADTF_PROPERTY_XML_ROAD_SIGN_FILE_NAME "xml file known sigins"
#define ADTF_PROPERTY_XML_ROAD_SIGN_FILE_DEFAULT \
        "/home/aadc/ADTF/src/configuration/roadSigns.xml"
#define ADTF_PROPERTY_XML_CONFIG_FILE_NAME "frAIburg xml configuration"
#define ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT \
        "/home/aadc/ADTF/configuration_files/frAIburg_configuration.xml"

#define ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME \
        "frAIburg xml configuration camera sensor target"
#define ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_DEFAULT "camera basler cropped"

#define ADTF_PROPERTY_XML_LOAD_ENABLED_NAME "Enable load known signs"
#define ADTF_PROPERTY_XML_LOAD_ENABLED_DEFAULT true

#define ADTF_PROPERTY_DEBUG_ENABLED_NAME "Enable debug mode"
#define ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT false

#define ADTF_PROPERTY_SCAN_RANGE_NAME \
  "In range distance to scan for corresponding signs [m]"
#define ADTF_PROPERTY_SCAN_RANGE_DEFAULT 0.5

#define ADTF_PROPERTY_PROJECTED_DIST_TO_INFORM_JURY_NAME \
  "Projected distance of a sign to inform the jury[m]"
//behind sign wait for repostion jumps
#define ADTF_PROPERTY_PROJECTED_DIST_TO_INFORM_JURY_DEFAULT -0.2




#define ADTF_PROPERTY_JURY_CHECK_INTERVAL_NAME \
  "check time intervall to check signs to inform the jury second [s]"
#define ADTF_PROPERTY_JURY_CHECK_INTERVAL_DEFAULT 0.1

#define ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME \
      "xml file with sign dimension"
#define ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_DEFAULT \
      "/home/aadc/ADTF/src/configuration/frAIburg_configuration_dimensions.xml"

//MAP sign properties, sgin box size and fuse data
#define ADTF_PROPERTY_ROADSGIN_FILTER_SIGN_MAX_FUSE_DIST_DEFAULT 1.0
#define ADTF_PROPERTY_ROADSGIN_FILTER_SIGN_MAX_FUSE_AREA_DEFAULT -1.
#define ROADSGIN_FILTER_SIGN_BOX_LENGTH_HALF_X 0.05
#define ROADSGIN_FILTER_SIGN_BOX_LENGTH_HALF_Y 0.15
//note if too small not able to add multiple diff sign to map:
#define ADTF_PROPERTY_MAP_SING_UPDATE_TIME_S_DEFAULT 0.0

#include <cmath>
#include <boost/assign/list_of.hpp>
#include "stdafx.h"
#include "aadc_structs.h"
#include "slim_pins.h"
#include "adtf_log_macros.h"
#include "opencv2/calib3d.hpp"
#include "map_element_types.h"
#include "map_types.h"
#include "global_map.hpp"
#include "map_element.hpp"
#include "map_helper.hpp"
#include "xml_helper.hpp"

class RoadSignFilter : public adtf::cFilter,
                       public frAIburg::map::MapEventListener
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_ROADSIGN_FILTER, ADTF_FILTER_ROADSIGN_FILTER_NAME,
                adtf::OBJCAT_DataFilter);

 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit RoadSignFilter(const tChar* __info);

    /*! default destructor */
    virtual ~RoadSignFilter();

    /*! called when any property has changed in adtf.*/
    tResult PropertyChanged(const tChar* str_name);
    /*! Implements the default cFilter state machine calls.*/
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls.*/
    tResult Stop(ucom::IException** __exception_ptr = NULL);

 protected:

    tResult ProcessRoadSignInputExt(IMediaSample* pMediaSample);

    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage stage,__exception = NULL);

    tResult OnPinEvent(adtf::IPin* source, tInt event_code, tInt param1,
                       tInt param2, adtf::IMediaSample* media_sample);

    /*! this function creates all the input pins of filter*/
    tResult CreateInputPins(__exception = NULL);

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(__exception = NULL);
    /*! this function creates all the  pins id order*/
    tResult SetPinIDs();

public:
  /*functions that will be called when the map is updated*/
  /*! get event if sign out of range, to inform the jury */
  void MapOnEventDistanceReached(frAIburg::map::tSptrMapElement,
                                frAIburg::map::tMapData threshold,
                                bool distance_lower_threshold);
  void MapOnEventRemoved(frAIburg::map::tSptrMapElement el);
  void MapOnEventAddedNew(frAIburg::map::tSptrMapElement el);

private:
    void InitMAP();
    void GetAllStaticProperties(void);
    void SetAllProperties(void);

    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;
    tResult Run(tInt nActivationCode,
                const tVoid* pvUserData, tInt szUserDataSize,
                __exception); //TODO(markus) NULL?
    void InitTimer();
    void DeleteTimer();

    float AxisAnglesToYawAngle(tFloat32 tvecFl32array[3]);

    /*! add or fuse a road sign box to map, SIGN_LANDMARK singn are added in
    the global frame,SIGN_SENSOR in the local */
    void AddSignToMap(tInt16 id_aadc,
                      frAIburg::map::tMapData x,//center
                      frAIburg::map::tMapData y,
                      frAIburg::map::tMapData angle,
                      frAIburg::map::tTimeMapStamp t,
                      frAIburg::map::MapRoadSignSubType type);

    /*! maps the addc sigin type to a map type*/
    frAIburg::map::MapElementType SignTypeToMapType(tInt16 i16ID);
    void LoadKnownLandmarkConfiguration();

    /*! find the landmark or sensor sign of el_r in the map, return NULL if non
     *  found
     */
    frAIburg::map::MapElementRoadSign* FindCorrespondingSign(
                            frAIburg::map::MapElementRoadSign* el_r);

    /*! inform the jury about a missing or unexpected sign
    *    the element
    *   \param jury_id -1 if missing, else the sign type
    *   \param x y center position
    *   \param angle Traffic sign facing direction in
    *                  degrees. 0..360° where 0 is the
    *                  direction of the positive y-axis.
    */
    bool TransmitJurySign(tInt16 jury_id, tFloat32 x,
                          tFloat32 y, tFloat32 angle);
    void CheckSign(frAIburg::map::tSptrMapElement &el);
    void ProcessMapEvent(frAIburg::map::tSptrMapElement &el);
    bool RemoveRelevantElement(frAIburg::map::tMapID id);
    /*! add debug point to map with changing color*/
    void AddDebugPointToMap(frAIburg::map::tMapData local_sign_center_x,
                            frAIburg::map::tMapData local_sign_center_y,
                            frAIburg::map::tMapData local_sign_angle_rad);
    bool IsUpdateTimeOkToAddNewSignToMap();

    /*! send an signn to the jury and updatet the status of the sign */
    void UpdateJurySign(frAIburg::map::MapElementRoadSign* el_r);

    /*! update both sigins with corresponding sign*/
    void UpdateSignWithCorrespondingSign(
                  frAIburg::map::MapElementRoadSign* el_r,
                  frAIburg::map::MapElementRoadSign* el_corresponding);

    frAIburg::map::GlobalMap* map_;
    cCriticalSection update_mutex_;
    std::vector<frAIburg::map::tSptrMapElement> relevant_signs_;
    /// INPUT PINS
    slim::InputPin pin_in_road_sign_ext_;
    slim::OutputPin pin_out_jury_sign_;
    /*! filter properties*/
    bool property_debug_enabled_;
    float property_range_scan_for_corresponding_sign_;
    float property_projected_dist_inform_jury_missing_sign_;
    bool first_sensor_sign_after_init_position_jump_;

    /*! offset of the camera to transform into the local car frame*/
    float config_camera_offse_x_;
    float config_camera_offse_y_;
    /*! sign size for the map box element*/
    float config_sign_box_size_half_x_;
    float config_sign_box_size_half_y_;

    /*! map fuse properties, set in adtf*/
    frAIburg::map::tMapData property_sign_fuse_dist_;
    frAIburg::map::tMapData property_sign_fuse_area_;
    /*! min update time to add sigin to the map*/
    float property_sign_update_time_s_map_;
    //update time map
    float last_update_time_map_sign_s_;

};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_ROADSIGN_FILTER_H_



/*!
*@}
*/
