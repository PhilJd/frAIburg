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
// emergency_filter with dynamic ultrasonic pins, position for a drivable
//    area, map collision detection
// - if the pin is connected the the emergency function will be enabled
// -  two xml configuration files can be set in the filter properties:
//    frAIburg config: calibration of the ultrasonic sensors
//    filter config: map collision box size, ultrasonic thresholds a
//        and position limits
// - Ultrasonic data is stored in a circular buffer, if a cycle is compleat the,
// 	mean is used to check if a threshold is reached. Buffer size and max value
// 	to add to buffer can be set in the filter properties
// - the time interval to check for map collision can be set in the filter
//    properties
// - in the debug modus the map collision detection box in show in green if no
//    collision else it will be remove_elements_over_distance
// - use the base ultrasonic pin names of the
//    addc arduino communication filter pin names to load the correct config.
//    in the filter xml
// - if a emergency is detected the speed signal 0 ander jury bool false signal
//    is transmitted

#ifndef AADCUSER_FRAIBURG_DYN_EMERGENCY_FILTER_H_
#define AADCUSER_FRAIBURG_DYN_EMERGENCY_FILTER_H_

#define ADTF_TEMPLATE_FILTER_NAME "frAIburg Dynamic Emergency Filter"
#define OID_ADTF_TEMPLATE_FILTER "adtf.frAIburg_dynamic_emergency_filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "Ultrasonic and postion emergency Filter"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "TemplateFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "dynamic US and postion emergency filter"

//max value to add elements to the us buffers
// in the calllback the mean value is added to the map
#define ADTF_PROPERTY_US_MAX_VAL_TO_NAME \
    "Ultrasonic max limit add to ring buffer in [m]"
#define ADTF_PROPERTY_US_MAX_VAL_TO_BUFFER 1.
#define ADTF_PROPERTY_DEBUG_ENABLED_NAME "Enbale filter debug modus"
#define ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT false
#define ADTF_PROPERTY_XML_CONFIG_FILE_NAME "frAIburg xml configuration"
#define ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT \
    "/home/aadc/ADTF/configuration_files/frAIburg_configuration.xml"

#define ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_NAME \
    "frAIburg emergency filter xml configuration"
#define ADTF_PROPERTY_FILTER_XML_CONFIG_FILE_DEFAULT \
    "/home/aadc/ADTF/src/configuration/frAIburg_configuration_dynamic_emergency_filter.xml"

#define ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_NAME \
    "set ring buffer size, if full emergency is check with mean"
#define ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_DEFAULT 2

#define ADTF_PROPERTY_MAP_COLLISION_CHECK_ENABLED "Enagle Map collision check"
#define ADTF_PROPERTY_MAP_COLLISION_CHECK_ENABLED_DEFAULT false

#define ADTF_PROPERTY_MAP_COLLISION_PLANNER_ENABLED "Use planner path for collision check"
#define ADTF_PROPERTY_MAP_COLLISION_PLANNER_ENABLED_DEFAULT true

#define ADTF_PROPERTY_MAP_COLLISION_CHECK_TIME_INTERVAL \
    "Time interval to check map collition in [s]"
#define ADTF_PROPERTY_MAP_COLLISION_CHECK_TIME_INTERVAL_DEFAULT 0.05

#include <cmath>
#include <boost/assign/list_of.hpp>
#include "stdafx.h"
#include "aadc_structs.h" //tSignalValue val for us input pins
#include "slim_pins.h"
#include "circular_buffer_ultrasonic.hpp"
#include "adtf_log_macros.h"
#include "map_types.h"
#include "global_map.hpp"
#include "map_element.hpp"
#include "map_helper.hpp"
#include "nonpin_types.h"
#include "customtypes.h"

typedef USCalibratedRingBuffer<float> tUSFilterBuffer;

// !  Template filter for AADC Teams
/*!
* This is an example filter for the AADC
*/
class DynamicEmergencyBreak : public adtf::cFilter,
                      frAIburg::utils::CircularBufferEventListener {
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, ADTF_TEMPLATE_FILTER_NAME,
                adtf::OBJCAT_DataFilter);

 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit DynamicEmergencyBreak(const tChar* __info);

    /*! default destructor */
    virtual ~DynamicEmergencyBreak();

    /*! alll the dynamic pins are added here*/
    tResult Connect(IPin* pSource, const tChar* strDestName,
                    __exception = NULL);

 protected:
    /// INPUT PINS
    /*! the input pin for us data */
    slim::DynamicInputPin dynamic_pins_input_us_;
    slim::InputPin car_pos_input_pin_;
    slim::InputPin car_curvature_input_pin_;
    slim::InputPin car_speed_input_pin_;

    /// OUTPUT PINS
    slim::OutputPin speed_pinoutput_;
    slim::OutputPin jury_bool_pinoutput_;
    slim::OutputPin pin_out_behaviour_planner_;
    slim::OutputPin jury_obstacle_output_pin_;

    //buff with pos vals to update
    vector<tVoid*> pos_rx_buff_;
    float current_car_pos_x_;
    float current_car_pos_y_;
    float current_car_curvature_;
    float current_car_speed_;
    float last_jury_obstacle_trans_time_;
    /** cFILTER STATEMACHINE***************************************************/
    /*! Implements the default cFilter state machine call.  e*/
    virtual tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Implements the default cFilter state machine call.*/
    virtual tResult Shutdown(tInitStage stage, __exception = NULL);

    /*! This Function will be called by all pins the filter is registered to.*/
    virtual tResult OnPinEvent(adtf::IPin* source, tInt event_code, tInt param1,
                               tInt param2, adtf::IMediaSample* media_sample);

    /** PIN METHODS  **********************************************************/
    /*! this function creates all the input pins of filter*/
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult SetPinIDs();

    /** cFILTER PROPERTIES*****************************************************/
    bool poperty_debug_enabled_;
    bool poperty_collision_check_planner_path_enabled_;

    //map collision check box set in the fitler config
    frAIburg::map::tMapData config_collision_box_local_center_x_;
    frAIburg::map::tMapData config_collision_box_local_center_y_;
    frAIburg::map::tMapData config_collision_box_size_half_x_;
    frAIburg::map::tMapData config_collision_box_size_half_y_;
    frAIburg::map::tMapData config_collision_box_angle_rad_;

    //postion limits form filter xml config
    float config_pos_limit_x_min_;
    float config_pos_limit_y_min_;
    float config_pos_limit_x_max_;
    float config_pos_limit_y_max_;

    void GetAllProperties(void);
    void SetAllProperties(void);

    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);

    /** USER  *****************************************************************/
private:
    frAIburg::map::GlobalMap* map_; //map as a singelton

    //buffer to clibrate the us signal
    //clibration is loaded based to the pin name
    std::vector<tUSFilterBuffer> us_calibrated_buffers_;
    std::vector<float> us_emergency_threshold_;

    void InitCalibrationBuffers();
    tResult ProcessDynamicdata(IPin* source, IMediaSample* media_sample,
                              slim::DynamicInputPin& dpins);
    void ProcessPosData(IPin* source, IMediaSample* mediaSample);
    void ProcessCurvatureData(IPin* source, IMediaSample* mediaSample);
    void ProcessSpeedData(IPin* source, IMediaSample* mediaSample);

    //buffer callbacks for buffer events
    void EventBufferFilled(int id);
    void EventCycleCompleat(int id);
    /*! send emergeny signal, zero speed and bool jury false and
    set the emergeny state of the filter*/
    void TransmitEmergencySignals(const frAIburg::map::tMapID *id = NULL);
    void TransmitDisabledEmergencySignals();
    void TransmitPlannerSignal(behaviour_type type,
                               const frAIburg::map::tMapID *id = NULL);
    void TransmitObjectAhead(const frAIburg::map::tMapID id);
    //check for map collision with the filter xml config box,
    // if collition return the map id of the closest collision element
    // else returns MAP_DEFAULT_ID (-1)
    frAIburg::map::tSptrMapElement MapBoxCollisionCheck(
            frAIburg::map::tSptrMapElement map_collision_check_box);

    /*! fixed rectangle box based on the xml file  */
    frAIburg::map::tSptrMapElement GetFixedRectCollisionBox();
    /*! Adaptive trapeze box based on the speed and curvature*/
    frAIburg::map::tSptrMapElement GetAdaptiveCollisionBox();
    frAIburg::map::tSptrMapElement GetPlannerCollisionPath();
    frAIburg::map::tSptrMapElement planner_path_;
    frAIburg::map::tMapID last_object_ahead_id_;

    //timer for map collision check
    void InitTimer();
    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;
    bool emergeny_state_triggerd_;
      tTimeStamp emergeny_state_triggerd_time_;
    tResult Run(tInt nActivationCode,
                const tVoid* pvUserData, tInt szUserDataSize,
                ucom::IException** __exception_ptr);
    void DeleteTimer();
    void DebugShowDrivableAreaInMap();
    void TransmitJuryObstacle(const frAIburg::map::tSptrMapElement & el_obstacle);
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_DYN_EMERGENCY_FILTER_H_

/*!
*@}
*/
