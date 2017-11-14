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
#ifndef AADCUSER_FRAIBURG_PARKINGSPACEDETECTION_PARKING_SPACE_DETECTION_H_
#define AADCUSER_FRAIBURG_PARKINGSPACEDETECTION_PARKING_SPACE_DETECTION_H_

#define ADTF_MAP_FILTER_NAME "frAIburg Parking Space Detection"
#define OID_ADTF_MAP_FILTER "adtf.frAIburg_Parking_Space_Detection"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "AADC user ParkingSpaceDetection"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "ParkingSpaceDetection"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "ParkingSpaceDetection"

//adtf filter propteries
#define ADTF_PROPERTY_DEBUG_ENABLED_NAME "Enbale filter debug modus"
#define ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT false

#define ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_NAME \
      "xml file with parking space dimension"
#define ADTF_PROPERTY_FILTER_XML_DIM_CONFIG_FILE_DEFAULT \
      "/home/aadc/ADTF/src/configuration/frAIburg_configuration_dimensions.xml"

#define ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_NAME \
      "xml file with known parking space configuration"
#define ADTF_PROPERTY_PARKING_SPACE_CONFIG_XML_DEFAULT \
      "/home/aadc/ADTF/src/configuration/roadSigns.xml"

#define ADTF_PROPERTY_TIME_CHECK_INTERVAL_NAME \
      "time intervall to check in range parking spaces of free  in [s]"
#define ADTF_PROPERTY_TIME_CHECK_INTERVAL_NAME_DEFAULT .1

#define ADTF_PROPERTY_PARKING_SPACE_TO_CHECK_RANGE_NAME \
      "Car range to scean if parking spot is free [m]"
#define ADTF_PROPERTY_PARKING_SPACE_TO_CHECK_RANGE_DEFAULT 1.

#define ADTF_PROPERTY_PARKING_JURY_RANGE_NAME \
      "Disabled! projected car distance to send status to jury [m]"
#define ADTF_PROPERTY_PARKING_JURY_RANGE_DEFAULT -0.3 //

#define ADTF_PROPERTY_PARKING_SPACE_MARGIN_X_NAME \
      "Parking space collision box margin half x [m]"
#define ADTF_PROPERTY_PARKING_SPACE_MARGIN_X_DEFAULT -0.05

#define ADTF_PROPERTY_PARKING_SPACE_MARGIN_Y_NAME \
      "Parking space collision box margin half Y [m]"
#define ADTF_PROPERTY_PARKING_SPACE_MARGIN_Y_DEFAULT 0.//TODO not wroking

#define ADTF_PROPERTY_PARKING_SPACE_US_HTIS_OCCUPIED_NAME \
      "Parking space ultrasonic hits to be occupied"
#define ADTF_PROPERTY_PARKING_SPACE_US_HTIS_OCCUPIED_DEFAULT 4

#define ADTF_PROPERTY_PARKING_SPACE_DEPTH_HTIS_OCCUPIED_NAME \
      "Parking space depth hits to be occupied"
#define ADTF_PROPERTY_PARKING_SPACE_DEPTH_HTIS_OCCUPIED_DEFAULT 9999// disabled for testing

#include <vector>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include "stdafx.h"
#include "slim_pins.h"
#include "adtf_log_macros.h"
#include "xml_helper.hpp"
#include "map_element.hpp"
#include "map_element_types.h"
#include "global_map.hpp"
#include "map_types.h"
#include "map_helper.hpp"
/*! @defgroup TemplateFilter
*  @{
*
*  ![Plugin Template Filter](User_Template.PNG)
*
* This is a small template which can be used by the AADC teams for their own
* filter implementations.
*
* **Dependencies** \n
* This plugin needs the following libraries:
*
*
* **Filter Properties**
* | Property | Description | Default |
* | -------- | ----------- | ------- |
* | bla      | blubb       | 9       |
*
*
*
* **Input Pins**
* | Pin            | Description          | MajorType           |
* | ---------------| -------------------- |tPosition            |
*
* | input_template | An example input pin | MEDIA_TYPE_TEMPLATE |
* MEDIA_TYPE_TEMPLATE |
*
*
* **Plugin Details**
* | | |
* |-|-|
* | Path    | src/aadcUser/frAiburg_TemplateFilter |
* | Filename| frAiburg_templateFilter.plb          |
* | Version | 1.0.0                                |
*
* _____________________________________________________________________________
*  Markdown examples --- Delete everything below here!
* _____________________________________________________________________________
*
*
*  [Full doxygen markdown
* documentation](https: // www.stack.nl/~dimitri/doxygen/manual/markdown.html)
*
* - Code Block
*  ~~~~~~~~~~~~~~~{.c}
*  int func(int a,int b) { return a*b; }
*  ~~~~~~~~~~~~~~~
* - [Link](https: // www.github.com)
* -
* ![Image](https: //
* www.audi-autonomous-driving-cup.com/wp-content/uploads/2017/02/Audi-emblem-2016-black-small.png)
*
*
*
*/

// !  Template filter for AADC Teams
/*!
* This is an example filter for the AADC
*/
class ParkingSpaceDetection : public adtf::cFilter,
                                  public frAIburg::map::MapEventListener
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_MAP_FILTER, ADTF_MAP_FILTER_NAME,
                adtf::OBJCAT_DataFilter);

 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit ParkingSpaceDetection(const tChar* __info);

    /*! default destructor */
    virtual ~ParkingSpaceDetection();

    /*! called when any property has changed in adtf.*/
    tResult PropertyChanged(const tChar* str_name);

 private:
    slim::OutputPin pin_out_jury_parking_;
    slim::OutputPin pin_out_us_enable_;

    void InitMAP();
    frAIburg::map::GlobalMap* map_; //map as a singelton

    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;
    tResult Run(tInt nActivationCode,
                const tVoid* pvUserData, tInt szUserDataSize,
                __exception); //TODO(markus) NULL?
    void InitTimer();
    void DeleteTimer();

    void InitUltrasonic();

    /*! filter properties, set in adtf*/
    bool property_debug_enabled_;
    float property_range_check_parking_;
    float property_range_send_jury_;
    unsigned int property_parking_space_us_hits_occupied_;
    unsigned int property_parking_space_hepth_hits_occupied_;
    float property_map_parking_space_size_half_x_;
    float property_map_parking_space_size_half_y_;

    void SetAllProperties(void);
    void GetAllStaticProperties(void);

 protected:
    cCriticalSection update_mutex_;
    bool us_enabled_;

    /** cFILTER STATE
     * MACHINE***************************************************/
    /*! Implements the default cFilter state machine call.  e*/
    virtual tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Implements the default cFilter state machine call.*/
    virtual tResult Shutdown(tInitStage stage, __exception = NULL);

    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    /** PIN METHOODS  *********************************************************/

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(__exception = NULL);

    /*! this function creates all the  pins id order*/
    tResult SetPinIDs();

    /*! infrom the jury about a the state of a parking spot*/
    bool TransmitJuryParking(tInt16 jury_id, tFloat32 x,
                             tFloat32 y, tUInt16 is_not_free);

    /*! add a parking space box to the map*/
    void AddParkingSpaceToMap(frAIburg::map::tMapData x,//center
                              frAIburg::map::tMapData y,
                              frAIburg::map::tMapData angle,
                              bool status,
                              tInt16 jury_id,
                              frAIburg::map::tTimeMapStamp t,
                              bool global_frame = false);

    void LoadKnownLandmarkConfiguration();

    std::vector<frAIburg::map::tSptrMapElement> parking_el_to_check_;

    bool RemoveElementCheck(frAIburg::map::tMapID id);
    /*! check all parking spaces in range if free*/
    void CheckParkingSpacesInRange();
    bool IsFree(frAIburg::map::tSptrMapElement &parking_el);
public:
    /*functions that will be called when the map is updated*/
    /*! add parking spaces in range to vector to check in the timer*/
    void MapOnEventDistanceReached(frAIburg::map::tSptrMapElement,
                                  frAIburg::map::tMapData threshold,
                                  bool distance_lower_threshold);
    void MapOnEventRemoved(frAIburg::map::tSptrMapElement el);
    void MapOnEventAddedNew(frAIburg::map::tSptrMapElement el);

private:
    void AddParkingSpaceToCheckList(frAIburg::map::tSptrMapElement &el);
    /*! enable or disable the us if based on the number of el in the check vec*/
    void SetUSPin();

    /*! update the status of the parking space and inform the jury*/
    void UpdateInformJuryParkingSpaceStatus(
        frAIburg::map::MapElementParking* p_space, bool space_free);
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_PARKINGSPACEDETECTION_PARKING_SPACE_DETECTION_H_

/*!
*@}
*/
