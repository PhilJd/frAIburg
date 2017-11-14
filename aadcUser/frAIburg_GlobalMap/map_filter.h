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
#ifndef AADCUSER_FRAIBURG_MAP_FILTER_H_
#define AADCUSER_FRAIBURG_MAP_FILTER_H_

#define ADTF_MAP_FILTER_NAME "frAIburg Singleton Map"  // keep frAIburg
#define OID_ADTF_MAP_FILTER "adtf.example.map_filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "AADC user map filter"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "MAPFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small Template Filter."
//adtf filter propteries
#define ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL_NAME \
                                      "Car postion update intervall in [s]"
#define ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL_DEFAULT_VAL 0.05

//distance to last postion from input pos pin to call map UpdateRepositionCar
#define ADTF_PROPERTY_MAP_REPOSITION_CAR_POS_DISTANCE_NAME \
                    "Car reposition jump distance between pin postion update[m]"
#define ADTF_PROPERTY_MAP_REPOSITION_CAR_POS_DISTANCE_DEFAULT_METER 0.1

#define ADTF_PROPERTY_MAP_REPOSITION_PAST_TIME_TO_UPDATE_NAME \
                    "Car reposition past elements to update [s]"
#define ADTF_PROPERTY_MAP_REPOSITION_PAST_TIME_TO_UPDATE_DISTANCE_DEFAULT_VAL 30

#define ADTF_PROPERTY_MAP_ELEMENT_REMOVE_DISTANCE_NAME \
                    "Car remove elements under protected distance to car [m]"
#define ADTF_PROPERTY_MAP_ELEMENT_REMOVE_DISTANCE_DEFAULT_METER -2.

#define ADTF_PROPERTY_MAP_POS_UNKNOWN_TIMEOUT \
                    "Car postion unknown timeout [s]"
#define ADTF_PROPERTY_MAP_POS_UNKNOWN_TIMEOUT_DEFAULT_SEC 60

#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include "slim_pins.h"
#include "adtf_log_macros.h"
#include "map_element.hpp"
#include "map_helper.hpp"
#include "global_map.hpp"


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
class MapFilter : public adtf::cFilter {
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_MAP_FILTER, ADTF_MAP_FILTER_NAME,
                adtf::OBJCAT_DataFilter);

 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit MapFilter(const tChar* __info);

    /*! default destructor */
    virtual ~MapFilter();

 private:
    void InitMAP();
    void InitTimer();
    frAIburg::map::GlobalMap* map_; //map as a singelton

    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;
    tResult Run(tInt nActivationCode,
                const tVoid* pvUserData, tInt szUserDataSize,
                ucom::IException** __exception_ptr);

    /*! filter properties, set in adtf*/
    tFloat32 property_map_car_pos_update_interval_in_s_;
    void SetAllProperties(void);
    void GetAllStaticProperties(void);

    frAIburg::map::tMapData property_el_remove_dist_;
    frAIburg::map::tMapData property_repostion_jump_dist_;
    float property_repostion_jump_time_to_update_;
    frAIburg::map::tTimeMapStamp property_pos_lost_timeout_time_micro_s;
 protected:
    cCriticalSection update_mutex_;
    /// INPUT PINS
    /*! the input pin for template data */
    slim::InputPin car_pos_input_pin_;
    vector<tVoid*> rx_buff_;
    /*! current car postion change  with the input pin*/
    frAIburg::map::tMapCarPosition current_car_pos_;

    /** cFILTER STATE
     * MACHINE***************************************************/
    /*! Implements the default cFilter state machine call.  e*/
    virtual tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Implements the default cFilter state machine call.*/
    virtual tResult Shutdown(tInitStage stage,
                             ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.*/
    virtual tResult OnPinEvent(adtf::IPin* source, tInt event_code, tInt param1,
                               tInt param2, adtf::IMediaSample* media_sample);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    /** PIN METHOODS  *********************************************************/
    /*! this function creates all the input pins of filter*/
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult SetPinIDs();
    void CheckForLostCarPos(frAIburg::map::tTimeMapStamp current_time_micro_s);
    static const std::vector<frAIburg::map::MapElementType>* GetExcludeTypesProjecteDistanceRemove(void);
    bool first_car_pos_received_;
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_MAP_FILTER_H_

/*!
*@}
*/
