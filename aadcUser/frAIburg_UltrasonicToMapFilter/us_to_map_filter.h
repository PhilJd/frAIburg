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
// us to map with dynamic pins
// - the pin name is used to load the us configuration:
//    use the base us addc arduino communication filter pin names
//



#ifndef AADCUSER_FRAIBURG_US_TO_MAP_TEMPLATE_FILTER_H_
#define AADCUSER_FRAIBURG_US_TO_MAP_TEMPLATE_FILTER_H_

#define ADTF_TEMPLATE_FILTER_NAME "frAIburg Ultrasonic To MapFilter"
#define OID_ADTF_TEMPLATE_FILTER "adtf.frAIburg_us_to_map_filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "AADC user template"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "TemplateFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "Ultrasonic input to the map, \
      keep the name of us pin addc arduino communication filter same to laod \
      correct calibration target"

//max value to add elements to the us buffers
// in the calllback the mean value is added to the map
#define ADTF_PROPERTY_US_MAX_VAL_TO_NAME "Max limit add data to the ring buffer in [m]"
#define ADTF_PROPERTY_US_MAX_VAL_TO_BUFFER 1.2 //detect spots on other side

#define ADTF_PROPERTY_DEBUG_ENABLED_NAME "Enbale filter debug modus"
#define ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT false
#define ADTF_PROPERTY_XML_CONFIG_FILE_NAME "frAIburg xml configuration"
#define ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT \
          "/home/aadc/ADTF/configuration_files/frAIburg_configuration.xml"

#define ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_NAME \
                      "set ring buffer size, if full map element is added"
#define ADTF_PROPERTY_US_CIRCULAR_BUFF_SIZE_DEFAULT 2

#define ADTF_PROPERTY_US_MAP_TIME_OF_LIFE_NAME \
                      "Map time of life, -1 to disable [s]"
#define ADTF_PROPERTY_US_MAP_TIME_OF_LIFE_DEFAULT 30

#define ADTF_PROPERTY_US_MAP_FUSE_MAX_DISTANCE_NAME \
                      "Map fuse max distance, -1 to disable [m]"
#define ADTF_PROPERTY_US_MAP_FUSE_MAX_DISTANCE_DEFAULT 0.05

#define ADTF_PROPERTY_US_MAP_FUSE_AREA_NAME \
                      "Map fuse max area difference, -1 to disable [m2]"
#define ADTF_PROPERTY_US_MAP_FUSE_AREA_DEFAULT -1

#include <cmath>
#include <boost/assign/list_of.hpp>
#include "stdafx.h"
#include "aadc_structs.h" //tSignalValue val for us input pins
#include "adtf_log_macros.h"
#include "slim_pins.h"
#include "circular_buffer_ultrasonic.hpp"
#include "global_map.hpp"
#include "map_element.hpp"
#include "map_helper.hpp"

// switch clang format off, to retain the markdown table
// clang-format off
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
* **Output Pins**
* | Pin            | Description           | MajorType           | SubType |
* | ---- ----------| --------------------- | ------------------- |
* ------------------- |
* |output_template | An example output pin | MEDIA_TYPE_TEMPLATE |
* MEDIA_TYPE_TEMPLATE |
*
*
* **Input Pins**
* | Pin            | Description          | MajorType           | SubType |
* | ---------------| -------------------- | ------------------- |
* ------------------- |
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
// clang-format off

typedef USCalibratedRingBuffer<float> tUSFilterBuffer;

// !  Template filter for AADC Teams
/*!
* This is an example filter for the AADC
*/
class USToMapFilter : public adtf::cFilter,
                      frAIburg::utils::CircularBufferEventListener {
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, ADTF_TEMPLATE_FILTER_NAME,
                adtf::OBJCAT_DataFilter);

 public:

    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit USToMapFilter(const tChar* __info);

    /*! default destructor */
    virtual ~USToMapFilter();

    /*! alll the dynamic pins are added here*/
    tResult Connect(IPin* pSource, const tChar* strDestName,
                    __exception = NULL);

    tResult PropertyChanged(const tChar* str_name);
 protected:
    /// INPUT PINS
    /*! the input pin for us data */
    slim::DynamicInputPin dynamic_pins_input_us_;
    slim::InputPin pin_in_enabled_us_to_map_;
    /** cFILTER STATEMACHINE***************************************************/
    /*! Implements the default cFilter state machine call.  e*/
    virtual tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    tResult Start(__exception = NULL);

    /*! Implements the default cFilter state machine call.*/
    virtual tResult Shutdown(tInitStage stage,
                             ucom::IException** __exception_ptr = NULL);

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
    bool property_debug_enabled_;
    bool property_use_us_pin_enabled_;
    float property_us_map_time_of_life_sec_;
    float property_us_map_fuse_max_distance_meter_;
    float property_us_map_fuse_area_diff_meter_;
    int property_us_parking_hits_occupied_;
    void GetAllStaticProperties(void);
    void SetAllProperties(void);


    /** USER  *****************************************************************/
    //buffer to clibrate the us signal
    //clibration is loaded based to the pin name
    std::vector<tUSFilterBuffer> us_calibrated_buffers_;
    bool pin_status_enabled_us_to_map_;

    void InitCalibrationBuffers();
    tResult ProcessDynamicdata(IPin* source, IMediaSample* media_sample,
                              slim::DynamicInputPin& dpins);
    //tResult YourFunc(IMediaSample* sample);

    //buffer callbacks for buffer events
    void EventBufferFilled(int id);// not used
    void EventCycleCompleat(int id);// add new map el to the map if full

    frAIburg::map::tSptrMapElement AddUSToMAP(float mean_dist,float trans_x,
                                              float trans_y,float rot,
                                             float field_of_view_angle_rad);
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_US_TO_MAP_TEMPLATE_FILTER_H_

/*!
*@}
*/
