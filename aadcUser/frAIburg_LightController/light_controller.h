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
#ifndef AADCUSER_FRAIBURG_LIGHT_CONTROLLER_FILTER_H_
#define AADCUSER_FRAIBURG_LIGHT_CONTROLLER_FILTER_H_

#define ADTF_TEMPLATE_FILTER_NAME "frAIburg Light Controller"  // keep frAIburg
#define OID_ADTF_TEMPLATE_FILTER "adtf.example.light_controller"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "Controls the lights of the car"

#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include "slim_pins.h"
#include "nonpin_types.h"
#include "adtf_tools.h"
#include "aadc_structs.h"
#include "adtf_tools.h"




class LightController : public adtf::cFilter {
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, ADTF_TEMPLATE_FILTER_NAME,
                adtf::OBJCAT_DataFilter);

 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit LightController(const tChar* __info);

    /*! default destructor */
    virtual ~LightController();

 public:
    /// INPUT PINS
    /*! the input pin for template data */
    slim::InputPin pin_in_light_command_;
    /// OUTPUT PINS
    /*! the output pin for template data */
    slim::OutputPin pin_out_turn_left_;
    slim::OutputPin pin_out_turn_right_;
    slim::OutputPin pin_out_hazzard_lights_;
    slim::OutputPin pin_out_head_lights_;
    slim::OutputPin pin_out_brake_lights_;
    slim::OutputPin pin_out_reverse_lights_;


    /** cFILTER STATEMACHINE***************************************************/
    /*! Implements the default cFilter state machine call.  e*/
    virtual tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

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

    tResult ProcessLightCommand(IMediaSample* pMediaSample);

    tResult TransmitBoolValue(slim::OutputPin& oPin, tBool bValue);


    /** USER  *****************************************************************/
    /*! func is doing ....
    *   \param sample the new media sample
    *   \return Standard Result Code.
    */
    //tResult YourFunc(IMediaSample* sample);
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_LIGHT_CONTROLLER_FILTER_H_

/*!
*@}
*/
