/*****************************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must
   display the following acknowledgement: “This product includes software
   developed by the Audi AG and its contributors for Audi Autonomous Driving
   Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef AADCUSER_FRAIBURG_STEERINGCONTROLLER_STEERINGCONTROLLER_H_
#define AADCUSER_FRAIBURG_STEERINGCONTROLLER_STEERINGCONTROLLER_H_

#define OID_ADTF_STEERINGCONTROLLER_FILTER "adtf.user.steeringcontroller"

#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include <cmath>
#include "aadc_structs.h"
#include "slim_pins.h"
#include "customtypes.h"
#include "nonpin_types.h"

using namespace adtf;  // NOLINT

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
* | ---- ----------| -------------------- | ------------------- |
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

// !  Template filter for AADC Teams
/*!
* This is an example filter for the AADC
*/
class SteeringController : public adtf::cFilter {
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_STEERINGCONTROLLER_FILTER, "Steering Controller",
                adtf::OBJCAT_DataFilter);

 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit SteeringController(const tChar* __info);

    /*! default destructor */
    virtual ~SteeringController();

    tResult TransmitSteeringAngle();
    tResult TransmitCurvature();

 protected:
    /*! the input pin for template data */
    slim::InputPin pin_in_goalpoint_;
    slim::InputPin pin_in_feedforward_;
    /*! the output pin for template data */
    slim::OutputPin pin_out_angle_;
    slim::OutputPin pin_out_curvature_;

    tResult Init(tInitStage e_stage, ucom::IException** __exception_ptr);

    tResult Shutdown(tInitStage e_stage,
                     ucom::IException** __exception_ptr = NULL);

    tResult OnPinEvent(adtf::IPin* p_source, tInt n_event_code, tInt nparam1,
                       tInt nparam2, adtf::IMediaSample* pmedia_sample);

    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);
    tResult SetPinIDs();

    tTimeStamp last_time_;

    tBool is_output_set_;

    tPoint lookahead_point_;
    tFloat32 u_feedforward_;

    tFloat32 LOOKAHEADDIST_;   // small values for exact tracking, high for low
                               // pass characteristic
   
    tFloat32 K_P_;
    tFloat32 K_I_;
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_STEERINGCONTROLLER_STEERINGCONTROLLER_H_

/*!
*@}
*/
