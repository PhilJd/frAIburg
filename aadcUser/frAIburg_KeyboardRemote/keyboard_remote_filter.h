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
#ifndef AADCUSER_FRAIBURG_K_REMOTE_FILTER_H_
#define AADCUSER_FRAIBURG_K_REMOTE_FILTER_H_

#define OID_ADTF_K_REMOTE_FILTER "adtf.example.keyboard_remote_filter"
#define ADTF_FILTER_K_REMOTE_FILTER_NAME "frAIburg Keyboard remote Filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "AADC user template"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "KeyboardRemoteFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small Template Filter."

#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include "slim_pins.h"
#include "adtf_log_macros.h"
#include "displaywidget.h"
#include "map_element.hpp"
#include "global_map.hpp"
#include "nonpin_types.h"

#include <QtCore/QtCore>
#include <QtGui/QtGui>
#ifdef WIN32
#ifdef _DEBUG
#pragma comment(lib, "qtmaind.lib")
#pragma comment(lib, "qtcored4.lib")
#pragma comment(lib, "qtguid4.lib")
#else  // _DEBUG
#pragma comment(lib, "qtmain.lib")
#pragma comment(lib, "qtcore4.lib")
#pragma comment(lib, "qtgui4.lib")
#endif
#endif

#define SPEED_CONTROLLER_DEFAULT_VALUE 14.
#define SPEED_CONTROLLER_MAX_VALUE 40.
#define SPEED_CONTROLLER_MIN_VALUE 0.
#define SPEED_CONTROLLER_INCREMENT_VALUE 2.

#define STEERING_ANGEL_DEFAULT_VALUE 85.
#define STEERING_ANGEL_MAX_VALUE 100.
#define STEERING_ANGEL_MIN_VALUE 0.
#define STEERING_ANGEL_INCREMENT_VALUE 5.

#define GOAL_GO_V_VALUE 3.
#define GOAL_GO_S_VALUE 5.

#define GOAL_STOP_V_VALUE 0.
#define GOAL_STOP_S_VALUE 0.5

// key for speed ang angle change
#define KEY_INC_SPEED 70         // f
#define KEY_DECREMENRT_SPEED 68  // d
#define KEY_INC_ANGEL 83         // s
#define KEY_DECREMENRT_ANGEL 65  // a
#define KEY_MAP_SAVE 80  // p
#define KEY_MAP_ADD_DEBUG_POINT 79  // o
#define KEY_LIGHT_CMD 76  // l

/*! @defgroup RemoteKeyboardFilter
*  @{
*
*
* Filter to controll aadc car with the arrow keys.
* to use enable the key first with the ui button.
* only if the qt ui window is in focus the keyboard can be used.
* if in adtf the workspace windows cann be scrolled with the arrow keys,
* click beside the enable button after enabling the keyboard
* when the button is pressed down the speed or strearing cmd is send ones.
* as the key is released the speed or stearing is set so zero
*
*
*
* **Dependencies** \n
* This plugin needs the following libraries: qt
*
*
* **Output Pins**
* | Smart Pin      | Description                        | MajorType |
* | ---- ----------| ---------------------              | ------------------- |
* |pin_out_speed_ |speed output for key up down         | tSignalValue |
* |               | connect to arduino commmunication,  | |
* |               | speed controller                    | |
* |pin_out_angle_ | angle output for key left right     | tSignalValue |
* |               | connect to arduino commmunication,  | |
* |               | steering controller                 | |
* |pin_out_goal_point_ |speed output for key up down    | tGoalSpeedDistance |
*
*
* **Plugin Details**
* | | |
* |-|-|
* | Path    | src/aadcUser/frAIburg_keyboard_remote|
* | Filename|frAIburg_keyboard_remote.plb          |
* | Version | 1.0.0                                |
*
* _____________________________________________________________________________
*  Markdown examples --- Delete everything below here!
* _____________________________________________________________________________
*
*/

// !  Template filter for AADC Teams
/*!
* This is an example filter for the AADC
*/
class KeyboardRemoteFilter : public QObject, public cBaseQtFilter {
    /*! set the filter ID and the version */

    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_K_REMOTE_FILTER,
                                ADTF_FILTER_K_REMOTE_FILTER_NAME,
                                OBJCAT_Auxiliary, "Keyboard Remote Filter", 1,
                                0, 0, "");
    Q_OBJECT
 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit KeyboardRemoteFilter(const tChar* __info);

    /*! default destructor */
    virtual ~KeyboardRemoteFilter();

 protected:
    /// OUTPUT PINS
    /*! the output pin for template data */
    slim::OutputPin pin_out_speed_;
    slim::OutputPin pin_out_angle_;
    slim::OutputPin pin_out_goal_point_;
    slim::OutputPin pin_out_light_cmd_;
    /*! The displayed widget*/
    DisplayWidget* widget_;

    bool keyborad_enabled_;
    bool go_speed_enabled_front_;
    bool go_speed_enabled_back_;
    bool go_steering_enabled_left_;
    bool go_steering_enabled_right_;
    tFloat32 transmit_speed_;
    tFloat32 transmit_steering_;

    /** cFILTER STATE
     * MACHINE***************************************************/

    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage stage,
                     ucom::IException** __exception_ptr = NULL);

    virtual tResult Start(ucom::IException** __exception_ptr = NULL);
    virtual tResult Stop(ucom::IException** __exception_ptr = NULL);
    tResult Run(tInt nActivationCode, const tVoid* pvUserData,
                tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

    /** PIN METHOODS  *********************************************************/
    /*! this function creates all the input pins of filter*/
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult SetPinIDs();

    /** USERER METHOODS *******************************************************/
 protected:  // Implement cBaseQtFilter
             /*! Creates the widget instance
             * \result Returns a standard result code.
             */
    tHandle CreateView();

    /*! Destroys the widget instance
    * \result Returns a standard result code.
    */
    tResult ReleaseView();

 public slots:  // func for button and key presses
    void OnTransmitValueTrue();

    void OnTransmitValueFalse();

    void keycmd(int k);

    // binded to the car wigdet arrow keys
    void KeyDiveGo();
    void KeyDriveStop();

    void KeyLeftGo();
    void KeyLeftStop();

    void KeyRightGo();
    void KeyRightStop();

    void KeyDiveBackGo();
    void KeyBackStop();
    void OnTransmiReset();
    
 private:
    // function to send speed
    void OnTransmitValuesDiveGo();
    void OnTransmitValuesDriveStop();

    void OnTransmitValuesLeftGo();
    void OnTransmitValuesLeftStop();

    void OnTransmitValuesRightGo();
    void OnTransmitValuesRightStop();

    void OnTransmitValuesDiveBackGo();
    void OnTransmitValuesDriveBackStop();



    void AddCarPosAsDebugPointToMap();
    /**
     * ************************************************************************/
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_K_REMOTE_FILTER_H_

/*!
*@}
*/
