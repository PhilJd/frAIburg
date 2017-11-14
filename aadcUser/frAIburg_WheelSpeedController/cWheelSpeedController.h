/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/


#ifndef _WHEELSPEEDCONTROLLER_H_
#define _WHEELSPEEDCONTROLLER_H_

#include "stdafx.h"

#define OID_ADTF_WHEELSPEEDCONTROLLER "adtf.frAIburg_wheelSpeedController"
/*! @defgroup WheelSpeedController Wheel Speed Controller
*  @{
*
* This filter implements a controller to set the wheelspeed of the vehicle with a P/PI/PID or PT1 algorithm. The input pin measured_wheelSpeed  has to be connected to the output pin of the Converter Wheels Filter and the desired wheel speed has to be set to the input pin set_WheelSpeed.
* The controller parameters have to be adapted to each individual car. The default values for the PT1 controller are good start for the controller but maybe have to be adapted to the individual team car. There are different methods to get the correct parameter, please refer to other literature.
* The output pin actuator_output have to be connected to a Calibration XML which maps the speed in m/〖sec〗^2  to servo angle of steering controller.
*
* This values are saved in the Sample XML SpeedController.xml in the folder configuration_files and can be used at the beginning.
*
*  \image html WheelSpeedController.PNG "Plugin Wheel Speed Controller"
*
* \b Dependencies \n
* This plugin needs the following libraries:
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Controller_Typ<td><td>PI
* <tr><td>Debug Mode<td>If true debug infos are plotted to registry<td>False
* <tr><td>PID::Kd_value <td>The differential factor Kd for the PID Controller<td>1
* <tr><td>PID::Ki_value <td>The integral factor Ki for the PID Controller<td>0.001
* <tr><td>PID::Kp_value <td>The proportional factor Kp for the PID Controller<td>1.1
* <tr><td>PID::Maximum output <td>The maximum allowed output for the wheel speed controller (speed in m/sec^2)<td>5
* <tr><td>PID::Minimum output <td>The minimum allowed output for the wheel speed controller (speed in m/sec^2)<td>-5
* <tr><td>PID::Sample_Interval_[msec] <td>The sample interval in msec used by the PID controller<td>0.025
* <tr><td>PT1::Correction Factor <td>Correction factor for input set point<td>1.15
* <tr><td>PT1::Gain <td>Gain for PT1 Controller<td>6
* <tr><td>PT1::OutputFactor <td>The factor to normalize the output value<td>1
* <tr><td>PT1::TimeConstant <td>Time Constant for PT1 Controller<td>1.5
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>actuator_output<td><td>tSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>set_WheelSpeed<td><td>tSignalValue
* <tr><td>measured_wheelSpeed<td><td>tSignalValue
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/algorithms/AADC_WheelSpeedController
* <tr><td>Filename<td>aadc_wheelSpeedController.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! the main class for the wheel speed controller plugin */
class cWheelSpeedController : public adtf::cFilter,
    public adtf::ISignalProvider
{
    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_WHEELSPEEDCONTROLLER, "frAIburg Wheel Speed Controller", OBJCAT_DataFilter, "Wheel Speed Controller", 1, 1, 0, "");

    /*! the input pin for the measured value */
    cInputPin m_oInputMeasWheelSpeed;

    /*! the input pin for the set point value */
    cInputPin m_oInputSetWheelSpeed;

    /*! the output pin for the manipulated value */
    cOutputPin m_oOutputActuator;

public:
    /*! constructor for  class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cWheelSpeedController(const tChar* __info);

    /*! Destructor. */
    virtual ~cWheelSpeedController();

protected: // overwrites cFilter

    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \return Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:
    /*! creates all the output Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! creates all the input Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! called if one of the properties is changed
    * \param strProperty the changed property
    * \return standard adtf error code
    */
    tResult PropertyChanged(const char* strProperty);

    /*! reads the properties and save to member variables
    * \param strPropertyName name of the property changed
    * \return standard adtf error code
    */
    tResult ReadProperties(const tChar* strPropertyName);

    /*! calculates the manipulated value for the given values, it uses the setpoint in m_setPoint
    * \param i_f64MeasuredValue    the measuredValue
    * \return the controller output for wheel speed
    */
    tFloat64 getControllerValue(tFloat64 i_f64MeasuredValue);

    /*!
     * Gets the time.
     *
     * \return  The time streamtime in milliseconds
     */
    tTimeStamp GetTime();

    /*! holds the last measuredValue */
    tFloat64 m_f64MeasuredVariable;
    /*! holds the last measured error */
    tFloat64 m_f64LastMeasuredError;
    /*! holds the last setpoint */
    tFloat64 m_f64SetPoint;
    /*! holds the last output */
    tFloat64 m_f64LastOutput;
    /*! holds the last sample time */
    tTimeStamp m_lastSampleTime;
    /*! holds the accumulatedVariable for the controller */
    tFloat64 m_f64accumulatedVariable;

    /*! media description for the input pin set speed */
    cObjectPtr<IMediaTypeDescription> m_pDescSetSpeed;
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDSetSpeedF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDSetSpeedArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bInputSetWheelSpeedGetID;


    /*! media description for the input pin measured speed */
    cObjectPtr<IMediaTypeDescription> m_pDescMeasSpeed;
    /*! the id for the f32value of the media description for input pin for the measured speed */
    tBufferID m_buIDMeasSpeedF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the measured speed */
    tBufferID m_buIDMeasSpeedArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bInputMeasWheelSpeedGetID;

    /*! the critical section for the on pin events */
    cCriticalSection m_critSecOnPinEvent;

    /*! media description for the output pin with speed */
    cObjectPtr<IMediaTypeDescription> m_pDescActuator;
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDActuatorF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDActuatorArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bInputActuatorGetID;

    // PID-Controller values
    //
    tFloat64 feedforward_gain_;
    /*! proportional factor for PID Controller */
    tFloat64    m_f64PIDKp;
    /*! integral factor for PID Controller */
    tFloat64    m_f64PIDKi;
    /*! differential factor for PID Controller */
    tFloat64    m_f64PIDKd;
    /*! the sampletime for the pid controller */
    tFloat64 m_f64PIDSampleTime;
    /*! the minimum output value for the controller */
    tFloat64 m_f64PIDMinimumOutput;
    /*! the maximum output value for the controller */
    tFloat64 m_f64PIDMaximumOutput;

    // PT1-Controller values
    /*! tau value for PT1 controller */
    tFloat64    m_f64PT1Tau;
    /*! sample time for PT1 Controller */
    tFloat64    m_f64PT1Sampletime;
    /*! gain factor for PT1 controller */
    tFloat64    m_f64PT1Gain;
    /*! time constant for pt1 controller */
    tFloat64 m_f64PT1TimeConstant;
    /*! input factor for PT1 */
    tFloat64 m_f64PT1OutputFactor;
    /*! input factor for PT1 */
    tFloat64 m_f64PT1InputErrorFactor;

    /*! the set point is multiplied with this factor, otherwise the set point is not reached by the controller. */
    tFloat64 m_f64PT1CorrectionFactor;

    /*! holds the last speed value */
    tFloat64 m_f64LastSpeedValue;

    /*! defines whether PID or PT1 is used */
    tInt32 m_i32ControllerMode;


public: // implements ISignalProvider
    /*!
    *  Returns the current signal value.
    *
    *  \param  nSignalID [in]  ID of signal
    *  \param  pValue    [out] Value of signal
    *
    *  \result Standard Result Code
    */
    virtual tResult GetSignalValue(tSignalID nSignalID, tSignalValue* pValue);

    /*!
    *  Activates events for specific signal.
    *  After this call is made the signal listener will receive notifications when a new
    *  signal value was received.
    *
    *  \param  nSignalID   [in] ID of signal
    *  \param  nUpdateRate [in] Update rate in microseconds
    *
    *  \result Standard Result Code
    */
    virtual tResult ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate = 0);


    /*!
    *  Deletes a link between signal provider and signal listener.
    *  After this call is made the signal listener will receive no further notifications.
    *
    *  \param  nSignalID [in] ID of signal
    *
    *  \result Standard Result Code
    */
    virtual tResult DeactivateSignalEvents(tSignalID nSignalID);

    /*!
     * this functions sends the signal data to the registry.
     *
     * \return  Standard Result Code
     */
    tResult SendSignalData();



public: // implements IObject

    /*!
     * Gets an interface.
     *
     * \param           idInterface The identifier interface.
     * \param [in,out]  ppvObject   If non-null, the ppv object.
     *
     * \return  The interface.
     */
    tResult GetInterface(const tChar* idInterface, tVoid** ppvObject);

    /*!
     * Gets the reference. ucom::IObject::Ref
     *
     * \return  A tUInt reference
     */
    tUInt Ref();

    /*!
     * Gets the unref. ucom::IObject::Unref
     *
     * \return  the unreferenced id
     */
    tUInt Unref();

    /*!
     * Destroys this object. ucom::IObject::Destroy
     *
     * \return  A tVoid.
     */
    tVoid Destroy();

    /*! members for signal registry. */
    typedef std::set<tSignalID> tActiveSignals;

    /*! The signal registry object */
    ucom::cObjectPtr<ISignalRegistryExtended> m_pISignalRegistry;
    /*! The lock for getting signal values*/
    cKernelMutex                              m_oLock;
    /*! The active signals for signal registry */
    tActiveSignals                            m_oActive;

    /*! indicates if the internal signals are also plotted to registry. */
    tBool       m_bShowDebug;
};

#endif /** @} */ // end of group // _cWheelSpeedController_H_

