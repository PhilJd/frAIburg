/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must
display the following acknowledgement: “This product includes software developed
by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 10:01:55#$ $Rev:: 63111   $
**********************************************************************/

// arduinofilter.cpp : Definiert die exportierten Funktionen für die
// DLL-Anwendung.
//
#include "stdafx.h"
#include "cWheelSpeedController.h"

#define WSC_PROP_PT1_TIMECONSTANT "PT1::TimeConstant"
#define WSC_PROP_PT1_GAIN "PT1::Gain"
#define WSC_PROP_PT1_OUTPUT_FACTOR "PT1::OutputFactor"
#define WSC_PROP_PT1_CORRECTION_FACTOR "PT1::Correction Factor"

#define WSC_PROP_PID_FEEDFORWARD "PID::Feedforward gain"
#define WSC_PROP_PID_KP "PID::Kp_value"
#define WSC_PROP_PID_KI "PID::Ki_value"
#define WSC_PROP_PID_KD "PID::Kd_value"
#define WSC_PROP_PID_SAMPLE_TIME "PID::Sample_Interval_[msec]"

// id and name definitions for signal registry (increase the id for new signals)
#define WSC_SIGREG_ID_WHEELSPEED_SETVALUE 0
#define WSC_SIGREG_NAME_WHEELSPEED_SETVALUE "wheel speed set value"
#define WSC_SIGREG_UNIT_WHEELSPEED_SETVALUE "m/sec2"

#define WSC_SIGREG_ID_WHEELSPEED_MEASVALUE 1
#define WSC_SIGREG_NAME_WHEELSPEED_MEASVALUE "wheel speed measured value"
#define WSC_SIGREG_UNIT_WHEELSPEED_MEASVALUE "m/sec2"

#define WSC_SIGREG_ID_WHEELSPEED_OUTPUTVALUE 2
#define WSC_SIGREG_NAME_WHEELSPEED_OUTPUTVALUE "wheel speed output value"
#define WSC_SIGREG_UNIT_WHEELSPEED_OUTPUTVALUE "m/sec2"

#define WSC_PROP_PID_MAXOUTPUT "PID::Maxiumum output"
#define WSC_PROP_PID_MINOUTPUT "PID::Minimum output"
#define WSC_PROP_DEBUG_MODE "Debug Mode"

ADTF_FILTER_PLUGIN("AADC Wheel Speed Controller", OID_ADTF_WHEELSPEEDCONTROLLER,
                   cWheelSpeedController)

cWheelSpeedController::cWheelSpeedController(const tChar* __info)
    : cFilter(__info),
      m_f64LastMeasuredError(0),
      m_f64SetPoint(0),
      m_lastSampleTime(0),
      m_f64LastSpeedValue(0) {
    SetPropertyFloat(WSC_PROP_PID_FEEDFORWARD, -1.0);
    SetPropertyBool(WSC_PROP_PID_FEEDFORWARD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_FEEDFORWARD NSSUBPROP_DESCRIPTION,
                   "The feedforward gain for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_KP, -30.0);
    SetPropertyBool(WSC_PROP_PID_KP NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KP NSSUBPROP_DESCRIPTION,
                   "The proportional factor Kp for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_KI, -110.);
    SetPropertyBool(WSC_PROP_PID_KI NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KI NSSUBPROP_DESCRIPTION,
                   "The integral factor Ki for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_KD, 1);
    SetPropertyBool(WSC_PROP_PID_KD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KD NSSUBPROP_DESCRIPTION,
                   "The differential factor Kd for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_SAMPLE_TIME, 0.025);
    SetPropertyBool(WSC_PROP_PID_SAMPLE_TIME NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_SAMPLE_TIME NSSUBPROP_DESCRIPTION,
                   "The sample interval in msec used by the PID controller");

    SetPropertyFloat(WSC_PROP_PID_MAXOUTPUT, 20);
    SetPropertyBool(WSC_PROP_PID_MAXOUTPUT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_MAXOUTPUT NSSUBPROP_DESCRIPTION,
                   "The maximum allowed output for the wheel speed controller "
                   "(speed in m/sec^2)");

    SetPropertyFloat(WSC_PROP_PID_MINOUTPUT, -30);
    SetPropertyBool(WSC_PROP_PID_MINOUTPUT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_MINOUTPUT NSSUBPROP_DESCRIPTION,
                   "The minimum allowed output for the wheel speed controller "
                   "(speed in m/sec^2)");

    SetPropertyBool(WSC_PROP_DEBUG_MODE, tFalse);
    SetPropertyStr(WSC_PROP_DEBUG_MODE NSSUBPROP_DESCRIPTION,
                   "If true debug infos are plotted to registry");

    SetPropertyFloat(WSC_PROP_PT1_OUTPUT_FACTOR, 1);
    SetPropertyBool(WSC_PROP_PT1_OUTPUT_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PT1_OUTPUT_FACTOR NSSUBPROP_DESCRIPTION,
                   "The factor to normalize the output value");

    SetPropertyFloat(WSC_PROP_PT1_TIMECONSTANT, 1.5);
    SetPropertyBool(WSC_PROP_PT1_TIMECONSTANT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PT1_TIMECONSTANT NSSUBPROP_DESCRIPTION,
                   "Time Constant for PT1 Controller");

    SetPropertyFloat(WSC_PROP_PT1_CORRECTION_FACTOR, 1.15);
    SetPropertyBool(WSC_PROP_PT1_CORRECTION_FACTOR NSSUBPROP_ISCHANGEABLE,
                    tTrue);
    SetPropertyStr(WSC_PROP_PT1_CORRECTION_FACTOR NSSUBPROP_DESCRIPTION,
                   "Correction factor for input set point");

    SetPropertyFloat(WSC_PROP_PT1_GAIN, 6);
    SetPropertyBool(WSC_PROP_PT1_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PT1_GAIN NSSUBPROP_DESCRIPTION,
                   "Gain for PT1 Controller");

    SetPropertyInt("Controller Typ", 2);
    SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT,
                   "1@P|2@PI|3@PID|4@PT1");

    // m_pISignalRegistry = NULL;
}

cWheelSpeedController::~cWheelSpeedController() {}

tResult cWheelSpeedController::PropertyChanged(const char* strProperty) {
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cWheelSpeedController::ReadProperties(const tChar* strPropertyName) {
    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PID_FEEDFORWARD)) {
        feedforward_gain_ = GetPropertyFloat(WSC_PROP_PID_FEEDFORWARD);
    }
    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PID_KP)) {
        m_f64PIDKp = GetPropertyFloat(WSC_PROP_PID_KP);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PID_KD)) {
        m_f64PIDKd = GetPropertyFloat(WSC_PROP_PID_KD);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PID_KI)) {
        m_f64PIDKi = GetPropertyFloat(WSC_PROP_PID_KI);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PID_SAMPLE_TIME)) {
        m_f64PIDSampleTime = GetPropertyFloat(WSC_PROP_PID_SAMPLE_TIME);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PT1_OUTPUT_FACTOR)) {
        m_f64PT1OutputFactor = GetPropertyFloat(WSC_PROP_PT1_OUTPUT_FACTOR);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PT1_GAIN)) {
        m_f64PT1Gain = GetPropertyFloat(WSC_PROP_PT1_GAIN);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PT1_TIMECONSTANT)) {
        m_f64PT1TimeConstant = GetPropertyFloat(WSC_PROP_PT1_TIMECONSTANT);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, "Controller Typ")) {
        m_i32ControllerMode = GetPropertyInt("Controller Typ");
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_DEBUG_MODE)) {
        m_bShowDebug = static_cast<tBool>(GetPropertyBool(WSC_PROP_DEBUG_MODE));
    }
    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PT1_CORRECTION_FACTOR)) {
        m_f64PT1CorrectionFactor =
            GetPropertyFloat(WSC_PROP_PT1_CORRECTION_FACTOR);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PID_MINOUTPUT)) {
        m_f64PIDMinimumOutput = GetPropertyFloat(WSC_PROP_PID_MINOUTPUT);
    }

    if (NULL == strPropertyName ||
        cString::IsEqual(strPropertyName, WSC_PROP_PID_MAXOUTPUT)) {
        m_f64PIDMaximumOutput = GetPropertyFloat(WSC_PROP_PID_MAXOUTPUT);
    }

    RETURN_NOERROR;
}

tResult cWheelSpeedController::CreateInputPins(__exception) {
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(
        OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
        (tVoid**)&pDescManager, __exception_ptr));

    // get media tayp
    tChar const* strDescSignalValue =
        pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue =
        new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,
                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set member media description
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(
        IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescMeasSpeed));
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(
        IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescSetSpeed));

    // create pins
    RETURN_IF_FAILED(m_oInputSetWheelSpeed.Create(
        "set_WheelSpeed", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetWheelSpeed));
    RETURN_IF_FAILED(
        m_oInputMeasWheelSpeed.Create("measured_wheelSpeed", pTypeSignalValue,
                                      static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputMeasWheelSpeed));

    RETURN_NOERROR;
}

tResult cWheelSpeedController::CreateOutputPins(__exception) {
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(
        OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
        (tVoid**)&pDescManager, __exception_ptr));

    // get media tayp
    tChar const* strDescSignalValue =
        pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue =
        new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,
                       IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set member media description
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(
        IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescActuator));

    // create pin
    RETURN_IF_FAILED(
        m_oOutputActuator.Create("actuator_output", pTypeSignalValue,
                                 static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputActuator));

    RETURN_NOERROR;
}

tResult cWheelSpeedController::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst) {
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    } else if (eStage == StageNormal) {
        ReadProperties(NULL);

        if (m_bShowDebug) {
            // create a kernel mutex
            THROW_IF_FAILED(m_oLock.Create(
                adtf_util::cString(OIGetInstanceName()) + ".active_signals"));

            // get the signal registry object
            RETURN_IF_FAILED(_runtime->GetObject(
                OID_ADTF_SIGNAL_REGISTRY, IID_ADTF_SIGNAL_REGISTRY_EXTENDED,
                (tVoid**)&m_pISignalRegistry, __exception_ptr));

            // register the provider at the registry
            RETURN_IF_FAILED(m_pISignalRegistry->RegisterProvider(
                this, OIGetInstanceName(), __exception_ptr));

            RETURN_IF_FAILED(m_pISignalRegistry->RegisterSignal(
                this, WSC_SIGREG_ID_WHEELSPEED_SETVALUE,
                WSC_SIGREG_NAME_WHEELSPEED_SETVALUE,
                WSC_SIGREG_UNIT_WHEELSPEED_SETVALUE,
                "the set value of the speed value", -35, +35.0,
                __exception_ptr));

            RETURN_IF_FAILED(m_pISignalRegistry->RegisterSignal(
                this, WSC_SIGREG_ID_WHEELSPEED_MEASVALUE,
                WSC_SIGREG_NAME_WHEELSPEED_MEASVALUE,
                WSC_SIGREG_UNIT_WHEELSPEED_MEASVALUE,
                "the measured value of the wheel speed controller", -50.0,
                +50.0, __exception_ptr));

            RETURN_IF_FAILED(m_pISignalRegistry->RegisterSignal(
                this, WSC_SIGREG_ID_WHEELSPEED_OUTPUTVALUE,
                WSC_SIGREG_NAME_WHEELSPEED_OUTPUTVALUE,
                WSC_SIGREG_UNIT_WHEELSPEED_OUTPUTVALUE,
                "the output value of the wheel speed controller", -50.0, +50.0,
                __exception_ptr));
        }
    } else if (eStage == StageGraphReady) {
        // set the flags which indicate if the media descriptions strings were
        // set
        m_bInputMeasWheelSpeedGetID = tFalse;
        m_bInputSetWheelSpeedGetID = tFalse;
        m_bInputActuatorGetID = tFalse;

        m_f64LastOutput = 0.0f;
    }

    RETURN_NOERROR;
}

tResult cWheelSpeedController::Start(__exception) {
    m_f64LastOutput = 0;
    m_f64LastMeasuredError = 0;
    m_f64SetPoint = 0;
    m_lastSampleTime = 0;
    m_f64LastSpeedValue = 0;
    m_f64accumulatedVariable = 0;

    return cFilter::Start(__exception_ptr);
}

tResult cWheelSpeedController::Stop(__exception) {
    return cFilter::Stop(__exception_ptr);
}

tResult cWheelSpeedController::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        ucom::cObjectPtr<ISignalRegistry> pSignalRegistry;
        if (IS_OK(_runtime->GetObject(OID_ADTF_SIGNAL_REGISTRY,
                                      IID_ADTF_SIGNAL_REGISTRY,
                                      (tVoid**)&pSignalRegistry))) {
            // Unregister the provider
            pSignalRegistry->UnregisterSignalProvider(this);
        }
        m_oActive.clear();

        m_oLock.Release();
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cWheelSpeedController::OnPinEvent(IPin* pSource, tInt nEventCode,
                                          tInt nParam1, tInt nParam2,
                                          IMediaSample* pMediaSample) {
    __synchronized_obj(m_critSecOnPinEvent);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived &&
        pMediaSample != NULL) {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &m_oInputMeasWheelSpeed) {
            // the controller here works only with 10 Hz therefore only every
            // fourth value is used
            /*static tUInt64 ui64_modCounter=0;
            ui64_modCounter++;
            if (ui64_modCounter%2!=0) RETURN_NOERROR;*/

            // write values with zero
            tFloat32 f32Value = 0;
            tUInt32 Ui32TimeStamp = 0;
            {
                // focus for sample write lock
                // read data from the media sample with the coder of the
                // descriptor
                __adtf_sample_read_lock_mediadescription(m_pDescMeasSpeed,
                                                         pMediaSample, pCoder);

                if (!m_bInputMeasWheelSpeedGetID) {
                    pCoder->GetID("f32Value", m_buIDMeasSpeedF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp",
                                  m_buIDMeasSpeedArduinoTimestamp);
                    m_bInputMeasWheelSpeedGetID = tTrue;
                }
                // get values from media sample
                pCoder->Get(m_buIDMeasSpeedF32Value, (tVoid*)&f32Value);
                pCoder->Get(m_buIDMeasSpeedArduinoTimestamp,
                            (tVoid*)&Ui32TimeStamp);
            }

            // write to member variable

            m_f64MeasuredVariable = f32Value;

            // calculation
            // if speed = 0 is requested output is immediately set to zero
            if (m_f64SetPoint == 0.) {
                m_f64LastOutput = 0;
                m_f64accumulatedVariable = 0;
                m_f64LastMeasuredError = 0;
            } else
                m_f64LastOutput =
                    getControllerValue(f32Value) * m_f64PT1OutputFactor;

            // create new media sample
            cObjectPtr<IMediaSample> pNewMediaSample;
            AllocMediaSample((tVoid**)&pNewMediaSample);

            // allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pDescActuator->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pNewMediaSample->AllocBuffer(nSize);
            tFloat32 outputValue = static_cast<tFloat32>(m_f64LastOutput);
            {
                // focus for sample write lock
                // read data from the media sample with the coder of the
                // descriptor
                __adtf_sample_write_lock_mediadescription(
                    m_pDescActuator, pNewMediaSample, pCoderOut);

                if (!m_bInputActuatorGetID) {
                    pCoderOut->GetID("f32Value", m_buIDActuatorF32Value);
                    pCoderOut->GetID("ui32ArduinoTimestamp",
                                     m_buIDActuatorArduinoTimestamp);
                    m_bInputActuatorGetID = tTrue;
                }
                // get values from media sample
                pCoderOut->Set(m_buIDActuatorF32Value, (tVoid*)&outputValue);
            }

            // transmit media sample over output pin
            RETURN_IF_FAILED(pNewMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oOutputActuator.Transmit(pNewMediaSample));
        } else if (pSource == &m_oInputSetWheelSpeed) {
            {
                // write values with zero
                tFloat32 f32Value = 0;
                tUInt32 ui32TimeStamp = 0;

                // focus for sample write lock
                __adtf_sample_read_lock_mediadescription(m_pDescSetSpeed,
                                                         pMediaSample, pCoder);

                if (!m_bInputSetWheelSpeedGetID) {
                    pCoder->GetID("f32Value", m_buIDSetSpeedF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp",
                                  m_buIDSetSpeedArduinoTimestamp);
                    m_bInputSetWheelSpeedGetID = tTrue;
                }

                // read data from the media sample with the coder of the
                // descriptor
                // get values from media sample
                pCoder->Get(m_buIDSetSpeedF32Value, (tVoid*)&f32Value);
                pCoder->Get(m_buIDSetSpeedArduinoTimestamp,
                            (tVoid*)&ui32TimeStamp);

                // write to member variable
                m_f64SetPoint = static_cast<tFloat64>(f32Value);
                if (m_i32ControllerMode == 4)
                    m_f64SetPoint = m_f64SetPoint * m_f64PT1CorrectionFactor;
            }
        }

        if (m_bShowDebug) {
            SendSignalData();
        }
    }
    RETURN_NOERROR;
}
tResult cWheelSpeedController::SendSignalData() {
    __synchronized_kernel(m_oLock);
    tTimeStamp tsStreamTime = _clock->GetStreamTime();

    for (tActiveSignals::iterator oSignal = m_oActive.begin();
         oSignal != m_oActive.end(); ++oSignal) {
        if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_MEASVALUE) {
            tSignalValue sValue;
            sValue.nRawValue = 0;
            sValue.nTimeStamp = tsStreamTime;
            sValue.strTextValue = 0;
            sValue.f64Value = m_f64MeasuredVariable;
            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
        } else if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_SETVALUE) {
            tSignalValue sValue;
            sValue.nRawValue = 0;
            sValue.nTimeStamp = tsStreamTime;
            sValue.strTextValue = 0;
            sValue.f64Value = m_f64SetPoint;

            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
        }

        else if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_OUTPUTVALUE) {
            tSignalValue sValue;
            sValue.nRawValue = 0;
            sValue.nTimeStamp = tsStreamTime;
            sValue.strTextValue = 0;
            sValue.f64Value = m_f64LastOutput;

            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
        }
    }
    RETURN_NOERROR;
}

tFloat64 cWheelSpeedController::getControllerValue(
    tFloat64 i_f64MeasuredValue) {
    tFloat64 u_feedforward = 0.;
    // i_f64MeasuredValue = (i_f64MeasuredValue +  m_f64LastSpeedValue) /2.0;
    if (m_f64SetPoint > 0.)
        i_f64MeasuredValue = std::max(1. * i_f64MeasuredValue, 0.);
    else if (m_f64SetPoint < 0.)
        i_f64MeasuredValue = std::min(1. * i_f64MeasuredValue, 0.);
    // m_f64LastSpeedValue = i_f64MeasuredValue;
    tFloat64 m_f64accumulatedVariable_old = 0.;

    tFloat f64Result = 0;

    // the three controller algorithms
    if (m_i32ControllerMode == 1) {
        // m_lastSampleTime = GetTime();

        // algorithm:
        // y = Kp * e
        // error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);

        f64Result = m_f64PIDKp * f64Error;
    } else if (m_i32ControllerMode == 2)  // PI- Regler
    {
        // m_lastSampleTime = GetTime();
        u_feedforward = feedforward_gain_ * m_f64SetPoint;
        // algorithm:
        // esum = esum + e
        // y = Kp * e + Ki * Ta * esum
        // error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:
        m_f64accumulatedVariable_old = m_f64accumulatedVariable;
        m_f64accumulatedVariable += (f64Error * m_f64PIDSampleTime);

        f64Result = m_f64PIDKp * f64Error +
                    (m_f64PIDKi * m_f64accumulatedVariable) + u_feedforward;

    } else if (m_i32ControllerMode == 3) {
        m_lastSampleTime = GetTime();
        tFloat64 f64SampleTime = m_f64PIDSampleTime;

        // algorithm:
        // esum = esum + e
        // y = Kp * e + Ki * Ta * esum + Kd * (e – ealt)/Ta
        // ealt = e

        // error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:
        m_f64accumulatedVariable_old = m_f64accumulatedVariable;
        m_f64accumulatedVariable += f64Error * m_f64PIDSampleTime;

        f64Result =
            m_f64PIDKp * f64Error + (m_f64PIDKi * m_f64accumulatedVariable) +
            m_f64PIDKd * (f64Error - m_f64LastMeasuredError) / f64SampleTime;

        m_f64LastMeasuredError = f64Error;
    } else if (m_i32ControllerMode == 4) {
        /*********************************
        * PT 1 discrete algorithm
        *
        *               Tau
        *       In +    ---  * LastOut
        *             Tsample
        * Out = ---------------------
        *               Tau
        *       1 +     ---
        *             Tsample
        *
        *                           Tau
        * here with     PT1Gain =   ---
        *                         Tsample
        *
        *                  T
        * y(k) = y(k-1) + --- ( v * e(k)  - y(k-1))
        *                  T1
        *
        * e(k):  input
        * y(k-1) : last output
        * v : gain
        * T/T1: time constant
        ********************************************/
        f64Result = m_f64LastOutput +
                    m_f64PT1TimeConstant *
                        (m_f64PT1Gain * (m_f64SetPoint - i_f64MeasuredValue) -
                         m_f64LastOutput);
    }
    // checking for minimum and maximum limits
    if (f64Result > m_f64PIDMaximumOutput) {
        f64Result = m_f64PIDMaximumOutput;
        m_f64accumulatedVariable =
            m_f64accumulatedVariable_old;  // F: Integral cut-off!
    } else if (f64Result < m_f64PIDMinimumOutput) {
        f64Result = m_f64PIDMinimumOutput;
        m_f64accumulatedVariable = m_f64accumulatedVariable_old;
    }

    m_f64LastOutput = f64Result;

    return f64Result;
}

tTimeStamp cWheelSpeedController::GetTime() {
    return (_clock != NULL) ? _clock->GetTime() : cSystem::GetTime();
}

/**
*   Returns the current value of a Signal.
*/
tResult cWheelSpeedController::GetSignalValue(tSignalID nSignalID,
                                              tSignalValue* pValue) {
    if (nSignalID == WSC_SIGREG_ID_WHEELSPEED_MEASVALUE) {
        pValue->f64Value = m_f64MeasuredVariable;
        tTimeStamp nStreamTime = _clock->GetStreamTime();
        pValue->nRawValue = 0;
        pValue->nTimeStamp = nStreamTime;
        pValue->strTextValue = 0;
    } else if (nSignalID == WSC_SIGREG_ID_WHEELSPEED_SETVALUE) {
        pValue->f64Value = m_f64SetPoint;
        tTimeStamp nStreamTime = _clock->GetStreamTime();
        pValue->nRawValue = 0;
        pValue->nTimeStamp = nStreamTime;
        pValue->strTextValue = 0;
    } else if (nSignalID == WSC_SIGREG_ID_WHEELSPEED_OUTPUTVALUE) {
        pValue->f64Value = m_f64LastOutput;
        tTimeStamp nStreamTime = _clock->GetStreamTime();
        pValue->nRawValue = 0;
        pValue->nTimeStamp = nStreamTime;
        pValue->strTextValue = 0;
    } else {
        RETURN_ERROR(ERR_NOT_FOUND);
    }

    RETURN_NOERROR;
}

/**
*   Activates a signal.
*   Activated signals send their values to the Signal Registry Service.
*/
tResult cWheelSpeedController::ActivateSignalEvents(tSignalID nSignalID,
                                                    tTimeStamp nUpdateRate) {
    if (nSignalID == WSC_SIGREG_ID_WHEELSPEED_MEASVALUE ||
        nSignalID == WSC_SIGREG_ID_WHEELSPEED_SETVALUE ||
        nSignalID == WSC_SIGREG_ID_WHEELSPEED_OUTPUTVALUE) {
        __synchronized_kernel(m_oLock);
        m_oActive.insert(nSignalID);
    } else {
        RETURN_ERROR(ERR_NOT_FOUND);
    }

    RETURN_NOERROR;
}

/**
*   Deactivates a signal.
*/
tResult cWheelSpeedController::DeactivateSignalEvents(tSignalID nSignalID) {
    __synchronized_kernel(m_oLock);
    m_oActive.erase(nSignalID);
    RETURN_NOERROR;
}

tResult cWheelSpeedController::GetInterface(const tChar* idInterface,
                                            tVoid** ppvObject) {
    if (idmatch(idInterface, IID_ADTF_SIGNAL_PROVIDER)) {
        *ppvObject = static_cast<ISignalProvider*>(this);
    } else {
        return cFilter::GetInterface(idInterface, ppvObject);
    }

    Ref();

    RETURN_NOERROR;
}

tUInt cWheelSpeedController::Ref() { return cFilter::Ref(); }

tUInt cWheelSpeedController::Unref() { return cFilter::Unref(); }

tVoid cWheelSpeedController::Destroy() { delete this; }
