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

#include "light_controller.h"
#define EM_LOG LOG_INFO

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_TEMPLATE_FILTER_NAME, OID_ADTF_TEMPLATE_FILTER,
                   LightController);

LightController::LightController(const tChar* __info) : cFilter(__info) {}

// ____________________________________________________________________________
LightController::~LightController() {}

// ____________________________________________________________________________
tResult LightController::Init(tInitStage stage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

    tResult nResult = ERR_NOERROR;

    if (stage == StageFirst) {
        // in StageFirst you can create and register your static pins.
        nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");

    } else if (stage == StageNormal) {
        // In this stage you would do further initialisation and/or create your
        // dynamic pins.
    } else if (stage == StageGraphReady) {
        // query your pins about their media types and additional meta data.
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LightController::Shutdown(tInitStage stage, __exception) {
    // In each stage clean up everything that you initialized in the
    // corresponding stage during Init. Pins are an exception:
    // - The base class takes care of static pins that are members of this
    //   class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please
    //   see the demo_dynamicpin example for further reference.
    if (stage == StageGraphReady) {
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult LightController::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                    tInt param2, IMediaSample* media_sample) {
    // first check what kind of event it is
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(media_sample);
        // by comparing it to our member pin variable we can find out which
        // pin received the sample
        if (pin_in_light_command_.isSource(source)) {
            ProcessLightCommand(media_sample);
        }
    }
    RETURN_NOERROR;
}
// ____________________________________________________________________________
tResult LightController::CreateInputPins(__exception) {
    slim::register_pin_func func = &LightController::RegisterPin;
    RETURN_IF_FAILED(pin_in_light_command_.FirstStageCreate(
        this, func, "light_command", "tLightCommand"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LightController::CreateOutputPins(__exception) {
    slim::register_pin_func func = &LightController::RegisterPin;
    RETURN_IF_FAILED(pin_out_turn_left_.FirstStageCreate(
        this, func, "turn_left", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_turn_right_.FirstStageCreate(
        this, func, "turn_right", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_hazzard_lights_.FirstStageCreate(
        this, func, "hazzard_lights", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_brake_lights_.FirstStageCreate(
        this, func, "brake_lights", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_head_lights_.FirstStageCreate(
        this, func, "head_lights", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_reverse_lights_.FirstStageCreate(
        this, func, "reverse_lights", "tBoolSignalValue"));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult LightController::SetPinIDs() {
    const vector<string> ids = boost::assign::list_of("bValue");
    RETURN_IF_FAILED(pin_out_turn_left_.StageGraphReadySetIDOrder(ids));
    RETURN_IF_FAILED(pin_out_turn_right_.StageGraphReadySetIDOrder(ids));
    RETURN_IF_FAILED(pin_out_hazzard_lights_.StageGraphReadySetIDOrder(ids));
    RETURN_IF_FAILED(pin_out_brake_lights_.StageGraphReadySetIDOrder(ids));
    RETURN_IF_FAILED(pin_out_head_lights_.StageGraphReadySetIDOrder(ids));
    RETURN_IF_FAILED(pin_out_reverse_lights_.StageGraphReadySetIDOrder(ids));

    RETURN_NOERROR;
}

tResult LightController::ProcessLightCommand(IMediaSample* pMediaSample) {
    tLightCommand* pSampleData = NULL;
    if (IS_OK(pin_in_light_command_.ReadNoID_start(pMediaSample,
                                                   (const tVoid**)&pSampleData,
                                                   sizeof(tLightCommand)))) {
        light_type light_command_id =
            static_cast<light_type>(pSampleData->light_id);
        tBool switch_on_off = pSampleData->switch_bool;

        pin_in_light_command_.ReadNoID_end(pMediaSample,
                                           (const tVoid**)pSampleData);

        switch (light_command_id) {
            case ALL_LIGHTS: {
                LOG_INFO(
                    A_UTILS_NS::cString::Format("sending al head lights \
                are on:%i",
                                                switch_on_off));
                TransmitBoolValue(pin_out_turn_left_, switch_on_off);
                TransmitBoolValue(pin_out_turn_left_, switch_on_off);
                TransmitBoolValue(pin_out_turn_right_, switch_on_off);
                TransmitBoolValue(pin_out_head_lights_, switch_on_off);
                TransmitBoolValue(pin_out_reverse_lights_, switch_on_off);
                TransmitBoolValue(pin_out_brake_lights_, switch_on_off);
                break;
            }
            case TURN_SIGNAL_LEFT: {
                TransmitBoolValue(pin_out_turn_left_, switch_on_off);
                break;
            }
            case TURN_SIGNAL_RIGHT: {
                TransmitBoolValue(pin_out_turn_right_, switch_on_off);
                break;
            }
            case HAZZARD_LIGHTS: {
                TransmitBoolValue(pin_out_hazzard_lights_, switch_on_off);
                break;
            }
            case HEAD_LIGHTS: {
                TransmitBoolValue(pin_out_head_lights_, switch_on_off);
                break;
            }
            case REVERSE_LIGHTS: {
                TransmitBoolValue(pin_out_reverse_lights_, switch_on_off);
                break;
            }
            case BRAKE_LIGHTS: {
                TransmitBoolValue(pin_out_brake_lights_, switch_on_off);
                break;
            }
        }
    } else
        EM_LOG("error read light command failed!");

    RETURN_NOERROR;
}

tResult LightController::TransmitBoolValue(slim::OutputPin& oPin,
                                           tBool bValue) {
    const vector<const void*> vals2 =
        boost::assign::list_of((const void*)&bValue);
    if (IS_FAILED(oPin.Transmit(vals2, _clock->GetStreamTime()))) {
        LOG_ERROR("failed sending bool ");
    }
    RETURN_NOERROR;
}
