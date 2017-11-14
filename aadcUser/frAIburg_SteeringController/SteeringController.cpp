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


Author:: Felix Plum
**********************************************************************/

/* BEFORE IMPLEMENTATION:
  include description/customtypes */

#include "SteeringController.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("Steering Controller", OID_ADTF_STEERINGCONTROLLER_FILTER,
                   SteeringController);

SteeringController::SteeringController(const tChar* __info) : cFilter(__info) {
    this->is_output_set_ = 0;
    this->u_feedforward_ = 0;
    this->K_P_ = 0;
    this->K_I_ = 0;  // not implemented yet
}

// ____________________________________________________________________________
SteeringController::~SteeringController() {}

tResult SteeringController::CreateInputPins(__exception) {
    slim::register_pin_func func = &SteeringController::RegisterPin;
    RETURN_IF_FAILED(
        pin_in_goalpoint_.FirstStageCreate(this, func, "goal_point", "tPoint"));

    slim::register_pin_func fun2 = &SteeringController::RegisterPin;
    RETURN_IF_FAILED(pin_in_feedforward_.FirstStageCreate(
        this, fun2, "angle_feedfwd", "tSignalValue"));

    RETURN_NOERROR;
}

tResult SteeringController::CreateOutputPins(__exception) {
    slim::register_pin_func func = &SteeringController::RegisterPin;
    RETURN_IF_FAILED(pin_out_angle_.FirstStageCreate(this, func, "servo_angle",
                                                     "tSignalValue"));
    slim::register_pin_func func2 = &SteeringController::RegisterPin;
    RETURN_IF_FAILED(pin_out_curvature_.FirstStageCreate(
        this, func2, "curvature", "tSignalValue"));

    RETURN_NOERROR;
}
// ____________________________________________________________________________
tResult SteeringController::Init(tInitStage e_stage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(e_stage, __exception_ptr))
    tResult nResult;
    // in StageFirst you can create and register your static pins.
    if (e_stage == StageFirst) {
        // LOG_ERROR(A_UTILS_NS::cString::Format("test %f", testpoint.y_value));

        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

    } else if (e_stage == StageNormal) {
        // In this stage you would do further initialisation and/or create your
        // dynamic pins. Please take a look at the demo_dynamicpin example for
        // further reference.
    } else if (e_stage == StageGraphReady) {
        // All pin connections have been established in this stage so you can
        // query your pins about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further
        // reference.
        nResult = SetPinIDs();

        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");
        lookahead_point_.x = 1;  // FOR TESTING. REMOVE ASAP!
        lookahead_point_.y = 0;
    }
    RETURN_NOERROR;
}

tResult SteeringController::SetPinIDs() {
    vector<string> ids = boost::assign::list_of("f32Value");
    RETURN_IF_FAILED(pin_out_curvature_.StageGraphReadySetIDOrder(ids));

    vector<string> ids2 = boost::assign::list_of("f32Value");
    RETURN_IF_FAILED(pin_out_angle_.StageGraphReadySetIDOrder(ids2));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult SteeringController::Shutdown(tInitStage e_stage, __exception) {
    // In each stage clean up everything that you initiaized in the
    // corresponding stage during Init. Pins are an exception:
    // - The base class takes care of static pins that are members of this
    //   class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please
    //   see the demo_dynamicpin example for further reference.
    if (e_stage == StageGraphReady) {
    } else if (e_stage == StageNormal) {
    } else if (e_stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(e_stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult SteeringController::OnPinEvent(IPin* source, tInt n_event_code,
                                       tInt nparam1, tInt nparam2,
                                       IMediaSample* media_sample) {
    // first check what kind of event it is

    if (n_event_code == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(media_sample);
        // by comparing it to our member pin variable we can find out which
        // pin received the sample

        // first pass curvature to calibration filter to get feedforward control
        if (pin_in_goalpoint_.isSource(source)) {
            tPoint* sample_data = NULL;
            if (IS_OK(pin_in_goalpoint_.ReadNoID_start(
                    media_sample, (const tVoid**)&sample_data,
                    sizeof(tPoint)))) {
                lookahead_point_ = *sample_data;
                pin_in_goalpoint_.ReadNoID_end(media_sample,
                                               (const tVoid**)sample_data);
                TransmitCurvature();
            }
            // this happens immediately after; now all values are combined.
            // BEWARE
            // ALGEBRAIC LOOPS!
        } else if (pin_in_feedforward_.isSource(source)) {
            tSignalValue* sample_data = NULL;
            if (IS_OK(pin_in_feedforward_.ReadNoID_start(
                    media_sample, (const tVoid**)&sample_data,
                    sizeof(tSignalValue)))) {
                u_feedforward_ = sample_data->f32Value;
                // printf("SController: Val to Arduino is %f\n",
                // u_feedforward_);
                TransmitSteeringAngle();
                pin_in_feedforward_.ReadNoID_end(media_sample,
                                                 (const tVoid**)sample_data);
            }
        }
    }
    RETURN_NOERROR;
}

tResult SteeringController::TransmitSteeringAngle() {
    tFloat32 steering_angle_out = 0;
    // tFloat32 anchor_lookahead_angle = atan2(lookahead_point_.y,
    // lookahead_point_.x-ANCHOR_XOFFSET_);
    // tFloat32 anchor_lookahead_dist =
    // sqrt(pow(lookahead_point_.x+ANCHOR_XOFFSET_, 2)+pow(lookahead_point_.y,
    // 2));
    // tFloat32 steering_angle_out = -57.3*atan2(WHEELBASE_ *
    // sin(anchor_lookahead_angle),
    //             0.5 * anchor_lookahead_dist + ANCHOR_XOFFSET_ *
    //             cos(anchor_lookahead_angle));
    // LOG_INFO(A_UTILS_NS::cString::Format("SC: Lookahead angle %f",
    // 57*anchor_lookahead_angle));

    // feedfoward (= reach lookahead) + distance deviation
    steering_angle_out = u_feedforward_ + K_P_ * lookahead_point_.y;
    vector<const tVoid*> vals =
        boost::assign::list_of((const tVoid*)&steering_angle_out);
    pin_out_angle_.Transmit(vals, _clock->GetStreamTime());
    RETURN_NOERROR;
}

tResult SteeringController::TransmitCurvature() {
    // static int cnt = 0;
    // calculate curv. = 1/R so that anchor point would move in circle in order
    // to  reach lookahead point

    tFloat32 curvature_out =
        lookahead_point_.y /
        (2 * (pow(lookahead_point_.x, 2.0) + pow(lookahead_point_.y, 2.0)));
    // printf("SteeringController: goal x y is %f %f, curvature %f\n", x_anchor,
    // lookahead_point_.y, curvature_out);
    // cnt++;
    // if (cnt==40) {
    //   printf("Steering controller aims at (%f, %f) with %f m from bumper\n",
    //   lookahead_point_.x, lookahead_point_.y, -CAR_ANCHOR_OFFSET_);
    //   printf("Steering controller wants to drive R = %f\n", 1/curvature_out);
    //   cnt = 0;
    // }

    // this is converted into servo angle by lookup/calibration filter
    vector<const tVoid*> vals =
        boost::assign::list_of((const tVoid*)&curvature_out);
    pin_out_curvature_.Transmit(vals, _clock->GetStreamTime());
    RETURN_NOERROR;
}
