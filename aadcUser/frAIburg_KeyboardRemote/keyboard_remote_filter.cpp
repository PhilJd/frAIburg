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

#include "keyboard_remote_filter.h"

using namespace frAIburg::map;
/// Create filter shell

ADTF_FILTER_PLUGIN(ADTF_FILTER_K_REMOTE_FILTER_NAME, OID_ADTF_K_REMOTE_FILTER,
                   KeyboardRemoteFilter);

KeyboardRemoteFilter::KeyboardRemoteFilter(const tChar* __info)
    : QObject(), cBaseQtFilter(__info) {
    keyborad_enabled_ = false;
    transmit_speed_ = SPEED_CONTROLLER_DEFAULT_VALUE;
    transmit_steering_ = STEERING_ANGEL_DEFAULT_VALUE;
    go_steering_enabled_left_ = false;
    go_steering_enabled_right_ = false;
    go_speed_enabled_front_ = false;
    go_speed_enabled_back_ = false;
}

// ____________________________________________________________________________
KeyboardRemoteFilter::~KeyboardRemoteFilter() {}

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::Init(tInitStage stage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cBaseQtFilter::Init(stage, __exception_ptr))

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
tResult KeyboardRemoteFilter::Shutdown(tInitStage stage, __exception) {
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
    return cBaseQtFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::Start(__exception) {
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));
    LOG_INFO("KeyboardRemoteFilter: enable controlls in the UI");
    LOG_INFO("KeyboardRemoteFilter: arrow keys: controll the car");
    LOG_INFO("KeyboardRemoteFilter: f keys: increment speed");
    LOG_INFO("KeyboardRemoteFilter: d keys: decrement speed");
    LOG_INFO("KeyboardRemoteFilter: s keys: increment steering");
    LOG_INFO("KeyboardRemoteFilter: a keys: decrement steering");
    LOG_INFO("KeyboardRemoteFilter: l keys: toggle and send light cmd");
    LOG_INFO("KeyboardRemoteFilter: p keys: save current map");
    LOG_INFO(
        "KeyboardRemoteFilter: o keys: add current car postion \
            as debug point to the map");
    keyborad_enabled_ = false;
    transmit_speed_ = SPEED_CONTROLLER_DEFAULT_VALUE;
    transmit_steering_ = STEERING_ANGEL_DEFAULT_VALUE;
    go_steering_enabled_left_ = false;
    go_steering_enabled_right_ = false;
    go_speed_enabled_front_ = false;
    go_speed_enabled_back_ = false;
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::Stop(__exception) {
    return cBaseQtFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::Run(
    tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize,
    ucom::IException** __exception_ptr /* =NULL */) {
    transmit_speed_ = SPEED_CONTROLLER_DEFAULT_VALUE;
    transmit_steering_ = STEERING_ANGEL_DEFAULT_VALUE;
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize,
                              __exception_ptr);
}

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::CreateInputPins(__exception) { RETURN_NOERROR; }

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::CreateOutputPins(__exception) {
    slim::register_pin_func func = &KeyboardRemoteFilter::RegisterPin;

    pin_out_angle_.FirstStageCreate(this, func, "SteeringController",
                                    "tSignalValue");
    pin_out_speed_.FirstStageCreate(this, func, "SpeedController",
                                    "tSignalValue");
    pin_out_goal_point_.FirstStageCreate(this, func, "GoalPoint",
                                         "tGoalSpeedDistance");
    pin_out_light_cmd_.FirstStageCreate(this, func, "light_command",
                                        "tLightCommand");
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::SetPinIDs() {
    vector<string> ids = boost::assign::list_of("f32Value");
    RETURN_IF_FAILED(pin_out_angle_.StageGraphReadySetIDOrder(ids));
    RETURN_IF_FAILED(pin_out_speed_.StageGraphReadySetIDOrder(ids));
    vector<string> id3s = boost::assign::list_of("v")("s");
    RETURN_IF_FAILED(pin_out_goal_point_.StageGraphReadySetIDOrder(id3s));
    vector<string> ids_light =
        boost::assign::list_of("light_id")("switch_bool");
    RETURN_IF_FAILED(pin_out_light_cmd_.StageGraphReadySetIDOrder(ids_light));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tHandle KeyboardRemoteFilter::CreateView() {
    // create the widget
    QWidget* pWidget = reinterpret_cast<QWidget*>(m_pViewport->VP_GetWindow());
    widget_ = new DisplayWidget(pWidget);

    // make the qt connections
    // to bool b
    connect(widget_->m_btSendValueFalse, SIGNAL(clicked()), this,
            SLOT(OnTransmitValueFalse()));
    connect(widget_->m_btSendValueTrue, SIGNAL(clicked()), this,
            SLOT(OnTransmitValueTrue()));
    // key events
    connect(widget_, SIGNAL(keyReceived(int)), this, SLOT(keycmd(int)));
    // press
    connect(widget_, SIGNAL(sendpressUp()), this, SLOT(KeyDiveGo()));
    connect(widget_, SIGNAL(sendpressDown()), this, SLOT(KeyDiveBackGo()));
    connect(widget_, SIGNAL(sendpressLeft()), this, SLOT(KeyLeftGo()));
    connect(widget_, SIGNAL(sendpressRight()), this, SLOT(KeyRightGo()));
    // realase
    connect(widget_, SIGNAL(sendreleaseUp()), this, SLOT(KeyDriveStop()));
    connect(widget_, SIGNAL(sendreleaseDown()), this, SLOT(KeyBackStop()));
    connect(widget_, SIGNAL(sendreleaseLeft()), this, SLOT(KeyLeftStop()));
    connect(widget_, SIGNAL(sendreleaseRight()), this, SLOT(KeyRightStop()));
    connect(widget_, SIGNAL(sendoutFocusWidget()), this,
            SLOT(OnTransmiReset()));

    return (tHandle)widget_;
}

// ____________________________________________________________________________
tResult KeyboardRemoteFilter::ReleaseView() {
    // delete the widget if present
    if (widget_ != NULL) {
        delete widget_;
        widget_ = NULL;
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void KeyboardRemoteFilter::keycmd(int key) {
    // LOG_INFO(A_UTILS_NS::cString::Format("Key: %i", key));
    // change speed and angel on key press
    // if the arrow is pressed down we send the updtaed singal
    if (keyborad_enabled_) {
        if (key == KEY_INC_SPEED) {
            if (transmit_speed_ + SPEED_CONTROLLER_INCREMENT_VALUE <=
                SPEED_CONTROLLER_MAX_VALUE)
                transmit_speed_ += SPEED_CONTROLLER_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("speed: %f", transmit_speed_));
            OnTransmitValuesDiveGo();
            OnTransmitValuesDiveBackGo();

        } else if (key == KEY_DECREMENRT_SPEED) {
            if (transmit_speed_ - SPEED_CONTROLLER_INCREMENT_VALUE >=
                SPEED_CONTROLLER_MIN_VALUE)
                transmit_speed_ -= SPEED_CONTROLLER_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("speed: %f", transmit_speed_));
            OnTransmitValuesDiveGo();
            OnTransmitValuesDiveBackGo();
        } else if (key == KEY_INC_ANGEL) {
            if (transmit_steering_ + STEERING_ANGEL_INCREMENT_VALUE <=
                STEERING_ANGEL_MAX_VALUE)
                transmit_steering_ += STEERING_ANGEL_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("steering: %f",
                                                 transmit_steering_));
            OnTransmitValuesLeftGo();
            OnTransmitValuesRightGo();
        } else if (key == KEY_DECREMENRT_ANGEL) {
            if (transmit_steering_ - STEERING_ANGEL_INCREMENT_VALUE >=
                STEERING_ANGEL_MIN_VALUE)
                transmit_steering_ -= STEERING_ANGEL_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("steering: %f",
                                                 transmit_steering_));
            OnTransmitValuesLeftGo();
            OnTransmitValuesRightGo();
        } else if (key == KEY_MAP_SAVE) {
            GlobalMap* map = frAIburg::map::getInstance();
            map->Print("keyborad print");

        } else if (key == KEY_MAP_ADD_DEBUG_POINT) {
            AddCarPosAsDebugPointToMap();
        } else if (key == KEY_LIGHT_CMD) {
            LOG_INFO("KeyboardRemoteFilter: sending light command");
            static tBool bValue = false;
            bValue = !bValue;
            tInt16 light_id = (tInt16)ALL_LIGHTS;
            const vector<const void*> vals2 = boost::assign::list_of(
                (const void*)&light_id)((const void*)&bValue);
            if (IS_FAILED(pin_out_light_cmd_.Transmit(
                    vals2, _clock->GetStreamTime()))) {
                LOG_ERROR("failed sending light val ");
            }
        }
    }
}
// ____________________________________________________________________________
void KeyboardRemoteFilter::AddCarPosAsDebugPointToMap() {
    // add debug point at current car postion
    GlobalMap* map = frAIburg::map::getInstance();
    tMapCarPosition car_pos;
    if (map->GetGlobalCarPosition(&car_pos)) {
        std::vector<tMapPoint> v1 =
            boost::assign::list_of(tMapPoint(car_pos.x, car_pos.y));
        tSptrMapElement el(
            new MapElement(DEBUG_POINT, v1, 0U, car_pos.heading));
        // set the local assignged pos to the gloable frame
        car_pos.y = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_X;
        car_pos.x = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_Y;
        car_pos.heading = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_HEADING;
        el->LocalToGlobal(car_pos);
        // add element to map with the gloale poly set allready
        map->AddElement(el);
        LOG_INFO_PRINTF(
            "map debug point car pos"
            "%s , delete after 10s",
            el->ToString().c_str());
        el->EnableTimeOfLife(10e6);

    } else {
        LOG_INFO("KeyboardRemoteFilter: unable to get map car pos");
    }
}

// ____________________________________________________________________________
void KeyboardRemoteFilter::KeyDiveGo() {
    go_speed_enabled_front_ = true;
    OnTransmitValuesDiveGo();
}
void KeyboardRemoteFilter::KeyDriveStop() {
    go_speed_enabled_front_ = false;
    OnTransmitValuesDriveStop();
}

void KeyboardRemoteFilter::KeyLeftGo() {
    go_steering_enabled_left_ = true;
    OnTransmitValuesLeftGo();
}
void KeyboardRemoteFilter::KeyLeftStop() {
    go_steering_enabled_left_ = false;
    OnTransmitValuesLeftStop();
}

void KeyboardRemoteFilter::KeyRightGo() {
    go_steering_enabled_right_ = true;
    OnTransmitValuesRightGo();
}

void KeyboardRemoteFilter::KeyRightStop() {
    go_steering_enabled_right_ = false;
    OnTransmitValuesRightStop();
}

void KeyboardRemoteFilter::KeyDiveBackGo() {
    go_speed_enabled_back_ = true;
    OnTransmitValuesDiveBackGo();
}
void KeyboardRemoteFilter::KeyBackStop() {
    go_speed_enabled_back_ = false;
    OnTransmitValuesDriveBackStop();
}

// Front
void KeyboardRemoteFilter::OnTransmitValuesDiveGo() {
    if (keyborad_enabled_ && go_speed_enabled_front_) {
        LOG_DUMP("DEBUG GO");
        tFloat32 speed = -transmit_speed_;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&speed);
        pin_out_speed_.Transmit(val, _clock->GetStreamTime());
        // transmit goalpoint
        tFloat32 v = GOAL_GO_V_VALUE;
        tFloat32 s = GOAL_GO_S_VALUE;
        vector<const tVoid*> val_goal =
            boost::assign::list_of((const tVoid*)&v)((const tVoid*)&s);
        pin_out_goal_point_.Transmit(val_goal, _clock->GetStreamTime());
    } else {
        // LOG_INFO("Keyborad controll disabled");
    }
}

void KeyboardRemoteFilter::OnTransmitValuesDriveStop() {
    if (keyborad_enabled_) {
        tFloat32 speed = 0;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&speed);
        pin_out_speed_.Transmit(val, _clock->GetStreamTime());
        // transmit goalpoint stop
        tFloat32 v = GOAL_STOP_V_VALUE;
        tFloat32 s = GOAL_STOP_S_VALUE;
        vector<const tVoid*> val_goal =
            boost::assign::list_of((const tVoid*)&v)((const tVoid*)&s);
        pin_out_goal_point_.Transmit(val_goal, _clock->GetStreamTime());
    }
}

// BACK
void KeyboardRemoteFilter::OnTransmitValuesDiveBackGo() {
    if (keyborad_enabled_ && go_speed_enabled_back_) {
        tFloat32 speed = transmit_speed_;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&speed);
        pin_out_speed_.Transmit(val, _clock->GetStreamTime());
        // transmit goalpoint
        tFloat32 v = -GOAL_GO_V_VALUE;
        tFloat32 s = GOAL_GO_S_VALUE;
        vector<const tVoid*> val_goal =
            boost::assign::list_of((const tVoid*)&v)((const tVoid*)&s);
        pin_out_goal_point_.Transmit(val_goal, _clock->GetStreamTime());
    } else {
        // LOG_INFO("Keyborad controll disabled");
    }
}

void KeyboardRemoteFilter::OnTransmitValuesDriveBackStop() {
    if (keyborad_enabled_) {
        tFloat32 speed = 0;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&speed);
        pin_out_speed_.Transmit(val, _clock->GetStreamTime());
        // transmit goalpoint stop
        tFloat32 v = GOAL_STOP_V_VALUE;
        tFloat32 s = GOAL_STOP_S_VALUE;
        vector<const tVoid*> val_goal =
            boost::assign::list_of((const tVoid*)&v)((const tVoid*)&s);
        pin_out_goal_point_.Transmit(val_goal, _clock->GetStreamTime());
    }
}

// LEFT
void KeyboardRemoteFilter::OnTransmitValuesLeftGo() {
    if (keyborad_enabled_ && go_steering_enabled_left_) {
        tFloat32 angle = -transmit_steering_;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&angle);
        pin_out_angle_.Transmit(val, _clock->GetStreamTime());
    } else {
        // LOG_INFO("Keyborad controll disabled");
    }
}
void KeyboardRemoteFilter::OnTransmitValuesLeftStop() {
    if (keyborad_enabled_) {
        tFloat32 angle = 0;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&angle);
        pin_out_angle_.Transmit(val, _clock->GetStreamTime());
    }
}

// Right
void KeyboardRemoteFilter::OnTransmitValuesRightGo() {
    if (keyborad_enabled_ && go_steering_enabled_right_) {
        tFloat32 angle = transmit_steering_;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&angle);
        pin_out_angle_.Transmit(val, _clock->GetStreamTime());
    } else {
        // LOG_INFO("Keyborad controll disabled");
    }
}

void KeyboardRemoteFilter::OnTransmitValuesRightStop() {
    if (keyborad_enabled_) {
        tFloat32 angle = 0;
        vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&angle);
        pin_out_angle_.Transmit(val, _clock->GetStreamTime());
    }
}

void KeyboardRemoteFilter::OnTransmiReset() {
    keyborad_enabled_ = false;
    go_steering_enabled_left_ = false;
    go_steering_enabled_right_ = false;
    go_speed_enabled_front_ = false;
    go_speed_enabled_back_ = false;

    // send angle zero
    tFloat32 angle = 0;
    vector<const tVoid*> val = boost::assign::list_of((const tVoid*)&angle);
    pin_out_angle_.Transmit(val, _clock->GetStreamTime());

    // send speed zero
    tFloat32 speed = 0;
    vector<const tVoid*> valspeed =
        boost::assign::list_of((const tVoid*)&speed);
    pin_out_speed_.Transmit(valspeed, _clock->GetStreamTime());

    // transmit goalpoint stop
    tFloat32 v = GOAL_STOP_V_VALUE;
    tFloat32 s = GOAL_STOP_S_VALUE;
    vector<const tVoid*> val_goal =
        boost::assign::list_of((const tVoid*)&v)((const tVoid*)&s);
    pin_out_goal_point_.Transmit(val_goal, _clock->GetStreamTime());
}

// bool b
void KeyboardRemoteFilter::OnTransmitValueTrue() { keyborad_enabled_ = true; }

void KeyboardRemoteFilter::OnTransmitValueFalse() { keyborad_enabled_ = false; }
