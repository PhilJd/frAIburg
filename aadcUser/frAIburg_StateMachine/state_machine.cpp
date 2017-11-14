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

#include "state_machine.h"
#include <boost/thread.hpp>

#define IF_DEBUG_LOG_PRINTF(...)     \
    if (poperty_debug_enabled_) {    \
        LOG_INFO_PRINTF(__VA_ARGS__) \
    }

namespace sc = boost::statechart;
namespace mpl = boost::mpl;
using namespace frAIburg::map;

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_FILTER_STATE_MACHINE_NAME, OID_ADTF_STATE_MACHINE,
                   StateMachine);

StateMachine::StateMachine(const tChar* __info) : cFilter(__info) {
    state_machine_.initiate();
    map_ = NULL;
    maneuver_requested_ = false;  // initializing request maneuver to false (no
                                  // maneuver is requested yet)
    SIGN_CROSSING_THRESHOLD_ = 1.2;
    PEDESTRIAN_AT_ZEBRA_THRESHOLD_ = 0.9;
    PEDESTRIAN_ON_ZEBRA_THRESHOLD_ = 0.4;
    CAR_ON_CROSSING_THRESHOLD_ = 0.5;
    CAR_AT_CROSSING_THRESHOLD_ = 1.2;
    current_maneuver_id_ = 0;
    jury_ready_request_ = false;
    current_highest_priority_ = std::numeric_limits<int>::max();
    event_change_locked_ = true;
    car_detected_ = false;
    SetAllProperties();
    send_behavior_thread_is_active = false;
}

// ____________________________________________________________________________
StateMachine::~StateMachine() {}
// ____________________________________________________________________________
//------------------------------------------------------------------------------------------------------


// ____________________________________________________________________________
void StateMachine::AddEventToPQ(tSptrMapElement el, event_type ev_type) {
    event new_event;
    new_event.event_id = el->GetID();
    new_event.event_priority = SetElementPriority(el);
    new_event.event_distance = el->GetDistanceToCar();
    new_event.event_type = ev_type;
    priority_queue_mutex_.lock();
    event_pq_.push(new_event);
    // ProcessEventQueue();
    current_highest_priority_ = SetElementPriority(el);
    priority_queue_mutex_.unlock();
    // LOG_INFO_PRINTF(cString::Format("Event type id %d was added to PQ",
    // ev_type));
    return;
}

// ____________________________________________________________________________
void StateMachine::EventQueueClear() {
    boost::mutex::scoped_lock scoped_lock(priority_queue_mutex_);
    event_pq_ = priority_queue<event>();
}

// ____________________________________________________________________________
bool StateMachine::EventQueueEmpty() {
    boost::mutex::scoped_lock scoped_lock(priority_queue_mutex_);
    return event_pq_.empty();
}

// ____________________________________________________________________________
event StateMachine::EventQueueTop() {
    boost::mutex::scoped_lock scoped_lock(priority_queue_mutex_);
    return event_pq_.top();
}

// ____________________________________________________________________________
void StateMachine::ProcessEvent(
        const boost::statechart::event_base& ev_type) {
    boost::mutex::scoped_lock scoped_lock(statemachine_mutex_);
    state_machine_.process_event(ev_type);
}

// ____________________________________________________________________________
size_t StateMachine::GetActiveStateHash() {
    boost::mutex::scoped_lock scoped_lock(statemachine_mutex_);
    return typeid(*state_machine_.state_begin()).hash_code();
}

// ____________________________________________________________________________
bool StateMachine::PedestrianOnCrossing(tSptrMapElement el) {
    tSptrMapElement zebra_element = map_->GetElement(current_event_.event_id);
	if(!zebra_element) return false;
    tMapPoint zebra_pos = zebra_element->GetGlobalPolyCenter();
    int child_id = GetObjectInRange(zebra_pos.get<0>(), zebra_pos.get<1>(), zebra_element->GetGlobalOrientation()+PI, PEDESTRIAN_CHILD, 0.3, -0.3, 0.5, -0.5);
    int adult_id = GetObjectInRange(zebra_pos.get<0>(), zebra_pos.get<1>(), zebra_element->GetGlobalOrientation()+PI, PEDESTRIAN_ADULT, 0.3, -0.3, 0.5, -0.5);

    if (child_id == -1 && adult_id == -1) return false;
    return true;
}

bool StateMachine::IsParkingOrChildAhead() {
    std::vector<tSptrMapElement> detected_elements;
            const std::vector<MapElementType> gettypes = boost::assign::list_of(
                LM_STREET_PARKING_SPOT)(PEDESTRIAN_CHILD);
            map_->GetAllElementsWithTypes(gettypes, detected_elements);
    tMapCarPosition pos;
    map_->GetGlobalCarPosition(&pos);
    BOOST_FOREACH (tSptrMapElement& s, detected_elements) {
        if(s->GetType() == LM_STREET_PARKING_SPOT){
            float projected_x = s->ProjectedDistanceToCar();
            float distance = s->GetDistanceToCar();
            float projected_y = sqrt(pow(distance,2)-pow(projected_x,2));


            float angle_diff = (pos.heading - s->GetGlobalOrientation());
            if(angle_diff > PI) {
                angle_diff -= 2*PI;
            } else if(angle_diff < -PI) {
                angle_diff += 2*PI;
            }
            bool is_in_angle_range  = fabs(angle_diff) > 0.4*PI && fabs(angle_diff) < 0.6*PI;
        // return (angle_diff >= threshold_max) ? true : false;
            if(projected_x > -0.5 && projected_x<2.5 && projected_y < 1.5 && projected_y > -1.0 && is_in_angle_range) {
                return true;
            }
        }
        if(s->GetType() == PEDESTRIAN_CHILD){
            float projected_x = s->ProjectedDistanceToCar();
            float distance = s->GetDistanceToCar();
            float projected_y = sqrt(pow(distance,2)-pow(projected_x,2));
            if(projected_x<2.5 && projected_y < 1) return true;
        }


    }
    return false;
}

bool StateMachine::PedestrianCrossingAllowedToGo(tSptrMapElement el)
{
    tMapPoint crossing_pos = el->GetGlobalPolyCenter();
    float global_orientation = el->GetGlobalOrientation()+PI;
    std::vector<tSptrMapElement> detected_pedestrian_list;
    const std::vector<MapElementType> get_types =
        boost::assign::list_of(PEDESTRIAN_ADULT)(PEDESTRIAN_CHILD);
    map_->GetAllElementsWithTypes(get_types, detected_pedestrian_list);
    BOOST_FOREACH (tSptrMapElement& s, detected_pedestrian_list) {
        tMapPoint element_center_point = s->GetGlobalPolyCenter();
        float dx = (element_center_point.get<0>() - crossing_pos.get<0>());
        float dy = (element_center_point.get<1>() - crossing_pos.get<1>());
        float pos_local_x =
            (dx)*cos(-(global_orientation)) - (dy)*sin(-(global_orientation));
        float pos_local_y =
            (dx)*sin(-(global_orientation)) + (dy)*cos(-(global_orientation));
    LOG_INFO_PRINTF("Car distance X,Y: %f,%f", pos_local_x, pos_local_y);
        if (pos_local_x < 0 && pos_local_x > -1  && pos_local_y > -1 && pos_local_y < 1) return false;
        if (pos_local_x < 1 && pos_local_x > -1  && pos_local_y > -1 && pos_local_y < 0 && (GetCurrentManeuver() == "right")) return false;
        if (pos_local_x < 1 && pos_local_x > 0  && pos_local_y > -1 && pos_local_y < 1 && (GetCurrentManeuver() == "straight")) return false;
        if (pos_local_x < 1 && pos_local_x > -1  && pos_local_y > 0 && pos_local_y < 1 && (GetCurrentManeuver() == "left")) return false;
    }
    return true;
}

bool StateMachine::PedestrianAtCrossingRight(tSptrMapElement el) {
    tSptrMapElement zebra_element = map_->GetElement(current_event_.event_id);
	if(!zebra_element) return false;
    tMapPoint zebra_pos = zebra_element->GetGlobalPolyCenter();
    int child_id = GetObjectInRange(zebra_pos.get<0>(), zebra_pos.get<1>(), zebra_element->GetGlobalOrientation()+PI, PEDESTRIAN_CHILD, 0.4, -0.4, -0.5, -0.9);
    int adult_id = GetObjectInRange(zebra_pos.get<0>(), zebra_pos.get<1>(), zebra_element->GetGlobalOrientation()+PI, PEDESTRIAN_ADULT, 0.4, -0.4, -0.5, -0.9);

    if(adult_id != -1){
		return true;
        tSptrMapElement pedestrian = map_->GetElement(adult_id);
        MapElementPedestrian* el_pedestrian = dynamic_cast<MapElementPedestrian*>(pedestrian.get());
        if(el_pedestrian && (el_pedestrian->GetPedestrianOrientation() == LEFT))
        {
            return true;
        }
    }
    if(child_id != -1){
		return true;
        tSptrMapElement pedestrian = map_->GetElement(child_id);
        MapElementPedestrian* el_pedestrian = dynamic_cast<MapElementPedestrian*>(pedestrian.get());
        if(el_pedestrian && (el_pedestrian->GetPedestrianOrientation() == LEFT)) return true;
    }
    return false;
    //TODO: test if orientation works, otherwise revert to this old version down that only checks position
//    if (child_id == -1 && adult_id == -1) return false;
//    return true;
}

bool StateMachine::PedestrianAtCrossingLeft(tSptrMapElement el) {
    tSptrMapElement zebra_element = map_->GetElement(current_event_.event_id);
	if(!zebra_element) return false;
    tMapPoint zebra_pos = zebra_element->GetGlobalPolyCenter();
    int child_id = GetObjectInRange(zebra_pos.get<0>(), zebra_pos.get<1>(), zebra_element->GetGlobalOrientation()+PI, PEDESTRIAN_CHILD, 0.4, -0.4, 0.9, 0.5);
    int adult_id = GetObjectInRange(zebra_pos.get<0>(), zebra_pos.get<1>(), zebra_element->GetGlobalOrientation()+PI, PEDESTRIAN_ADULT, 0.4, -0.4, 0.9, 0.5);

    if(adult_id != -1){
		return true;
        tSptrMapElement pedestrian = map_->GetElement(adult_id);
        MapElementPedestrian* el_pedestrian = dynamic_cast<MapElementPedestrian*>(pedestrian.get());
        if(el_pedestrian && (el_pedestrian->GetPedestrianOrientation() == RIGHT))
        {
            return true;
        }
    }
    if(child_id != -1){
		return true;
        tSptrMapElement pedestrian = map_->GetElement(child_id);
        MapElementPedestrian* el_pedestrian = dynamic_cast<MapElementPedestrian*>(pedestrian.get());
        if(el_pedestrian && (el_pedestrian->GetPedestrianOrientation() == RIGHT)) return true;
    }
    return false;
    //TODO: test if orientation works, otherwise revert to this old version down that only checks position

//    if (child_id == -1 && adult_id == -1) return false;
//    return true;
}

// ____________________________________________________________________________
bool StateMachine::CarOnCrossing(tSptrMapElement el) {
    tMapPoint crossing_pos = el->GetGlobalPolyCenter();
    int car_id = GetObjectInRange(crossing_pos.get<0>(), crossing_pos.get<1>(), el->GetGlobalOrientation()+PI, CAR, 0.45, -0.45, 0.45, -0.45);
    if (car_id == -1) return false;
    return true;

    // tSptrMapElement crossing_element =
    //     map_->GetElement(current_event_.event_id);
    // std::vector<tSptrMapElement> detected_cars;
    // map_->GetAllElementsWithType(CAR, detected_cars);
    // BOOST_FOREACH (const tSptrMapElement& s, detected_cars) {
    //     if (s->GlobalDistance(*crossing_element) < CAR_ON_CROSSING_THRESHOLD_)
    //         return true;
    // }
    // return false;
}

bool StateMachine::PullOutAllowed() {
    tMapCarPosition pos;
    map_->GetGlobalCarPosition(&pos);
    int car_id = GetObjectInRange(pos.x, pos.y, pos.heading, CAR, 1.2, 0.3, 2.0, -2.0);
    if (car_id == -1) return true;
    return false;
}

// ____________________________________________________________________________
bool StateMachine::AllowedToGo(
    string
        command) {

    tSptrMapElement el = map_->GetElement(current_event_.event_id);
    if (CarOnCrossing(map_->GetElement(current_event_.event_id))) {  // check this condition. Otherwise we might crash
                            // into other cars?
        return false;
    }

    if (command == "straight") {
        switch (current_event_.event_type) {
            case T_CROSSING_STOP:
            case X_CROSSING_STOP:
            case T_CROSSING_GIVE_WAY:
            case X_CROSSING_GIVE_WAY:
                return (!CarOnLeft(el) && !CarOnRight(el));
            case X_CROSSING_RIGHT_OF_WAY_RIGHT:
            case T_CROSSING_RIGHT_OF_WAY_RIGHT:
                return !CarOnRight(el);
            default:
                return true;
        }

    } else if (command == "left") {
        switch (current_event_.event_type) {
            case T_CROSSING_STOP:  // todo: what if we drive straight? For now
                                   // assume no orientation
            case X_CROSSING_STOP:
            case T_CROSSING_GIVE_WAY:
            case X_CROSSING_GIVE_WAY:
                return (!CarOnLeft(el) && !CarOnRight(el) &&
                        !CarTowardsAhead(el));
            case X_CROSSING_RIGHT_OF_WAY_RIGHT:
            case T_CROSSING_RIGHT_OF_WAY_RIGHT:
                return (!CarOnRight(el) && !CarTowardsAhead(el));
            default:
                return true;
        }
    } else if (command == "right") {
        switch (el->GetType()) {
            case X_CROSSING_STOP:
            case T_CROSSING_STOP:
            case X_CROSSING_GIVE_WAY:
            case T_CROSSING_GIVE_WAY:
                return !CarOnLeft(el);
            case X_CROSSING_RIGHT_OF_WAY_RIGHT:
            case T_CROSSING_RIGHT_OF_WAY_RIGHT:
                return true;
            default:
                return true;
        }
    }
    return true;
}

// ____________________________________________________________________________
void StateMachine::SendBehaviour() {
    //IF_DEBUG_LOG_PRINTF("StateMachine: SendBehavior active");
    size_t state_hash = GetActiveStateHash();
    if (current_state_hash_ == state_hash) {
        // EM_LOG("Send behaviour: state have not changed");
        send_behavior_thread_is_active = false;
        return;
    } else {
        std::string jury_action = GetCurrentManeuver();

        if (state_hash == typeid(PStEmergency_stop).hash_code()) {
            IF_DEBUG_LOG_PRINTF("Sending behaviour for PStEmergency_stop");
            TransmitBehaviour(STOP, -1, -1, ST_STOP);
            TransmitLightCommand(HAZZARD_LIGHTS, tTrue);
        } else if (state_hash == typeid(StNormal_driving).hash_code()) {
            IF_DEBUG_LOG_PRINTF("SM: Sending behaviour for StNormal_driving");
            TransmitLightCommand(ALL_LIGHTS, tFalse);
            TransmitLightCommand(HEAD_LIGHTS, tTrue);
            TransmitBehaviour(FOLLOW_LANE, -1, -1, ST_SPEED_HIGH);
        } else if (state_hash == typeid(StStop_crossing_ahead).hash_code()) {
            if (jury_action == "left") {
                TransmitBehaviour(STOP_AT_OBJECT, TURN_LEFT,
                                  current_event_.event_id, ST_STOP);
                IF_DEBUG_LOG_PRINTF(
                    "SM: Sending a stop at object %d "
                    "and after turn left",
                    current_event_.event_id);
            } else if (jury_action == "right") {
                TransmitBehaviour(STOP_AT_OBJECT, TURN_RIGHT,
                                  current_event_.event_id, ST_STOP);
                IF_DEBUG_LOG_PRINTF(
                    "SM: Sending a stop at object %d "
                    "and after turn right",
                    current_event_.event_id);
            } else if (jury_action == "straight") {
                TransmitBehaviour(STOP_AT_OBJECT, GO_STRAIGHT,
                                  current_event_.event_id, ST_STOP);
                IF_DEBUG_LOG_PRINTF(
                    "SM: Sending a stop at object %d "
                    "and after go straight",
                    current_event_.event_id);
            }

        } else if (state_hash == typeid(StGoTo_crossing_ahead).hash_code()) {
            if (jury_action == "left") {
                TransmitBehaviour(GO_TO_OBJECT, TURN_LEFT,
                                  current_event_.event_id, ST_SPEED_LOW);
                IF_DEBUG_LOG_PRINTF(
                    "SM: Sending go to crossing %d "
                    "and after turn left",
                    current_event_.event_id);
            } else if (jury_action == "right") {
                TransmitBehaviour(GO_TO_OBJECT, TURN_RIGHT,
                                  current_event_.event_id, ST_SPEED_LOW);
                IF_DEBUG_LOG_PRINTF(
                    "SM: Sending go to crossing %d"
                    "and after turn right",
                    current_event_.event_id);
            } else if (jury_action == "straight") {
                TransmitBehaviour(GO_TO_OBJECT, GO_STRAIGHT,
                                  current_event_.event_id, ST_SPEED_LOW);
                IF_DEBUG_LOG_PRINTF(
                    "SM: Sending go to crossing %d "
                    "and after turn go straight",
                    current_event_.event_id);
            }
        } else if (state_hash == typeid(StObstacle_ahead).hash_code()) {
            TransmitBehaviour(GO_AROUND_OBJECT, -1, current_event_.event_id,
                              ST_SPEED_LOW);
            IF_DEBUG_LOG_PRINTF("SM: Obstacle ahead, trying to go around it");
            TransmitLightCommand(HAZZARD_LIGHTS, tTrue);
        } else if (state_hash == typeid(StStop_crossing_at).hash_code()) {
            if (jury_action == "left") {
            LOG_INFO_PRINTF("NEW: Stopped at crossing");
                TransmitLightCommand(TURN_SIGNAL_LEFT, tTrue);
                boost::this_thread::sleep_for(boost::chrono::seconds(3));

                unsigned int timer = 0;
            tSptrMapElement el = map_->GetElement(current_event_.event_id);
            LOG_INFO_PRINTF("NEW: CarOnRight: %i", CarOnRight(el));
                LOG_INFO_PRINTF("NEW: AllowedToGo: %i", AllowedToGo("left"));

                while ((!AllowedToGo("left") && ++timer <= 15) || (!PedestrianCrossingAllowedToGo(map_->GetElement(current_event_.event_id)) && ++timer <= 15) ||
                       (CarOnCrossing(map_->GetElement(current_event_.event_id)) && ++timer <= 60)) {

                    LOG_INFO_PRINTF("NEW: CarOnRight: %i", CarOnRight(el));
                    LOG_INFO_PRINTF("NEW: AllowedToGo: %i", AllowedToGo("left"));

                    boost::this_thread::sleep_for(boost::chrono::seconds(1));
                }
                LOG_INFO_PRINTF("TURNING LEFT");
                TransmitBehaviour(TURN_LEFT, -1, current_event_.event_id,
                                  ST_SPEED_LOW);
                event_change_locked_ = true;
                maneuver_requested_ = true;
            } else if (jury_action == "right") {
                std::cout << "line 284\n" << std::endl;
                TransmitLightCommand(TURN_SIGNAL_RIGHT, tTrue);
                boost::this_thread::sleep_for(boost::chrono::seconds(3));
                unsigned int timer = 0;
                while ((!AllowedToGo("right") && ++timer <= 15) || (!PedestrianCrossingAllowedToGo(map_->GetElement(current_event_.event_id)) && ++timer <= 15) ||
                       (CarOnCrossing(map_->GetElement(current_event_.event_id)) && ++timer <= 60)) {
                    boost::this_thread::sleep_for(boost::chrono::seconds(1));
                }
                TransmitBehaviour(TURN_RIGHT, -1, current_event_.event_id,
                                  ST_SPEED_LOW);
                event_change_locked_ = true;
                LOG_INFO_PRINTF("TURNING RIGHTTT");
                maneuver_requested_ = true;
            } else if (jury_action == "straight") {
                boost::this_thread::sleep_for(boost::chrono::seconds(3));
                unsigned int timer = 0;
                while ((!AllowedToGo("straight") && ++timer <= 15) || (!PedestrianCrossingAllowedToGo(map_->GetElement(current_event_.event_id)) && ++timer <= 15) ||
                       (CarOnCrossing(map_->GetElement(current_event_.event_id)) && ++timer <= 60)) {
                    boost::this_thread::sleep_for(boost::chrono::seconds(1));
                }
                TransmitBehaviour(GO_STRAIGHT, -1, current_event_.event_id,
                                  ST_SPEED_LOW);
                LOG_INFO_PRINTF("GO STRAIGHT");
                event_change_locked_ = true;
                maneuver_requested_ = true;
            } else {
                LOG_ERROR_PRINTF(
                    "ERROR: at crossing but next maneuver doesn't make sense");
            }
        } else if (state_hash == typeid(StGoTo_crossing_at).hash_code()) {
            if (jury_action == "left") {
                TransmitLightCommand(TURN_SIGNAL_LEFT, tTrue);
                IF_DEBUG_LOG_PRINTF("TURNING LEFT");
                TransmitBehaviour(TURN_LEFT, -1, current_event_.event_id,
                                  ST_SPEED_LOW);
                event_change_locked_ = true;
                maneuver_requested_ = true;
            } else if (jury_action == "right") {
                TransmitLightCommand(TURN_SIGNAL_RIGHT, tTrue);
                IF_DEBUG_LOG_PRINTF("TURNING RIGHT");
                TransmitBehaviour(TURN_RIGHT, -1, current_event_.event_id,
                                  ST_SPEED_LOW);
                event_change_locked_ = true;
                maneuver_requested_ = true;
            } else if (jury_action == "straight") {
                TransmitBehaviour(GO_STRAIGHT, -1, current_event_.event_id,
                                  ST_SPEED_LOW);
                IF_DEBUG_LOG_PRINTF("GO STRAIGHT");
                event_change_locked_ = true;
                maneuver_requested_ = true;
            } else {
                // TODO: should stop?
                LOG_ERROR_PRINTF(
                    "ERROR: at crossing but next "
                    "manuever doesn't make sense");
            }
        } else if (state_hash == typeid(StPull_left).hash_code()) {
            TransmitLightCommand(TURN_SIGNAL_LEFT, tTrue);
            TransmitBehaviour(PULL_LEFT, -1, -1, ST_SPEED_LOW);
            IF_DEBUG_LOG_PRINTF("Pullin out left from parking");
            event_change_locked_ = true;
            maneuver_requested_ = true;

        } else if (state_hash == typeid(StPull_right).hash_code()) {
            TransmitLightCommand(TURN_SIGNAL_RIGHT, tTrue);
            TransmitBehaviour(PULL_RIGHT, -1, -1, ST_SPEED_LOW);
            IF_DEBUG_LOG_PRINTF("Pullin out right from parking");
            event_change_locked_ = true;
            maneuver_requested_ = true;
        } else if (state_hash == typeid(StEmpty_Zebra_crossing).hash_code()) {
            TransmitBehaviour(FOLLOW_LANE, -1, -1, ST_SPEED_LOW);
            //TransmitLightCommand(BRAKE_LIGHTS, tTrue);
            LOG_INFO_PRINTF(
                "SM: Empty Zebra crossing, follow lane, speed low");
        } else if (state_hash ==
                       typeid(StStop_zebra_ahead_pedestrian_at).hash_code() ||
                   state_hash ==
                       typeid(StStop_zebra_ahead_pedestrian_on).hash_code()) {
            TransmitBehaviour(STOP_AT_OBJECT, -1, current_event_.event_id,
                              ST_STOP);
            //TransmitLightCommand(BRAKE_LIGHTS, tTrue);
            IF_DEBUG_LOG_PRINTF("SM: Stop at zebra crossing");
        } else if (state_hash ==
                   typeid(StStop_zebra_at_pedestrian_at).hash_code()) {
            LOG_INFO_PRINTF(
                "SM: waiting for 10 seconds for the person to cross");
            unsigned int timer = 0;
            //TransmitLightCommand(BRAKE_LIGHTS, tTrue);
            while (++timer <= 10) {
                //if (PedestrianOnCrossing())
                //    ; TODO : WTF?!!?!!
                //{
                //    ProcessEvent(EvPedestrian_on_crossing());
                //    current_state_name_ = state_hash.name();
                //    send_behavior_thread_is_active = false;
                //    return;
                //}
                boost::this_thread::sleep_for(boost::chrono::seconds(1));
            }
            LOG_INFO_PRINTF("SM: waiting time is over");
            time_last_zebra_crossing_ = _clock->GetStreamTime();
            ProcessEvent(EvTime_up());
            SendBehaviour();

        } else if (state_hash ==
                   typeid(StStop_zebra_at_pedestrian_on).hash_code()) {
            LOG_INFO_PRINTF(
                "SM: pedestrian on crossing, waiting for some seconds ");
            // TransmitLightCommand(BRAKE_LIGHTS, tTrue);
            unsigned int timer = 0;
            while (++timer <= 10)  //
            {
                //if (!PedestrianOnCrossing()) {
                //    ProcessEvent(EvEmpty_zebra_crossing());
                //    current_state_name_ = state_hash.name();
                //    send_behavior_thread_is_active = false;
                //    return;
                //}
                boost::this_thread::sleep_for(boost::chrono::seconds(1));
            }
            time_last_zebra_crossing_ = _clock->GetStreamTime();
            //TransmitBehaviour(FOLLOW_LANE, -1, -1,
            //                  ST_SPEED_HIGH);
            LOG_INFO_PRINTF(
                "waited for some seconds, pedestrian still on crossing");
        ProcessEvent(EvTime_up());
        SendBehaviour();
        } else if (state_hash == typeid(StNext_maneuver_parking).hash_code()) {
            if (IsParkingRequested(GetCurrentManeuver())) {
                tMapID map_parking_id = MAP_DEFAULT_ID;
                if (GetParkingMapID(GetCurrentManeuver(), &map_parking_id)) {
                    TransmitBehaviour(PARK, -1,
                                      map_parking_id, ST_SPEED_LOW);
                    event_change_locked_ = true;
                    maneuver_requested_ = true;
                    IF_DEBUG_LOG_PRINTF(
                        "SM: Park at: "
                        "parking spot with id %i",
                        map_parking_id);
                }
            } else {
                IF_DEBUG_LOG_PRINTF(
                    "SM: Stop at point infront of parking spot, but no parking "
                    "request");
            }
        } else if (state_hash == typeid(StParked).hash_code()) {
            LOG_INFO_PRINTF("SM: Car Is Parked");
            TransmitLightCommand(HAZZARD_LIGHTS, tTrue);
            unsigned int timer = 0;
            while (++timer <= 5)  //
            {
                boost::this_thread::sleep_for(boost::chrono::seconds(1));
            }
            TransmitLightCommand(HAZZARD_LIGHTS, tFalse);
        LOG_INFO_PRINTF("SM: Parked for 5 second");
            if (GetCurrentManeuver() == "pull_out_left") {
                unsigned int timer = 0;
        while ((!PullOutAllowed() && ++timer <= 20)) {
            boost::this_thread::sleep_for(boost::chrono::seconds(1));
        }
                ProcessEvent(EvNext_pull_left());
            } else if (GetCurrentManeuver() == "pull_out_right") {
        unsigned int timer = 0;
        while ((!PullOutAllowed() && ++timer <= 20)) {
        boost::this_thread::sleep_for(boost::chrono::seconds(1));
            }
                ProcessEvent(EvNext_pull_right());
            }
            SendBehaviour();

        } else if (state_hash == typeid(StSlow_driving).hash_code()) {
            TransmitBehaviour(FOLLOW_LANE, -1, -1, ST_SPEED_LOW);
            IF_DEBUG_LOG_PRINTF("scan parking, follow lane slow");
        }
    }
    current_state_hash_ = state_hash;
    send_behavior_thread_is_active = false;
    return;
}

// ____________________________________________________________________________
void StateMachine::SkipJuryStates() {
    maneuver_list_vector_.clear();
    tAADC_Maneuver man;
    man.id = 0;
    man.action = "left";  // cross_parking
    maneuver_list_vector_.push_back(man);
    man.id = 1;
    man.action = "right";  // cross_parking
    maneuver_list_vector_.push_back(man);

    LOG_INFO_PRINTF("Skip Jury next Action is %s",
                    GetCurrentManeuver().c_str());
    ProcessEvent(EvManeuver_recieved());
    ProcessEvent(EvJury_get_ready());
    ProcessEvent(EvJury_start());
    event_change_locked_ = false;
}



int StateMachine::GetObjectInRange(float x_global, float y_global, float global_orientation, MapElementType type, float x_max, float x_min, float y_max, float y_min){
    std::vector<tSptrMapElement> detected_elements;
    map_->GetAllElementsWithType(type, detected_elements);
    BOOST_FOREACH (tSptrMapElement& s, detected_elements) {
        tMapPoint element_center_point = s->GetGlobalPolyCenter();
        float dx = (element_center_point.get<0>() - x_global);
        float dy = (element_center_point.get<1>() - y_global);
        float pos_local_x =
            (dx)*cos(-(global_orientation)) - (dy)*sin(-(global_orientation));
        float pos_local_y =
            (dx)*sin(-(global_orientation)) + (dy)*cos(-(global_orientation));


        if (pos_local_x > x_min && pos_local_x < x_max && pos_local_y > y_min && pos_local_y < y_max) return s->GetID();
    }
    return -1;
}

// ____________________________________________________________________________
bool StateMachine::CarOnRight(tSptrMapElement el) {
    tMapPoint crossing_pos = el->GetGlobalPolyCenter();
    int car_id = GetObjectInRange(crossing_pos.get<0>(), crossing_pos.get<1>(), el->GetGlobalOrientation()+PI, CAR, 0.5, -0.5, -0.5, -2.5);
    if (car_id == -1) return false;
    return true;

//     std::vector<tSptrMapElement> detected_cars;
//     map_->GetAllElementsWithType(CAR, detected_cars);
//     BOOST_FOREACH (tSptrMapElement& s, detected_cars) {
//      // LOG_INFO_PRINTF("Car distance %f", s->GlobalDistance(*el));
//      //    LOG_INFO_PRINTF("Car angle is %f", s->GetOrientationAngleToCar() );
//         tMapPoint crossing_center_global = el->GetGlobalPolyCenter();
//         tMapPoint car_center_global = s->GetGlobalPolyCenter();
//         float dx = (car_center_global.get<0>() - crossing_center_global.get<0>());
//         float dy = (car_center_global.get<1>() - crossing_center_global.get<1>());

//         float pos_local_x =
//             (dx)*cos(-(el->GetGlobalOrientation()+PI)) - (dy)*sin(-(el->GetGlobalOrientation()+PI));
//         float pos_local_y =
//             (dx)*sin(-(el->GetGlobalOrientation()+PI)) + (dy)*cos(-(el->GetGlobalOrientation()+PI));
//         LOG_INFO_PRINTF("Car distance X,Y: %f,%f", pos_local_x, pos_local_y);
//         // if (s->GlobalDistance(*el) <= CAR_AT_CROSSING_THRESHOLD_ &&
//         //     s->GlobalDistance(*el) > CAR_ON_CROSSING_THRESHOLD_ &&
//         //     s->GetOrientationAngleToCar() < 0 &&
//         //     s->GetOrientationAngleToCar() > -M_PI_4)

// //        if(pos_local_x < 1 && pos_local_x > -1 && pos_local_y < -0.5 && pos_local_y > -2.5 )
//         if(pos_local_x < 1 && pos_local_x > -1 && pos_local_y < -0.5 && pos_local_y > -2.5 )
//             return true;
//     }
//     return false;
}

// ____________________________________________________________________________
bool StateMachine::CarOnLeft(tSptrMapElement el) {
    tMapPoint crossing_pos = el->GetGlobalPolyCenter();
    int car_id = GetObjectInRange(crossing_pos.get<0>(), crossing_pos.get<1>(), el->GetGlobalOrientation()+PI, CAR, 0.5, -0.5, 2.5, 0.5);
    if (car_id == -1) return false;
    return true;
    // std::vector<tSptrMapElement> detected_cars;
    // map_->GetAllElementsWithType(CAR, detected_cars);
    // BOOST_FOREACH (tSptrMapElement& s, detected_cars) {
    //     //                LOG_INFO_PRINTF(cString::Format("Distance car to
    //     //                sign:%f, Angle car to car: %f",
    //     //                s->GlobalDistance(*el),
    //     //                s->GetOrientationAngleToCar()));
    //     // 90 degrees are M_PI_2 (= 0.5 * PI), counterclockwise
    //     if (s->GlobalDistance(*el) < CAR_AT_CROSSING_THRESHOLD_ &&
    //         s->GlobalDistance(*el) > CAR_ON_CROSSING_THRESHOLD_ &&
    //         s->GetOrientationAngleToCar() > M_PI_2 * 0.5 &&
    //         s->GetOrientationAngleToCar() < M_PI_2)
    //         return true;
    // }
    // return false;
}

// ____________________________________________________________________________
bool StateMachine::CarTowardsAhead(tSptrMapElement el) {
    tMapPoint crossing_pos = el->GetGlobalPolyCenter();
    int car_id = GetObjectInRange(crossing_pos.get<0>(), crossing_pos.get<1>(), el->GetGlobalOrientation()+PI, CAR, 2.5, 0.45, 0.5, -0.5);
    if (car_id == -1) return false;
    return true;

    // std::vector<tSptrMapElement> detected_cars;
    // map_->GetAllElementsWithType(CAR, detected_cars);
    // BOOST_FOREACH (tSptrMapElement& s, detected_cars) {
    //     //                LOG_INFO_PRINTF(cString::Format("Distance car to
    //     //                sign:%f, Angle car to car: %f",
    //     //                s->GlobalDistance(*el),
    //     //                s->GetOrientationAngleToCar()));
    //     // angle between 0 and 45 degrees
    //     if (s->GlobalDistance(*el) < CAR_AT_CROSSING_THRESHOLD_ &&
    //         s->GlobalDistance(*el) > CAR_ON_CROSSING_THRESHOLD_ &&
    //         s->GetOrientationAngleToCar() > 0  // todo
    //         && s->GetOrientationAngleToCar() < M_PI_2 * 0.5) {
    //         return true;
    //     }
    // }
    // return false;
}

// ____________________________________________________________________________
float StateMachine::Car2ObjectPlaneViewAngle(
    tSptrMapElement el) {  // TODO:Change this function. markus has a new one
    // not sure if correct
    tMapCarPosition p;
    map_->GetGlobalCarPosition(&p);
    if (map_->GetGlobalCarPosition(&p) && el->IsOrientationUsed()) {
        return fabs(p.heading - el->GetGlobalOrientation());
    } else {
        return -1;
    }
}

// ____________________________________________________________________________
void StateMachine::ProcessEventQueue() {
    if (EventQueueEmpty()) {
        // std::cout << "line 519 event qeueue empty"<< std::endl;
        ProcessEvent(EvRoad_is_free());
        SendBehaviour();
        return;
    }
    // contunu if event changed and no car was detected
    if ((EventQueueTop().event_id == current_event_.event_id &&
         EventQueueTop().event_type == current_event_.event_type) ||
        event_change_locked_) {  //|| car_detected_
        //  std::cout << "this event is already top priority or event changing
        //  is locked" << std::endl;
        // std::cout << "line 525"<< std::endl;
        if (!car_detected_) return;
    }

    {
        // check if map elements was found
        tSptrMapElement previous_el = map_->GetElement(current_event_.event_id);
        if (previous_el) {
            IF_DEBUG_LOG_PRINTF("previous event_id element: %s",
                                previous_el->ToString().c_str());
        } else {
            LOG_ERROR_PRINTF(
                "StateMachine previous el "
                "not found, id: %d",
                current_event_.event_id);
        }
        tSptrMapElement eltop = map_->GetElement(EventQueueTop().event_id);
        if (eltop) {
            IF_DEBUG_LOG_PRINTF("top event id element: %s",
                                eltop->ToString().c_str());
        } else {
            LOG_ERROR_PRINTF(
                "StateMachine top event id el "
                "not found, id: %d",
                EventQueueTop().event_id);
        }

        std::string jury_action = GetCurrentManeuver();
        current_event_ = EventQueueTop();
        tSptrMapElement el = map_->GetElement(current_event_.event_id);
        switch (current_event_.event_type) {
            case EMPTY_ZEBRA_CROSSING: {
                ProcessEvent(EvEmpty_zebra_crossing());
                break;
            }
            case PEDESTRIAN_ON_ZEBRA_CROSSING: {
                ProcessEvent(EvPedestrian_on_crossing());
                break;
            }
            case PEDESTRAIN_WAITS_AT_ZEBRA_CROSSING: {

                ProcessEvent(EvPedestrian_at_crossing());
                break;
            }
            case X_CROSSING_STOP: {
                ProcessEvent(EvStop_at_crossing_ahead());
                break;
            }
            case X_CROSSING_RIGHT_OF_WAY_RIGHT: {
                if ((CarOnRight(el) && !(jury_action == "right")) ||
                    CarOnCrossing(el) || (CarTowardsAhead(el)&& jury_action == "left") || !PedestrianCrossingAllowedToGo(el)) {
                    ProcessEvent(EvStop_at_crossing_ahead());
                } else {
                    ProcessEvent(EvGoTo_crossing_ahead());
                }
                break;
            }
            case X_CROSSING_RIGHT_OF_WAY: {
                if (CarOnCrossing(el) || (CarTowardsAhead(el)&& jury_action == "left") || !PedestrianCrossingAllowedToGo(el)) {
                    ProcessEvent(EvStop_at_crossing_ahead());
                    IF_DEBUG_LOG_PRINTF(
                        "Other car inside crossing, "
                        "stopping at crossing");
                } else
                    ProcessEvent(EvGoTo_crossing_ahead());

                break;
            }
            case X_CROSSING_GIVE_WAY: {
                if ((CarOnRight(el) && !(jury_action == "right")) ||
                    CarOnLeft(el) || CarOnCrossing(el) || (CarTowardsAhead(el)&& (jury_action == "left")) || !PedestrianCrossingAllowedToGo(el)) {
                    ProcessEvent(EvStop_at_crossing_ahead());
                } else {
                    ProcessEvent(EvGoTo_crossing_ahead());
                }
                break;
            }
            case T_CROSSING_STOP: {
                ProcessEvent(EvStop_at_crossing_ahead());
                break;
            }
            case T_CROSSING_RIGHT_OF_WAY_RIGHT: {
                IF_DEBUG_LOG_PRINTF("IM IN T_CROSSING_RIGHT_OF_WAY_RIGHT");
                if ((CarOnRight(el) && !(jury_action == "right")) ||
                    CarOnCrossing(el)) {
                    ProcessEvent(EvStop_at_crossing_ahead());
                } else {
                    ProcessEvent(EvGoTo_crossing_ahead());
                }
                break;
            }
            case T_CROSSING_RIGHT_OF_WAY: {
                if (CarOnCrossing(el)) {
                    ProcessEvent(EvStop_at_crossing_ahead());
                    IF_DEBUG_LOG_PRINTF(
                        "Other car inside crossing, stopping at crossing");
                } else
                    ProcessEvent(EvGoTo_crossing_ahead());
                break;
            }
            case T_CROSSING_GIVE_WAY: {
                if ((CarOnRight(el) && !(jury_action == "right")) ||
                    CarOnLeft(el) || CarOnCrossing(el)) {
                    IF_DEBUG_LOG_PRINTF(
                        "SM process ev. q. case 1 "
                        "T_CROSSING_GIVE_WAY");
                    ProcessEvent(EvStop_at_crossing_ahead());
                } else {
                    IF_DEBUG_LOG_PRINTF(
                        "SM process ev. q. case 2"
                        " T_CROSSING_GIVE_WAY");
                    ProcessEvent(EvGoTo_crossing_ahead());
                }
                break;
            }
            //                case PARKING_SIGN: {
            //                    ProcessEvent(EvNext_maneuver_park());
            //                    break;
            //                }
            case PARKING_SPOT: {
               LOG_INFO_PRINTF("SM PARKING SPOT, exec. maneuver park");
                if(IsParkingRequested(GetCurrentManeuver())) ProcessEvent(EvNext_maneuver_park());
//                else ProcessEvent(EvSlow_down());
                break;
            }
            case SLOW_DOWN: {
                //ProcessEvent(EvSlow_down());
                TransmitBehaviour(FOLLOW_LANE, -1, -1, ST_SPEED_MEDIUM);
                break;
            }
            case SPEED_UP: {
                //ProcessEvent(EvSpeed_Up());
                TransmitBehaviour(FOLLOW_LANE, -1, -1, ST_SPEED_ULTRAHIGH);

                break;
            }
        }
        SendBehaviour();
        car_detected_ = false;
        return;
    }
}

// ____________________________________________________________________________
int StateMachine::SetElementPriority(tSptrMapElement el) {
    float distance = el->GetDistanceToCar();

    float max_priority_detection_distance = 1.0;  // 0.9 before
    float max_priority_detection_angle = M_PI_2;
    float min_priority_detection_angle = -M_PI_2;
    float priority_border_distance = 0.45;
    // float left_priority_border_angle = -M_PI_4;
    // float right_priority_border_angle = M_PI_4;

    float angle = el->GetOrientationAngleToCar();
    if (distance > max_priority_detection_distance ||
        angle < min_priority_detection_angle ||
        angle > max_priority_detection_angle) {
        return 0;  // outside range of instereset, thus ignored
    }
    if (distance > 0 && distance <= priority_border_distance) {
        if (-M_PI_2 < angle && angle <= -M_PI_4) {
            return 2;
        } else if (-M_PI_4 < angle && angle <= M_PI_4) {
            return 1;
        } else if (M_PI_4 < angle && angle <= M_PI_2) {
            return 2; //TODO: I changed from 3 to 2, check if better
        }
    } else if (distance > priority_border_distance &&
               distance <= max_priority_detection_distance) {
        if (-M_PI_2 < angle && angle <= -M_PI_4) {
            return 2;
        } else if (-M_PI_4 < angle && angle <= M_PI_4) {
            return 3; //TODO: I changed from 2 to 3, check if better
        } else if (M_PI_4 < angle && angle <= M_PI_2) {
            return 3;
        }
    }
    return 0;
}

void StateMachine::ProcessElement(const tSptrMapElement el) {
    if (!el || !map_) {
        LOG_ERROR_PRINTF("SM ProcessElement or map not exist");
    }
    if ((SetElementPriority(el) == 0 ||
         SetElementPriority(el) > current_highest_priority_) &&
        !(el->GetType() ==
          STREET_TRAFFIC_SIGN_POSITION_MARKER)) {  //    // this would be use to
                                                   //    filter higher priority
                                                   //    elements we dont care
                                                   //    about cuz they are more
                                                   //    far
        if(el->GetType() != CAR) return;
    }

    // bool stop_sign_found = false;
    std::vector<tSptrMapElement> detected_crossing_signs_tmp;
    std::vector<MapElementType> gettypes = boost::assign::list_of(
        STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT)(STREET_TRAFFIC_SIGN_STOP)(
        STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC)(STREET_TRAFFIC_SIGN_GIVE_WAY);

    switch (el->GetType()) {

        case STREET_MARKER_CROSSING_T:
        case STREET_MARKER_CROSSING_X: {

            std::vector<tSptrMapElement> detected_crossing_signs;
            const std::vector<MapElementType> gettypes = boost::assign::list_of(
                STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT)(STREET_TRAFFIC_SIGN_STOP)(
                STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC)(
                STREET_TRAFFIC_SIGN_GIVE_WAY);
            map_->GetAllElementsWithTypes(gettypes, detected_crossing_signs);
            if (detected_crossing_signs.empty()) {
                LOG_ERROR_PRINTF("SM no sigin at CROSSING");
            }


            BOOST_FOREACH (const tSptrMapElement& s, detected_crossing_signs) {
        MapElementRoadSign* el_sign = dynamic_cast<MapElementRoadSign*>(s.get());
                if (el_sign && (s->GlobalDistance(*el) < SIGN_CROSSING_THRESHOLD_ ) &&
                    MapHelper::IsElementOrientatedToCar(
                        map_, *(s.get()), (tMapData)0.8 * M_PI) && el_sign->GetSignSubType() == SIGN_SENSOR){
                    switch (s->GetType()) {
                        case STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT: {
                            AddEventToPQ(el, X_CROSSING_RIGHT_OF_WAY_RIGHT);
                            break;
                        }
                        case STREET_TRAFFIC_SIGN_STOP: {
                            AddEventToPQ(el, X_CROSSING_STOP);
                            break;
                        }
                        case STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC: {
                            AddEventToPQ(el, X_CROSSING_RIGHT_OF_WAY);
                            break;
                        }
                        case STREET_TRAFFIC_SIGN_GIVE_WAY: {
                            AddEventToPQ(el, X_CROSSING_GIVE_WAY);
                            break;
                        }
                        default: {
                            AddEventToPQ(el, X_CROSSING_RIGHT_OF_WAY_RIGHT);
                            break;
                        }
                    }
                }
            }
        AddEventToPQ(el, X_CROSSING_RIGHT_OF_WAY_RIGHT);
            break;
        }

        case STREET_MARKER_ZEBRA: {
            std::vector<tSptrMapElement> detected_pedestrian_list;
            const std::vector<MapElementType> get_types =
                boost::assign::list_of(PEDESTRIAN_ADULT)(PEDESTRIAN_CHILD);
            map_->GetAllElementsWithTypes(get_types, detected_pedestrian_list);

            // print out and compare distances with filter
            //LOG_WARNING_PRINTF("TIME PASSED %f", (_clock->GetStreamTime() - time_last_zebra_crossing_));
        // std::cout << "the time passed:" << (_clock->GetStreamTime() - time_last_zebra_crossing_) << std::endl;
            if (!detected_pedestrian_list.empty() && ((_clock->GetStreamTime() - time_last_zebra_crossing_) >
        10 * 1e6)) {
                IF_DEBUG_LOG_PRINTF("SM pedestiran at zebra");
                    if (PedestrianOnCrossing(el)) {
                        AddEventToPQ(el, PEDESTRIAN_ON_ZEBRA_CROSSING);
                    } else if (PedestrianAtCrossingLeft(el) || PedestrianAtCrossingRight(el)) {
                        AddEventToPQ(el, PEDESTRAIN_WAITS_AT_ZEBRA_CROSSING);
                    } else {
                        AddEventToPQ(el, EMPTY_ZEBRA_CROSSING);
                    }
            } else {
                IF_DEBUG_LOG_PRINTF("SM NO pedestiran at zebra");
                AddEventToPQ(el, EMPTY_ZEBRA_CROSSING);
            }
            break;
        }

        case STREET_TRAFFIC_SIGN_STOP: {
            std::vector<tSptrMapElement> detected_crossings;
            const std::vector<MapElementType> get_types =
                boost::assign::list_of(STREET_MARKER_CROSSING_T)(
                    STREET_MARKER_CROSSING_X);
            map_->GetAllElementsWithTypes(get_types, detected_crossings);

            if (!detected_crossings.empty() &&
                    MapHelper::IsElementOrientatedToCar(
                        map_, *(el.get()), (tMapData)0.8 * M_PI)) {
                BOOST_FOREACH (tSptrMapElement& s, detected_crossings) {
                    if (s->GlobalDistance(*el) <
                        SIGN_CROSSING_THRESHOLD_) {
                        if (s->GetType() == STREET_MARKER_CROSSING_T)
                            AddEventToPQ(s, T_CROSSING_STOP);
                        else if (s->GetType() == STREET_MARKER_CROSSING_X)
                            AddEventToPQ(s, X_CROSSING_STOP);
                        break;
                    }
                }
            } else {
                LOG_ERROR_PRINTF("SM no crossing at stop sign detected");
            }
            // AddEventToPQ(el, STOP_SIGN);
            break;
        }
        case STREET_TRAFFIC_SIGN_GIVE_WAY_RIGHT: {
            std::vector<tSptrMapElement> detected_crossings;
            const std::vector<MapElementType> get_types =
                boost::assign::list_of(STREET_MARKER_CROSSING_T)(
                    STREET_MARKER_CROSSING_X);
            map_->GetAllElementsWithTypes(get_types, detected_crossings);
            if (!detected_crossings.empty() &&
                    MapHelper::IsElementOrientatedToCar(
                        map_, *(el.get()), (tMapData)0.8 * M_PI)) {
                BOOST_FOREACH (tSptrMapElement& s, detected_crossings) {
                    if (s->GlobalDistance(*el) < SIGN_CROSSING_THRESHOLD_) {
                        if (s->GetType() == STREET_MARKER_CROSSING_T)
                            AddEventToPQ(s, T_CROSSING_RIGHT_OF_WAY_RIGHT);
                        else if (s->GetType() == STREET_MARKER_CROSSING_X)
                            AddEventToPQ(s, X_CROSSING_RIGHT_OF_WAY_RIGHT);
                        break;
                    }
                }
            } else {
                LOG_ERROR_PRINTF("SM no crossing at give way sign detected");
            }
            break;
        }
        case STREET_TRAFFIC_SIGN_GIVE_WAY: {
            std::vector<tSptrMapElement> detected_crossings;
            const std::vector<MapElementType> get_types =
                boost::assign::list_of(STREET_MARKER_CROSSING_T)(
                    STREET_MARKER_CROSSING_X);
            map_->GetAllElementsWithTypes(get_types, detected_crossings);
            if (!detected_crossings.empty() &&
                    MapHelper::IsElementOrientatedToCar(
                        map_, *(el.get()), (tMapData)0.8 * M_PI)) {
                BOOST_FOREACH (tSptrMapElement& s, detected_crossings) {
                    if (s->GlobalDistance(*el) < SIGN_CROSSING_THRESHOLD_) {
                        if (s->GetType() == STREET_MARKER_CROSSING_T)
                            AddEventToPQ(s, T_CROSSING_GIVE_WAY);
                        else if (s->GetType() == STREET_MARKER_CROSSING_X)
                            AddEventToPQ(s, X_CROSSING_GIVE_WAY);
                        break;
                    }
                }
            } else {
                LOG_ERROR_PRINTF("SM no crossing at give way sign detected");
            }
            break;
        }
        case STREET_TRAFFIC_SIGN_PRIORITY_IN_TRAFFIC: {
            std::vector<tSptrMapElement> detected_crossings;
            const std::vector<MapElementType> get_types =
                boost::assign::list_of(STREET_MARKER_CROSSING_T)(
                    STREET_MARKER_CROSSING_X);
            map_->GetAllElementsWithTypes(get_types, detected_crossings);
            if (!detected_crossings.empty() &&
                    MapHelper::IsElementOrientatedToCar(
                        map_, *(el.get()), (tMapData)0.8 * M_PI)) {
                BOOST_FOREACH (const tSptrMapElement& s, detected_crossings) {
                    if (s->GlobalDistance(*el) < SIGN_CROSSING_THRESHOLD_) {
                        if (s->GetType() == STREET_MARKER_CROSSING_T)
                            AddEventToPQ(s, T_CROSSING_RIGHT_OF_WAY);
                        else if (s->GetType() == STREET_MARKER_CROSSING_X)
                            AddEventToPQ(s, X_CROSSING_RIGHT_OF_WAY);
                        break;
                    }
                }
            } else {
                LOG_ERROR_PRINTF("SM no crossing at give way sign detected");
            }

            break;
        }
        case STREET_PARKING_SPOT:
        case LM_STREET_PARKING_SPOT:
               AddEventToPQ(el, PARKING_SPOT);
            break;

        //case PEDESTRIAN_CHILD:
        case STREET_TRAFFIC_SIGN_SPEED_50:
        case STREET_TRAFFIC_ROAD_WORKS:
            AddEventToPQ(el, SLOW_DOWN);
            break;
        case STREET_TRAFFIC_SIGN_SPEED_100:
            AddEventToPQ(el, SPEED_UP);
            break;
        case PEDESTRIAN_CHILD:
        case PEDESTRIAN_ADULT:{
            std::vector<tSptrMapElement> detected_zebra_crossings;
            const std::vector<MapElementType> get_types =
                boost::assign::list_of(STREET_MARKER_ZEBRA);
            map_->GetAllElementsWithTypes(get_types, detected_zebra_crossings);

            // print out and compare distances with filter
        std::cout << "the time passed:" << (_clock->GetStreamTime() - time_last_zebra_crossing_) << std::endl;
            if (!detected_zebra_crossings.empty() && ((_clock->GetStreamTime() - time_last_zebra_crossing_) >
        10 * 1e6)) {
                BOOST_FOREACH (const tSptrMapElement& s,
                               detected_zebra_crossings) {
                    if (PedestrianOnCrossing(s)) {
                        AddEventToPQ(s, PEDESTRIAN_ON_ZEBRA_CROSSING);
                        break;
                    } else if (PedestrianAtCrossingLeft(el) || PedestrianAtCrossingRight(el)) {
                        AddEventToPQ(s, PEDESTRAIN_WAITS_AT_ZEBRA_CROSSING);
                        break;
                    }
                }
            }
        if (current_event_.event_type == X_CROSSING_RIGHT_OF_WAY ||
                current_event_.event_type == X_CROSSING_GIVE_WAY ||
                current_event_.event_type == X_CROSSING_STOP ||
                current_event_.event_type == X_CROSSING_RIGHT_OF_WAY_RIGHT ||
                current_event_.event_type == T_CROSSING_RIGHT_OF_WAY ||
                current_event_.event_type == T_CROSSING_GIVE_WAY ||
                current_event_.event_type == T_CROSSING_STOP ||
                current_event_.event_type == T_CROSSING_RIGHT_OF_WAY_RIGHT) {
                car_detected_ = true;
                LOG_INFO_PRINTF("second: brabie is detected");
                ProcessEventQueue();
            }
            break;
        }

        case CAR: {
            //LOG_WARNING_PRINTF("TODO car");
            if (current_event_.event_type == X_CROSSING_RIGHT_OF_WAY ||
                current_event_.event_type == X_CROSSING_GIVE_WAY ||
                current_event_.event_type == X_CROSSING_STOP ||
                current_event_.event_type == X_CROSSING_RIGHT_OF_WAY_RIGHT ||
                current_event_.event_type == T_CROSSING_RIGHT_OF_WAY ||
                current_event_.event_type == T_CROSSING_GIVE_WAY ||
                current_event_.event_type == T_CROSSING_STOP ||
                current_event_.event_type == T_CROSSING_RIGHT_OF_WAY_RIGHT) {
                car_detected_ = true;
                //LOG_INFO_PRINTF("car is detected");
                ProcessEventQueue();
            }
            break;
        }
        default:
            // todo default
            break;
    }
}

//--------------------------------------------------------------------------------------------------------
tResult StateMachine::Init(tInitStage stage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))
    // in StageFirst you can create and register your static pins.

    if (stage == StageFirst) {
        tResult nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");

    } else if (stage == StageNormal) {
        map_ = frAIburg::map::getInstance();

        // In this stage you would do further initialisation and/or create your
        // dynamic pins. Please take a look at the demo_dynamicpin example for
        // further reference.
        // map_->RegisterEventListener(this);
    } else if (stage == StageGraphReady) {
        GetAllProperties();
        // All pin connections have been established in this stage so you can
        // query your pins about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further
        // reference.

        tResult nResult = InitBufferPins();
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create buffer");
        // SkipJuryStates();
        // SendBehaviour();
        time_last_zebra_crossing_ = 0;
        StartThread();
    }
    RETURN_NOERROR;
}

tResult StateMachine::InitBufferPins() {
    vector<string> ids =
        boost::assign::list_of("f32Value")("ui32ArduinoTimestamp");
    pin_out_emergency_stop_speed_.StageGraphReadySetIDOrder(ids);

    vector<string> id2s = boost::assign::list_of("bEmergencyStop");
    pin_out_emergency_stop_bool_.StageGraphReadySetIDOrder(id2s);

    vector<string> id3s = boost::assign::list_of("bValue");
    pin_out_set_state_ready_.StageGraphReadySetIDOrder(id3s);
    pin_out_set_state_running_.StageGraphReadySetIDOrder(id3s);
    pin_out_set_state_complete_.StageGraphReadySetIDOrder(id3s);
    pin_out_maneuver_completed_.StageGraphReadySetIDOrder(id3s);

    vector<string> id4s = boost::assign::list_of("behaviour_id")(
        "behaviour_next_id")("object_id")("speed_id")("timestamp")("is_finished");
    pin_out_behaviour_.StageGraphReadySetIDOrder(id4s);

    vector<string> id5s = boost::assign::list_of("light_id")("switch_bool");
    pin_out_light_controller_.StageGraphReadySetIDOrder(id5s);

    RETURN_NOERROR;
}

tResult StateMachine::CreateInputPins(__exception) {
    slim::register_pin_func func = &StateMachine::RegisterPin;

    RETURN_IF_FAILED(pin_in_jurystruct_.FirstStageCreate(
        this, func, "Jury_Struct", "tJuryStruct"));
    RETURN_IF_FAILED(pin_in_emergency_stop_.FirstStageCreate(
        this, func, "EmergencyStopBool", "tJuryEmergencyStop"));
    RETURN_IF_FAILED(pin_in_bool_test_.FirstStageCreate(
        this, func, "test bool input", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_in_maneuver_list_.FirstStageCreate(
        this, func, "Maneuver_List", "tManeuverList"));
    RETURN_IF_FAILED(pin_in_planner_status_.FirstStageCreate(
        this, func, "Planner_status", "tPlannerStatus"));
    RETURN_NOERROR;
}

tResult StateMachine::CreateOutputPins(__exception) {
    slim::register_pin_func func = &StateMachine::RegisterPin;

    // tJuryStruct added as output to block sending to jury if the car has
    // no postion (so that the state ready is not reached if it is send in the
    // jury tool)
    RETURN_IF_FAILED(pin_out_jurystruct_.FirstStageCreate(
        this, func, "Jury_Struct_Out", "tJuryStruct"));
    RETURN_IF_FAILED(pin_out_emergency_stop_bool_.FirstStageCreate(
        this, func, "EmergencyStopJuryBoolStateout", "tJuryEmergencyStop"));
    // create speed outout zero empergency
    RETURN_IF_FAILED(pin_out_emergency_stop_speed_.FirstStageCreate(
        this, func, "EmergencyStopSpeed", "tSignalValue"));
    // create speed outout zero empergency
    RETURN_IF_FAILED(pin_out_set_state_ready_.FirstStageCreate(
        this, func, "Set_State_Ready", "tBoolSignalValue"));
    // create speed outout zero empergency
    RETURN_IF_FAILED(pin_out_set_state_running_.FirstStageCreate(
        this, func, "Set_State_Running", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_set_state_complete_.FirstStageCreate(
        this, func, "Set_State_Complete", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_maneuver_completed_.FirstStageCreate(
        this, func, "Increment_manouver", "tBoolSignalValue"));
    RETURN_IF_FAILED(pin_out_behaviour_.FirstStageCreate(
        this, func, "behaviour_output", "tBehaviour"));
    RETURN_IF_FAILED(pin_out_light_controller_.FirstStageCreate(
        this, func, "light_output", "tLightCommand"));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult StateMachine::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));
    get_all_properties();

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult StateMachine::Stop(__exception) {
    // running_ok_ = false; TODO
    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}

tResult StateMachine::UpdateProperties(ucom::IException** __exception_ptr) {
    get_all_properties();
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void StateMachine::set_all_properties(void) {}

// ____________________________________________________________________________
void StateMachine::get_all_properties(void) {}

// ____________________________________________________________________________
tResult StateMachine::Shutdown(tInitStage stage, __exception) {
    // In each stage clean up everything that you initialized in the
    // corresponding stage during Init. Pins are an exception:
    // - The base class takes care of static pins that are members of this
    //   class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please
    //   see the demo_dynamicpin example for further reference.
    if (stage == StageGraphReady) {
        StopThread();
        TransmitLightCommand(ALL_LIGHTS, tFalse);
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult StateMachine::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                 tInt param2, IMediaSample* pMediaSample) {
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(source);
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        // by comparing it to our member pin variable we can find out which
        // pin received the sample
        if (pin_in_emergency_stop_.isSource(source)) {
            ProcessEmergencyStop(pMediaSample);
        } else if (pin_in_jurystruct_.isSource(source)) {
            ProcessJuryStructInput(pMediaSample);
        } else if (pin_in_maneuver_list_.isSource(source)) {
            ProcessManeuverListInput(pMediaSample);
        } else if (pin_in_bool_test_.isSource(source)) {
//            if (true)  // maneuver_requested_ == TODO
//            {
//                IncrementManeuver();
//                maneuver_requested_ = false;
//            }
//            ProcessEvent(EvPlannerCompleted());
//            event_change_locked_ = false;
//            IF_DEBUG_LOG_PRINTF("SM Recieved Completed and acting");

//            SendBehaviour();
        } else if (pin_in_planner_status_.isSource(source)) {
            ProcessPlannerStatus(pMediaSample);
        }
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult StateMachine::ProcessEmergencyStop(IMediaSample* pMediaSample) {
    // TODO changed for test event only to turn on off the break lights
    tJuryEmergencyStop* pSampleData = NULL;
    if (IS_OK(pin_in_emergency_stop_.ReadNoID_start(
            pMediaSample, (const tVoid**)&pSampleData,
            sizeof(tJuryEmergencyStop)))) {
        tBool is_emergency_stop = tFalse;
        is_emergency_stop = pSampleData->bEmergencyStop;
        pin_in_emergency_stop_.ReadNoID_end(pMediaSample,
                                            (const tVoid**)pSampleData);
        if (is_emergency_stop == tTrue) {
            TransmitLightCommand(BRAKE_LIGHTS, tTrue);
            // ProcessEvent(EvEmergency_stop());
            // TransmitFloatValue(pin_out_emergency_stop_speed_, 0, 0);
        } else if (is_emergency_stop == tFalse) {
            TransmitLightCommand(BRAKE_LIGHTS, tFalse);
            ProcessEvent(EvRoad_is_free());
        }

    } else
        LOG_WARNING_PRINTF("error read emergency stop failed!");

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult StateMachine::ProcessJuryStructInput(IMediaSample* pMediaSample) {
    tJuryStruct sampleData;
    tJuryStruct* pSampleData = NULL;
    sampleData.i8ActionID = -2;
    sampleData.i16ManeuverEntry = -1;
    if (IS_OK(pin_in_jurystruct_.ReadNoID_start(
            pMediaSample, (const tVoid**)&pSampleData, sizeof(tJuryStruct)))) {
        // sampleData = *pSampleData;
        sampleData.i8ActionID = pSampleData->i8ActionID;
        sampleData.i16ManeuverEntry = pSampleData->i16ManeuverEntry;
        pin_in_jurystruct_.ReadNoID_end(pMediaSample,
                                        (const tVoid**)pSampleData);

        //            LOG_INFO(cString::Format("ACTION Id -> %d: yoyoo ",
        //            i8ActionID));
        //            LOG_INFO(cString::Format("Maneouver Id -> %d: ",
        //            i16entry));
        switch (sampleData.i8ActionID) {
            case -1:  // action :get ready
                // EM_LOG("JURY: STOPPPPP");
                ProcessEvent(EvJury_stop());
                // TransmitBoolValue(pin_out_set_state_ready_, tFalse);
                TransmitBehaviour(STOP, -1, -1, ST_STOP);
                TransmitBoolValue(pin_out_set_state_running_, tFalse);
                break;
            case 0:  // action :get ready
                IF_DEBUG_LOG_PRINTF("JURY: GET READY");

                // the jury is sending the the start command
                // quick fix when map pos known the ready state send to the jury
                // todo(phil, markus): make this nice
                jury_ready_request_ = true;
                break;
            case 1:  // action :start
                IF_DEBUG_LOG_PRINTF("JURY: START");
                current_maneuver_id_ = sampleData.i16ManeuverEntry;
                IF_DEBUG_LOG_PRINTF("Jury: Next Action is -> %s",
                                    GetCurrentManeuver().c_str());
                event_change_locked_ = false;
                //ProcessEvent(EvJury_start()); //TODO:bring it back?

                TransmitBoolValue(pin_out_set_state_running_, tTrue);
                std::string jury_action = GetCurrentManeuver();
                if (jury_action == "left") {
			ProcessEvent(EvJury_start());
                } else if (jury_action == "right") {
            		ProcessEvent(EvJury_start());
                } else if (jury_action == "straight") {
            		ProcessEvent(EvJury_start());
                } else if (jury_action ==
                           "cross_parking") {  // todo: this does nothing
            		ProcessEvent(EvJury_start());
                } else if (jury_action == "pull_out_left") {
            unsigned int timer = 0;
            while ((!PullOutAllowed() && ++timer <= 20)) {
                    boost::this_thread::sleep_for(boost::chrono::seconds(1));
                }
            ProcessEvent(EvJury_start());
                    ProcessEvent(EvNext_pull_left());
                } else if (jury_action == "pull_out_right") {
            unsigned int timer = 0;
            while ((!PullOutAllowed() && ++timer <= 20)) {
                    boost::this_thread::sleep_for(boost::chrono::seconds(1));
                }
            ProcessEvent(EvJury_start());
                    ProcessEvent(EvNext_pull_right());
                }
                else{
                    ProcessEvent(EvJury_start());
                }
                if (map_->IsCarPosKnown()) {
                    pin_out_jurystruct_.TransmitStruct<tJuryStruct>(
                        &sampleData, _clock->GetStreamTime());
                }
                break;
        }

    } else
        LOG_ERROR_PRINTF("SM read jury struct failed!");
    SendBehaviour();
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult StateMachine::ProcessPlannerStatus(IMediaSample* pMediaSample) {
    tPlannerStatus* pSampleData = NULL;
    if (IS_OK(pin_in_planner_status_.ReadNoID_start(pMediaSample,
                                                    (const tVoid**)&pSampleData,
                                                    sizeof(tPlannerStatus)))) {
        planner_status_ = static_cast<planner_status>(pSampleData->status);
        //      if (LOG_DEBUG_) {printf("Planner: Received behaviour %s and ID
        //      %i\n", behaviour_type_string[behaviour_curr_], objectid_curr_);}
        pin_in_planner_status_.ReadNoID_end(pMediaSample,
                                            (const tVoid**)pSampleData);
        switch (planner_status_) {
            case PS_SUCCESSFUL: {
                // ProcessEvent(EvPlannerSuccsesful());
                IF_DEBUG_LOG_PRINTF("Received successful and ignored");
                break;
            }
            case PS_COMPLETED: {
                if (maneuver_requested_ == true) {
                    IncrementManeuver();
                    maneuver_requested_ = false;
                }
                ProcessEvent(EvPlannerCompleted());
                event_change_locked_ = false;
                IF_DEBUG_LOG_PRINTF("Recieved Completed and acting");

                // Phil: SendBehavior sleeps sometimes. As this function here
                // runs in OnPinEvent it
                // blocks other stuff. Quick fix is to run in a thread. In
                // theory there
                // should be only one thread but we print an error if there is
                // practice.
                if (send_behavior_thread_is_active) {
                    std::cout << "\n\n\nERROR! THIS SHOULD NOT HAPPEN! The "
                                 "planner sent a"
                              << "PS_COMPLETED while the state machine was "
                                 "still in function SendBehaviour."
                              << "\n\n\n"
                              << std::endl;
                }
                send_behavior_thread_is_active = true;
                IF_DEBUG_LOG_PRINTF(
                    "StateMachine: Starting SendBehavior Thread");
                boost::thread(&StateMachine::SendBehaviour, this);
                break;
            }
            default:
                break;
        }
        RETURN_NOERROR;
    }
    LOG_ERROR_PRINTF("StateMachine::Planner Status failed");
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult StateMachine::ProcessManeuverListInput(IMediaSample* pMediaSample) {
    // focus for sample read lock
    cObjectPtr<IMediaTypeDescription> description_ =
        (pin_in_maneuver_list_.description_);
    __adtf_sample_read_lock_mediadescription(description_, pMediaSample,
                                             pCoder);

    std::vector<tSize> vecDynamicIDs;

    // retrieve number of elements by providing NULL as first paramter
    tSize szBufferSize = 0;
    if (IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize))) {
        // create a buffer depending on the size element
        std::cout << "ProcessManeuverListInput! This print is here to"
                  << "check if the delete causes problems. " << std::endl;
        tChar* pcBuffer = new tChar[szBufferSize];
        std::cout << "tChar* pcBuffer = new tChar[szBufferSize];" << std::endl;
        vecDynamicIDs.resize(szBufferSize);
        // get the dynamic ids (we already got the first "static" size element)
        if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()),
                                              szBufferSize))) {
            // iterate over all elements
            for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx) {
                // get the value and put it into the buffer
                pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
            }

            // set the resulting char buffer to the string object
            m_strManeuverFileString = (const tChar*)pcBuffer;

            // EM_LOG(cString::Format(m_strManeuverFileString));
        }

        // cleanup the buffer
        delete pcBuffer;
        std::cout << "delete pcBuffer;" << std::endl;
    }
    LoadManeuverList();

    RETURN_NOERROR;
}

tResult StateMachine::LoadManeuverList() {
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;
    // read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems))) {
        maneuver_list_vector_.clear();
        // iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin();
             itSectorElem != oSectorElems.end(); ++itSectorElem) {
            // if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");
            int maneuver_counter = 0;
            if (IS_OK((*itSectorElem)
                          ->FindNodes("AADC-Maneuver", oManeuverElems))) {
                // iterate through maneuvers
                for (cDOMElementRefList::iterator itManeuverElem =
                         oManeuverElems.begin();
                     itManeuverElem != oManeuverElems.end(); ++itManeuverElem) {
                    tAADC_Maneuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = (*itManeuverElem)->GetAttribute("action");
                    sector.maneuverList.push_back(man);
                    maneuver_counter++;
                    maneuver_list_vector_.push_back(man);
                }
            }
        }
    }
    if (oSectorElems.size() > 0) {
        ProcessEvent(EvManeuver_recieved());

    } else {
        LOG_ERROR_PRINTF("State Machine: no valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

tResult StateMachine::TransmitFloatValue(slim::OutputPin& oPin, tFloat32 value,
                                         tUInt32 timestamp) {
    vector<const void*> vals2 =
        boost::assign::list_of((const void*)&value)((const void*)&timestamp);
    if (IS_FAILED(oPin.Transmit(vals2, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF("failed sending float");
    }
    RETURN_NOERROR;
}

tResult StateMachine::TransmitBoolValue(slim::OutputPin& oPin, tBool bValue) {
    vector<const void*> vals2 = boost::assign::list_of((const void*)&bValue);
    if (IS_FAILED(oPin.Transmit(vals2, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF("failed sending bool ");
    }
    RETURN_NOERROR;
}

tResult StateMachine::TransmitBehaviour(tInt32 behaviour_id,
                                        tInt32 behaviour_next_id,
                                        tInt32 object_id, tInt32 speed_id) {
    tTimeStamp curr_time = _clock->GetStreamTime();
    tBool is_finished = false;
    vector<const void*> vals1 =
        boost::assign::list_of((const void*)&behaviour_id)(
            (const void*)&behaviour_next_id)((const void*)&object_id)(
            (const void*)&speed_id)((const void*)(&curr_time))((const void*)(&is_finished));
    IF_DEBUG_LOG_PRINTF("sending behaviour type %i", behaviour_id);
    if (IS_FAILED(
            pin_out_behaviour_.Transmit(vals1, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF("failed sending behaviour ");
    }
    RETURN_NOERROR;
}
tResult StateMachine::TransmitLightCommand(tInt32 light_id, tBool switch_bool) {
    vector<const void*> vals1 = boost::assign::list_of((const void*)&light_id)(
        (const void*)&switch_bool);
    if (IS_FAILED(pin_out_light_controller_.Transmit(
            vals1, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF("failed sending light command ");
    }
    RETURN_NOERROR;
}

void StateMachine::IncrementManeuver() {
    if (current_maneuver_id_ + 1 ==
        maneuver_list_vector_.size())  // if(last maneuver)
    {
        LOG_INFO_PRINTF("All Maneuvers are completed");
        TransmitBoolValue(pin_out_set_state_complete_, tTrue);
        TransmitBehaviour(STOP, -1, -1, ST_STOP);
        ProcessEvent(EvCompleted_all_maneuvers());
        return;
    } else if (current_maneuver_id_ + 1 > maneuver_list_vector_.size()) {
        LOG_ERROR_PRINTF("SM out of maneuver");
    } else {
        current_maneuver_id_++;
        TransmitBoolValue(pin_out_maneuver_completed_, tTrue);
        return;
    }
}

std::string StateMachine::GetCurrentManeuver() {
    std::string jury_action = "noop";  // TODO check if default
    if (current_maneuver_id_ < maneuver_list_vector_.size()) {
        jury_action = maneuver_list_vector_[current_maneuver_id_].action;
    } else {
        LOG_ERROR_PRINTF("SM out of maneuver");
    }
    return jury_action;
}

// void StateMachine::IncrementManeuver()
//{
//    if(current_maneuver_pos_+1 ==
//    maneuver_counter_vector_.at(current_sector_pos_)) //if(last maneuver in a
//    sector)
//    {
//        if(current_sector_pos_+1 == sector_counter_) //if(last sector)
//        {
//            LOG_INFO_PRINTF("All Maneuvers are completed");
//            TransmitBoolValue(pin_out_set_state_complete_, tTrue);
//            return;

//        }
//        else
//        {
//            current_sector_pos_++;
//            current_maneuver_pos_ = 0;
//        }
//    }
//    else
//    {
//        current_maneuver_pos_++;
//    }
//    TransmitBoolValue(pin_out_maneuver_completed_, tTrue);
//    IF_DEBUG_LOG_PRINTF("Current Sector ID:%i, Current Maneuver ID: %i",
//                        current_sector_pos_, current_maneuver_pos_);
//    //TODO? send behaviour?
//}

// ____________________________________________________________________________
void StateMachine::StartThread() {
    running_ok_ = true;
    thread_process_queue_ =
        boost::thread(boost::bind(&StateMachine::Run, this));
}

// ____________________________________________________________________________
void StateMachine::Run() {
    // init delay
    boost::this_thread::sleep_for(boost::chrono::seconds(5));

    while (running_ok_) {
        if (jury_ready_request_ && map_->IsCarPosKnown()) {
            LOG_INFO_PRINTF("SM transmitting jury ready now");
            TransmitBoolValue(pin_out_set_state_ready_, tTrue);
            jury_ready_request_ = false;  // TODO we can never restart
            // This should not be in the run call, HOTFIX!
            ProcessEvent(EvJury_get_ready());
        }


        const std::vector<tSptrMapElement> current_map_elements =
            map_->GetAllElements();

        EventQueueClear();  // empty priority queue
        current_highest_priority_ = std::numeric_limits<int>::max();
        BOOST_FOREACH (const tSptrMapElement& el, current_map_elements) {
            if (MapElementRoadSign::IsRoadSign(el->GetType())) {
                if (el->GetType() != STREET_TRAFFIC_SIGN_POSITION_MARKER) {
                    // cast to MapElementRoadSign
                    // add SIGN_SENSOR and ignore SIGN_LANDMARK
                    MapElementRoadSign* el_sign =
                        dynamic_cast<MapElementRoadSign*>(el.get());
                    if (el_sign && el_sign->GetSignSubType() == SIGN_SENSOR) {
                        ProcessElement(el);
                    }
                }
            } else {
                ProcessElement(el);
            }
        }

        size_t state_hash = GetActiveStateHash();
        if(IsParkingOrChildAhead() && (state_hash == typeid(StNormal_driving).hash_code())) {
            //LOG_INFO_PRINTF("TRIGGERED SLOW DOWN");
            TransmitBehaviour(FOLLOW_LANE, -1, -1, ST_SPEED_LOW);
        }else if(!IsParkingOrChildAhead() && (state_hash == typeid(StNormal_driving).hash_code()) && current_event_.event_type != SPEED_UP){
        TransmitBehaviour(FOLLOW_LANE, -1, -1, ST_SPEED_HIGH);
    }

        // highlight the map el with the highest prio in a different color
        if (poperty_debug_enabled_) {
            if (!EventQueueEmpty()) {
                // change color the map el with the highest prio
                DebugMapElementHighLight(EventQueueTop().event_id, "darkblue");
            }
        }

        ProcessEventQueue();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
}

// ____________________________________________________________________________
void StateMachine::StopThread() {
    running_ok_ = false;
    // pause shutdown until queue thread finished to prevent crashes
    // WIll fail thread_process_queue_.join();
}

// ____________________________________________________________________________
void StateMachine::DebugMapElementHighLight(tMapID id_to_highlight,
                                            const char* color) {
    // ONLY call in debug modus!
    static tMapID prev_id = MAP_DEFAULT_ID;
    static std::string prev_color;
    // reset old color
    if (id_to_highlight != prev_id) {
        // clear previous color
        if (prev_id != MAP_DEFAULT_ID) {
            tSptrMapElement prev_el = map_->GetElement(prev_id);
            if (!prev_el) return;
            if (prev_el) prev_el->user_color_ui_ = prev_color;
        }
        // set color for the new element
        tSptrMapElement prev_el = map_->GetElement(id_to_highlight);
        if (!prev_el) return;
        prev_color = prev_el->user_color_ui_;
        if (prev_el) prev_el->user_color_ui_ = color;
        prev_id = id_to_highlight;
    }
}

// ____________________________________________________________________________
void StateMachine::GetAllProperties(void) {
    poperty_debug_enabled_ =
        GetPropertyBool("ADTF_PROPERTY_DEBUG_ENABLED_NAME");
}

// ____________________________________________________________________________
void StateMachine::SetAllProperties(void) {
    SetPropertyBool("ADTF_PROPERTY_DEBUG_ENABLED_NAME", false);
}

bool StateMachine::IsParkingRequested(const std::string& jury_command) {
    return boost::contains(jury_command, "cross_parking");
}

bool StateMachine::GetParkingMapID(const std::string& jury_command,
                                   tMapID* map_parking_id) {
    // string cat then
    // boost::lexical_cast<tdata>( with catch!
    int jury_id = -1;
    try {
        jury_id = boost::lexical_cast<int>(
            jury_command.substr(jury_command.find(" ") + 1));
    } catch (std::exception const& e) {
        LOG_ERROR_PRINTF("SM jury cmd get parking id failed");
        return false;
    }
    LOG_INFO_PRINTF("JURY IDDDD:%i", jury_id);

    std::vector<tSptrMapElement> parking_spaces;
    const std::vector<MapElementType> get_types =
        boost::assign::list_of(LM_STREET_PARKING_SPOT)(STREET_PARKING_SPOT);
    map_->GetAllElementsWithTypes(get_types, parking_spaces);

    if (parking_spaces.empty()) {
        LOG_ERROR_PRINTF("SM: Couldn't find parking spaces");
    }

    BOOST_FOREACH (const tSptrMapElement& p, parking_spaces) {
        const MapElementParking* el_f =
            dynamic_cast<const MapElementParking*>(p.get());
        if (el_f) {
            // safely casted to MapElementParking
            if (el_f->GetJuryID() == jury_id) {
                *map_parking_id = el_f->GetID();
                LOG_INFO_PRINTF("SM parking map el found %s",
                                    el_f->ToString().c_str());
                return true;
            }
        } else {
            LOG_ERROR_PRINTF("SM parking cased failed");
        }
    }
    LOG_ERROR_PRINTF("SM jury parking ID not foud in map for jury id %d",
                     jury_id);
    return false;
}
