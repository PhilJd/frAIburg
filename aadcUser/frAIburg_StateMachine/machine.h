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
#ifndef MACHINE_HPP
#define MACHINE_HPP

#include <boost/mpl/list.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>
#include "adtf_log_macros.h"

#include "state_machine.h"

namespace sc = boost::statechart;
namespace mpl = boost::mpl;
/* St: For leaf state, PS for parent state */
struct StInitial;  // Initial state has to be declared first

struct Machine : sc::state_machine<Machine, StInitial> {
    Machine (){    }

};  // Declare the state machine < Machine_name, Initial_state >


// For simplicity of reading, all states are declared at the start

// Jury communication states
struct StStart_up;
struct StManeuver_recieved;
struct StReady_to_start;
struct StAll_maneuver_complete;

struct PStDriving; // main parent state

//lane following states
struct PStLane_keeping; // parent of below states
struct StNormal_driving;
struct StObstacle_ahead;
struct StSlow_driving;

//emergy stop states
struct PStEmergency_stop;

//crossing states
struct PStCrossing;
struct StStop_crossing_ahead;
struct StGoTo_crossing_ahead;
struct StStop_crossing_at;
struct StGoTo_crossing_at;
struct StCrossing_in;

//Parking states
struct PStParking;
struct StNext_maneuver_parking;
struct StParking_infront;
struct StParking_at;
struct StParked;
struct StPull_left;
struct StPull_right;

//Zebrea crossing states
struct PStZebra_crossing;
struct StEmpty_Zebra_crossing;
struct StStop_zebra_ahead_pedestrian_at;
struct StStop_zebra_ahead_pedestrian_on;
struct StStop_zebra_at_pedestrian_at;
struct StStop_zebra_at_pedestrian_on;
/*  EVENTS BELOW */
// Jury communication events
struct EvManeuver_recieved : sc::event<EvManeuver_recieved> {};
struct EvJury_get_ready : sc::event<EvJury_get_ready> {};
struct EvJury_start : sc::event<EvJury_start> {};
struct EvJury_stop : sc::event<EvJury_stop> {};
struct EvIncrement_maneuver_id : sc::event<EvIncrement_maneuver_id> {};
struct EvCompleted_all_maneuvers : sc::event<EvCompleted_all_maneuvers> {};
struct EvRestart_section : sc::event<EvRestart_section> {};

struct EvEmergency_stop : sc::event<EvEmergency_stop> {};
struct EvRoad_is_free : sc::event<EvRoad_is_free> {};
struct EvSlow_down : sc::event<EvSlow_down> {};


struct EvStop_at_crossing_ahead : sc::event<EvStop_at_crossing_ahead> {};
struct EvGoTo_crossing_ahead : sc::event<EvGoTo_crossing_ahead> {};
struct EvCrossing_at : sc::event<EvCrossing_at> {};
struct EvCrossing_in : sc::event<EvCrossing_in> {};
struct EvCrossing_finished : sc::event<EvCrossing_finished> {};

struct EvNext_maneuver_park : sc::event<EvNext_maneuver_park> {};
struct EvParking_GoTo_infront : sc::event<EvParking_GoTo_infront> {};
struct EvParking_GoTo_parking : sc::event<EvParking_GoTo_parking> {};
struct EvParking_stop_at_parking : sc::event<EvParking_stop_at_parking> {};
struct EvPark : sc::event<EvPark> {};


struct EvEmpty_zebra_crossing : sc::event<EvEmpty_zebra_crossing> {};
struct EvPedestrian_on_crossing : sc::event<EvPedestrian_on_crossing> {};
struct EvPedestrian_at_crossing : sc::event<EvPedestrian_at_crossing> {};
struct EvTime_up : sc::event<EvTime_up> {};


struct EvNext_pull_right : sc::event<EvNext_pull_right> {};
struct EvNext_pull_left : sc::event<EvNext_pull_left> {};

struct EvPlannerSuccsesful : sc::event<EvPlannerSuccsesful> {};
struct EvPlannerCompleted : sc::event<EvPlannerCompleted> {};

struct EvObstacle_ahead : sc::event<EvObstacle_ahead> {};




struct StInitial : sc::simple_state<StInitial, Machine> {
    StInitial() {
        LOG_INFO_PRINTF("State Machine: Car Is On !");  // entry
    }
    typedef sc::transition<EvManeuver_recieved /*trans. to state StStart_up*/, StStart_up> reactions;
};

struct PStDriving : sc::simple_state<PStDriving /* Defined state*/, Machine /*Parent*/, 
                            PStLane_keeping /* init. Child */, sc::has_deep_history> /*keep last state*/ {
    PStDriving() {}
    typedef mpl::list<sc::transition<EvEmergency_stop, PStEmergency_stop>,
                      sc::transition<EvJury_stop, StReady_to_start>,
                      sc::transition<EvCompleted_all_maneuvers, StAll_maneuver_complete> >
        reactions;
};
// _________________________________________Jury_States________________________________________________________
struct StStart_up : sc::simple_state<StStart_up, Machine> {
    StStart_up() {
        LOG_INFO_PRINTF(
            "State Machine: Car is at StartUp state - Maneuver list recieved "
            "!");  // entry
    }
    typedef sc::transition<EvJury_get_ready, StReady_to_start> reactions;
};

struct StReady_to_start : sc::simple_state<StReady_to_start, Machine> {
    StReady_to_start() {
        LOG_INFO_PRINTF(
            "State Machine: Car is at get_ready state - waiting for jury start "
            "command !");  // entry
    }
    /*history: if history, go to last state !? */                                            // LOOK IT UP!
    typedef sc::transition<EvJury_start, sc::deep_history<PStLane_keeping> >
        reactions;
};

struct StAll_maneuver_complete : sc::simple_state<StAll_maneuver_complete, Machine> {
    StAll_maneuver_complete() {
        LOG_INFO_PRINTF(
            "State Machine: all maneuvers are completed!");  // entry
    }
};

// _________________________________________Lane_keeping_States________________________________________________________
struct PStLane_keeping
    : sc::simple_state<PStLane_keeping, PStDriving, StNormal_driving> {
    PStLane_keeping() {
        LOG_INFO_PRINTF(
            "State Machine: Im driving and keeping lane !");  // entry
    }
    typedef mpl::list<
        sc::transition<EvStop_at_crossing_ahead, StStop_crossing_ahead>,
        sc::transition<EvGoTo_crossing_ahead, StGoTo_crossing_ahead>,
        sc::transition<EvEmergency_stop, PStEmergency_stop>,
        sc::transition<EvNext_maneuver_park, PStParking>,
        sc::transition<EvJury_stop, StReady_to_start>,
        sc::transition<EvNext_pull_right, StPull_right>,
        sc::transition<EvNext_pull_left, StPull_left>,
        sc::transition<EvObstacle_ahead, StObstacle_ahead>,
        sc::transition<EvEmpty_zebra_crossing, StEmpty_Zebra_crossing>,
        sc::transition<EvPedestrian_on_crossing, StStop_zebra_ahead_pedestrian_on>,
        sc::transition<EvPedestrian_at_crossing, StStop_zebra_ahead_pedestrian_at>,
        sc::transition<EvNext_maneuver_park, StNext_maneuver_parking>,
        sc::transition<EvSlow_down, StSlow_driving> >

        reactions;
        /* Connections defined for parent also apply for children! */
};

struct StNormal_driving : sc::simple_state<StNormal_driving, PStLane_keeping> {
    StNormal_driving() {}  // entry
};

struct StSlow_driving : sc::simple_state<StSlow_driving, PStLane_keeping> {
    StSlow_driving() {
        LOG_INFO_PRINTF("State Machine: follow lane slow!");  // entry
    }  // entry
    typedef sc::transition<EvRoad_is_free, StNormal_driving >
        reactions;
};

// _________________________________________Zebra_crossing_States________________________________________________________
struct PStZebra_crossing
    : sc::simple_state<PStZebra_crossing, PStDriving, StEmpty_Zebra_crossing> {
    PStZebra_crossing() {
        LOG_INFO_PRINTF("State Machine: Zebra Crossing ahead!");  // entry
    }
    typedef mpl::list<sc::transition<EvStop_at_crossing_ahead, StStop_crossing_ahead>,
        //sc::transition<EvGoTo_crossing_ahead, StGoTo_crossing_ahead>,
        sc::transition<EvEmergency_stop, PStEmergency_stop>,
        sc::transition<EvNext_maneuver_park, PStParking>,
        sc::transition<EvJury_stop, StReady_to_start>,
        sc::transition<EvNext_pull_right, StPull_right>,
        sc::transition<EvNext_pull_left, StPull_left>,
        sc::transition<EvObstacle_ahead, StObstacle_ahead>,
        sc::transition<EvNext_maneuver_park, StNext_maneuver_parking>,
        sc::transition<EvSlow_down, StSlow_driving> >
        reactions;
};

struct StEmpty_Zebra_crossing
    : sc::simple_state<StEmpty_Zebra_crossing, PStZebra_crossing> {
    StEmpty_Zebra_crossing() {
        LOG_INFO_PRINTF("State Machine: Zebra crossing is empty !");  // entry
    }
    typedef mpl::list<
		sc::transition<EvStop_at_crossing_ahead, StStop_crossing_ahead>,
        sc::transition<EvGoTo_crossing_ahead, StGoTo_crossing_ahead>,
        sc::transition<EvPedestrian_at_crossing, StStop_zebra_ahead_pedestrian_at>,
        sc::transition<EvPedestrian_on_crossing, StStop_zebra_ahead_pedestrian_on>,
        sc::transition<EvRoad_is_free, PStLane_keeping> >
        reactions;
};

struct StStop_zebra_ahead_pedestrian_at
    : sc::simple_state<StStop_zebra_ahead_pedestrian_at, PStZebra_crossing> {
    StStop_zebra_ahead_pedestrian_at() {
        LOG_INFO_PRINTF("State Machine: Stop at Zebra ahead, pedestrian at crossing !");  // entry
    }
    typedef mpl::list<
        sc::transition<EvPlannerCompleted, StStop_zebra_at_pedestrian_at>,
        sc::transition<EvPedestrian_on_crossing, StStop_zebra_ahead_pedestrian_on> >
        reactions;
};

struct StStop_zebra_ahead_pedestrian_on
    : sc::simple_state<StStop_zebra_ahead_pedestrian_on, PStZebra_crossing> {
    StStop_zebra_ahead_pedestrian_on() {
        LOG_INFO_PRINTF("State Machine: Stop at Zebra ahead, pedestrian on crossing !");  // entry
    }
    typedef mpl::list<
        sc::transition<EvPlannerCompleted, StStop_zebra_at_pedestrian_on>,
        //sc::transition<EvEmpty_zebra_crossing, PStLane_keeping>,
        sc::transition<EvPedestrian_at_crossing, PStLane_keeping> >
        reactions;
};

struct StStop_zebra_at_pedestrian_at
    : sc::simple_state<StStop_zebra_at_pedestrian_at, PStZebra_crossing> {
    StStop_zebra_at_pedestrian_at() {
        LOG_INFO_PRINTF(
            "State Machine: Standing at crossing, pederiation at crossing!");  // entry
    }
    typedef mpl::list<
        sc::transition<EvTime_up, PStLane_keeping>,
        sc::transition<EvPedestrian_on_crossing, StStop_zebra_at_pedestrian_on> >
        //sc::transition<EvEmpty_zebra_crossing, StEmpty_Zebra_crossing> >
        reactions;
};
struct StStop_zebra_at_pedestrian_on
    : sc::simple_state<StStop_zebra_at_pedestrian_on, PStZebra_crossing> {
    StStop_zebra_at_pedestrian_on() {
        LOG_INFO_PRINTF(
            "State Machine: Standing at crossing, pederiation on crossing!");  // entry
    }
    typedef mpl::list<
        sc::transition<EvTime_up, PStLane_keeping>,
        sc::transition<EvPedestrian_at_crossing, PStLane_keeping> >
        //sc::transition<EvEmpty_zebra_crossing, PStLane_keeping> >
        reactions;
};

// _________________________________________Emergency_stop_States________________________________________________________
struct PStEmergency_stop : sc::simple_state<PStEmergency_stop, Machine> {
    PStEmergency_stop() {
        LOG_INFO_PRINTF("State Machine: Emergency Stop !");  // entry
    }
    ~PStEmergency_stop() {
        LOG_INFO_PRINTF("State Machine: Emergency Stop Cleared !");  // entry
    }
    typedef sc::transition<EvRoad_is_free, sc::deep_history<PStLane_keeping> >
        reactions;
};

// _________________________________________Obstacles_States________________________________________________________

struct StObstacle_ahead : sc::simple_state<StObstacle_ahead, Machine> {
    StObstacle_ahead() {
        LOG_INFO_PRINTF("State Machine: Obstacle ahead !");  // entry
    }
    typedef sc::transition<EvPlannerCompleted, PStLane_keeping >
        reactions;
};

// _________________________________________Parking_States________________________________________________________
struct PStParking
    : sc::simple_state<PStParking, PStDriving, StNext_maneuver_parking,
        sc::has_deep_history> {
    PStParking() {
//        LOG_INFO_PRINTF(
//            "State Machine: Next Manouver - Find Parking !");  // entry
    }
    typedef mpl::list<sc::transition<EvEmergency_stop, PStEmergency_stop>,
                      sc::transition<EvJury_stop, StReady_to_start> >
        reactions;
};

struct StNext_maneuver_parking
    : sc::simple_state<StNext_maneuver_parking, PStParking> {
    StNext_maneuver_parking() {
        LOG_INFO_PRINTF("State Machine: Next maneuver is parking !");  // entry
    }
    typedef sc::transition<EvPlannerCompleted, StParked>
        reactions;
};

struct StParking_infront
    : sc::simple_state<StParking_infront, PStParking> {
    StParking_infront() {
        LOG_INFO_PRINTF("State Machine: Car is infront parking !");  // entry
    }
     typedef sc::transition< EvPlannerCompleted, StParking_at >
     reactions;
};

struct StParking_at
    : sc::simple_state<StParking_at, PStParking> {
    StParking_at() {
        LOG_INFO_PRINTF("State Machine: Car inside parking box !");  // entry
    }
     typedef sc::transition< EvPlannerCompleted, StParked >
     reactions;
};

struct StParked
    : sc::simple_state<StParked, PStParking> {
    StParked() {
        LOG_INFO_PRINTF("State Machine: Car is now parking !");  // entry
    }
    typedef mpl::list<
        sc::transition<EvNext_pull_left, StPull_left>,
        sc::transition<EvNext_pull_right, StPull_right> >
        reactions;

};

struct StPull_left
    : sc::simple_state<StPull_left, PStParking> {
    StPull_left() {
        LOG_INFO_PRINTF("State Machine: Pull out left from parking! !");  // entry
    }
    typedef sc::transition< EvPlannerCompleted, StNormal_driving >
    reactions;
};

struct StPull_right
    : sc::simple_state<StPull_right, PStParking> {
    StPull_right() {
        LOG_INFO_PRINTF("State Machine: Pull out right from parking! !");  // entry
    }
    typedef sc::transition< EvPlannerCompleted, StNormal_driving >
    reactions;
};

// _________________________________________Road_Crossing_States________________________________________________________
struct PStCrossing : sc::simple_state<PStCrossing, PStDriving, StStop_crossing_ahead,
                                      sc::has_deep_history> {
    PStCrossing() {}  // entry
    typedef mpl::list<
        sc::transition<EvEmergency_stop, PStEmergency_stop>,
        sc::transition<EvNext_maneuver_park, PStParking>,
        sc::transition<EvJury_stop, StReady_to_start>,
        sc::transition<EvNext_pull_right, StPull_right>,
        sc::transition<EvNext_pull_left, StPull_left>,
        sc::transition<EvEmpty_zebra_crossing, StEmpty_Zebra_crossing>,
        sc::transition<EvPedestrian_on_crossing, StStop_zebra_ahead_pedestrian_on>,
        sc::transition<EvPedestrian_at_crossing, StStop_zebra_ahead_pedestrian_at>,
        sc::transition<EvNext_maneuver_park, StNext_maneuver_parking> >

        reactions;
};

struct StStop_crossing_ahead : sc::simple_state<StStop_crossing_ahead, PStCrossing> {
    StStop_crossing_ahead() {
        LOG_INFO_PRINTF("State Machine: Stop at Crossing far ahead !");  // entry
    }
    typedef mpl::list<sc::transition<EvPlannerCompleted, StStop_crossing_at>,
                      sc::transition<EvCrossing_in, StCrossing_in>,
                      sc::transition<EvCrossing_finished, PStLane_keeping> >
        reactions;
};

struct StGoTo_crossing_ahead : sc::simple_state<StGoTo_crossing_ahead, PStCrossing> {
    StGoTo_crossing_ahead() {
        LOG_INFO_PRINTF("State Machine: Go to Crossing far ahead !");  // entry
    }
    typedef mpl::list<sc::transition<EvPlannerCompleted, StGoTo_crossing_at>,
                      sc::transition<EvStop_at_crossing_ahead, StStop_crossing_ahead>,
                      sc::transition<EvCrossing_in, StCrossing_in>,
                      sc::transition<EvCrossing_finished, PStLane_keeping> >
        reactions;
};


struct StStop_crossing_at : sc::simple_state<StStop_crossing_at, PStCrossing> {
    StStop_crossing_at() {
        LOG_INFO_PRINTF("State Machine: Car Stopped at Crossing !");  // entry
    }
    typedef mpl::list<sc::transition<EvCrossing_in, StCrossing_in>,
                      sc::transition<EvPlannerCompleted, PStLane_keeping> >
        reactions;
};

struct StGoTo_crossing_at : sc::simple_state<StGoTo_crossing_at, PStCrossing> {
    StGoTo_crossing_at() {
        LOG_INFO_PRINTF("State Machine: Car arrived to Crossing !");  // entry
    }
    typedef mpl::list<sc::transition<EvCrossing_in, StCrossing_in>,
                      sc::transition<EvPlannerCompleted, PStLane_keeping> >
        reactions;
};

struct StCrossing_in : sc::simple_state<StCrossing_in, PStCrossing> {
    StCrossing_in() {
        LOG_INFO_PRINTF("State Machine: Car in Crossing !");  // entry
    }
    typedef mpl::list<sc::transition<EvStop_at_crossing_ahead, StStop_crossing_ahead>,
                      sc::transition<EvCrossing_at, StStop_crossing_at>,
                      sc::transition<EvPlannerCompleted, PStLane_keeping> >
        reactions;
};

#endif
