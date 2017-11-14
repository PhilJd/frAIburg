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
#ifndef _STATE_MACHINE_NEW_H_
#define _STATE_MACHINE_NEW_H_

#define OID_ADTF_STATE_MACHINE "adtf.example.state_machine_new"
#define ADTF_FILTER_STATE_MACHINE_NAME "frAIburg State Machine"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "frAIburg State Machine"


#include<unistd.h>
#include <ctime>
#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include "ManeuverList.h"
#include "adtf_tools.h"
#include "machine.h"
#include "slim_pins.h"

#include "aadc_enums.h"
#include "aadc_structs.h"
#include "adtf_tools.h"

#include "aadc_juryEnums.h"
#include "global_map.hpp"
#include "map_types.h"
#include "map_element.hpp"
#include "map_element_types.h"
#include "map_event_listener.hpp"
#include "map_helper.hpp"

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <queue>

#include "event_struct.h"

//include for state machine
#include <boost/mpl/list.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>
#include "adtf_log_macros.h"
//#include "behaviour_enum.h"
#include "nonpin_types.h"
#include <limits>
#include "boost/algorithm/string.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/atomic.hpp>  // use atomic<bool> to control thread launch


using namespace frAIburg::map;//NOT in the header!

class StateMachine : public adtf::cFilter {

    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_STATE_MACHINE, ADTF_FILTER_STATE_MACHINE_NAME,
                adtf::OBJCAT_DataFilter);

 public:
     StateMachine(const tChar* __info);

    /*! default destructor */
    virtual ~StateMachine();

    /*! Implements the default cFilter state machine calls. It will be  */
    tResult Start(ucom::IException** __exception_ptr = NULL);
    tResult Stop(__exception);
    /*! This Function is always called when any property has changed. This
     * should
     * be the only place  */
    tResult UpdateProperties(ucom::IException** __exception_ptr = NULL);

    //from here is listener functions
    void SkipJuryStates(); // skipping the jury communication commands, this should not be used in the competition

    tResult TransmitBehaviour(tInt32 behaviour_id, tInt32 behaviour_next_id, tInt32 object_id, tInt32 speed_id);
    tResult TransmitLightCommand(tInt32 light_id, tBool switch_bool); // transmitting the light command, (enum light_id, bool on or off)

    Machine state_machine_; // state machine object
    bool poperty_debug_enabled_;
    void GetAllProperties(void);
    void SetAllProperties(void);



    private:

    bool car_detected_; // this bool triggers a second "processevent" round for the case a car was detected late. this should prevents the case where car are ignored in crossing when detected late.

    float __attribute__ ((deprecated("use MapHelper"))) Car2ObjectPlaneViewAngle(tSptrMapElement el);
    bool CarOnRight(tSptrMapElement el); // returns true if there is a car on the right side of the crossing, otherwise false
    bool CarOnLeft(tSptrMapElement el); // returns true if there is a car on the left side of the crossing, otherwise false
    bool CarOnCrossing(tSptrMapElement); // returns true if there is car inside the crossing
    bool CarTowardsAhead(tSptrMapElement el);
    bool PedestrianOnCrossing(tSptrMapElement el); //returns true if there is a pedestrian ON the zebra crossing
    bool PedestrianAtCrossingRight(tSptrMapElement el);
    bool PedestrianAtCrossingLeft(tSptrMapElement el);
    bool IsParkingRequested(const std::string& jury_command);
    bool GetParkingMapID(const std::string& jury_command, tMapID* map_parking_id);

    void ProcessEventQueue(); //after all events are in the event queue. the event is ordered and the first event in handeled
    int SetElementPriority(tSptrMapElement el); // returns the priority of the element depending on its distance to the car and angle. 1 being the highest priority. 0 priority is to ignore.
    void SendBehaviour(); // sends the current behaviour to the planner
    bool AllowedToGo(string command); // checks if our car in the crossing is allowed to cross. depending on the trafic signs and other cars in the crossing.

    std::priority_queue<event> event_pq_; // event queue element, here all the event are stored and ordered (first by priority then by distance)
    event current_event_; // here the current event the car is acting upon is stored.
    frAIburg::map::GlobalMap *map_;
    int current_highest_priority_; // Lowest number is higest priorty (1 priority is highest!)
    bool event_change_locked_; // locking the current event from changing (for example when we turn left, this is locked until the turning is completed)

    int GetObjectInRange(float x_global, float y_global, float global_orientation, MapElementType type, float x_max, float x_min, float y_max, float y_min);
    bool IsParkingOrChildAhead();
    bool PedestrianCrossingAllowedToGo(tSptrMapElement el);
    bool PullOutAllowed();


 private:
     // mutex to lock read/write ops on the boost state chart
    boost::mutex statemachine_mutex_;

    // mutex to lock read/write ops on the priority qeue
    boost::mutex priority_queue_mutex_;
    // thread safe boost statechart  functions
    void ProcessEvent(const boost::statechart::event_base& ev_type);
    size_t GetActiveStateHash();

    // thread safe event queue read/write functions
    void EventQueueClear();
    bool EventQueueEmpty();
    event EventQueueTop();
    void AddEventToPQ(tSptrMapElement el, event_type ev_type); // adding an event to the priority queue
    void ProcessElement(const tSptrMapElement el); // processing an incoming element from the map (when element is added to the near map or distance is changed)

    /*! this function creates all the input pins of filter*/
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);
    tResult InitBufferPins();

    tResult ProcessEmergencyStop(IMediaSample* pMediaSample);
    tResult ProcessPlannerStatus(IMediaSample* pMediaSample);

    tResult TransmitFloatValue(slim::OutputPin& oPin, tFloat32 value,
                               tUInt32 timestamp);

    tResult TransmitBoolValue(slim::OutputPin& oPin, tBool bValue);

    planner_status planner_status_; //the status sent from the planner
    bool maneuver_requested_; // when a maneuver is requested from the planner this is true (turn left, right, straight, park, pull out)

    /*! The maneuver file string */
    cString m_strManeuverFileString;


    tResult ProcessJuryStructInput(IMediaSample* pMediaSample); // process data from JuryStruct pin
    tResult ProcessManeuverListInput(IMediaSample* pMediaSample); // process data from maneuverlist pin
    void IncrementManeuver(); // when a maneuver is completed the current maneuver is increased (also updates the jury)
    std::string GetCurrentManeuver();

    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tAADC_Maneuver> maneuver_list_vector_; // vector with all the maneuver sent by the jury
    unsigned int current_maneuver_id_; // the current maneuver we are driving
    int current_behaviour_id_;
    size_t current_state_hash_; // current state hash in the state machine


    /*! Atomic bool, true when a SendBehavior thread is running. */
    boost::atomic<bool> send_behavior_thread_is_active;

    tResult LoadManeuverList(); // after the data is extracted from the pin it is processed in this function

    slim::InputPin pin_in_jurystruct_;
    slim::InputPin pin_in_emergency_stop_;
    slim::InputPin pin_in_maneuver_list_;
    slim::InputPin pin_in_bool_test_; // this pin is used for testing
    slim::InputPin pin_in_planner_status_;

    slim::OutputPin pin_out_jurystruct_;
    slim::OutputPin pin_out_emergency_stop_bool_;
    slim::OutputPin pin_out_emergency_stop_speed_;
    slim::OutputPin pin_out_set_state_ready_;
    slim::OutputPin pin_out_set_state_running_;
    slim::OutputPin pin_out_set_state_complete_;
    slim::OutputPin pin_out_maneuver_completed_;
    slim::OutputPin pin_out_behaviour_;
    slim::OutputPin pin_out_light_controller_;

    float SIGN_CROSSING_THRESHOLD_; // the maximum allowed distance for a sign from a crossing, if there is no sign in this radius the crossing is acted as a crossing with no sign.
    float PEDESTRIAN_AT_ZEBRA_THRESHOLD_;  //
    float SIGN_STOP_LINE_THRESHOLD_;
    float CAR_ON_CROSSING_THRESHOLD_;
    float CAR_AT_CROSSING_THRESHOLD_;
    float PEDESTRIAN_ON_ZEBRA_THRESHOLD_;

    tTimeStamp time_last_zebra_crossing_;

 protected:
    void get_all_properties(void);
    void set_all_properties(void);

    /*! Implements the default cFilter state machine call. */
    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. */
    tResult Shutdown(tInitStage stage,
                     ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to. */
    tResult OnPinEvent(adtf::IPin* source, tInt event_code, tInt param1,
                       tInt param2, adtf::IMediaSample* media_sample);

   //timer added by markus
   void StartThread();
   boost::thread thread_process_queue_;
   tBool running_ok_;
   void Run();
   void StopThread();
   /*! hight light one map element, only use in the debug mode*/
   void DebugMapElementHighLight(tMapID id_to_highlight, const char * color);
   //TODO fix for test event
   bool jury_ready_request_;
 };
// *****************************************************************************
#endif  // _STATE_MACHINE_H_
