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
#ifndef AADCUSER_FRAIBURG_VIRTUALCROSSINGCREATOR_CROSSING_CREATOR_H_
#define AADCUSER_FRAIBURG_VIRTUALCROSSINGCREATOR_CROSSING_CREATOR_H_

#include "stdafx.h"
#include "customtypes.h"
#include "nonpin_types.h"


#include <vector>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include "slim_pins.h"
#include "adtf_log_macros.h"
#include "map_element.hpp"
#include "map_element_types.h"
#include "global_map.hpp"
#include "map_types.h"
#include "map_helper.hpp"

#define OID_ADTF_FILTER_DEF \
    "adtf.user.aadc_frAIburg_CrossingCreator"  // unique for a filter
#define ADTF_FILTER_DESC \
    "frAIburg CrossingCreator"  // this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME \
    "CrossingCreator"  // must match with accepted_version_...
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"  // version string
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small Crossing Creation Filter \n$Rev:: 62948"

class CrossingCreator : public adtf::cFilter,
                               public frAIburg::map::MapEventListener {
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC,
                        adtf::OBJCAT_DataFilter);

 protected:
 public:
    /*! Sends bool, enable 1 or disable 0 this filter */
    slim::InputPin crossing_input_pin_;

    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    explicit CrossingCreator(const tChar* __info);

    /*! default destructor */
    virtual ~CrossingCreator();

    /*! Implements the default cFilter state machine call.
    */
    virtual tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call.
    *
    */
    virtual tResult Shutdown(tInitStage eStage,
                     ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1,
                       tInt nParam2, IMediaSample* pMediaSample);

    /*! Implements the default cFilter state machine calls.*/
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls.*/
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*! This Function is always called when any property has changed.
    */
    tResult PropertyChanged(const tChar* str_name);

    /*functions that will be called when the map is updated*/
    /*! add elements relevant for crossing and in range to vector to check in the timer*/
    void MapOnEventDistanceReached(frAIburg::map::tSptrMapElement,
                                  frAIburg::map::tMapData threshold,
                                  bool distance_lower_threshold);
    void MapOnEventRemoved(frAIburg::map::tSptrMapElement el);
    void MapOnEventAddedNew(frAIburg::map::tSptrMapElement el);

 private:
    enum TOrientation { LEFT = 0 , BOTTOM = 1 ,RIGHT = 2 };

    //
    cCriticalSection update_mutex_;
    std::vector<frAIburg::map::tSptrMapElement> crossing_relevant_signs_;
    std::vector<frAIburg::map::tSptrMapElement> crossing_relevant_markings_;

    /*! Map pointer */
    frAIburg::map::GlobalMap* map_;
    void InitMAP();

    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;
    tResult Run(tInt nActivationCode,
                const tVoid* pvUserData, tInt szUserDataSize,
                __exception); // TODO(markus) NULL?
    void InitTimer();
    void DeleteTimer();

    /*! is debug mode enabled */
    bool property_debug_enabled_;

    /*! the struct with all the properties*/
    struct FilterProperties {
        /*! from car perspective: sign-offset to crossing-bottom-right-corner in y direction */
        float sign_offset_y;
        /*! from car perspective: sign-offset to crossing-bottom-right-corner in x direction */
        float sign_offset_x;
        /*! from car perspective: stopline template-offset to crossing-bottom-right-corner in x direction */
        float zero_sign_angle_offset_x_stopline;
        /*! from car perspective: stopline template offset to crossing-bottom-right-corner in y direction */
        float zero_sign_angle_offset_y_stopline;
        /*! used to compute center of crossing */
        float half_crossing_width;
        /*! used to compute center of crossing */
        float half_crossing_height;
        /*! range for map element notification */
        float property_map_range_check_marker;
        float property_map_range_check_sign;
        float pl2;
        bool use_fix_xml_crossings_;
        bool validate_crossings_with_map_;
    }
    /*! the filter properties of this class */
    filter_properties_;

    /*!  Write the crossing to the map*/
    frAIburg::map::tSptrMapElement CreateCrossingElementFromSign(
          const frAIburg::map::tSptrMapElement &sign_detected,
          frAIburg::map::MapElementType crossing_type,
          frAIburg::map::tTimeMapStamp t);
    /*! Process the elements that come from MapOnEventDistanceReached */
    void HandleMapSignElementsWithinDistance();

    /*!  adds a crossing box to the map*/
    frAIburg::map::tSptrMapElement AddCrossingBoxToMap(
                const frAIburg::map::tMapPoint &center_of_sign,
                frAIburg::map::MapElementType crossing_type,
                frAIburg::map::tMapData grid_orientation,
                frAIburg::map::tTimeMapStamp t);


    /*! Remove the element from crossing_relevant_els_ */
    bool RemoveElementCheck(std::vector<frAIburg::map::tSptrMapElement> &map_vec,
                            frAIburg::map::tMapID id);

    /*! Put the relevant element in the vector, relevant signs and relevant lane info */
    void UpdateRelevantElementToSignVector(frAIburg::map::tSptrMapElement &el);

    /*! Find closest grid orientation */
    void GetFixedGridOrientation(const frAIburg::map::tMapData &sign_orientation, frAIburg::map::tMapData *return_grid_orientation);
    tResult AddCrossingToVector(IMediaSample* sample);

    /*! Only to test that sending works */
    bool AddLaneCrossingToMap(const tCrossing &crossing);
    /*! check the map for crossing signs and return true if there co*/
    bool isValid(const frAIburg::map::tSptrMapElement &center_of_crossing,
        frAIburg::map::tMapData max_distance_to_crossing_sign_center);
};

#endif  // AADCUSER_FRAIBURG_VIRTUALCROSSINGCREATOR_CROSSING_CREATOR_H_
