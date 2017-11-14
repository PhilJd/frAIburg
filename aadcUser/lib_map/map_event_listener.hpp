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
//
//  map_event_listener.hpp
//  MapEventListener
//     GlobalMap calls diffrent function if elements are adde, changed fused
//      in an deffinded range
//
//  Created by Markus on 05.07.17.
//  Copyright © 2017 me. All rights reserved.
//

#ifndef AADCUSER_FRAIBURG_MAP_LISTENER_LIB_H_
#define AADCUSER_FRAIBURG_MAP_LISTENER_LIB_H_

// #include <stdio.h>
#include <boost/foreach.hpp>
// #include <fstream>
// #include <iostream>
#include <vector>
#include "adtf_log_macros.h"
#include "global_map.hpp"
#include "map_element.hpp"
#include "map_types.h"

namespace frAIburg {
namespace map {

class MapEventListener {
    /* abstract class, implement that car geti called by the map
        if the eventlistener is registered */
 public:
    MapEventListener() : max_range_(0), min_range_(0) {}

    /// set diffrent parameter, to define when event function are being called:
    /*distance ranges form the center of an object to the car front for which
      which map_on_event_changed is call when the postion or angleto the car
      changes*/
    void SetMaxRange(float r) { max_range_ = r; }
    tMapData GetMaxRange() const { return max_range_; }
    void SetMinRange(float r) { min_range_ = r; }
    tMapData GetMinRange() const { return min_range_; }
    /* add multiple thresholds distances for which map_on_event_distance_reached
     is called,wehen the dist. form the centerpoint of map object
     to the car "moves over" the threshold */
    void AddDistanceThreshold(float d) { distance_threshold_.push_back(d); }
    const std::vector<tMapData>& GetAllDistanceThresholds() const {
        return distance_threshold_;
    }

    // TODO(markus) change to const tSptrMapElement &
    /// abstract event function being call by Simple map when registered as an
    /// listener when the map is updated
    /* a new el has been added to the map*/
    virtual void MapOnEventAddedNew(tSptrMapElement el) = 0;

    // element will be removed from map
    virtual void MapOnEventRemoved(tSptrMapElement el) = 0;

    // element distance is lower or higher the the set distance_threshold_, max
    // min range will ignored:
    virtual void MapOnEventDistanceReached(tSptrMapElement el,
                                           const tMapData threshold,
                                           bool distance_lower_threshold) = 0;

 private:
    tMapData max_range_;  // if zero disabled
    tMapData min_range_;
    std::vector<tMapData> distance_threshold_;  // if empty disabled
};
}
} /*NAME SPACE frAIburg,map*/

#endif /* AADCUSER_FRAIBURG_MAP_LISTENER_LIB_H_ */
