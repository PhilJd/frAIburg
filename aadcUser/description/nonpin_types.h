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

#ifndef AADCUSER_DESCRIPTION_NONPIN_TYPES_H_
#define AADCUSER_DESCRIPTION_NONPIN_TYPES_H_
#include "customtypes.h"

// LANEWIDTH_ use /home/aadc/ADTF/src/configuration/frAIburg_configuration_dimensions.xml
const float LANEWIDTH_ = 0.47; // from center marking to center marking
// CAR_WIDTH_ use /home/aadc/ADTF/src/configuration/frAIburg_configuration_dimensions.xml
const float CAR_WIDTH_ = 0.35;
const float CAR_LENGTH_ = 0.6;
const float PI = 3.14159265359;
const float RADIUS_MIN_ = 0.6;

const float SPEED_STOPPED_ = 0.01;
const float SPEED_LOW_ = 0.5; //0.4
const float SPEED_MEDIUM_ = 0.6;
const float SPEED_HIGH_ = 1.0; // 1.
const float SPEED_ULTRAHIGH_ = 1.2;
const float SPEED_MAX_ = 1.;
const float ACC_MAX_ = 0.8; //1 // dv/ds, not dv/dt
// x offset between front bumper and anchor point used
// in steering controller:
const float CAR_ANCHOR_OFFSET_ = 0.05;
const float CAR_ANCHOR_OFFSET_BWD_ = 0.8;

const float STOPLINE_OFFSET_ = 0.15; // from crossing begin

enum speed_type {ST_BACKWARDS, ST_STOP, ST_SPEED_LOW, ST_SPEED_MEDIUM, ST_SPEED_HIGH, ST_SPEED_ULTRAHIGH};

enum orientation_type {vertical, horizontal, unknown};

enum planner_status {PS_STAT_OBSTACLE, PS_DYN_OBSTACLE, PS_SUCCESSFUL,
		             PS_FAILED, PS_NO_LANES, PS_COMPLETED};


enum behaviour_type {BT_UNKNOWN, STOP, EM_ENABLED, EM_DISABLED, EM_OBSTACLE_AHEAD, FOLLOW_LANE, GO_TO_OBJECT, STOP_AT_OBJECT, GO_AROUND_OBJECT,
		             TURN_LEFT, TURN_RIGHT, GO_STRAIGHT, PARK_AT_OBJECT, PULL_LEFT, PULL_RIGHT, PULL_RIGHT_FWD, PULL_RIGHT_BWD,
	 	             GO_BACKWARDS, STOP_AT_PARKING_INFRONT, PARK, CHANGELANE_TO_LEFT, CHANGELANE_TO_RIGHT, FOLLOW_LANE_TO_CLEAR, FOLLOW_CAR,
                     DEMO_GO_TO_PERSON, DEMO_GO_TO_INIT};

enum light_type {	ALL_LIGHTS,
									TURN_SIGNAL_LEFT,
									TURN_SIGNAL_RIGHT,
									HEAD_LIGHTS,
									HAZZARD_LIGHTS,
									BRAKE_LIGHTS,
									REVERSE_LIGHTS };

typedef std::vector<tPoint> tPath;

struct tPose {
    float x;
    float y;
    float heading;
};

struct Lane {
    std::vector<float> coeff;
    orientation_type orientation;
    int alignment;  // 2, 1, -1, -2 (2,1, ..  left from car
    int no_of_samples;
    float dist;              // either vertical or lateral distance from car
    float angle_origin_rad;  // angle at x=0; pos. in direction of y
    float lane_quality;  // 0: don't trust if possible! 1: good fit, good length
    tPoint start_pnt;
    tPoint end_pnt;
};

#endif  // AADCUSER_DESCRIPTION_NONPIN_TYPES_H_
