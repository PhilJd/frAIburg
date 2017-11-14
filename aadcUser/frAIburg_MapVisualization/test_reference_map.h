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
#ifndef AADCUSER_FRAIBURG_TEST_REF_MAP_H_
#define AADCUSER_FRAIBURG_TEST_REF_MAP_H_

#define ADTF_MAP_BLOCK_SIDE_OFFSET 0.05L
#define ADTF_MAP_LINE_SIZE 0.02L
#define ADTF_MAP_TEXT_ELEMENT_COLOR "mediumaquamarine"

#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include "slim_pins.h"
#include "map_element.hpp"
#include "global_map.hpp"
#include "map_types.h"
#include "map_helper.hpp"

/*! add test lane markers to the map */
void AddReferenceMap(frAIburg::map::GlobalMap * map);
/*! add test element to the map for testing the state machine*/
void AddTestElementsStateMachine(frAIburg::map::GlobalMap * map);

#endif /*AADCUSER_FRAIBURG_TEST_REF_MAP_H_*/
