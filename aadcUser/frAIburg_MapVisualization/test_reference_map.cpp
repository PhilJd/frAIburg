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

#include "test_reference_map.h"

using namespace frAIburg::map;

void AddMapElementGlobal(GlobalMap *map, tSptrMapElement &el,
                         const char *color = NULL) {
    tMapCarPosition zero_pos;
    zero_pos.x = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_X;
    zero_pos.y = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_Y;
    zero_pos.heading = MAP_LOCAL_IS_GLOBAL_CAR_POSITION_HEADING;

    el->LocalToGlobal(zero_pos);

    if (color) {
        el->user_color_ui_ = color;
    }
    map->AddElement(el);  // add ele new to mam
    el->SetUpdateWithRepostionJumps(false);
}

// ____________________________________________________________________________
void AddReferenceMap(GlobalMap *map) {
    MapElementType lanetype = LM_STREET_LANE;
    // add reference lines to the map
    std::vector<tMapPoint> lane_b = boost::assign::list_of(tMapPoint(-6L, 0L))(
        tMapPoint(-6L, ADTF_MAP_LINE_SIZE))(tMapPoint(-2L, ADTF_MAP_LINE_SIZE))(
        tMapPoint(-2, 0L));
    std::vector<tMapPoint> block_c_b = boost::assign::list_of(tMapPoint(-5, 1))(
        tMapPoint(-5, 2))(tMapPoint(-3, 2))(tMapPoint(-3, 1));
    std::vector<tMapPoint> block_r_b = boost::assign::list_of(tMapPoint(-2, 1))(
        tMapPoint(-2, 2))(tMapPoint(-1, 2))(tMapPoint(-1 - 0.2, 1 + 0.2));
    std::vector<tMapPoint> block_r_t = boost::assign::list_of(tMapPoint(-2, 3))(
        tMapPoint(-2, 4))(tMapPoint(-1 - 0.2, 4 - 0.2))(tMapPoint(-1, 3));
    // center top block
    std::vector<tMapPoint> block_c_t = boost::assign::list_of(tMapPoint(-5, 3))(
        tMapPoint(-5, 4))(tMapPoint(-3, 4))(tMapPoint(-3, 3));
    std::vector<tMapPoint> lane_l_ =
        boost::assign::list_of(tMapPoint(-6, 1))(tMapPoint(-6, 4))(tMapPoint(
            -6 - ADTF_MAP_LINE_SIZE, 4))(tMapPoint(-6 - ADTF_MAP_LINE_SIZE, 1));
    std::vector<tMapPoint> lane_t = boost::assign::list_of(
        tMapPoint(-6L, 5L - ADTF_MAP_LINE_SIZE))(tMapPoint(-6L, 5))(
        tMapPoint(-2L, 5))(tMapPoint(-2, 5L - ADTF_MAP_LINE_SIZE));

    tSptrMapElement el_box_b(new MapElement(lanetype, lane_b));
    AddMapElementGlobal(map, el_box_b, MAP_ELEMENT_COLOR_LANDMARK);
    tSptrMapElement el_box_c_b(new MapElement(lanetype, block_c_b));
    AddMapElementGlobal(map, el_box_c_b, MAP_ELEMENT_COLOR_LANDMARK);
    tSptrMapElement el_box_r_t(new MapElement(lanetype, block_r_t));
    AddMapElementGlobal(map, el_box_r_t, MAP_ELEMENT_COLOR_LANDMARK);
    tSptrMapElement el_box_r_b(new MapElement(lanetype, block_r_b));
    AddMapElementGlobal(map, el_box_r_b, MAP_ELEMENT_COLOR_LANDMARK);
    tSptrMapElement el_box_t(new MapElement(lanetype, block_c_t));
    AddMapElementGlobal(map, el_box_t, MAP_ELEMENT_COLOR_LANDMARK);
    tSptrMapElement el_lane_l(new MapElement(lanetype, lane_l_));
    AddMapElementGlobal(map, el_lane_l, MAP_ELEMENT_COLOR_LANDMARK);
    tSptrMapElement el_lane_t(new MapElement(lanetype, lane_t));
    AddMapElementGlobal(map, el_lane_t, MAP_ELEMENT_COLOR_LANDMARK);
}

void AddTestElementsStateMachine(GlobalMap *map) {
    // add crossign
    const char *color = ADTF_MAP_TEXT_ELEMENT_COLOR;

    tSptrMapElement t_crossing_t_l =
        MapHelper::CreatBox(STREET_MARKER_CROSSING_T, -5.5, 4.5,  // center
                            0.5, 0.5);                            // size
    AddMapElementGlobal(map, t_crossing_t_l, color);

    tSptrMapElement t_crossing_t_m =
        MapHelper::CreatBox(STREET_MARKER_CROSSING_T, -2.5, 4.5,  // center
                            0.5, 0.5);                            // size
    AddMapElementGlobal(map, t_crossing_t_m, color);

    tSptrMapElement x_crossing_m_l =
        MapHelper::CreatBox(STREET_MARKER_CROSSING_T, -5.5, 2.5,  // center
                            0.5, 0.5);                            // size
    AddMapElementGlobal(map, x_crossing_m_l, color);

    /*tSptrMapElement x_crossing_m_m
    =MapHelper::CreatBox(STREET_MARKER_CROSSING_X,
                                                          -2.5,2.5, //center
                                                          0.5,0.5); //size
    AddMapElementGlobal(map,x_crossing_m_m,color);*/

    tSptrMapElement t_crossing_m_r =
        MapHelper::CreatBox(STREET_MARKER_CROSSING_T, -0.5, 2.5,  // center
                            0.5, 0.5);                            // size
    AddMapElementGlobal(map, t_crossing_m_r, color);

    tSptrMapElement t_crossing_b_r =
        MapHelper::CreatBox(STREET_MARKER_CROSSING_T, -2.5, 0.5,  // center
                            0.5, 0.5);                            // size
    AddMapElementGlobal(map, t_crossing_b_r, color);

    float zebra_orientation = 0*3.1415;
    tSptrMapElement t_zebra_t_m = MapHelper::CreatBox(STREET_MARKER_ZEBRA,
                                                      // -4.1,4.5, //center
                                                      -4.3, 4.5,  // center
                                                      0.2, 0.5,0, NULL, &zebra_orientation);  // size
    AddMapElementGlobal(map, t_zebra_t_m, color);
    tSptrMapElement t_crossing_b_ll =
        MapHelper::CreatBox(STREET_MARKER_CROSSING_T,
                            //-5.5,.5, //center
                            -7, .5,     // center
                            0.5, 0.5);  // size
    AddMapElementGlobal(map, t_crossing_b_ll, color);
    tSptrMapElement t_crossing_b_l =
        MapHelper::CreatBox(STREET_MARKER_CROSSING_T, -8.5, .5,  // center
                            0.5, 0.5);                           // size
    AddMapElementGlobal(map, t_crossing_b_l, color);
}
