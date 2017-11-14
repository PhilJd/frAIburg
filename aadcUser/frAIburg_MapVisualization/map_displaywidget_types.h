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
#ifndef AADCUSER_DISPLAY_TYPES_H_
#define AADCUSER_DISPLAY_TYPES_H_


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#define MAP_V_CAR_RADIUS_IN_METER 0.15L
#define MAP_V_ADTF_CAR_OFFSET_X_METER -MAP_V_CAR_RADIUS_IN_METER
#define MAP_V_ADTF_CAR_OFFSET_Y_METER 0.

#define MAP_V_HIDE_TEXT_MAX_ELEMENTS 500L
//SIZES
#define MAP_V_TIME_BEFORE_REPAINT_MSEC 100L

#define MAP_V_GRID_STEP_SIZE_METER 1L
#define MAP_V_POINT_RADIUS_METER 0.05L
#define MAP_V_DEFAULT_CAR_CENTER_VIEW_SIZE_METER 0.05L
#define MAP_V_DEFAULT_FULL_SCALE_MAP_SIDE_MARING_METER 0.5L
#define MAP_V_LENGTH_ELMENT_MARKER 0.05L
//COLORS
#define MAP_V_QT_COLOR_BACKGROUND  43,43,43 //Qt::white//
#define MAP_V_QT_COLOR_GRID_NORMAL Qt::lightGray
#define MAP_V_QT_COLOR_GRID_ORIGIN Qt::darkGray
#define MAP_V_QT_COLOR_CAR_FILL Qt::red
#define MAP_V_QT_COLOR_CAR_OUTER_LINE Qt::red
#define MAP_V_QT_COLOR_CAR_HEADING_LINE Qt::white
#define MAP_V_QT_COLOR_ELEMENT_TEXT Qt::red
#define MAP_V_QT_COLOR__ELEMENT_CENTER_MARKER Qt::green
#define MAP_V_QT_COLOR_ELEMENT_POLY_FILL_DEFAULT Qt::red
#define MAP_V_QT_COLOR_ELEMENT_POLY_OUTER_LINE_DEFAULT Qt::green
#define MAP_V_QT_COLOR_ELEMENT_ALPHA_DEFAULT 80
#define MAP_V_QT_COLOR_ELEMENTT_ALPHA_TEXT_DEFAULT 200


//TEXT
#define MAP_V_QT_TEXT_SIZE_ELEMENT 10


typedef boost::geometry::strategy::transform::\
        translate_transformer<frAIburg::map::tMapCoords, 2, 2> tMapTransformer;
typedef boost::geometry::model::box<frAIburg::map::tMapPoint> tMapBox;




#endif //AADCUSER_DISPLAY_TYPES_H_
