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
//  qt elemnt to draw map element to a qt scene
//
//  Created by Markus on 06.07.17.
//
#include <QGraphicsEllipseItem>
#include <QGraphicsItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QMessageBox>
#include <QString>
#include <string>
#include "adtf_log_macros.h"
#include "global_map.hpp"
#include "map_displaywidget_types.h"
#include "map_element.hpp"
#include "map_helper.hpp"
#include "map_types.h"

#ifndef AADCUSER_DISPLAY_EL_H_
#define AADCUSER_DISPLAY_EL_H_

class MapSceneElement {
    // helper class for the ui
    // holds all colos and strings
 public:
    frAIburg::map::tSptrMapElement el_;

    MapSceneElement(QGraphicsScene *scene,
                    frAIburg::map::tSptrMapElement &elmap)
        : el_(elmap),
          scene_(scene),
          poly_(NULL),
          point_(NULL),
          text_(NULL),
          text_color_(MAP_V_QT_COLOR_ELEMENT_TEXT) {
        // default name to display, not used if el user ui tag is set
        el_type_name_id_ =
            frAIburg::map::MapHelper::TypeToString(el_->GetType()) +
            QString::number(el_->GetID());
        // colors
        SetColors();

        text_font_ = QFont("Arial", MAP_V_QT_TEXT_SIZE_ELEMENT);
    }

    /*!draw the map element to the Qscene*/
    void Draw(float m_pixels_per_m_x_scaling, float m_pixels_per_m_y_scaling,
              const tMapTransformer &translate_map_to_qt, bool show_text) {
        namespace bg = boost::geometry;
        if (!scene_) return;

        if (el_->GetID() == MAP_DEFAULT_ID){
          //map id is set to defualt id if el was removed from map
          el_type_name_id_ = "ERROR_DEFAULT_ID";
        }

        if (!el_->GetGlobalPoly().outer().empty()) {
            // check if color changed:
            if (user_el_color_ != el_->user_color_ui_) {
                SetColors();
            }

            DrawPoly(m_pixels_per_m_x_scaling, m_pixels_per_m_y_scaling,
                     translate_map_to_qt);

            // draw the type name at the poly center
            frAIburg::map::tMapPoint p_center(0, 0);
            bg::transform(el_->GetGlobalPolyCenter(), p_center,
                          translate_map_to_qt);
            // move text beside the marker
            const float center_qt_x =
                bg::get<0>(p_center) * m_pixels_per_m_x_scaling;
            const float center_qt_y =
                bg::get<1>(p_center) * m_pixels_per_m_y_scaling;
            const float marker_length_half_qt_x =
                MAP_V_LENGTH_ELMENT_MARKER * m_pixels_per_m_x_scaling;
            const float marker_length_half_qt_y =
                MAP_V_LENGTH_ELMENT_MARKER * m_pixels_per_m_x_scaling;

            if (show_text) {
                DrawText(center_qt_x, center_qt_y, marker_length_half_qt_x);
                DrawMarker(center_qt_x, center_qt_y, marker_length_half_qt_x,
                           marker_length_half_qt_y, m_pixels_per_m_x_scaling,
                           m_pixels_per_m_y_scaling);
            }
            if (el_->IsOrientationUsed())
                DrawOrientationArrow(center_qt_x, center_qt_y,
                                     m_pixels_per_m_x_scaling,
                                     m_pixels_per_m_y_scaling);
        }
    }

 private:
    QGraphicsScene *scene_;
    QGraphicsPolygonItem *poly_;
    QGraphicsEllipseItem *point_;
    QGraphicsSimpleTextItem *text_;
    // normalsnsor info
    static const QPen pen_poly_normal;
    // known landanmakrs
    static const QPen pen_poly_landmarks;

    QBrush brush_fill_ploy_;
    QPen pen_out_line_poly_;
    QPen pen_center_marker_;
    QPen pen_center_angle_;
    QColor text_color_;
    std::string user_el_color_;

    QString el_type_name_id_;

    QFont text_font_;

    void DrawPoly(float m_pixels_per_m_x_scaling,
                  float m_pixels_per_m_y_scaling,
                  const tMapTransformer &translate_map_to_qt) {
        namespace bg = boost::geometry;

        frAIburg::map::tMapPolygon bosotqtpoly;
        // transform the gloabal coords to the qt frame

        bg::transform(el_->GetGlobalPoly(), bosotqtpoly, translate_map_to_qt);

        if (bosotqtpoly.outer().size() == 1) {  // for one wie draw a circle
            const float r = MAP_V_POINT_RADIUS_METER;
            point_ = scene_->addEllipse(
                (bg::get<0>(bosotqtpoly.outer().back()) - r) *
                    m_pixels_per_m_x_scaling,
                (bg::get<1>(bosotqtpoly.outer().back()) - r) *
                    m_pixels_per_m_y_scaling,
                r * 2.0 * m_pixels_per_m_x_scaling,
                r * 2.0 * m_pixels_per_m_y_scaling, pen_out_line_poly_,
                brush_fill_ploy_);
        } else {
            // draw al poly for more points
            QPolygonF qring;
            // add the point to the q polygon with a x and y scaling
            BOOST_FOREACH (const frAIburg::map::tMapPoint &p,
                           bosotqtpoly.outer()) {
                qring << QPointF(
                    bg::get<0>(p) * m_pixels_per_m_x_scaling,   // x
                    bg::get<1>(p) * m_pixels_per_m_y_scaling);  // y
            }
            // draw the polygon
            poly_ =
                scene_->addPolygon(qring, pen_out_line_poly_, brush_fill_ploy_);
            // poly->setFlag(QGraphicsItem::ItemIsMovable, true);
        }
    }

    void DrawText(float center_qt_x, float center_qt_y,
                  float marker_length_half_qt_x) {
        QString *el_qstr_draw;
        QString el_ui_tag;
        if (el_->user_tag_ui_.empty()) {
            // tpye name and id
            el_qstr_draw = &el_type_name_id_;
        } else {
            // tage and id
            el_ui_tag =
                el_->user_tag_ui_.c_str() + QString::number(el_->GetID());
            el_qstr_draw = &el_ui_tag;
        }
        text_ = scene_->addSimpleText(*el_qstr_draw, text_font_);
        if (text_) {
            text_->setPos(center_qt_x + marker_length_half_qt_x, center_qt_y);
            text_->setBrush(text_color_);
        }
    }

    void DrawOrientationArrow(float center_qt_x, float center_qt_y,
                              float m_pixels_per_m_x_scaling,
                              float m_pixels_per_m_y_scaling) {
        // add arrow for el orientation
        const float angle = el_->GetGlobalOrientation();
        const float angle_head_arrow_left = angle + 20 * (M_PI / 180);
        const float angle_head_arrow_right = angle - 20 * (M_PI / 180);
        const float r_line = MAP_V_LENGTH_ELMENT_MARKER * 1.5;
        const float r_arrow_head = r_line * 0.8;

        const float line_end_pos_x =
            r_line * cos(angle) * m_pixels_per_m_x_scaling + center_qt_x;
        const float line_end_pos_y =
            r_line * sin(angle) * m_pixels_per_m_y_scaling + center_qt_y;
        // line in oritation direction from center
        scene_->addLine(center_qt_x, center_qt_y, line_end_pos_x,
                        line_end_pos_y, pen_center_angle_);
        // line arrow to left
        scene_->addLine(line_end_pos_x, line_end_pos_y,
                        r_arrow_head * cos(angle_head_arrow_left) *
                                m_pixels_per_m_x_scaling +
                            center_qt_x,
                        r_arrow_head * sin(angle_head_arrow_left) *
                                m_pixels_per_m_y_scaling +
                            center_qt_y,
                        pen_center_angle_);
        scene_->addLine(line_end_pos_x, line_end_pos_y,
                        r_arrow_head * cos(angle_head_arrow_right) *
                                m_pixels_per_m_x_scaling +
                            center_qt_x,
                        r_arrow_head * sin(angle_head_arrow_right) *
                                m_pixels_per_m_y_scaling +
                            center_qt_y,
                        pen_center_angle_);
    }

    void DrawMarker(float center_qt_x, float center_qt_y,
                    float marker_length_half_qt_x,
                    float marker_length_half_qt_y,
                    float m_pixels_per_m_x_scaling,
                    float m_pixels_per_m_y_scaling) {
        // center marker line x
        scene_->addLine(center_qt_x - marker_length_half_qt_x, center_qt_y,
                        center_qt_x + marker_length_half_qt_y, center_qt_y,
                        pen_center_marker_);
        // center marker y line
        scene_->addLine(center_qt_x, center_qt_y - marker_length_half_qt_x,
                        center_qt_x, center_qt_y + marker_length_half_qt_y,
                        pen_center_marker_);
    }

    void SetColors() {
        QColor color_poly_fill(MAP_V_QT_COLOR_ELEMENT_POLY_FILL_DEFAULT);
        QColor color_marker(MAP_V_QT_COLOR__ELEMENT_CENTER_MARKER);
        QColor color_poly_outer_line(
            MAP_V_QT_COLOR_ELEMENT_POLY_OUTER_LINE_DEFAULT);
        if (!el_->user_color_ui_.empty()) {
            user_el_color_ = el_->user_color_ui_;
            QColor color_user(user_el_color_.c_str());
            if (color_user.isValid()) {
                color_poly_fill = color_user;
                color_marker = color_user;
                color_poly_outer_line = color_user;
            } else {
                LOG_WARNING_PRINTF("element ui user color not vaild: %s",
                                   el_->user_color_ui_.c_str());
            }
        }
        pen_center_angle_ = QPen(color_marker);  // no alpha

        text_color_.setAlpha(MAP_V_QT_COLOR_ELEMENTT_ALPHA_TEXT_DEFAULT);
        color_marker.setAlpha(MAP_V_QT_COLOR_ELEMENT_ALPHA_DEFAULT);
        color_poly_fill.setAlpha(MAP_V_QT_COLOR_ELEMENT_ALPHA_DEFAULT);

        brush_fill_ploy_ = QBrush(color_poly_fill);
        brush_fill_ploy_.setStyle(Qt::SolidPattern);  // filled
        pen_out_line_poly_ = QPen(color_poly_outer_line);
        pen_center_marker_ = QPen(color_marker);
        pen_center_angle_.setWidth(2);
    }
};

#endif  // AADCUSER_DISPLAY_EL_H_
