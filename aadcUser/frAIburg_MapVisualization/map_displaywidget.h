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
//  map_types.h
//  types for the map and map elments
//
//  Created by Markus on 06.07.17.
//
#ifndef AADCUSER_FRAIBURG_MAP_DISPLAYWIDGET_H_
#define AADCUSER_FRAIBURG_MAP_DISPLAYWIDGET_H_



#include <iostream>
#include <map>
#include <deque>

#include <QObject>
#include <QtCore/QtCore>
#include <QtGui/QtGui>
#include <QGraphicsItem>
#include <QGraphicsPolygonItem>
#include <QString>
#include <QLayout>
#include <QLayout>
#include <QCheckBox>
#include <QSlider>
#include <QPushButton>
#include <QEvent>
#include <QScrollBar>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QMessageBox>
#include <QFileDialog>
#include <ctime>


#include "map_types.h"
#include "map_element.hpp"
#include "global_map.hpp"
#include "map_helper.hpp"
#include "adtf_log_macros.h"
#include "map_display_scene_element.hpp"
#include "map_displaywidget_types.h"

#ifndef MAP_BUILD_ADTF //TODO(markus)
#include <boost/qvm/mat_operations.hpp>
#endif
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/timer/timer.hpp> //time draw
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/multi/geometries/multi_geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#ifdef MAP_BUILD_ADTF
#include "stdafx.h"
#endif

#ifdef WIN32
#ifdef _DEBUG
#pragma comment(lib, "qtmaind.lib")
#pragma comment(lib, "qtcored4.lib")
#pragma comment(lib, "qtguid4.lib")
#else  // _DEBUG
#pragma comment(lib, "qtmain.lib")
#pragma comment(lib, "qtcore4.lib")
#pragma comment(lib, "qtgui4.lib")
#endif
#endif

class DisplayWidgetMap: public QWidget, public frAIburg::map::MapEventListener {
    Q_OBJECT

    public:
        /*! if save_file_name the map is saved as an img at
        */
        DisplayWidgetMap(QWidget* obj,frAIburg::map::GlobalMap* map,
                        bool show_text = true,
                        const char * save_file_name = NULL);
        ~DisplayWidgetMap();

        virtual void paintEvent(QPaintEvent *);

        //do paint just after time
        void doRepaint();
    private slots:
        /*! slot for ui changes Changes*/
        void changeZoomMode(int mode);
        void changeMapEltxt(int mode);
        void slider_zoom_value_changed (int k);
        void handle_push_button_clear();
        void handle_push_button_recenter_car();
        void HandlePushButtonSaveImg();
    signals:
        void doSignalRepaint();

private:
        std::vector<MapSceneElement> elements_;
        frAIburg::map::GlobalMap* map_;

        //var changed in ui callbacks
        bool autofullZoom_;
        bool show_map_el_txt_;
        int slider_zoom_val_;

        /*! the widget */
        QCheckBox* switch_full_map_;
        QCheckBox* switch_show_map_el_txt_;
        QSlider *slider_zoom_;
        QString save_file_name_;
        QPoint prevPos;

        float m_x_maxval;
        float m_x_minval;
        float m_y_maxval;
        float m_y_minval;

        boost::posix_time::ptime time_last_ui_update_;
        QMutex update_mutex_;
        /*! the main font for the widget */
        QFont* m_mainFont;
        QGridLayout* grid_layout_;
        QGraphicsScene* scene_;//plot the map
        //Graphics View for widget
        QGraphicsView* view_;//view the the scene
        QPushButton *push_button_clear_;
        QPushButton *push_button_recenter_car_;
        QPushButton *push_button_save_img_;
        bool recenter_car_;
        frAIburg::map::tMapCarPosition pos_;
        //bounding  for the map global size of the map
        //used for scaling
        tMapBox bounding_box_map_;
        tMapBox bounding_box_map_car_;
        tMapTransformer setup_zoom_transform();
        void draw_grid(const tMapTransformer  &trans,
                                      const int m_pixels_per_m_x_scaling,
                                      const int m_pixels_per_m_y_scaling);
        void draw_car(const tMapTransformer  &trans,
                                      const int m_pixels_per_m_x_scaling,
                                      const int m_pixels_per_m_y_scaling);
        void draw_map(const tMapTransformer &translate_map_to_qt,
                                      const int m_pixels_per_m_x_scaling,
                                      const int m_pixels_per_m_y_scaling);
        void set_scense_box(const tMapTransformer &translate_map_to_qt,
                                      const int m_pixels_per_m_x_scaling,
                                      const int m_pixels_per_m_y_scaling);
        void set_scroll_pos(const tMapTransformer  &translate_map_to_qt,
                                       const int m_pixels_per_m_x_scaling,
                                      const int m_pixels_per_m_y_scaling);


public:
        /*function that will be called when the map is updated*/
         void MapOnEventAddedNew(frAIburg::map::tSptrMapElement el);
         void MapOnEventRemoved(frAIburg::map::tSptrMapElement el);
         void MapOnEventDistanceReached(frAIburg::map::tSptrMapElement,
                                        frAIburg::map::tMapData threshold,
                                        bool distance_lower_threshold);



        void SaveMapToVid();
};


#endif /*AADCUSER_FRAIBURG_MAP_DISPLAYWIDGET_H_*/
