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
#include "map_displaywidget.h"
#include <QDebug>
#include <cmath>
#include <iostream>

namespace bg = boost::geometry;
namespace trans = boost::geometry::strategy::transform;
typedef trans::translate_transformer<frAIburg::map::tMapCoords, 2, 2>
    tMapTransformer;
using namespace frAIburg::map;

// ____________________________________________________________________________
DisplayWidgetMap::DisplayWidgetMap(QWidget *obj, GlobalMap *map,
                                   bool show_text /*= true*/,
                                   const char *save_file_name /*= NULL*/)
    : QWidget(obj),
      frAIburg::map::MapEventListener(),
      save_file_name_(save_file_name) {
    map_ = map;
    bounding_box_map_car_ = tMapBox();
    bounding_box_map_ = tMapBox();

    QPalette pal;
    pal.setColor(QPalette::Background, Qt::white);
    this->setPalette(pal);
    // set gird layout with input options at the top, map in scene below
    grid_layout_ = new QGridLayout(this);
    // add checkboxes and silder to a grid layout to the top in one row
    autofullZoom_ = false;  // change to show full map
    switch_full_map_ = new QCheckBox("Show full map");
    switch_full_map_->setChecked(autofullZoom_);
    grid_layout_->addWidget(switch_full_map_, 0 /*row*/, 0 /*col*/,
                            1 /*rowSpan*/, 1 /*colSpan*/, 0);

    show_map_el_txt_ = show_text;
    switch_show_map_el_txt_ = new QCheckBox("show text");
    switch_show_map_el_txt_->setChecked(show_map_el_txt_);
    grid_layout_->addWidget(switch_show_map_el_txt_, 0 /*row*/, 1 /*col*/,
                            1 /*rowSpan*/, 1 /*colSpan*/, 0);

    slider_zoom_val_ = 60;
    slider_zoom_ = new QSlider(Qt::Horizontal);
    slider_zoom_->setValue(slider_zoom_val_);
    grid_layout_->addWidget(slider_zoom_, 0 /*row*/, 2 /*col*/, 1 /*rowSpan*/,
                            1 /*colSpan*/, 0);

    push_button_clear_ = new QPushButton("clear");
    grid_layout_->addWidget(push_button_clear_, 0 /*row*/, 3 /*col*/,
                            1 /*rowSpan*/, 1 /*colSpan*/, 0);
    recenter_car_ = false;
    push_button_recenter_car_ = new QPushButton("recenter car");
    grid_layout_->addWidget(push_button_recenter_car_, 0 /*row*/, 4 /*col*/,
                            1 /*rowSpan*/, 1 /*colSpan*/, 0);
    push_button_save_img_ = new QPushButton("save img");
    grid_layout_->addWidget(push_button_save_img_, 0 /*row*/, 5 /*col*/,
                            1 /*rowSpan*/, 1 /*colSpan*/, 0);
    // initialize the fonts
    m_mainFont = new QFont("Arial", 12);
    // Set Font
    setFont(*m_mainFont);

    // add the map plot under the widgets
    scene_ = new QGraphicsScene(this);
    // Set Backgroud Coloe
    scene_->setBackgroundBrush(QColor(MAP_V_QT_COLOR_BACKGROUND));
    view_ = new QGraphicsView(scene_, this);
    // don t show the scroll bars
    // view->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    // view->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
    // Map scent to view
    view_->mapToScene(0, 0);
    // Create View
    view_->mapToScene(view_->viewport()->geometry()).boundingRect();
    grid_layout_->addWidget(view_, 2 /*row*/, 0 /*col*/, 1 /*rowSpan*/,
                            0 /*colSpan*/, 0);

    setLayout(grid_layout_);

    // connect callbacks to ui widgets
    connect(switch_full_map_, SIGNAL(stateChanged(int)), this,
            SLOT(changeZoomMode(int)));
    connect(slider_zoom_, SIGNAL(valueChanged(int)), this,
            SLOT(slider_zoom_value_changed(int)));
    connect(switch_show_map_el_txt_, SIGNAL(stateChanged(int)), this,
            SLOT(changeMapEltxt(int)));
    connect(push_button_clear_, SIGNAL(released()), this,
            SLOT(handle_push_button_clear()));
    connect(push_button_recenter_car_, SIGNAL(released()), this,
            SLOT(handle_push_button_recenter_car()));
    connect(push_button_save_img_, SIGNAL(released()), this,
            SLOT(HandlePushButtonSaveImg()));
    // time to ensure that the ui is not drawn all the time
    time_last_ui_update_ = boost::posix_time::microsec_clock::local_time();

    bounding_box_map_ = tMapBox(tMapPoint(-1, -1), tMapPoint(1.0, 1.0));
}

// ____________________________________________________________________________
DisplayWidgetMap::~DisplayWidgetMap() {
    // Qt will handle deleting the widget if created wiht parent
    elements_.clear();
}

// ____________________________________________________________________________
void DisplayWidgetMap::doRepaint() {
    boost::posix_time::ptime t =
        boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = t - time_last_ui_update_;
    // not repaint before MAP_V_TIME_BEFORE_REPAINT_MSEC msec

    if (diff.total_milliseconds() > MAP_V_TIME_BEFORE_REPAINT_MSEC) {
        time_last_ui_update_ = t;
        // repaint();
        emit update();
    }
    // std::cout <<"timer: "<< diff.total_milliseconds() << std::endl;
}

// ____________________________________________________________________________
void DisplayWidgetMap::paintEvent(QPaintEvent *) {
    if (update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)) {
        try {
            scene_->clear();
            view_->viewport()->update();
            map_->GetGlobalCarPosition(&pos_);

            tMapTransformer translate_map_to_qt = setup_zoom_transform();

            int m_pixels_per_m_x_scaling =
                std::floor(view_->width() / (m_x_maxval - m_x_minval));
            int m_pixels_per_m_y_scaling =
                std::floor(view_->height() / (m_y_maxval - m_y_minval));
            m_pixels_per_m_x_scaling =
                std::min(m_pixels_per_m_x_scaling, m_pixels_per_m_y_scaling);
            m_pixels_per_m_y_scaling =
                m_pixels_per_m_x_scaling * -1;  // in qt y is flipped down

            // hide map element text if too many elements
            if (show_map_el_txt_ &&
                map_->GetElementCnt() > MAP_V_HIDE_TEXT_MAX_ELEMENTS) {
                switch_show_map_el_txt_->setChecked(false);
                show_map_el_txt_ = false;
            }
            // draw all the content
            draw_grid(translate_map_to_qt, m_pixels_per_m_x_scaling,
                      m_pixels_per_m_y_scaling);
            draw_map(translate_map_to_qt, m_pixels_per_m_x_scaling,
                     m_pixels_per_m_y_scaling);
            draw_car(translate_map_to_qt, m_pixels_per_m_x_scaling,
                     m_pixels_per_m_y_scaling);
            // set max min values for the scnece, no scaling back
            set_scense_box(translate_map_to_qt, m_pixels_per_m_x_scaling,
                           m_pixels_per_m_y_scaling);
            // recenter car of buttoom was triggered
            set_scroll_pos(translate_map_to_qt, m_pixels_per_m_x_scaling,
                           m_pixels_per_m_y_scaling);
        } catch (std::exception e) {
            std::stringstream ss;
            ss << "map ui exception paintEvent: " << e.what() << std::endl;
            LOG_ERROR_PRINTF("%s", ss.str().c_str());
        }
        if (!save_file_name_.isEmpty()) SaveMapToVid();
        update_mutex_.unlock();
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::SaveMapToVid() {
    static bool set_start_time = false;
    static boost::posix_time::ptime start_rec_time;
    if (!set_start_time) {
        start_rec_time = boost::posix_time::microsec_clock::local_time();
        set_start_time = true;
    }

    boost::posix_time::ptime t =
        boost::posix_time::microsec_clock::local_time();
    QString fileName = "recordings/frAIburg_MapVisualization/";  // TODO

    std::stringstream sstream;
    sstream << t;
    fileName += QString(sstream.str().c_str()) + "_" + save_file_name_;
    view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    update_mutex_.lock();
    const QPixmap pixmap = QPixmap::grabWidget(view_);
    update_mutex_.unlock();
    view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    view_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    pixmap.save(fileName);

    boost::posix_time::time_duration diff =
        boost::posix_time::microsec_clock::local_time() - t;
    // not repaint before MAP_V_TIME_BEFORE_REPAINT_MSEC msec

    LOG_INFO_PRINTF("saving map as img took %f s to %s",
                    diff.total_milliseconds() / 1e3,
                    fileName.toStdString().c_str());
}

// ____________________________________________________________________________
void DisplayWidgetMap::set_scroll_pos(
    const tMapTransformer &translate_map_to_qt,
    const int m_pixels_per_m_x_scaling, const int m_pixels_per_m_y_scaling) {
    if (recenter_car_) {
        recenter_car_ = false;
        // set scroll bar to show car
        tMapPoint carpos_qt(pos_.x, pos_.y);
        bg::transform(carpos_qt, carpos_qt, translate_map_to_qt);
        view_->verticalScrollBar()->setValue(bg::get<1>(carpos_qt) *
                                             m_pixels_per_m_y_scaling);
        view_->horizontalScrollBar()->setValue(bg::get<0>(carpos_qt) *
                                               m_pixels_per_m_x_scaling);
    }
}
// ____________________________________________________________________________
void DisplayWidgetMap::set_scense_box(
    const tMapTransformer &translate_map_to_qt,
    const int m_pixels_per_m_x_scaling, const int m_pixels_per_m_y_scaling) {
    tMapBox bounding_box_map_car_qt;
    // set scene frame
    float margin = MAP_V_DEFAULT_FULL_SCALE_MAP_SIDE_MARING_METER;
    bg::transform(bounding_box_map_car_, bounding_box_map_car_qt,
                  translate_map_to_qt);
    double m_x_max =
        bg::get<bg::max_corner, 0>(bounding_box_map_car_qt) + margin;
    double m_x_min =
        bg::get<bg::min_corner, 0>(bounding_box_map_car_qt) - margin;
    double m_y_max =
        bg::get<bg::max_corner, 1>(bounding_box_map_car_qt) + margin;
    double m_y_min =
        bg::get<bg::min_corner, 1>(bounding_box_map_car_qt) - margin;
    scene_->setSceneRect(QRectF(QPointF(m_x_min * m_pixels_per_m_x_scaling,
                                        m_y_max * m_pixels_per_m_y_scaling),
                                QPointF(m_x_max * m_pixels_per_m_x_scaling,
                                        m_y_min * m_pixels_per_m_y_scaling)));
}

// ____________________________________________________________________________
tMapTransformer DisplayWidgetMap::setup_zoom_transform() {
    // get a boost trans to transform the map poly to the qt frame

    // auto scaling so that full map and car are shown
    bounding_box_map_car_ = bounding_box_map_;
    // expand the bounding box with the car postion
    bg::expand(bounding_box_map_car_,
               bg::return_envelope<tMapBox>(tMapPoint(pos_.x, pos_.y)));
    // set min min max meter values with // margin to siedes

    if (autofullZoom_) {
        float margin = MAP_V_DEFAULT_FULL_SCALE_MAP_SIDE_MARING_METER;
        // get the max values form all elements in the map by the
        // boudning boxes set up while adding
        m_x_maxval = bg::get<bg::max_corner, 0>(bounding_box_map_car_) + margin;
        m_x_minval = bg::get<bg::min_corner, 0>(bounding_box_map_car_) - margin;
        m_y_maxval = bg::get<bg::max_corner, 1>(bounding_box_map_car_) + margin;
        m_y_minval = bg::get<bg::min_corner, 1>(bounding_box_map_car_) - margin;

        return tMapTransformer(m_x_minval, m_y_minval);
    } else {
        // keep the car in the center
        // set transfomation so that car is the center in the view
        double car_view_size =
            MAP_V_DEFAULT_CAR_CENTER_VIEW_SIZE_METER * slider_zoom_val_;
        m_x_maxval = pos_.x + car_view_size;
        m_x_minval = pos_.x - car_view_size;
        m_y_maxval = pos_.y + car_view_size;
        m_y_minval = pos_.y - car_view_size;
        // move view with the car
        return tMapTransformer(-pos_.x, -pos_.y);
    }

    int m_pixels_per_m_x_scaling =
        std::floor(view_->width() / (m_x_maxval - m_x_minval));
    int m_pixels_per_m_y_scaling =
        std::floor(view_->height() / (m_y_maxval - m_y_minval));
    m_pixels_per_m_x_scaling =
        std::min(m_pixels_per_m_x_scaling, m_pixels_per_m_y_scaling);
    m_pixels_per_m_y_scaling =
        m_pixels_per_m_x_scaling * -1;  // in qt y is flipped down
}

// ____________________________________________________________________________
void DisplayWidgetMap::draw_grid(const tMapTransformer &translate_map_to_qt,
                                 const int m_pixels_per_m_x_scaling,
                                 const int m_pixels_per_m_y_scaling) {
    // set colors for orgin an grid
    QPen pen_grid;
    pen_grid.setWidth(1);
    QColor gridcolor(MAP_V_QT_COLOR_GRID_NORMAL);
    gridcolor.setAlpha(150);
    pen_grid.setColor(gridcolor);
    pen_grid.setStyle(Qt::DotLine);
    QPen pen_grid_origin;
    pen_grid_origin.setWidth(3);
    QColor gridcolor_origin(MAP_V_QT_COLOR_GRID_ORIGIN);
    gridcolor_origin.setAlpha(200);
    pen_grid_origin.setStyle(Qt::DotLine);
    pen_grid_origin.setColor(gridcolor_origin);

    // get grid max min values for the whole map
    // draw the whole grid
    float margin = MAP_V_DEFAULT_FULL_SCALE_MAP_SIDE_MARING_METER;
    float m_x_maxval =
        bg::get<bg::max_corner, 0>(bounding_box_map_car_) + margin;
    float m_x_minval =
        bg::get<bg::min_corner, 0>(bounding_box_map_car_) - margin;
    float m_y_maxval =
        bg::get<bg::max_corner, 1>(bounding_box_map_car_) + margin;
    float m_y_minval =
        bg::get<bg::min_corner, 1>(bounding_box_map_car_) - margin;

    float grid_steps_meter = MAP_V_GRID_STEP_SIZE_METER;
    // draw x grid
    for (float i_meter = std::ceil(m_x_minval); i_meter < std::ceil(m_x_maxval);
         i_meter += grid_steps_meter) {
        tMapPoint global_start(i_meter, m_y_minval);
        tMapPoint global_end(i_meter, m_y_maxval);
        tMapPoint qt_start(0, 0);
        tMapPoint qt_end(0, 0);
        bg::transform(global_start, qt_start, translate_map_to_qt);
        bg::transform(global_end, qt_end, translate_map_to_qt);
        double qt_start_x = bg::get<0>(qt_start) * m_pixels_per_m_x_scaling;
        double qt_start_y = bg::get<1>(qt_start) * m_pixels_per_m_y_scaling;
        double qt_end_x = bg::get<0>(qt_end) * m_pixels_per_m_x_scaling;
        double qt_end_y = bg::get<1>(qt_end) * m_pixels_per_m_y_scaling;

        // down is pos in the coord system
        QPen *pen_grid_color;
        if (i_meter == 0) {
            pen_grid_color = &pen_grid_origin;
        } else {
            pen_grid_color = &pen_grid;
        }
        scene_->addLine(qt_start_x, qt_start_y, qt_end_x, qt_end_y,
                        *pen_grid_color);
    }
    // draw y grid
    for (float i_meter = std::ceil(m_y_minval); i_meter < std::ceil(m_y_maxval);
         i_meter += grid_steps_meter) {
        tMapPoint global_start(m_x_minval, i_meter);
        tMapPoint global_end(m_x_maxval, i_meter);
        tMapPoint qt_start(0, 0);
        tMapPoint qt_end(0, 0);
        bg::transform(global_start, qt_start, translate_map_to_qt);
        bg::transform(global_end, qt_end, translate_map_to_qt);
        double qt_start_x = bg::get<0>(qt_start) * m_pixels_per_m_x_scaling;
        double qt_start_y = bg::get<1>(qt_start) * m_pixels_per_m_y_scaling;
        double qt_end_x = bg::get<0>(qt_end) * m_pixels_per_m_x_scaling;
        double qt_end_y = bg::get<1>(qt_end) * m_pixels_per_m_y_scaling;

        // down is pos in the coord system
        QPen *pen_grid_color;
        if (i_meter == 0) {
            pen_grid_color = &pen_grid_origin;
        } else {
            pen_grid_color = &pen_grid;
        }
        scene_->addLine(qt_start_x, qt_start_y, qt_end_x, qt_end_y,
                        *pen_grid_color);
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::draw_map(const tMapTransformer &translate_map_to_qt,
                                const int m_pixels_per_m_x_scaling,
                                const int m_pixels_per_m_y_scaling) {
    // draw map poly
    BOOST_FOREACH (MapSceneElement &s, elements_) {
        s.Draw(m_pixels_per_m_x_scaling, m_pixels_per_m_y_scaling,
               translate_map_to_qt, show_map_el_txt_);
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::draw_car(const tMapTransformer &translate_map_to_qt,
                                const int m_pixels_per_m_x_scaling,
                                const int m_pixels_per_m_y_scaling) {
    // draw car pos if know in the map as a circle wit line to heading

    QColor car_fill_color(MAP_V_QT_COLOR_CAR_FILL);
    car_fill_color.setAlpha(180);
    QBrush brush_car(car_fill_color);
    brush_car.setStyle(Qt::SolidPattern);  // filled

    const QPen pen_car((MAP_V_QT_COLOR_CAR_OUTER_LINE));
    QPen pen_car_heading_line((MAP_V_QT_COLOR_CAR_HEADING_LINE));
    pen_car_heading_line.setWidth(2);

    if (map_->IsCarPosKnown()) {
        // car pos is defined at the front of the car
        // the car pos is known in the map

        tMapData d_x = (MAP_V_ADTF_CAR_OFFSET_X_METER)*cos(pos_.heading) -
                       (MAP_V_ADTF_CAR_OFFSET_Y_METER)*sin(pos_.heading);
        tMapData d_y = (MAP_V_ADTF_CAR_OFFSET_Y_METER)*cos(pos_.heading) +
                       (MAP_V_ADTF_CAR_OFFSET_X_METER)*sin(pos_.heading);
        tMapData car_center_x = pos_.x + d_x;
        tMapData car_center_y = pos_.y + d_y;
        // car_center_x += pos_.x;
        // car_center_y += pos_.y;
        tMapPoint car_pos(car_center_x, car_center_y);
        // tMapPoint car_pos(pos_.x, pos_.y);
        tMapPoint car_pos_qt(0., 0.);
        bg::transform(car_pos, car_pos_qt, translate_map_to_qt);

        double radius_car = MAP_V_CAR_RADIUS_IN_METER;  // car radius in meter
        double qt_x =
            bg::get<0>(car_pos_qt) * m_pixels_per_m_x_scaling - radius_car;
        double qt_y = bg::get<1>(car_pos_qt) * m_pixels_per_m_y_scaling;
        scene_->addEllipse(qt_x - radius_car * m_pixels_per_m_x_scaling,
                           qt_y - radius_car * m_pixels_per_m_y_scaling,
                           radius_car * 2. * m_pixels_per_m_x_scaling,
                           radius_car * 2. * m_pixels_per_m_y_scaling, pen_car,
                           brush_car);
        // add line for direction of the car
        scene_->addLine(
            qt_x, qt_y,
            radius_car * cos(pos_.heading) * m_pixels_per_m_x_scaling + qt_x,
            radius_car * sin(pos_.heading) * m_pixels_per_m_y_scaling + qt_y,
            pen_car_heading_line);
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::changeZoomMode(int mode) {
    if (update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)) {
        if (mode == 2)
            autofullZoom_ = true;
        else if (mode == 0)
            autofullZoom_ = false;
        update_mutex_.unlock();
        doRepaint();
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::HandlePushButtonSaveImg() {
    // save img
    QPixmap pixmap;
    view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    if (update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)) {
        pixmap = QPixmap::grabWidget(view_);
        update_mutex_.unlock();
    }
    view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    view_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    QString filters("png files (*.png)");
    QString defaultFilter("png (*.png)");

    QString file_name = QFileDialog::getSaveFileName(
        this, "Select one or more files to open", QDir::currentPath(),
        "Images (*.png )");
    if (!file_name.isNull()) {
        pixmap.save(file_name);
    } else {
        LOG_ERROR_PRINTF("save file name is null");
    }
}

void DisplayWidgetMap::handle_push_button_clear(){
  //
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(this, "FrAIburg Map",
                                  "Clear all map elements?",
                                QMessageBox::Yes|QMessageBox::No);

  if (reply == QMessageBox::Yes &&
          update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)){
    map_->ClearElements();
    elements_.clear();
    //set grid bounding box to car
    tMapCarPosition pos;
    if (map_->GetGlobalCarPosition(&pos)){
      bounding_box_map_ = tMapBox(tMapPoint(pos.x-1, pos.y-1),
                                 tMapPoint(pos.x+1, pos.y+1));
    }
  }
}

void DisplayWidgetMap::handle_push_button_recenter_car() {
    recenter_car_ = true;
}

// ____________________________________________________________________________
void DisplayWidgetMap::changeMapEltxt(int mode) {
    if (update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)) {
        if (mode == 2)
            show_map_el_txt_ = true;
        else if (mode == 0)
            show_map_el_txt_ = false;
        update_mutex_.unlock();
        doRepaint();
    }
}

void DisplayWidgetMap::slider_zoom_value_changed(int k) {
    // update_mutex_.lock();
    if (k % 2 == 0 && k > 0 &&
        update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)) {
        slider_zoom_val_ = k;

        update_mutex_.unlock();
        doRepaint();
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::MapOnEventAddedNew(tSptrMapElement el) {
    if (update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)) {
        elements_.push_back(MapSceneElement(scene_, el));

        // xepend bounding boxes for the map
        bg::expand(bounding_box_map_,
                   bg::return_envelope<tMapBox>(el->GetGlobalPoly()));
        // emit doRepaint();
        update_mutex_.unlock();
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::MapOnEventRemoved(tSptrMapElement el) {
    if (update_mutex_.tryLock(MAP_V_TIME_BEFORE_REPAINT_MSEC)) {
        // remove element from list
        std::vector<MapSceneElement>::iterator it = elements_.begin();
        for (; it != elements_.end(); ++it) {
            if ((*it).el_->GetID() == el->GetID()) {
                elements_.erase(it);
                break;  // not return, call unlock
            }
        }
        update_mutex_.unlock();
    }
}

// ____________________________________________________________________________
void DisplayWidgetMap::MapOnEventDistanceReached(
    tSptrMapElement el, const tMapData threshold,
    bool distance_lower_threshold) {
    boost::ignore_unused(el, threshold, distance_lower_threshold);
}
