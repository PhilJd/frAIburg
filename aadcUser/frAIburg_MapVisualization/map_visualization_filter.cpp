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

#include "map_visualization_filter.h"
using namespace frAIburg::map;

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_MAP_VISUALIZATION_FILTER_NAME,
                   OID_ADTF_MAP_VISUALIZATION_FILTER, MapVisualizationFilter);
// ____________________________________________________________________________
MapVisualizationFilter::MapVisualizationFilter(const tChar* __info)
    : QObject(), cBaseQtFilter(__info) {
    InitMAP();
    set_all_properties();
    LOG_INFO_PRINTF("boost verion %d", BOOST_VERSION);
}

// ____________________________________________________________________________
MapVisualizationFilter::~MapVisualizationFilter() {
    // map_->clear_elements();//clean the map with all smart ptr
}

// ____________________________________________________________________________
void MapVisualizationFilter::InitMAP() { map_ = frAIburg::map::getInstance(); }

// ____________________________________________________________________________
void MapVisualizationFilter::InitTimer() {
    tUInt32 nFlags = 0;
    // Create our cyclic timer.
    cString strTimerName = OIGetInstanceName();
    strTimerName += ".rttimerCSVwriter";
    tTimeStamp tmPeriod = 1e6 * property_map_car_pos_update_interval_in_s_;
    tTimeStamp tmstartdelay = 1;
    timer_ = _kernel->TimerCreate(tmPeriod,
                                  tmstartdelay,  // dely for the first start
                                  static_cast<IRunnable*>(this),  // call run
                                  NULL, NULL, 0, nFlags, strTimerName);
    if (!timer_) {
        LOG_ERROR("map ui unable to create timer");
    }
}

// ____________________________________________________________________________
tResult MapVisualizationFilter::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cBaseQtFilter::Init(stage, __exception_ptr))

    tResult nResult = ERR_NOERROR;

    if (stage == StageFirst) {
        // in StageFirst you can create and register your static pins.
        nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");

    } else if (stage == StageNormal) {
    } else if (stage == StageGraphReady) {
        get_all_properties();
        // query your pins about their media types and additional meta data.
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");

        InitTimer();
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
// ____________________________________________________________________________
tResult MapVisualizationFilter::Shutdown(tInitStage stage, __exception) {
    // Destroy the timer
    if (timer_ && stage == StageGraphReady) {
        __synchronized_obj(update_mutex_);
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    // call the base class implementation
    return cBaseQtFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult MapVisualizationFilter::Start(__exception) {
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));

    running_ok_ = true;
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapVisualizationFilter::Stop(__exception) {
    running_ok_ = false;

    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapVisualizationFilter::OnPinEvent(IPin* source, tInt event_code,
                                           tInt param1, tInt param2,
                                           IMediaSample* media_sample) {
    // first check what kind of event it is
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(media_sample);
        // by comparing it to our member pin variable we can find out which
        // pin received the sample
    }
    RETURN_NOERROR;
}
// ____________________________________________________________________________
tResult MapVisualizationFilter::CreateInputPins(__exception) { RETURN_NOERROR; }

// ____________________________________________________________________________
tResult MapVisualizationFilter::CreateOutputPins(__exception) {
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult MapVisualizationFilter::SetPinIDs() { RETURN_NOERROR; }

// ____________________________________________________________________________
tResult MapVisualizationFilter::Run(tInt nActivationCode,
                                    const tVoid* pvUserData,
                                    tInt szUserDataSize,
                                    ucom::IException** __exception_ptr) {
    if (running_ok_ && qtwidget_ != NULL) {
        // update the ui
        __synchronized_obj(update_mutex_);
        qtwidget_->doRepaint();
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void MapVisualizationFilter::get_all_properties(void) {
    // sample time
    property_map_car_pos_update_interval_in_s_ =
        GetPropertyFloat(ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL);
    // LOG_DUMP(A_UTILS_NS::cString::Format("map car postion update intervall in
    // s: ",
    //                                      property_map_update_interval_in_s_));
}

// ____________________________________________________________________________
void MapVisualizationFilter::set_all_properties(void) {
    // sample time
    SetPropertyFloat(ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL,
                     ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL_DEFAULT_VAL);
    SetPropertyBool(ADTF_PROPERTY_LOAD_ENABLED_TEST_MAP_NAME,
                    ADTF_PROPERTY_LOAD_ENABLED_TEST_MAP_DEFAULT);

    SetPropertyStr(ADTF_PROPERTY_SAVE_TO_PNG_NAME,
                   ADTF_PROPERTY_SAVE_TO_PNG_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_SAVE_TO_PNG_NAME NSSUBPROP_DESCRIPTION,
                   "set name.png, in not empty the map is saved as a "
                   "an img each update intervall");

    SetPropertyBool("Show text at start", true);
}

// ____________________________________________________________________________
tResult MapVisualizationFilter::UpdateProperties(
    ucom::IException** __exception_ptr) {
    RETURN_IF_FAILED(cBaseQtFilter::UpdateProperties(__exception_ptr));
    get_all_properties();
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tHandle MapVisualizationFilter::CreateView() {
    QWidget* pWidget = reinterpret_cast<QWidget*>(m_pViewport->VP_GetWindow());
    bool show_txt = GetPropertyBool("Show text at start");

    qtwidget_ =
        new DisplayWidgetMap(pWidget, map_, show_txt,
                             GetPropertyStr(ADTF_PROPERTY_SAVE_TO_PNG_NAME));
    if (!qtwidget_) {
        LOG_ERROR_PRINTF("create DisplayWidgetMap failed ");
    }
    // register the widget to the map to get updates form changes
    map_->RegisterEventListener(qtwidget_);
    LOG_INFO("MapVisualizationFilter: adding test map ");
    // add test map
    if (GetPropertyBool(ADTF_PROPERTY_LOAD_ENABLED_TEST_MAP_NAME)) {
        AddReferenceMap(map_);
        AddTestElementsStateMachine(map_);
    }
    return (tHandle)qtwidget_;
}

// ____________________________________________________________________________
tResult MapVisualizationFilter::ReleaseView() {
    std::cout << "MapVisualizationFilter: release view" << std::endl;
    running_ok_ = false;
    if (qtwidget_ != NULL) {
        // qtwidget_->clearAll();
        map_->DeregisterEventListener(qtwidget_);
        delete qtwidget_;
        // TODO(markus) check adtf exmaple if delete is need
        qtwidget_ = NULL;
    }
    RETURN_NOERROR;
}
