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
// filter with a live display of the map

#ifndef AADCUSER_FRAIBURG_MAP_VISUALIZATION_FILTER_H_
#define AADCUSER_FRAIBURG_MAP_VISUALIZATION_FILTER_H_

#define ADTF_MAP_VISUALIZATION_FILTER_NAME \
                "frAIburg Singleton Map Visualization"
#define OID_ADTF_MAP_VISUALIZATION_FILTER \
                "adtf.example.map_visualization_filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "AADC user map Visualizationfilter"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "MAPVisualizationFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small Template Filter."
//adtf filter propteries
#define ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL \
    "UI update intervall in [s]"
#define ADTF_PROPERTY_MAP_CAR_POS_UPDATA_INTERVALL_DEFAULT_VAL 0.05

#define ADTF_PROPERTY_LOAD_ENABLED_TEST_MAP_NAME "Enable load frAIburg test map"
#define ADTF_PROPERTY_LOAD_ENABLED_TEST_MAP_DEFAULT false

#define ADTF_PROPERTY_SAVE_TO_PNG_NAME \
  "save png name video: if string in not null each update intervall saved"
#define ADTF_PROPERTY_SAVE_TO_PNG_DEFAULT ""


#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include "slim_pins.h"

#include "map_displaywidget.h"
#include "map_element.hpp"
#include "global_map.hpp"
#include "test_reference_map.h"

#include <QtCore/QtCore>
#include <QtGui/QtGui>
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

/*! @defgroup MapVisualizationFilter TODO
*  @{
*
*  ![Plugin Template Filter](User_Template.PNG)
*
* This is a small template which can be used by the AADC teams for their own
* filter implementations.
*
* **Dependencies** \n
* This plugin needs the following libraries:
*
*
* **Filter Properties**
* | Property | Description | Default |
* | -------- | ----------- | ------- |
* | bla      | blubb       | 9       |
*
*
*
* **Input Pins**
* | Pin            | Description          | MajorType           |
* | ---------------| -------------------- |tPosition            |
*
* | input_template | An example input pin | MEDIA_TYPE_TEMPLATE |
* MEDIA_TYPE_TEMPLATE |
*
*
* **Plugin Details**
* | | |
* |-|-|
* | Path    | src/aadcUser/frAiburg_TemplateFilter |
* | Filename| frAiburg_templateFilter.plb          |
* | Version | 1.0.0                                |
*
* _____________________________________________________________________________
*  Markdown examples --- Delete everything below here!
* _____________________________________________________________________________
*
*
*  [Full doxygen markdown
* documentation](https: // www.stack.nl/~dimitri/doxygen/manual/markdown.html)
*
* - Code Block
*  ~~~~~~~~~~~~~~~{.c}
*  int func(int a,int b) { return a*b; }
*  ~~~~~~~~~~~~~~~
* - [Link](https: // www.github.com)
* -
* ![Image](https: //
* www.audi-autonomous-driving-cup.com/wp-content/uploads/2017/02/Audi-emblem-2016-black-small.png)
*
*
*
*/

class MapVisualizationFilter : public QObject, public cBaseQtFilter{
    Q_OBJECT
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_MAP_VISUALIZATION_FILTER,
                ADTF_MAP_VISUALIZATION_FILTER_NAME,
                adtf::OBJCAT_Tool);

 public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    explicit MapVisualizationFilter(const tChar* __info);

    /*! default destructor */
    virtual ~MapVisualizationFilter();
    /*! This Function is always called when any property has changed.*/
    tResult UpdateProperties(ucom::IException** __exception_ptr = NULL);

 private:
    void InitMAP();
    void InitTimer();
    frAIburg::map::GlobalMap* map_; //map as a singelton

    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;
    tResult Run(tInt nActivationCode,
                const tVoid* pvUserData, tInt szUserDataSize,
                ucom::IException** __exception_ptr);

    /*! filter properties, set in adtf*/
    tFloat32 property_map_car_pos_update_interval_in_s_;
    void set_all_properties(void);
    void get_all_properties(void);
 protected:
    cCriticalSection update_mutex_;
    /// This filter has no PINS

    /** cFILTER STATE
     * MACHINE***************************************************/
    /*! Implements the default cFilter state machine call.  e*/
    virtual tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Implements the default cFilter state machine call.*/
    virtual tResult Shutdown(tInitStage stage,
                             ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.*/
    virtual tResult OnPinEvent(adtf::IPin* source, tInt event_code, tInt param1,
                               tInt param2, adtf::IMediaSample* media_sample);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    /** PIN METHOODS  *********************************************************/
    /*! this function creates all the input pins of filter*/
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter*/
    tResult SetPinIDs();
    /** QT  *********************************************************/
    /*! Creates the widget instance*/
    /*! The displayed widget*/
 protected:
    DisplayWidgetMap *qtwidget_;
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();

};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_MAP_VISUALIZATION_FILTER_H_

/*!
*@}
*/
