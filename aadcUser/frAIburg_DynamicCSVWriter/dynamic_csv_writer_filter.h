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
#ifndef AADCUSER_FRAIBURG_CSV_FILTER_H_
#define AADCUSER_FRAIBURG_CSV_FILTER_H_

#define ADTF_CSW_WRITER_FILTER_NAME "frAIburg dynamic csv writer"
#define OID_ADTF_CSV_FILTER "adtf.example.dynamic_csv_writer_filter"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "AADC user dynamic csv writer of tSignal vals"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "DynamicCSVWriterFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "DynamicCSVWriterFilter."

#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include <fstream>
#include <sstream>  // std::ostringstream

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include "aadc_structs.h"
#include "slim_pins.h"
#include "adtf_log_macros.h"

/// Filter Properties
#define ADTF_PROPERTY_CSV_SAMPLE_TIME_NAME "Sample time in [s]"
#define ADTF_PROPERTY_CSV_SAMPLE_TIME_DEFAULT 0.1
#define ADTF_PROPERTY_CSV_FILE_NAME "Output file name"
#define ADTF_PROPERTY_CSV_FILE_NAME_DEFAULT "data.csv"
#define CSV_FILE_PATH_DEFAULT \
    "recordings/frAIburg_dynamic_csv_writer/"  // no "/" at the beginning
// note this filter is not able to create recordings in the home dir

/*! @defgroup frAIburg_dynamic_csv_writer_filter
*  @{
*
* **Dependencies** \n
* This plugin needs the following libraries:
*
*
* **Filter Properties**
* | Property | Description | Default |
* | -------- | ----------- | ------- |
* | Sample time |   -      | 0.1      |
* | outputfilename |  -       | data.csv      |
*
*
* **Input Pins**
* | Pin            | Description          | MajorType           |              |
* | ---------------| -------------------- | ------------------- |
* ------------------- |
* | dynamipin | data to write to the csv | tSignalValue,tPosition,tWheelData | |
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
// not def in "aadc_structs.h", def for simpler acc without ids,
// not the postion must be the same in mediatype description
typedef struct {
    tFloat32 f32XPos;
    tFloat32 f32YPos;
    tFloat32 f32radius;
    tFloat32 f32speed;
    tFloat32 f32heading;
} tCSVPostionData;

// !  Template filter for AADC Teams
/*!
* This is an example filter for the AADC
*/
class DyamicCSVWriterFilter : public adtf::cFilter {
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_CSV_FILTER, ADTF_CSW_WRITER_FILTER_NAME,
                adtf::OBJCAT_DataFilter);

 public:
    explicit DyamicCSVWriterFilter(const tChar* __info);

    /*! default destructor */
    virtual ~DyamicCSVWriterFilter();

    /*! alll the dynamic pins are added here*/
    tResult Connect(IPin* pSource, const tChar* strDestName,
                    __exception = NULL);
    /*! This Function is always called when any property has changed.*/
    tResult UpdateProperties(ucom::IException** __exception_ptr = NULL);

 protected:
    /// INPUT PINS
    slim::DynamicInputPin dynamic_pins_in_tsignal_;
    slim::DynamicInputPin dynamic_pins_in_tpostion_;
    slim::DynamicInputPin dynamic_pins_in_twheel_;
    slim::DynamicInputPin dynamic_pins_in_imu;
    /*! timer to call run func with a property set in adtf */
    tHandle timer_;  /// every property_sample_time_in_s_ run is called
    tBool running_ok_;

    /*! csv files*/
    ofstream csvfile_;
    cString csv_header_;
    uint64_t line_cnt_;

    /*! data saved for the pins at pin event*/
    tTimeStamp start_time_micro_s_;
    std::vector<tSignalValue>
        pin_data_tsignal_;  // data for dynamic_pins_in_tsignal_
    std::vector<tCSVPostionData>
        pin_data_postion_;  // data for dynamic_pins_in_postion_
    std::vector<tWheelData>
        pin_data_wheel_;  // data for dynamic_pins_in_twheel_
    std::vector<tInerMeasUnitData>
            pin_data_imu_; // data for dynamic_pins_in_inerMeasUnit

    /*! filter properties, set in adtf*/
    tFloat32 property_sample_time_in_s_;
    string property_output_file_name;
    void GetAllProperties(void);
    void SetAllProperties(void);

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

    /** USERER METHOODS *******************************************************/
    /*! runnable for timer */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData,
                tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);
    template <typename Tdata>
    tResult ProcessDynamicdata(IPin* source, IMediaSample* media_sample,
                               slim::DynamicInputPin& dpin,
                               std::vector<Tdata>& data_vec);
    tResult AppendCSVHeaderPinNames(slim::DynamicInputPin& dpins,
                                    uint8_t val_cnt = 1);
    tResult AppendCSVHeaderPinNames(slim::DynamicInputPin& dpins,
                                    vector<string>& subtypes_names);

    void InitDynamicPinBuffers();
    tResult WriteCSVHeader();

    /**
     * ************************************************************************/
};

// *****************************************************************************
#endif  // AADCUSER_FRAIBURG_CSV_FILTER_H_

/*!
*@}
*/
