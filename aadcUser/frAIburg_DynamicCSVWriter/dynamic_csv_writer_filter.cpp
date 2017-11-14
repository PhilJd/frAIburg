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

#include "dynamic_csv_writer_filter.h"

/// Create filter shell
ADTF_FILTER_PLUGIN(ADTF_CSW_WRITER_FILTER_NAME, OID_ADTF_CSV_FILTER,
                   DyamicCSVWriterFilter);

DyamicCSVWriterFilter::DyamicCSVWriterFilter(const tChar* __info)
    : cFilter(__info) {
    // using convenience method for configuring dynamic connection pins ...
    ConfigureConnectionPins(0);
    property_sample_time_in_s_ = ADTF_PROPERTY_CSV_SAMPLE_TIME_DEFAULT;
    property_output_file_name = string(ADTF_PROPERTY_CSV_FILE_NAME_DEFAULT);
    SetAllProperties();
    start_time_micro_s_ = 0;
    line_cnt_ = 0;
}

// ____________________________________________________________________________
DyamicCSVWriterFilter::~DyamicCSVWriterFilter() {}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::Init(tInitStage stage, __exception) {
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))

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
        // In this stage you would do further initialisation and/or create your
        // dynamic pins.
    } else if (stage == StageGraphReady) {
        GetAllProperties();

        // query your pins about their media types and additional meta data.
        nResult = SetPinIDs();
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to set IDs");

        tUInt32 nFlags = 0;
        // Create our cyclic timer.
        // we do this here and not in Start() to prevent the impact of
        // the thread creation during RL_Running
        cString strTimerName = OIGetInstanceName();
        strTimerName += ".rttimerCSVwriter";
        tTimeStamp tmPeriod = 1e6 * property_sample_time_in_s_;
        tTimeStamp tmstartdelay = 1e6 * 0.3;
        timer_ =
            _kernel->TimerCreate(tmPeriod,
                                 tmstartdelay,  // dely for the first start
                                 static_cast<IRunnable*>(this),  // call run
                                 NULL, NULL, 0, nFlags, strTimerName);

        if (!timer_) {
            THROW_ERROR_DESC(ERR_UNEXPECTED, "Unable to create timer");
        } else {
            // create and open the .csv file set in the properties
            // the filenam is changed if the file exist
            csvfile_.open(property_output_file_name.c_str());

            WriteCSVHeader();
        }
        InitDynamicPinBuffers();
    }
    RETURN_NOERROR;
}

tResult DyamicCSVWriterFilter::WriteCSVHeader() {
    tResult nResult = ERR_NOERROR;
    // write header to file in same order as written later
    csv_header_.Append("time in s,");
    nResult = AppendCSVHeaderPinNames(dynamic_pins_in_tsignal_);
    if (IS_FAILED(nResult))
        LOG_ERROR("Failed append header dynamic_pins_in_tsignal_");
    //            nResult =
    //            AppendCSVHeaderPinNames(dynamic_pins_in_tpostion_,5);
    //            if (IS_FAILED(nResult)) LOG_ERROR("Failed append
    //            header
    //            dynamic_pins_in_tpostion_");
    vector<string> names_subtypes =
        boost::assign::list_of("X")("Y")("Radius")("Speed")("Heading");
    nResult =
        AppendCSVHeaderPinNames(dynamic_pins_in_tpostion_, names_subtypes);
    if (IS_FAILED(nResult))
        LOG_ERROR("Failed append header dynamic_pins_in_tpostion_");
    /// wheel data
    vector<string> names_subtypes_wheel = boost::assign::list_of("Tach")("Dir");
    nResult =
        AppendCSVHeaderPinNames(dynamic_pins_in_twheel_, names_subtypes_wheel);
    if (IS_FAILED(nResult))
        LOG_ERROR("Failed append header dynamic_pins_in_twheel_");
    /// imu header data
    vector<string> names_subtypes_imu = boost::assign::list_of("f32A_x")(
        "f32A_y")("f32A_z")("f32G_x")("f32G_y")("f32G_z")("f32M_x")("f32M_y")(
        "f32M_z")("f32roll")("f32pitch")("f32yaw");
    nResult = AppendCSVHeaderPinNames(dynamic_pins_in_imu, names_subtypes_imu);
    if (IS_FAILED(nResult))
        LOG_ERROR("Failed append header dynamic_pins_in_imu");

    // write header to file
    csvfile_ << csv_header_ << std::endl;
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::AppendCSVHeaderPinNames(
    slim::DynamicInputPin& dpins, uint8_t val_cnt) {
    for (uint32_t i = 0; i < dpins.getDynamicPinCnt(); i++) {
        cObjectPtr<cDynamicInputPin> pin;
        RETURN_IF_FAILED(dpins.getDynamicPin(i, pin));

        for (uint8_t j = 0; j < val_cnt; ++j) {
            csv_header_.Append(pin->GetName());
            if (val_cnt > 1) {
                char buffer[1];
                sprintf(buffer, "%u", j);
                csv_header_.Append(buffer);
            }
            csv_header_.Append(", ");
        }
    }

    RETURN_NOERROR;
}
// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::AppendCSVHeaderPinNames(
    slim::DynamicInputPin& dpins, vector<string>& subtypes_names) {
    for (uint32_t i = 0; i < dpins.getDynamicPinCnt(); i++) {
        cObjectPtr<cDynamicInputPin> pin;
        RETURN_IF_FAILED(dpins.getDynamicPin(i, pin));

        for (uint8_t j = 0; j < subtypes_names.size(); j++) {
            csv_header_.Append(pin->GetName());
            csv_header_.Append(subtypes_names[j].c_str());
            csv_header_.Append(", ");
        }
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::Start(__exception) {
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    // set intial start time
    if (start_time_micro_s_ == 0) start_time_micro_s_ = _clock->GetStreamTime();
    // enable the timer function  to write data
    running_ok_ = true;
    LOG_INFO(A_UTILS_NS::cString::Format("start writing to %s ",
                                         property_output_file_name.c_str()));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void DyamicCSVWriterFilter::InitDynamicPinBuffers() {
    // resize buffer for pin size
    pin_data_tsignal_.resize(dynamic_pins_in_tsignal_.getDynamicPinCnt());
    pin_data_postion_.resize(dynamic_pins_in_tpostion_.getDynamicPinCnt());
    pin_data_wheel_.resize(dynamic_pins_in_twheel_.getDynamicPinCnt());
    pin_data_imu_.resize(dynamic_pins_in_imu.getDynamicPinCnt());
}
// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::Stop(__exception) {
    running_ok_ = false;
    LOG_INFO(A_UTILS_NS::cString::Format("paused writing to %s ",
                                         property_output_file_name.c_str()));
    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}
// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::Shutdown(tInitStage stage, __exception) {
    // Destroy the timer
    if (timer_ && stage == StageGraphReady) {
        _kernel->TimerDestroy(timer_);
        timer_ = NULL;
        // close the output file
        csvfile_.close();

        if (!boost::filesystem::exists(property_output_file_name)) {
            LOG_ERROR("DyamicCSVWriterFilter faild to save file");
        } else {
            boost::filesystem::path full_path(property_output_file_name);
            LOG_INFO(A_UTILS_NS::cString::Format(
                "csv file saved %s , \
                                                %u lines written",
                full_path.string().c_str(), line_cnt_));
        }
    }
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::OnPinEvent(IPin* source, tInt event_code,
                                          tInt param1, tInt param2,
                                          IMediaSample* media_sample) {
    // first check what kind of event it is
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        RETURN_IF_POINTER_NULL(media_sample);
        // get data if the pins are the source
        ProcessDynamicdata<tSignalValue>(
            source, media_sample, dynamic_pins_in_tsignal_, pin_data_tsignal_);
        ProcessDynamicdata<tCSVPostionData>(
            source, media_sample, dynamic_pins_in_tpostion_, pin_data_postion_);
        ProcessDynamicdata<tWheelData>(
            source, media_sample, dynamic_pins_in_twheel_, pin_data_wheel_);
        ProcessDynamicdata<tInerMeasUnitData>(
            source, media_sample, dynamic_pins_in_imu, pin_data_imu_);
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
template <typename Tdata>
tResult DyamicCSVWriterFilter::ProcessDynamicdata(
    IPin* source, IMediaSample* media_sample, slim::DynamicInputPin& dpins,
    std::vector<Tdata>& data_vec) {
    // check all the dynamic pins for data and save it the the corres modning
    // vector
    uint32_t index;
    if (dpins.IsSource(source, &index)) {
        cObjectPtr<cDynamicInputPin> pin;
        dpins.getDynamicPin(index, pin);
        Tdata* data;
        if (IS_OK(dpins.OnPinEventRead_start(media_sample, (const tVoid**)&data,
                                             sizeof(Tdata)))) {
            if (index <= data_vec.size()) {
                data_vec[index] = *data;

                // std::string name;
                // dpins.getDynamicPinName(index,name);
                // LOG_INFO(A_UTILS_NS::cString::Format("index: %d,name %s",
                //                index,
                //                //pin->GetName(),
                //                name.c_str()));
                dpins.OnPinEventRead_end(media_sample, (const tVoid**)&data);

            } else {
                LOG_ERROR("vetor size not correct for index");
                RETURN_ERROR(ERR_INVALID_ARG);
            }

        } else {
            LOG_ERROR("read dynamic pin failed, check types");
            RETURN_ERROR(ERR_IO_INCOMPLETE);
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::CreateInputPins(__exception) {
    slim::register_pin_func func = &DyamicCSVWriterFilter::RegisterPin;

    RETURN_IF_FAILED(dynamic_pins_in_tsignal_.StageFirstCreate(
        this, func, slim::FIXED_MEDIA_TYPE, "tSignalValue"));

    RETURN_IF_FAILED(dynamic_pins_in_tpostion_.StageFirstCreate(
        this, func, slim::FIXED_MEDIA_TYPE, "tPosition"));

    RETURN_IF_FAILED(dynamic_pins_in_twheel_.StageFirstCreate(
        this, func, slim::FIXED_MEDIA_TYPE, "tWheelData"));

    RETURN_IF_FAILED(dynamic_pins_in_imu.StageFirstCreate(
        this, func, slim::FIXED_MEDIA_TYPE, "tInerMeasUnitData"));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::CreateOutputPins(__exception) {
    // no output
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::SetPinIDs() {
    // no ids for tx and rx are needed
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::Connect(IPin* source, const tChar* strDestName,
                                       __exception) {
    THROW_IF_POINTER_NULL(source);
    // connect all dynamicpin witht there media type here
    // LOG_INFO(A_UTILS_NS::cString::Format("connect new pin: %s",strDestName));
    if (dynamic_pins_in_tsignal_.IsEqualMediaType(source)) {
        if (IS_FAILED(
                dynamic_pins_in_tsignal_.StageConnect(source, strDestName))) {
            LOG_ERROR("dyn pin failed connect");
        }

    } else if (dynamic_pins_in_tpostion_.IsEqualMediaType(source)) {
        if (IS_FAILED(
                dynamic_pins_in_tpostion_.StageConnect(source, strDestName))) {
            LOG_ERROR("dyn pin failed connect");
        }

    } else if (dynamic_pins_in_twheel_.IsEqualMediaType(source)) {
        if (IS_FAILED(
                dynamic_pins_in_twheel_.StageConnect(source, strDestName))) {
            LOG_ERROR("dyn pin failed connect");
        }

    } else if (dynamic_pins_in_imu.IsEqualMediaType(source)) {
        if (IS_FAILED(dynamic_pins_in_imu.StageConnect(source, strDestName))) {
            LOG_ERROR("dynamic_pins_in_imu pin failed connect");
        }

    } else {
        LOG_INFO(A_UTILS_NS::cString::Format(
            "mediatype for %s is not supported", strDestName));
    }
    return (cFilter::Connect(source, strDestName, __exception_ptr));
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::Run(tInt nActivationCode,
                                   const tVoid* pvUserData, tInt szUserDataSize,
                                   ucom::IException** __exception_ptr) {
    // TODO lock!

    if (running_ok_) {
        // LOG_INFO("Timer running");
        // check if smaple time is ok
        tTimeStamp current_time_micro_s = _clock->GetStreamTime();

        // write the data to the output file
        std::ostringstream line;
        // write time in s
        line << (current_time_micro_s - start_time_micro_s_) * 1 / 1e6 << ",";
        // write data for tSignalValue
        vector<tSignalValue>::iterator it = pin_data_tsignal_.begin();
        for (; it != pin_data_tsignal_.end(); ++it) {
            line << it->f32Value << ",";
        }
        // write data for tPostion
        vector<tCSVPostionData>::iterator it_pos = pin_data_postion_.begin();
        for (; it_pos != pin_data_postion_.end(); ++it_pos) {
            line << it_pos->f32XPos << "," << it_pos->f32YPos << ","
                 << it_pos->f32radius << "," << it_pos->f32speed << ","
                 << it_pos->f32heading << ",";
        }
        vector<tWheelData>::iterator it_w = pin_data_wheel_.begin();
        for (; it_w != pin_data_wheel_.end(); ++it_w) {
            line << it_w->ui32WheelTach << ","
                 << boost::lexical_cast<std::string, int>(it_w->i8WheelDir)
                 << ",";
        }
        vector<tInerMeasUnitData>::iterator it_imu = pin_data_imu_.begin();
        for (; it_imu != pin_data_imu_.end(); ++it_imu) {
            line << it_imu->f32A_x << "," << it_imu->f32A_y << ","
                 << it_imu->f32A_z << "," << it_imu->f32G_x << ","
                 << it_imu->f32G_y << "," << it_imu->f32G_z << ","
                 << it_imu->f32M_x << "," << it_imu->f32M_y << ","
                 << it_imu->f32M_z << "," << it_imu->f32Roll << ","
                 << it_imu->f32Pitch << "," << it_imu->f32Yaw << ",";
        }
        // end of line
        line << std::endl;  // tdodo flush not all the time
        ++line_cnt_;
        csvfile_ << line.str();

        tFloat32 write_time_s =
            (_clock->GetStreamTime() - current_time_micro_s) * 1 / 1e6;
        if (write_time_s >= property_sample_time_in_s_)
            LOG_WARNING("csw writer sample time to fast for write");
        // LOG_INFO(A_UTILS_NS::cString::Format("time writing in s
        // %f",write_time_s));
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void DyamicCSVWriterFilter::SetAllProperties(void) {
    // sample time
    SetPropertyFloat(ADTF_PROPERTY_CSV_SAMPLE_TIME_NAME,
                     ADTF_PROPERTY_CSV_SAMPLE_TIME_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_CSV_SAMPLE_TIME_NAME NSSUBPROP_DESCRIPTION,
                   "set after which time in s all the data received is "
                   "written to the csv file");
    SetPropertyFloat(ADTF_PROPERTY_CSV_SAMPLE_TIME_NAME NSSUBPROP_MIN, 0.00001);
    SetPropertyFloat(ADTF_PROPERTY_CSV_SAMPLE_TIME_NAME NSSUBPROP_MAX, 10);

    // csv output file
    SetPropertyStr(ADTF_PROPERTY_CSV_FILE_NAME,
                   ADTF_PROPERTY_CSV_FILE_NAME_DEFAULT);
    SetPropertyStr(ADTF_PROPERTY_CSV_FILE_NAME NSSUBPROP_DESCRIPTION,
                   "set the output file name (no path supported for now)");
}

// ____________________________________________________________________________
void DyamicCSVWriterFilter::GetAllProperties(void) {
    // sample time
    property_sample_time_in_s_ =
        GetPropertyFloat(ADTF_PROPERTY_CSV_SAMPLE_TIME_NAME);
    LOG_DUMP(A_UTILS_NS::cString::Format("csv writer sample time in s: ",
                                         property_sample_time_in_s_));
    // csv output file
    const char* file_name = GetPropertyStr(ADTF_PROPERTY_CSV_FILE_NAME);

    std::ostringstream ostr_first;
    ostr_first << CSV_FILE_PATH_DEFAULT << file_name;
    property_output_file_name = ostr_first.str();

    boost::filesystem::path dir(CSV_FILE_PATH_DEFAULT);
    if (!(boost::filesystem::exists(dir))) {
        try {
            boost::filesystem::create_directory(dir);
        } catch (...) {
            LOG_ERROR(
                "DyamicCSVWriterFilter can't creat dir, "
                "create folder recordings in home dir");
        }
    }

    if (boost::filesystem::exists(property_output_file_name)) {
        // file the file exists we change the name by addin a numer to it
        uint32_t max = 50;
        uint32_t i = 0;
        string newname;
        do {
            ++i;
            std::ostringstream ostr;
            ostr << CSV_FILE_PATH_DEFAULT << i << file_name;
            newname = ostr.str();
        } while (boost::filesystem::exists(newname) && i <= max);

        property_output_file_name = newname;
        LOG_INFO(A_UTILS_NS::cString::Format(
            "file newname nex: %s", property_output_file_name.c_str()));
    }
}

// ____________________________________________________________________________
tResult DyamicCSVWriterFilter::UpdateProperties(
    ucom::IException** __exception_ptr) {
    RETURN_IF_FAILED(cFilter::UpdateProperties(__exception_ptr));
    GetAllProperties();
    RETURN_NOERROR;
}
