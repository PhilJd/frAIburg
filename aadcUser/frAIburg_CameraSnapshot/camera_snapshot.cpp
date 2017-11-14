/*****************************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must
   display the following acknowledgement: “This product includes software
   developed by the Audi AG and its contributors for Audi Autonomous Driving
   Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Author: Philipp Jund
**********************************************************************/
#include "stdafx.h"
#include "camera_snapshot.h"
#include <sys/stat.h>
#include <stdexcept>
#include "adtf_tools.h"
#include "log_tools.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv_tools.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, CameraSnapshot)

CameraSnapshot::CameraSnapshot(const tChar* __info) : cFilter(__info) {
    trigger_pressed_ = false;
    image_counter_ = 0;
    // Expose the storage folder property to the ADTF GUI.
    SetPropertyStr("storage_path", "/home/aadc/Pictures/");
    cString info =
        "Path to the parent folder. Here, a subfolder will be"
        "created for all images taken during the activ session";
    SetPropertyStr("storage_path" NSSUBPROP_DESCRIPTION, info);
}

// ____________________________________________________________________________
CameraSnapshot::~CameraSnapshot() {}

// ____________________________________________________________________________
tResult CameraSnapshot::Start(__exception) {
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult CameraSnapshot::Stop(__exception) {
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult CameraSnapshot::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));
    if (stage == StageFirst) {
        // Video input
        RETURN_IF_FAILED(video_input_pin_.Create(
            "Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&video_input_pin_));
        // Trigger Input
        RETURN_IF_FAILED(trigger_pin_.FirstStageCreate(
            this, &CameraSnapshot::RegisterPin, "trigger", "tBoolSignalValue"));
    } else if (stage == StageNormal) {
        ComposeStoragePath();
    } else if (stage == StageGraphReady) {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(video_input_pin_.GetMediaType(&pType));
        cObjectPtr<IMediaTypeVideo> type_video;
        RETURN_IF_FAILED(pType->GetInterface(
            IID_ADTF_MEDIA_TYPE_VIDEO, reinterpret_cast<tVoid**>(&type_video)));
        if (IS_FAILED(slim::cvtools::UpdateCvMatFormat(type_video->GetFormat(),
                                                       input_image_))) {
            LOG_ERROR("Invalid Input Format for this filter");
        }
        input_format_ = *(type_video->GetFormat());
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult CameraSnapshot::Shutdown(tInitStage stage,
                                 ucom::IException** __exception_ptr) {
    if (stage == StageGraphReady) {
    } else if (stage == StageNormal) {
    } else if (stage == StageFirst) {
    }
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult CameraSnapshot::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                   tInt param2, IMediaSample* media_sample) {
    RETURN_IF_POINTER_NULL(media_sample);
    RETURN_IF_POINTER_NULL(source);
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        if (trigger_pin_.isSource(source)) {
            trigger_pressed_ = true;
        } else if (source == &video_input_pin_ && trigger_pressed_) {
            // check if video format is still unknown
            if (input_format_.nPixelFormat == IImage::PF_UNKNOWN) {
                RETURN_IF_FAILED(slim::cvtools::UpdateCvMatFormat(
                    video_input_pin_.GetFormat(), input_image_));
            }
            SaveImage(media_sample);
            image_counter_++;
            trigger_pressed_ = false;
        }
    } else if (event_code == IPinEventSink::PE_MediaTypeChanged) {
        if (source == &video_input_pin_) {
            RETURN_IF_FAILED(slim::cvtools::UpdateCvMatFormat(
                video_input_pin_.GetFormat(), input_image_));
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult CameraSnapshot::SaveImage(IMediaSample* sample) {
    RETURN_IF_POINTER_NULL(sample);
    input_image_ = slim::cvtools::AdtfMediaSampleToCvMat(sample, input_format_);
    if (input_image_.rows == 0 || input_image_.cols == 0) {
        LOG_ERROR("Mat cols and rows are zero. Conversion failed.");
        RETURN_NOERROR;
    }
    cString path =
        storage_path_ + cString::Format("img%d.tiff", image_counter_);
    std::string stdpath(path.GetPtr());
    if (input_image_.type() == CV_8UC3) {
        cv::cvtColor(input_image_, input_image_, CV_RGB2BGR);
    }
    cv::imwrite(stdpath.c_str(), input_image_);
}

// ____________________________________________________________________________
tResult CameraSnapshot::ComposeStoragePath() {
    storage_path_ = GetPropertyStr("storage_path");
    if (!storage_path_.EndsWith("/")) {
        storage_path_ = storage_path_ + "/";
    }
    storage_path_ =
        storage_path_ + LogTools::GetTimeStampString().c_str() + "/";
    // create the directory

    const int error = mkdir(storage_path_.GetPtr(),
                            S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);  // flags
    if (error == -1) {
        LOG_ERROR("Error creating directory!\n");
        RETURN_ERROR(-1);
    }
    RETURN_NOERROR;
}
