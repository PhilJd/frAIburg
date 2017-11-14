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

Author: spiesra,   $Date:: 2017-05-16 10:06:17#$ $Rev:: 63289   $
Author: Phil
**********************************************************************/

#include "realsense_camera.h"
#include <iostream>  // std::cout

const cString RealsenseCamera::PropEnableDebugName = "Enable Debug Logging";
const tBool RealsenseCamera::PropEnableDebugDefault = false;

const cString RealsenseCamera::PropFPSDepthName =
    "DepthStream::Depth Stream FPS";
const cString RealsenseCamera::PropFPSColorName =
    "ColorStream::Color Stream FPS";
const int RealsenseCamera::PropFPSDefault = 30;

const cString RealsenseCamera::PropResolutionColorName =
    "ColorStream::Color Stream Resolution";
const cString RealsenseCamera::PropResolutionDefault = "640x480";

const cString RealsenseCamera::PropEnableColorName =
    "ColorStream::Enable Color Stream";
const cString RealsenseCamera::PropEnableDepthName =
    "DepthStream::Enable Depth Stream";
const tBool RealsenseCamera::PropEnableStreamDefault = true;

/// Create filter shell
ADTF_FILTER_PLUGIN("AADC Realsense Camera", OID_ADTF_REALSENSE_FILTER,
                   RealsenseCamera);

// ____________________________________________________________________________
RealsenseCamera::RealsenseCamera(const tChar* __info)
    : cFilter(__info), camera_found_(tFalse) {
    // Setting Property for Debug Output
    SetPropertyBool(PropEnableDebugName, PropEnableDebugDefault);
    // Setting properties for frame rates of Depth and Color Stream
    SetPropertyInt(PropFPSDepthName, PropFPSDefault);
    SetPropertyStr(PropFPSDepthName + NSSUBPROP_VALUELIST, "30@30|60@60");
    SetPropertyInt(PropFPSColorName, PropFPSDefault);
    SetPropertyStr(PropFPSColorName + NSSUBPROP_VALUELIST, "30@30|60@60");
    // Setting Properties for Resolution of Depth and Color Stream
    SetPropertyStr(PropResolutionColorName, PropResolutionDefault);
    SetPropertyStr(PropResolutionColorName + NSSUBPROP_VALUELIST,
                   "1920x1080@1920x1080|640x480@640x480|320x240@320x240");
    // Setting Properties for Enabling Depth, Color
    SetPropertyBool(PropEnableColorName, PropEnableStreamDefault);
    SetPropertyBool(PropEnableDepthName, PropEnableStreamDefault);

    // Checking if device is connected
    try {
        // member variable to save the Camera in
        context_ = new rs::context();
        device_ = context_->get_device(0);
        // Automatically Creating Properties from CameraProperties of
        // Librealsense
        for (int j = 0; j < NUMBER_AVAILABLE_PROPERTIES; j++) {
            // Checking if Camera Supports Current Property
            if (device_->supports_option(static_cast<rs::option>(j))) {
                CreateProperty(j);
            }
        }
    } catch (const rs::error& e) {
        LOG_WARNING("No Realsense device found. Plugin will not be available");
    } catch (const std::exception& e) {
        LOG_WARNING("No Realsense device found. Plugin will not be available");
    }
}

// ____________________________________________________________________________
RealsenseCamera::~RealsenseCamera() {}

// ____________________________________________________________________________
tResult RealsenseCamera::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    if (eStage == StageFirst) {
        slim::register_pin_func func = &RealsenseCamera::RegisterPin;
        RETURN_IF_FAILED(video_out_rgb_.StageFirst(this, func, "outputRGB"));
        RETURN_IF_FAILED(
            video_out_depthvis_.StageFirst(this, func, "outputDepthVis"));
        output_depth_raw_.Create(
            "outputDepthRaw",
            new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA,
                                 MEDIA_SUBTYPE_STRUCT_STRUCTURED),
            static_cast<IPinEventSink*>(this));
        RegisterPin(&output_depth_raw_);
    } else if (eStage == StageNormal) {
        m_filterProperties.enableDebugOutput =
            GetPropertyBool(PropEnableDebugName);
        // Sets Member Variables with the Values of the Properties
        // Framerate of Depth and Color Stream
        m_filterProperties.DepthFPS = GetPropertyInt(PropFPSDepthName);
        // m_filterProperties.ColorFPS = GetPropertyInt(PropFPSColorName);
        // Resolution of Depth and Color Stream
        m_filterProperties.ColorResolution =
            GetPropertyStr(PropResolutionColorName);
        // enable depth and color stream
        // m_filterProperties.Depth = GetPropertyBool(PropEnableDepthName);
        // m_filterProperties.Color = GetPropertyBool(PropEnableColorName);
        try {
            if (context_->get_device_count() == 0) {
                THROW_ERROR_DESC(ERR_NOT_CONNECTED,
                                 "Probably no Realsense"
                                 "camera connected. Connect camera and restart"
                                 "ADTF");
            } else {
                device_ = context_->get_device(0);
                camera_found_ = tTrue;
            }
            // Sets the Supported Options of the Camera with the Values of the
            // Properties
            for (int i = 0; i < NUMBER_AVAILABLE_PROPERTIES; i++) {
                if (device_->supports_option(static_cast<rs::option>(i))) {
                    device_->set_option(
                        static_cast<rs::option>(i),
                        static_cast<double>(GetPropertyFloat(OptionName(i))));
                }
            }
        } catch (const rs::error& e) {
            std::stringstream error_desc;
            error_desc << "Probably no Realsense camera connected: "
                       << e.get_failed_function() << "(" << e.get_failed_args()
                       << "):\n " << e.what();
            THROW_ERROR_DESC(ERR_NOT_CONNECTED, error_desc.str().c_str());
            camera_found_ = tFalse;
        } catch (const std::exception& e) {
            std::stringstream error_desc;
            error_desc << "Probably no Realsense camera connected:" << e.what();
            THROW_ERROR_DESC(ERR_NOT_CONNECTED, error_desc.str().c_str());
            camera_found_ = tFalse;
        }
        if (camera_found_) {
            PrintDeviceInfo();
            // Bitmat Format for the RBG Output VideoPin
            m_BitmapFormatRGBOut.nWidth =
                WidthOutOfResolution(m_filterProperties.ColorResolution);
            m_BitmapFormatRGBOut.nHeight =
                HeightOutOfResolution(m_filterProperties.ColorResolution);
            m_BitmapFormatRGBOut.nBitsPerPixel = 24;
            m_BitmapFormatRGBOut.nPixelFormat = IImage::PF_RGB_888;
            m_BitmapFormatRGBOut.nBytesPerLine =
                m_BitmapFormatRGBOut.nWidth *
                m_BitmapFormatRGBOut.nBitsPerPixel / 8;
            m_BitmapFormatRGBOut.nSize = m_BitmapFormatRGBOut.nBytesPerLine *
                                         m_BitmapFormatRGBOut.nHeight;
            m_BitmapFormatRGBOut.nPaletteSize = 0;
            // Bitmap Format for the Visualized Depth Output VideoPin
            m_BitmapFormatDepthOut.nWidth = 320;
            m_BitmapFormatDepthOut.nHeight = 240;
            m_BitmapFormatDepthOut.nBitsPerPixel = 16;
            m_BitmapFormatDepthOut.nPixelFormat = IImage::PF_GREYSCALE_16;
            m_BitmapFormatDepthOut.nBytesPerLine =
                m_BitmapFormatDepthOut.nWidth *
                m_BitmapFormatDepthOut.nBitsPerPixel / 8;
            m_BitmapFormatDepthOut.nSize =
                m_BitmapFormatDepthOut.nBytesPerLine *
                m_BitmapFormatDepthOut.nHeight;
            m_BitmapFormatDepthOut.nPaletteSize = 0;

            // Setting the Bitmap Formats to the fitting Pins
            video_out_rgb_.SetFormat(m_BitmapFormatRGBOut);
            video_out_depthvis_.SetFormat(m_BitmapFormatDepthOut);
        }
    } else if (eStage == StageGraphReady) {
        StartStreams();
        visualize_depth_ = video_out_depthvis_.IsConnected();
        m_Thread.Create(cKernelThread::TF_Suspended,
                        static_cast<IKernelThreadFunc*>(this), NULL, 0);
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult RealsenseCamera::Start(__exception) {
    if (!camera_found_) {
        return cFilter::Start(__exception_ptr);
    }
    // only start thread if there is a camera.
    device_->start();  // Start the camera
    // starting the Thread
    if (m_Thread.GetState() != cKernelThread::TS_Running) {
        m_Thread.Run();
    }
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult RealsenseCamera::Stop(__exception) {
    // suspend the thread
    if (m_Thread.GetState() == cKernelThread::TS_Running) {
        m_Thread.Suspend(tTrue);
    }
    if (camera_found_) {
        device_->stop();
    }
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult RealsenseCamera::Shutdown(tInitStage eStage, __exception) {
    if (eStage == StageNormal) {
        m_Thread.Terminate(tTrue);
        m_Thread.Release();
    }
    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ____________________________________________________________________________
tResult RealsenseCamera::ThreadFunc(adtf::cKernelThread* Thread, tVoid* data,
                                    tSize size) {
    device_->wait_for_frames();
    // transmit RGB stream
    if (device_->is_stream_enabled(rs::stream::color)) {
        video_out_rgb_.Transmit(device_->get_frame_data(rs::stream::color),
                                _clock->GetStreamTime());
    }
    // transmit depth stream
    if (device_->is_stream_enabled(rs::stream::depth)) {
        // Raw Depth Data
        TransmitDepthRaw(device_->get_frame_data(rs::stream::depth));
        if (visualize_depth_) {
            // Calling Convert Function for Visualization
            // std::vector<uchar> image;
            // image.resize(m_BitmapFormatDepthOut.nSize);
            // memcpy(image.data(), device_->get_frame_data(rs::stream::depth),
            //        m_BitmapFormatDepthOut.nSize);
            ConvertDepthAndTransmit(device_->get_frame_data(rs::stream::depth));
        }
    }
    RETURN_NOERROR;
}

// Automatically creates properties from the camera options. The option number
// is connected to the enum of camera options provided by librealsense.
// ____________________________________________________________________________
void RealsenseCamera::CreateProperty(int OptionNumber) {
    // Save the min, max, step, and default values of the option and put
    // them into the description of the property
    double min, max, step, def;
    cString name = OptionName(OptionNumber);
    rs::option option = static_cast<rs::option>(OptionNumber);
    std::stringstream desc;
    desc << device_->get_option_description(option);
    device_->get_option_range(option, min, max, step, def);
    desc << ", Min: " << min << ", Max: " << max << ", Step: " << step;
    // Setting the Property and its description
    SetPropertyFloat(name, def);
    SetPropertyStr(name + NSSUBPROP_DESCRIPTION, desc.str().c_str());
}

// ____________________________________________________________________________
tResult RealsenseCamera::ConvertDepthAndTransmit(const void* data) {
    cv::Mat img = cv::Mat(cv::Size(320, 240), CV_16U, const_cast<void*>(data),
                          m_BitmapFormatDepthOut.nBytesPerLine);
    double min, max;
    cv::minMaxLoc(img, &min, &max);
    img.convertTo(img, CV_16U, 65535 / (max - min));
    video_out_depthvis_.Transmit(img.data, _clock->GetStreamTime());
    RETURN_NOERROR;
}

// Transmit Function for Raw Depth Data
// ____________________________________________________________________________
tResult RealsenseCamera::TransmitDepthRaw(const void* data) {
    // creating new media sample
    cObjectPtr<IMediaSample> mediasample;
    RETURN_IF_FAILED(AllocMediaSample(reinterpret_cast<tVoid**>(&mediasample)));
    int num_bytes = 320 * 240 * 2;
    RETURN_IF_FAILED(mediasample->AllocBuffer(num_bytes));
    // updating media sample
    RETURN_IF_FAILED(mediasample->Update(_clock->GetStreamTime(), data,
                                         num_bytes, IMediaSample::MSF_None));
    RETURN_IF_FAILED(output_depth_raw_.Transmit(mediasample));
    RETURN_NOERROR;
}

// Returns int Height from cString Resolution
// ____________________________________________________________________________
int RealsenseCamera::HeightOutOfResolution(cString Resolution) {
    int Height = 0;
    if (Resolution == "1920x1080") {
        Height = 1080;
    } else if (Resolution == "1280x720") {
        Height = 720;
    } else if (Resolution == "640x480") {
        Height = 480;
    } else if (Resolution == "320x240") {
        Height = 240;
    }
    return Height;
}

// Returns int Width from cString Resolution
// ____________________________________________________________________________
int RealsenseCamera::WidthOutOfResolution(cString Resolution) {
    int Width = 0;
    if (Resolution == "1920x1080") {
        Width = 1920;
    } else if (Resolution == "1280x720") {
        Width = 1280;
    } else if (Resolution == "640x480") {
        Width = 640;
    } else if (Resolution == "320x240") {
        Width = 320;
    }
    return Width;
}

// ____________________________________________________________________________
void RealsenseCamera::StartStreams() {
    std::stringstream loginfo;
    if (GetPropertyBool(PropEnableDepthName)) {
        device_->enable_stream(rs::stream::depth, 320, 240, rs::format::z16,
                               GetPropertyInt(PropFPSDepthName));
        rs_apply_depth_control_preset(reinterpret_cast<rs_device*>(device_), 0);
    }
    if (GetPropertyBool(PropEnableColorName)) {
        device_->enable_stream(
            rs::stream::color,
            WidthOutOfResolution(m_filterProperties.ColorResolution),
            HeightOutOfResolution(m_filterProperties.ColorResolution),
            rs::format::bgr8, GetPropertyInt(PropFPSColorName));
    }
    // Setting Emitter Enabled for better Depth Stream in near Range
    device_->set_option(rs::option::r200_emitter_enabled, 1);
}

// ____________________________________________________________________________
void RealsenseCamera::PrintDeviceInfo() {
    for (int i = 0; i < context_->get_device_count(); i++) {
        rs::device* dev = context_->get_device(i);
        std::stringstream deviceDescription;
        deviceDescription << "Realsense Device: " << i << " - "
                          << dev->get_name() << "\n";
        deviceDescription << "Serial number: " << dev->get_serial() << "\n";
        deviceDescription << "Firmware Version: "
                          << dev->get_firmware_version();
        if (!m_filterProperties.enableDebugOutput) {
            LOG_INFO(deviceDescription.str().c_str());
            return;
        }
        // also print the available properties in debug mode
        deviceDescription << "\n";
        for (int j = 0; j < NUMBER_AVAILABLE_PROPERTIES; j++) {
            rs::option option = static_cast<rs::option>(j);
            if (!dev->supports_option(option)) {
                continue;
            }
            double min, max, step, def;
            deviceDescription << dev->get_option_description(option);
            deviceDescription << ", ";
            dev->get_option_range(option, min, max, step, def);
            deviceDescription << "min: " << min << ", max:" << max
                              << ", step:" << step << ", def:" << def << "\n";
        }
        LOG_INFO(deviceDescription.str().c_str());
    }
}
