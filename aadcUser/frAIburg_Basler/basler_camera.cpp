/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must
display the following acknowledgement: “This product includes software developed
by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/

#include "basler_camera.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, BaslerCamera)

// ____________________________________________________________________________
BaslerCamera::BaslerCamera(const tChar* __info) : cFilter(__info) {
    SetProperties();
}

// ____________________________________________________________________________
BaslerCamera::~BaslerCamera() {
}

// ____________________________________________________________________________
tResult BaslerCamera::Start(__exception) {
    try {
        camera_.StartGrabbing(GrabStrategy_LatestImageOnly);
    } catch (GenICam::GenericException& e) {
        THROW_ERROR_DESC(ERR_FAILED,
                         cString::Format("Camera cannot start grabbing: %s",
                                         e.GetDescription()));
    }
    // starting the Thread
    if (stream_thread_.GetState() != cKernelThread::TS_Running) {
        stream_thread_.Run();
    }
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult BaslerCamera::Stop(__exception) {
    // suspend the thread
    if (stream_thread_.GetState() == cKernelThread::TS_Running) {
        stream_thread_.Suspend(tTrue);
    }
    // stops grabbing
    camera_.StopGrabbing();

    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult BaslerCamera::Shutdown(tInitStage eStage,
                                ucom::IException** __exception_ptr) {
    if (eStage == StageNormal) {
        stream_thread_.Terminate(tTrue);
        stream_thread_.Release();
        if (camera_.IsOpen()) {
            camera_.Close();
            camera_.DetachDevice();
            camera_.DestroyDevice();
        }
        try {
            PylonTerminate();  // terminates the pylon sdk
        } catch (GenICam::GenericException& e) {
            THROW_ERROR_DESC(ERR_FAILED,
                             cString::Format("Pylon not deinitialized: %s",
                                             e.GetDescription()));
        }
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ____________________________________________________________________________
tResult BaslerCamera::Init(tInitStage eStage, __exception) {
    // never miss calling the parent implementation first!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst) {
        slim::register_pin_func func = &BaslerCamera::RegisterPin;

        RETURN_IF_FAILED(video_output_bgr_.StageFirst(this, func, "outputBGR"));
        RETURN_IF_FAILED(video_output_ipm_.StageFirst(this, func, "outputIPM"));

        gcl_commands_.Create(
            "ROI_GCL",
            new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL),
            static_cast<IPinEventSink*>(this));
        RegisterPin(&gcl_commands_);
    }
    if (eStage == StageNormal) {
        GetProperties();
        // Setting Bitmapformat for Output
        format_bgr_out_.nPixelFormat = adtf_util::IImage::PF_BGR_888;
        format_bgr_out_.nBitsPerPixel = 24;
        format_bgr_out_.nWidth = filter_properties_.Width;
        format_bgr_out_.nHeight = filter_properties_.Height;
        format_bgr_out_.nBytesPerLine =
            format_bgr_out_.nWidth * format_bgr_out_.nBitsPerPixel / 8;
        format_bgr_out_.nSize =
            format_bgr_out_.nBytesPerLine * format_bgr_out_.nHeight;
        format_bgr_out_.nPaletteSize = 0;
        video_output_bgr_.SetFormat(format_bgr_out_);
    } else if (eStage == StageGraphReady) {
        // Initializing Pylon5
        // Needs to be called before any Pylon5 functions can be used
        try {
            PylonInitialize();
        } catch (GenICam::GenericException& e) {
            THROW_ERROR_DESC(ERR_NOT_CONNECTED,
                             cString::Format("Pylon not Initialized: %s",
                                             e.GetDescription()));
        }
        SetCameraProperties();
        std::string xml_path =
            GetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME);
        basler_transform_ = CameraTransformations(xml_path,
                                        "camera basler cropped");
        InitUndistortion();
        // Creating Thread to grab Camera Results in
        stream_thread_.Create(cKernelThread::TF_Suspended,
                              static_cast<IKernelThreadFunc*>(this), NULL, 0);
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult BaslerCamera::PropertyChanged(const tChar* strName) {
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    // associate the properties to the member
    if (cString::IsEqual(strName, "ROIBrightness::Show ROI")) {
        filter_properties_.ROIShow = GetPropertyBool("ROIBrightness::Show ROI");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void BaslerCamera::SetProperties() {
    SetPropertyBool("Enable_GPU_undistort", false);
    SetPropertyStr("Enable_GPU_undistort" NSSUBPROP_DESCRIPTION,
                   "If true, undistort is computed on the GPU.");
    SetPropertyBool("Enable_GPU_undistort" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("Stream Width", 1280);
    SetPropertyStr("Stream Width" NSSUBPROP_DESCRIPTION, "Width of the Stream");

    SetPropertyInt("Stream Height", 400);
    SetPropertyStr("Stream Height" NSSUBPROP_DESCRIPTION,
                   "Height of the Stream");

    SetPropertyInt("Stream column offset", 0);
    SetPropertyStr("Stream column offset" NSSUBPROP_DESCRIPTION,
                   "Columns offset of the stream");

    SetPropertyInt("Stream row offset", 380);
    SetPropertyStr("Stream row offset" NSSUBPROP_DESCRIPTION,
                   "Row offset of the stream");

    SetPropertyFloat("FPS max", 30.0);
    SetPropertyStr("FPS max" NSSUBPROP_DESCRIPTION, "Max. fps");

    SetPropertyFloat("Exposure time", 0.005);
    SetPropertyStr("Exposure time" NSSUBPROP_DESCRIPTION,
                   "Exposure time in seconds");

    SetPropertyFloat("Brightness", 0.3);
    SetPropertyStr("Brightness" NSSUBPROP_DESCRIPTION,
                   "Target Brightness for Auto Brightness of Camera");

    SetPropertyInt("ROIBrightness::XOffset", 0);
    SetPropertyStr("ROIBrightness::XOffset" NSSUBPROP_DESCRIPTION,
                   "X Offset for camera brightness adjustment area");

    SetPropertyInt("ROIBrightness::YOffset", 100);
    SetPropertyStr("ROIBrightness::YOffset" NSSUBPROP_DESCRIPTION,
                   "Y Offset for camera brightness adjustment area");

    SetPropertyInt("ROIBrightness::Width", 1280);
    SetPropertyStr("ROIBrightness::Width" NSSUBPROP_DESCRIPTION,
                   "Width of the camera brightness adjustment area");

    SetPropertyInt("ROIBrightness::Height", 300);
    SetPropertyStr("ROIBrightness::Height" NSSUBPROP_DESCRIPTION,
                   "Height of the camera brightness adjustment area");

    SetPropertyBool("ROIBrightness::Show ROI", false);
    SetPropertyStr("ROIBrightness::Show ROI" NSSUBPROP_DESCRIPTION,
                   "Shows ROI as a Rectangle in Video");
    SetPropertyBool("ROIBrightness::Show ROI" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyStr(ADTF_PROPERTY_XML_CONFIG_FILE_NAME,
                   ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT);

    SetPropertyStr("Calibration File", "");
    SetPropertyStr("Calibration File" NSSUBPROP_DESCRIPTION,
                   "Set the undistortion calibration file here");
    SetPropertyBool("Calibration File" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(
        "Calibration File" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER,
        "YML Files (*.yml)");
}

// ____________________________________________________________________________
void BaslerCamera::GetProperties() {
    gpu_undistort_ = GetPropertyBool("Enable_GPU_undistort");
    filter_properties_.Width = GetPropertyInt("Stream Width");
    filter_properties_.Height = GetPropertyInt("Stream Height");
    filter_properties_.OffsetX = GetPropertyInt("Stream column offset");
    filter_properties_.OffsetY = GetPropertyInt("Stream row offset");
    filter_properties_.ROIWidth = GetPropertyInt("ROIBrightness::Width");
    filter_properties_.ROIHeight = GetPropertyInt("ROIBrightness::Height");
    filter_properties_.ROIOffsetX = GetPropertyInt("ROIBrightness::XOffset");
    filter_properties_.ROIOffsetY = GetPropertyInt("ROIBrightness::YOffset");
    filter_properties_.Brightness = GetPropertyFloat("Brightness");
    filter_properties_.ROIShow = GetPropertyBool("ROIBrightness::Show ROI");
    filter_properties_.fps_max_ = GetPropertyFloat("FPS max");
    filter_properties_.exposure_time_ = GetPropertyFloat("Exposure time");

    // Get path of calibration file with camera paramters
    std::string undistort_calib_file = GetPropertyStr("Calibration File");
    ReadUndistortCalibFile(undistort_calib_file);
}

// ____________________________________________________________________________
void BaslerCamera::SetCameraProperties() {
    // Attach the camera to camera instance member
    CDeviceInfo info;  // Only look for cameras supported by Camera_t.
    info.SetDeviceClass(CBaslerUsbInstantCamera::DeviceClass());
    // Create a camera object with the first found camera
    // device that matches the specified device class.
    camera_.Attach(CTlFactory::GetInstance().CreateFirstDevice(info));
    camera_.Open();
    // Pixel Format for camera Output
    camera_.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
    // Setting Camera options from Properties
    camera_.Width.SetValue(filter_properties_.Width);
    camera_.Height.SetValue(filter_properties_.Height);
    camera_.OffsetX.SetValue(filter_properties_.OffsetX);
    camera_.OffsetY.SetValue(filter_properties_.OffsetY);
    // The Autofunction ROI defines the area used for e.g. auto
    // brightness adjustment
    camera_.AutoFunctionROIWidth.SetValue(filter_properties_.ROIWidth);
    camera_.AutoFunctionROIHeight.SetValue(
        filter_properties_.ROIHeight);
    camera_.AutoFunctionROIOffsetX.SetValue(
        filter_properties_.ROIOffsetX);
    camera_.AutoFunctionROIOffsetY.SetValue(
        filter_properties_.ROIOffsetY);
    camera_.AutoTargetBrightness.SetValue(
        filter_properties_.Brightness);
    // Custom FPS
    camera_.AcquisitionMode.SetValue(
        Basler_UsbCameraParams::AcquisitionMode_Continuous);
    camera_.TriggerMode.SetValue(
        Basler_UsbCameraParams::TriggerMode_Off);
    camera_.ExposureTime.SetValue(
        filter_properties_.exposure_time_ *
        1e6);  // in microsec.; 1/50s = 20000musec.
    camera_.AcquisitionFrameRate.SetValue(filter_properties_.fps_max_);
    camera_.AcquisitionStart.Execute();
    // camera_.SensorReadoutMode.SetValue(Basler_UsbCameraParams::SensorReadoutMode_Fast);
    // camera_.BslUSBSpeedMode.SetValue(Basler_UsbCameraParams::BslUSBSpeedMode_LowSpeed);
}

// ____________________________________________________________________________
tResult BaslerCamera::ThreadFunc(adtf::cKernelThread* Thread, tVoid* data,
                                  tSize size) {
    namespace slimcv = slim::cvtools;
    CGrabResultPtr ptrGrabResult;  // PTR for the Result
    if (!camera_.IsOpen() || !camera_.IsGrabbing()) {
        RETURN_NOERROR;
    }
    // Waiting up to 5000ms to get Result
    try {
        camera_.RetrieveResult(1000, ptrGrabResult,
                               TimeoutHandling_ThrowException);
    } catch (GenICam::GenericException& e) {
        LOG_ERROR(cString::Format("An exception occurred.  %s",
                                  e.GetDescription()));
        RETURN_NOERROR;
    }
    tInt32 grab_img_size = tInt32(ptrGrabResult->GetImageSize());
    tInt32 format_size = format_bgr_out_.nSize;
    if (ptrGrabResult->GrabSucceeded() && format_size == grab_img_size) {
        tVoid* camera_buffer = ptrGrabResult->GetBuffer();
        tTimeStamp time_stamp = _clock->GetStreamTime();
        cv::Mat img = slimcv::NoCopyAdtfMediaSampleToCvMat(camera_buffer,
                                                           format_bgr_out_);
        cv::Mat ipm, undistorted;
        Undistort(img, &undistorted);
        if (video_output_bgr_.IsConnected())
            video_output_bgr_.TransmitBGR(undistorted, time_stamp);

        if (video_output_ipm_.IsConnected()) {
            // inverse perspective mapping
            cv::Mat img_gray;
            cv::cvtColor(undistorted, img_gray, CV_BGR2GRAY);
            basler_transform_.PixelToStreetImage(img_gray, &ipm);
            video_output_ipm_.Transmit(ipm, time_stamp);
        }


        if (filter_properties_.ROIShow) {
            transmitGCLROI();
        }
    }

    ptrGrabResult.Release();
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void BaslerCamera::InitUndistortion() {
    // estimate the new, undistorted camera matrix
    cv::Mat newCameraMatrix;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        camera_matrix_, distortion_matrix_,
        cv::Size(format_bgr_out_.nWidth, format_bgr_out_.nHeight),
        cv::Matx33d::eye(), newCameraMatrix, 1.0,
        cv::Size(format_bgr_out_.nWidth, format_bgr_out_.nHeight),
        0.6);  // 0.6 This is the fixed zoom-in value, changing requires new ipm
    cv::fisheye::initUndistortRectifyMap(
        camera_matrix_, distortion_matrix_, cv::Matx33d::eye(),
        newCameraMatrix,
        cv::Size(format_bgr_out_.nWidth, format_bgr_out_.nHeight), CV_16SC2,
        rectify_map1_, rectify_map2_);
    // if a gpu is available and it's active, upload to gpu
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        gpu_available_ = true;
    }
    if (gpu_available_ && gpu_undistort_) {
        cv::Mat mapX_32FC1, mapY_32FC1;
        cv::convertMaps(rectify_map1_, rectify_map2_,
                        mapX_32FC1, mapY_32FC1, CV_32FC1);
        // transfer maps to gpu
        rectify_map1_gpu_.upload(mapX_32FC1);
        rectify_map2_gpu_.upload(mapY_32FC1);
    }
}

// ____________________________________________________________________________
void BaslerCamera::Undistort(const cv::Mat& src, cv::Mat* undistorted) {
    try {
        if (gpu_available_ && gpu_undistort_) {
            // stream used for queueing the instructions on the gpu
            cv::cuda::Stream stream;
            gpu_input_.upload(src, stream);
            cv::cuda::remap(gpu_input_, gpu_output_,
                            rectify_map1_gpu_, rectify_map2_gpu_,
                            cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                            cv::Scalar::all(0), stream);
            gpu_output_.download(*undistorted, stream);
            stream.waitForCompletion();  // wait for the GPU
        } else {
            cv::remap(src, *undistorted, rectify_map1_, rectify_map2_,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                      cv::Scalar::all(0));
        }
    } catch (cv::Exception& e) {
        LOG_ERROR(cString::Format("OpenCV exception: %s", e.what()));
    }
}

// ____________________________________________________________________________
tResult BaslerCamera::transmitGCLROI() {
    tUInt32* aGCLProc;
    tUInt32* GCLCommands;
    // Creating new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(
        AllocMediaSample(reinterpret_cast<tVoid**>(&pMediaSample)));
    // allocating buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(4096));
    // updating media sample
    pMediaSample->SetTime(_clock->GetStreamTime());
    // locking GCL memory
    RETURN_IF_FAILED(pMediaSample->WriteLock((tVoid**)&aGCLProc));
    GCLCommands = aGCLProc;
    // Setting GCL commands to show ROI in video
    cGCLWriter::StoreCommand(GCLCommands, GCL_CMD_CLEAR);
    cGCLWriter::StoreCommand(GCLCommands, GCL_CMD_FGCOL,
                             cColor(255, 0, 0).GetRGBA());
    cGCLWriter::StoreCommand(
        GCLCommands, GCL_CMD_DRAWRECT, filter_properties_.ROIOffsetX,
        filter_properties_.ROIOffsetY,
        filter_properties_.ROIOffsetX + filter_properties_.ROIWidth,
        filter_properties_.ROIOffsetY + filter_properties_.ROIHeight);
    cGCLWriter::StoreCommand(GCLCommands, GCL_CMD_END);
    // unlocking GCL memory
    pMediaSample->Unlock(aGCLProc);
    // transmitting
    RETURN_IF_FAILED(gcl_commands_.Transmit(pMediaSample));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
void BaslerCamera::ReadUndistortCalibFile(const std::string &calib_filename) {
    // check if calibration file with camera paramters exits
    if (calib_filename.empty()) {
        LOG_ERROR_PRINTF("ReadUndistortCalibFile: filepath is empty");
        return;
    }

    // read calibration file with camera params, save to member variables
    cv::FileStorage camera_params(calib_filename, cv::FileStorage::READ);
    if (!camera_params.isOpened()) {
        LOG_ERROR_PRINTF("ReadUndistortCalibFile: file %s not found,"
        " or relativ path, or not a .yml", calib_filename.c_str());
        // If this error occurs, cv::remap will crash
        return;
    }
    camera_params["camera_matrix"] >> camera_matrix_;
    camera_params["distortion_coefficients"] >> distortion_matrix_;
//    std::cout << "camera matrix" << std::endl;
//    std::cout << camera_matrix_ << std::endl << std::endl;
//    std::cout << "distortion matrix" << std::endl;
//    std::cout << distortion_matrix_ << std::endl << std::endl;
}
