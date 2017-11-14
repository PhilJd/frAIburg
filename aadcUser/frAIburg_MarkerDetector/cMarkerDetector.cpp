/**
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
* $Author:: kuckal  $  $Date:: 2017-05-22 10:14:17#$ $Rev:: 63667   $
**********************************************************************/

#include "stdafx.h"
#include <boost/thread.hpp>
#include <unistd.h>   // usleep, wait for thread to finish
#include "opencv_tools.h"  // AdtfMediaSampleToCvMat
#include "cMarkerDetector.h"
#include "aadc_roadSign_enums.h"

using namespace cv;

ADTF_FILTER_PLUGIN("frAiburg Marker Detector Plugin", OID_FRAIBURG_MARKERDETECTFILTER,
                   cMarkerDetector)

cMarkerDetector::cMarkerDetector(const tChar* __info) : cFilter(__info) {
    SetPropertyBool("Debug Output to Console", tFalse);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION,
                   "If enabled additional debug information is printed to the "
                   "console (Warning: decreases performance).");

    SetPropertyStr("Calibration File for used Camera", "");
    SetPropertyBool("Calibration File for used Camera" NSSUBPROP_FILENAME,
                    tTrue);
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_FILENAME
                       NSSUBSUBPROP_EXTENSIONFILTER,
                   "YML Files (*.yml)");
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_DESCRIPTION,
                   "Here you have to set the file with calibration paraemters "
                   "of the used camera");

    SetPropertyStr("Detector Paramater File", "");
    SetPropertyBool("Detector Paramater File" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Detector Paramater File" NSSUBPROP_FILENAME
                       NSSUBSUBPROP_EXTENSIONFILTER,
                   "YML Files (*.yml)");
    SetPropertyStr("Detector Paramater File" NSSUBPROP_DESCRIPTION,
                   "Here you have to set the file with the parameters with the "
                   "detector params");

    SetPropertyFloat("Size of Markers", 0.117f);
    SetPropertyStr("Size of Markers" NSSUBPROP_DESCRIPTION,
                   "Size (length of one side) of markers in m");

    SetPropertyFloat("FPS max", 10.0);
    SetPropertyStr("FPS max" NSSUBPROP_DESCRIPTION, "Max. fps");

    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::Start",
                              m_oProcessStart);
    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::End",
                              m_oProcessEnd);
    UCOM_REGISTER_TIMING_SPOT(
        cString(OIGetInstanceName()) + "::MarkerDetection::Start",
        m_oPreMarkerDetect);
    UCOM_REGISTER_TIMING_SPOT(
        cString(OIGetInstanceName()) + "::MarkerDetection::End",
        m_oPostMarkerDetect);
    UCOM_REGISTER_TIMING_SPOT(
        cString(OIGetInstanceName()) + "::PoseEstimation::Start",
        m_oPrePoseEstimation);
    UCOM_REGISTER_TIMING_SPOT(
        cString(OIGetInstanceName()) + "::PoseEstimation::End",
        m_oPostPoseEstimation);

    thread_is_active_ = false;
    timestamp_last_update_ = 0;
}

cMarkerDetector::~cMarkerDetector() {}

tResult cMarkerDetector::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst) {
        // create the video rgb input pin
        RETURN_IF_FAILED(
            m_oPinInputVideo.Create("Video_RGB_input", IPin::PD_Input,
                                    static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

        // create the video rgb output pin
        RETURN_IF_FAILED(
            m_oPinOutputVideo.Create("Video_RGB_output", IPin::PD_Output,
                                     static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));

        // create the description manager
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid**)&pDescManager,
                                             __exception_ptr));

        // create the description for the road sign pin
        tChar const* strDesc = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strDesc);
        cObjectPtr<IMediaType> pType =
            new cMediaType(0, 0, 0, "tRoadSign", strDesc,
                           IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // create the road sign OutputPin
        RETURN_IF_FAILED(m_oPinRoadSign.Create("RoadSign", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSign));
        // set the description for the road sign pin
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
                                             (tVoid**)&m_pDescriptionRoadSign));

        // create the description for the road sign pin
        tChar const* strDescExt =
            pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strDescExt);
        cObjectPtr<IMediaType> pTypeExt =
            new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt,
                           IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // create the extended road sign OutputPin
        RETURN_IF_FAILED(
            m_oPinRoadSignExt.Create("RoadSign_ext", pTypeExt, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSignExt));
        // set the description for the extended road sign pin
        RETURN_IF_FAILED(
            pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
                                   (tVoid**)&m_pDescriptionRoadSignExt));

        //*** create and register the output pin 1***//
        cObjectPtr<IMediaType> pTypeOut_GCL;
        RETURN_IF_FAILED(AllocMediaType(
            (tVoid**)&pTypeOut_GCL, MEDIA_TYPE_COMMAND,
            MEDIA_SUBTYPE_COMMAND_GCL, NULL, NULL, __exception_ptr));
        RETURN_IF_FAILED(m_outputPinGCL.Create(
            "GCL_Markers", pTypeOut_GCL, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_outputPinGCL));

    } else if (eStage == StageNormal) {
        // get the propeerties
        m_f32MarkerSize =
            static_cast<tFloat32>(GetPropertyFloat("Size of Markers"));
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
        m_bCamaraParamsLoaded = tFalse;

        // Get path of detector parameter file
        cFilename fileDetectorParameter =
            GetPropertyStr("Detector Paramater File");
        if (fileDetectorParameter.IsEmpty()) {
            THROW_ERROR_DESC(ERR_INVALID_FILE,
                             "Detector Parameter File for Markers not set");
        }
        // create absolute path for marker configuration file
        ADTF_GET_CONFIG_FILENAME(fileDetectorParameter);
        fileDetectorParameter = fileDetectorParameter.CreateAbsolutePath(".");
        // check if marker configuration file exits
        if (!(cFileSystem::Exists(fileDetectorParameter))) {
            THROW_ERROR_DESC(ERR_INVALID_FILE,
                             "Detector Parameter file for Markers not found");
        }
        // create the detector params
        m_detectorParams = aruco::DetectorParameters::create();
        if (!(readDetectorParameters(fileDetectorParameter.GetPtr(),
                                     m_detectorParams))) {
            THROW_ERROR_DESC(ERR_INVALID_FILE,
                             "Detector Parameter file not valid");
        }

        m_Dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

        // Get path of calibration file with camera paramters
        cFilename fileCalibration =
            GetPropertyStr("Calibration File for used Camera");

        if (fileCalibration.IsEmpty()) {
            THROW_ERROR_DESC(ERR_INVALID_FILE,
                             "Calibration File for camera not set");
        }

        // Get path of calibration file with camera paramters
        ADTF_GET_CONFIG_FILENAME(fileCalibration);
        fileCalibration = fileCalibration.CreateAbsolutePath(".");
        // check if calibration file with camera paramters exits
        if (!(cFileSystem::Exists(fileCalibration))) {
            THROW_ERROR_DESC(ERR_INVALID_FILE,
                             "Calibration File for camera not found");
        }

        // read the calibration file with camera paramters exits and save to
        // member variable
        readCameraParameters(fileCalibration.GetPtr(), m_Intrinsics,
                             m_Distorsion);
        m_bCamaraParamsLoaded = tTrue;

        max_fps_ = GetPropertyFloat("FPS max");

    } else if (eStage == StageGraphReady) {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO,
                                             (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());

        // set the image format of the output video pin
        UpdateOutputImageFormat(pTypeVideo->GetFormat());

        // IDs were not set yet
        m_bIDsRoadSignExtSet = tFalse;
        m_bIDsRoadSignSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult cMarkerDetector::Shutdown(tInitStage eStage, __exception) {
    while (thread_is_active_) {
        usleep(100000);
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cMarkerDetector::OnPinEvent(IPin* pSource, tInt nEventCode,
                                    tInt nParam1, tInt nParam2,
                                    IMediaSample* pMediaSample) {
    switch (nEventCode) {
        case IPinEventSink::PE_MediaSampleReceived:
            // a new image was received so the processing is started
            if (pSource == &m_oPinInputVideo) {
                float span = (_clock->GetStreamTime() - timestamp_last_update_);
                float fps_span = ((1.0 / max_fps_) * 1e6);
                if (thread_is_active_ | (span < fps_span)) {
                    RETURN_NOERROR;
                }
                timestamp_last_update_ = _clock->GetStreamTime();
                UCOM_TIMING_SPOT(m_oProcessStart);
                cv::Mat img = slim::cvtools::AdtfMediaSampleToCvMat(pMediaSample, m_sInputFormat);
                thread_is_active_ = true;
                boost::thread(&cMarkerDetector::ProcessVideo, this,
                              img, pMediaSample->GetTime());
                UCOM_TIMING_SPOT(m_oProcessEnd);
            }
            break;
        case IPinEventSink::PE_MediaTypeChanged:
            if (pSource == &m_oPinInputVideo) {
                // the input format was changed, so the imageformat has to
                // changed in this filter also
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO,
                                                     (tVoid**)&pTypeVideo));

                UpdateInputImageFormat(m_oPinInputVideo.GetFormat());
                UpdateOutputImageFormat(m_oPinInputVideo.GetFormat());
            }
            break;
        default:
            break;
    }
    RETURN_NOERROR;
}

tResult cMarkerDetector::ProcessVideo(cv::Mat img, tTimeStamp timestamp) {
    // the results from aruco detection
    vector<int> ids;
    vector<vector<Point2f> > corners, rejected;
    vector<Vec3d> rvecs, tvecs;
    // receiving data from input sample, and saving to inputImage
    UCOM_TIMING_SPOT(m_oPreMarkerDetect);
    // cv::Rect roi(300, 300, 400, 400);
    // cv::Mat roiImage(inputImage, roi);

    // doing the detection of markers in image
    aruco::detectMarkers(img, m_Dictionary, corners, ids,
                         m_detectorParams, rejected);
    // aruco::detectMarkers(roiImage, m_Dictionary, corners, ids,
    // m_detectorParams, rejected);

    UCOM_TIMING_SPOT(m_oPostMarkerDetect);
    UCOM_TIMING_SPOT(m_oPrePoseEstimation);

    // if we have the camera pararmeter available we calculate the pose
    if (m_bCamaraParamsLoaded && ids.size() > 0) {
        aruco::estimatePoseSingleMarkers(corners, m_f32MarkerSize, m_Intrinsics,
                                         m_Distorsion, rvecs, tvecs);
    }

    UCOM_TIMING_SPOT(m_oPostPoseEstimation);

    transmitGCL(ids, corners);

    if (m_oPinOutputVideo.IsConnected()) {
        // draw the marker on the image
        aruco::drawDetectedMarkers(img, corners, ids);
        if (m_bCamaraParamsLoaded && ids.size() > 0) {
            for (unsigned int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(img, m_Intrinsics, m_Distorsion,
                                rvecs[i], tvecs[i], m_f32MarkerSize * 0.5f);
            }
        }
        // creating new media sample for output
        cObjectPtr<IMediaSample> pNewSample;
        if (IS_FAILED(AllocMediaSample((tVoid**)&pNewSample))) {
            thread_is_active_ = false;
            RETURN_NOERROR;
        }
        pNewSample->Update(timestamp, img.data,
                           m_sOutputFormat.nSize, 0);
        m_oPinOutputVideo.Transmit(pNewSample);
    }

    // print marker info and draw the markers in image
    for (unsigned int i = 0; i < ids.size(); i++) {
        // call the function to transmit a road sign sample with the detected
        // marker
        // call the function to transmit a extended road sign sample with the
        // detected marker if the Tvec in the marker was correctly set
        if (m_bCamaraParamsLoaded) {
            sendRoadSignStructExt(static_cast<tInt16>(ids[i]),
                                  getMarkerArea(corners[i]),
                                  timestamp, tvecs[i], rvecs[i]);
        } else {
            sendRoadSignStruct(static_cast<tInt16>(ids[i]),
                               getMarkerArea(corners[i]), timestamp);
        }
    }
    thread_is_active_ = false;
    RETURN_NOERROR;
}

tResult cMarkerDetector::UpdateInputImageFormat(const tBitmapFormat* pFormat) {
    if (pFormat != NULL) {
        m_sInputFormat = (*pFormat);

        // LOG_INFO(adtf_util::cString::Format(
        //     "Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , "
        //     "PixelFormat; %d",
        //     m_sInputFormat.nWidth, m_sInputFormat.nHeight,
        //     m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize,
        //     m_sInputFormat.nPixelFormat));
    }

    RETURN_NOERROR;
}

tResult cMarkerDetector::UpdateOutputImageFormat(const tBitmapFormat* pFormat) {
    if (pFormat != NULL) {
        m_sOutputFormat = (*pFormat);

        // LOG_INFO(adtf_util::cString::Format(
        //     "Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, "
        //     "PixelFormat; %d",
        //     m_sOutputFormat.nWidth, m_sOutputFormat.nHeight,
        //     m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize,
        //     m_sOutputFormat.nPixelFormat));

        m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);
    }

    RETURN_NOERROR;
}

tResult cMarkerDetector::sendRoadSignStruct(const tInt16& i16ID,
                                            const tFloat32& f32MarkerSize,
                                            const tTimeStamp& timeOfFrame) {
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionRoadSign->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        // focus for sample write lock
        // write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSign,
                                                  pMediaSample, pCoder);

        // get IDs
        if (!m_bIDsRoadSignSet) {
            pCoder->GetID("i16Identifier", m_szIDRoadSignI16Identifier);
            pCoder->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
            m_bIDsRoadSignSet = tTrue;
        }

        pCoder->Set(m_szIDRoadSignI16Identifier, (tVoid*)&i16ID);
        pCoder->Set(m_szIDRoadSignF32Imagesize, (tVoid*)&f32MarkerSize);

        pMediaSample->SetTime(timeOfFrame);
    }

    // doing the transmit
    RETURN_IF_FAILED(m_oPinRoadSign.Transmit(pMediaSample));

    // print debug info if activated
    if (m_bDebugModeEnabled)
        LOG_INFO(cString::Format("Sign ID %d detected. Area is: %f", i16ID,
                                 f32MarkerSize));

    RETURN_NOERROR;
}

tResult cMarkerDetector::sendRoadSignStructExt(const tInt16& i16ID,
                                               const tFloat32& f32MarkerSize,
                                               const tTimeStamp& timeOfFrame,
                                               const Vec3d& Tvec,
                                               const Vec3d& Rvec) {
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionRoadSignExt->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        // focus for sample write lock
        // write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSignExt,
                                                  pMediaSample, pCoder);

        // get IDs
        if (!m_bIDsRoadSignExtSet) {
            pCoder->GetID("i16Identifier", m_szIDRoadSignExtI16Identifier);
            pCoder->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoder->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoder->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            m_bIDsRoadSignExtSet = tTrue;
        }

        pCoder->Set(m_szIDRoadSignExtI16Identifier, (tVoid*)&i16ID);
        pCoder->Set(m_szIDRoadSignExtF32Imagesize, (tVoid*)&f32MarkerSize);
        // convert from cv::Vec3D to array
        tFloat32 rvecFl32array[3] = {tFloat32(Rvec[0]), tFloat32(Rvec[1]),
                                     tFloat32(Rvec[2])};
        tFloat32 tvecFl32array[3] = {tFloat32(Tvec[0]), tFloat32(Tvec[1]),
                                     tFloat32(Tvec[2])};
        pCoder->Set("af32TVec", (tVoid*)&tvecFl32array[0]);
        pCoder->Set("af32RVec", (tVoid*)&rvecFl32array[0]);

        pMediaSample->SetTime(timeOfFrame);
    }
    // doing the transmit
    RETURN_IF_FAILED(m_oPinRoadSignExt.Transmit(pMediaSample));

    // print debug info if activated
    if (m_bDebugModeEnabled)
        LOG_INFO(
            cString::Format("Sign ID %d detected, translation is: %f, %f, %f",
                            i16ID, Tvec[0], Tvec[1], Tvec[2]));
    RETURN_NOERROR;
}

tResult cMarkerDetector::transmitGCL(const vector<int>& ids,
                                     const vector<vector<Point2f> >& corners) {
    IDynamicMemoryBlock* pGCLCmdDebugInfo;

    if ((ids.size() != corners.size()) || ids.size() == 0 ||
        corners.size() == 0) {
        cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);
        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_CLEAR);
        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);
    } else {
        cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);

        // set color
        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
                                 cColor::Red.GetRGBA());

        // iterate through all corners
        vector<int>::const_iterator itIds = ids.begin();
        for (vector<vector<Point2f> >::const_iterator it = corners.begin();
             it != corners.end(); it++, itIds++) {
            // add the ID as text to middle of marker
            tInt centerMarkerX = 0;
            tInt centerMarkerY = 0;
            for (int p = 0; p < 4; p++) {
                centerMarkerX += tInt(it->at(p).x);
                centerMarkerY += tInt(it->at(p).y);
            }
            centerMarkerX = centerMarkerX / 4;
            centerMarkerY = centerMarkerY / 4;
            cString idTest = cString::FromInt(*itIds);
            cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_TEXT_SIZE_HUGE);
            cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_TEXT,
                                     centerMarkerX, centerMarkerY,
                                     idTest.GetLength());
            cGCLWriter::StoreData(pGCLCmdDebugInfo, idTest.GetLength(),
                                  idTest.GetPtr());

            // draw marker sides as lines
            for (int j = 0; j < 4; j++) {
                Point2i p0, p1;
                p0 = it->at(j);
                p1 = it->at((j + 1) % 4);
                cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE,
                                         p0.x, p0.y, p1.x, p1.y);
            }
            //// draw first corner as big circle to check rotation
            Point2i pFirst;
            pFirst = it->at(0);
            cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE,
                                     pFirst.x, pFirst.y, 10, 10);
        }

        cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);
    }

    // alloc media sample and transmit it over output pin
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));
    RETURN_IF_FAILED(pSample->Update(
        _clock->GetStreamTime(), pGCLCmdDebugInfo->GetPtr(),
        (tInt)pGCLCmdDebugInfo->GetSize(), IMediaSample::MSF_None));
    RETURN_IF_FAILED(m_outputPinGCL.Transmit(pSample));

    cGCLWriter::FreeDynamicMemoryBlock(pGCLCmdDebugInfo);

    RETURN_NOERROR;
}
