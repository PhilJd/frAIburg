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

Author:
**********************************************************************/
#ifndef AADCUSER_FRAIBURG_CAMERASNAPSHOT_CAMERA_SNAPSHOT_H_
#define AADCUSER_FRAIBURG_CAMERASNAPSHOT_CAMERA_SNAPSHOT_H_

#include "stdafx.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ADTF_OpenCV_helper.h"
#include "slim_pins.h"
#include "adtf_tools.h"

// unique filter name
#define OID_ADTF_FILTER_DEF "adtf.frAIburg_camera_snapshot"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "Camera Snapshot Filter"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "CameraSnapshotFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "Filter to save an image from camera. "

// switch clang format off, to retain the markdown table
// clang-format off 
/*! @defgroup CameraSnapshotFilter
*  @{
*    This filter grabs the camera output and saves it as a .png file to disk.
*
* **Dependencies** \n
* This plugin needs the following libraries:
* - OpenCV  v.3.2.0
*
* **Filter Properties**
* | Property | Description | Default |
* | -------- | ----------- | ------- |
* |          |             |         |
*
*
* **Output Pins**
* | Pin   | Description  | MajorType  | SubType  |
* | ---- -| ------------ | ---------- | -------- |
* | None  |              |            |          |
*
*
* **Input Pins**
* | Pin          | Description                    | MajorType        | SubType                          |
* | ------------ | ------------------------------ | -----------------|--------------------------------- |
* | Video_Input  | Video Pin for data from camera | MEDIA_TYPE_VIDEO | MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED |
* | Trigger Input| Input Pin to trigger snapshot  | tBool            |                                  |
*
*
* **Plugin Details**
* | | |
* |-|-|
* | Path    | src/aadcUser/frAIburg_CameraSnapshot |
* | Filename| frAIburg_CameraSnapshot.plb          |
* | Version | 1.0.0                                |
*
*/
// clang-format on

// ! Filter to save current camera image to disk.
/*!
* This class grabs the camera output and saves it as a .png file to disk.
*
*/
class CameraSnapshot : public adtf::cFilter {
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary, ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major, ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build, ADTF_FILTER_VERSION_LABEL);

 protected:
    /*! input for rgb image */
    cVideoPin video_input_pin_;

    /*! input for the image snapshot trigger */
    slim::InputPin trigger_pin_;

 public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    explicit CameraSnapshot(const tChar* __info);

    /*! default destructor */
    virtual ~CameraSnapshot();

    /*! Init stage.
    *    \param [in,out] __exception_ptr n Exception pointer where exceptions
    *        will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] stage The Init function will be called when the filter
    *        state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Shutdown stage.
    *   \param [in,out] __exception_ptr An Exception pointer where exceptions
    *       will be put when failed. If not using the cException smart pointer,
    *       the interface has to be released by calling Unref().
    *   \param  [in] stage The Init function will be called when the filter
    *       state changes as follows:\n
    *   \result Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage stage,
                     ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] source Pointer to the sending pin's IPin interface.
    *   \param [in] event_code Event code. For allowed values see
    *       IPinEventSink::tPievent_code
    *   \param [in] param1 Optional integer parameter.
    *   \param [in] param2 Optional integer parameter.
    *   \param [in] media_sample Address of an IMediaSample interface pointer.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization
    *        between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to
    *        adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
                       IMediaSample* media_sample);

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters stat
    *    IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions
    *        will be put when failed. If not using the cException smart pointer,
    *        the interface has to be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a
    *        configuration if the Filter is connected to the Message Bus.
    *        (see:  section_message_bus)! This has to be done, to disconnect the
    *        Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Stop Stage.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions
    *        will be put when failed. If not using the cException smart pointer,
    *        the interface has to be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a
    *       configuration if the Filter is connected to the Message Bus.
    *       (see: section_message_bus)! This has to be done, to disconnect
    *       the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

 private:
    /*! Saves the camera image.
    *   \param sample the new media sample
    *   \return Standard Result Code.
    */
    tResult SaveImage(IMediaSample* sample);

    /* Sets the storage, path, including checks and adding timestamp */
    tResult ComposeStoragePath();

    /*! Bitmap format of input pin */
    tBitmapFormat input_format_;

    /*! Stores the current snapshot */
    cv::Mat input_image_;

    /*! True after trigger event, is set to false after saving the image */
    bool trigger_pressed_;

    /*! Holds the number of snapshots taken durnig the current session. */
    tInt32 image_counter_;

    cString storage_path_;
};

/** @} */  // end of group

#endif  // AADCUSER_FRAIBURG_CAMERASNAPSHOT_CAMERA_SNAPSHOT_H_
