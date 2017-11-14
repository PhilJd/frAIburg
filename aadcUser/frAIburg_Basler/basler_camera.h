/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 12:53:59#$ $Rev:: 63132   $
**********************************************************************/

#ifndef AADCUSER_FRAIBURG_BASLER_BASLER_CAMERA_H_
#define AADCUSER_FRAIBURG_BASLER_BASLER_CAMERA_H_



#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <opencv2/opencv.hpp>
#include "stdafx.h"
#include "slim_pins.h"
#include "opencv_tools.h"
#include "xml_helper.hpp"
#include "camera_transformations.h"

using namespace Pylon;


#define OID_ADTF_FILTER_DEF "adtf.frAIburg.baslercamera"
#define ADTF_FILTER_DESC "frAIburg Basler Camera"
#define ADTF_FILTER_VERSION_SUB_NAME "BaslerCamera_filter"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
#define ADTF_FILTER_VERSION_LABEL "A basler camera filter \n$Rev:: 62962"

#define ADTF_PROPERTY_XML_CONFIG_FILE_NAME "frAIburg xml configuration"
#define ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT \
    "/home/aadc/ADTF/configuration_files/frAIburg_configuration.xml"


// necessary because WinGDI.h redefines it
#undef GetObject

/*!
* @defgroup BaslerCamera Basler Camera
* @{
*  Grabber Filter for Basler Camera
*
* This filter grabs images from the Basler Camera. The grabbing parameter for the camera can be set by the filterproperties. The grabbing procedure needs the pylon5 SDK which is provided by Basler.
* \image html BaslerCamera.PNG "Plugin BaslerCamera"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li Pylon5  v.5.0.5
*
* <b> Filter Properties</b>
* The filter has the following properties:
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Stream Width<td>Width of the Stream<td>1280
* <tr><td>Stream Height<td>Height of the Stream<td>960
* <tr><td>Brightness<td>target Brightness for Auto Gain Function<td>0.3
* <tr><td>ROI::XOffset <td>x-Offset of the ROI<td>440
* <tr><td>ROI::YOffset <td>y-Offset of the ROI<td>330
* <tr><td>ROI::Width <td>Width of the ROI<td>400
* <tr><td>ROI::Height <td>Height of the ROI<td>300
* </table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>video_output_rgb_<td>Outputpin for Camera Stream Imagetype: PF_BGR_888<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>gcl_commands_<td>Outputpin for Camera GCL Stream<td>MEDIA_TYPE_COMMAND<td>MEDIA_SUBTYPE_COMMAND_GCL
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/camera/AADC_Basler
* <tr><td>Filename<td>aadc_BaslerCamera.plb
* <tr><td>Version<td>1.0.0
* </table>
*/

//!  Class of Basler Camera Filter
/*!
* This class is the main class of the Basler Camera Filter
*/
class BaslerCamera : public adtf::cFilter, adtf::IKernelThreadFunc {
 public:
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_SensorDevice,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL);

 public:
    /*! constructor for BaslerCamera class
        \param __info   [in] This is the name of the filter instance.
        */
    explicit BaslerCamera(const tChar* __info);

    /*! the destructor for this class
    */
    ~BaslerCamera();

    /*! Init stage. */
    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Shutdown stage. */
    tResult Shutdown(tInitStage stage,
                     ucom::IException** __exception_ptr = NULL);

    /*! Start Stage. */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Stop Stage. */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*! */
    tResult PropertyChanged(const tChar* strName);


 private:
    // OUTPUT PINS
    /*! output pin for undistorted BGR stream */
    slim::VideoOutputPin video_output_bgr_;

    /*! pin for undistorted, inverse perspective mapping grayscale img */
    slim::VideoOutputPin video_output_ipm_;

    /*! Holds the transformation matrix of the Basler camera. */
    CameraTransformations basler_transform_;

    /*! output for GCL Commands*/
    cOutputPin gcl_commands_;

    /*! The bitmap format of the undistorted bgr output. */
    tBitmapFormat format_bgr_out_;

    /*! the struct with all the properties*/
    struct filterProperties {
        /*! Width of the Stream*/
        int Width;
        /*! Height of the Stream*/
        int Height;
        /*! Offset of the Stream*/
        int OffsetX;
        /*! Offset of the Stream*/
        int OffsetY;
        /*! Offset of the ROI in the Stream*/
        int ROIOffsetX;
        /*! Offset of the ROI in the Stream*/
        int ROIOffsetY;
        /*! Width of the ROI*/
        int ROIWidth;
        /*! Height of the ROI*/
        int ROIHeight;
        float fps_max_;
        float exposure_time_;
        /*! Target Brightness for Auto Gain Function*/
        tFloat64 Brightness;
        /*! Shows ROI as a Rectangle in the Video*/
        tBool ROIShow;
    }
    /*! the filter properties of this class */
    filter_properties_;

    /*! Thread Function to grab the Stream of the Camera
     *   \param Thread Thread associated with the funtion
     *   \param data external data for the Thread
     *   \param size of the data
     *   \result Returns a standard result code.
     */
    tResult ThreadFunc(adtf::cKernelThread* Thread, tVoid* data, tSize size);

    /*! Prepares the undistortion matrices. */
    void InitUndistortion();

    /*! Applies cv::remap to undistort the fisheye effect. */
    void Undistort(const cv::Mat& src, cv::Mat* undistorted);

    /*! Camera Instance */
    CBaslerUsbInstantCamera camera_;

    /*! Thread to grab the Stream in*/
    cKernelThread stream_thread_;

    // CAMERA PARAMS
    /*! camera matrix from calibration file */
    cv::Mat camera_matrix_;
    cv::Mat distortion_matrix_;
    cv::Mat rectify_map1_;
    cv::Mat rectify_map2_;
    cv::cuda::GpuMat rectify_map1_gpu_;
    cv::cuda::GpuMat rectify_map2_gpu_;

    /*! GPU matrices to store the incoming images. No in-place operations on
     *  GPU, so we need another one to fetch.  */
    cv::cuda::GpuMat gpu_input_;
    cv::cuda::GpuMat gpu_output_;

    // set to true if it's enabled in the filter properties
    bool gpu_undistort_;

    // true if the system has a gpu
    bool gpu_available_;

    /*! transmits GCL commands to Show ROI
     *    \return Returns a standard result code.
     */
    tResult transmitGCLROI();

    /*! Sets the ADTF properties. */
    void SetProperties();

    /*! Gets the ADTF properties. */
    void GetProperties();

    /*! Sets the basler camera parameters via the pylon interface. */
    void SetCameraProperties();

    /*! Read the camera params from calibration file */
    void ReadUndistortCalibFile(const std::string &calib_filename);
};

/*!
*@}
*/

#endif  // AADCUSER_FRAIBURG_BASLER_BASLER_CAMERA_H_
