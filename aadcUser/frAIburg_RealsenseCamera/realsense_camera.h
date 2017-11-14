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
   display the following acknowledgment: “This product includes software
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

#ifndef AADCUSER_FRAIBURG_REALSENSECAMERA_REALSENSECAMERA_H_
#define AADCUSER_FRAIBURG_REALSENSECAMERA_REALSENSECAMERA_H_

#define OID_ADTF_REALSENSE_FILTER "adtf.aadc.frAIburg_realsense"

#include "stdafx.h"
#include "realsense_helper.h"
#include "librealsense/rs.hpp"
#include "opencv2/highgui.hpp"
#include "slim_pins.h"  // SimpleVideoPin


#define NUMBER_AVAILABLE_PROPERTIES 68
/*!
* @defgroup RealsenseCamera
* @{
* Output filter for Realsense Camera
*
* This filter grabs images from the Intel Realsense Camera. The camera options can be set by the filterproperties. The filter uses LibRealsense provided by Intel. The Camera provides RGB, Infrared and Depth stream.
*
*  \image html RealsenseCamera.PNG "Plugin Realsense Camera"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li Boost   v.1.58.0
* \li OpenCV  v.3.2.0
* \li LibRealsense  v1.11.0
*
* <b> Filter Properties</b>
* The filter has the following properties:
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Enable Debug Logging<td>Logs Debug Information to the ADTF Console<td>false
* <tr><td>Depth Stream FPS<td>Frame rate of the Depth Stream<td>30
* <tr><td>Color Stream FPS<td>Frame rate of the Color Stream<td>30
* <tr><td>Depth Stream Resolution<td>Resolution for the Depth Stream<td>640x480
* <tr><td>Color Stream Resolution<td>Resolution for the Color Stream<td>640x480
* <tr><td>Enable Infrared 1/2 Stream<td>Enables the Infrared Streams<td>true
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
* <tr><td>outputRGB<td>output for RGB Stream of Camera Imagetype: PF_RGB_888<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>outputInfraRed1<td>output for Infrared Stream of Camera Imagetype: PF_GREYSCALE_8<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>outputInfraRed2<td>output for Infrared Stream of Camera Imagetype: PF_GREYSCALE_8<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>outputoutputDepthVisualization<td>output for Visualization of Depth Stream of Camera Imagetype: PF_GREYSCALE_16<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>outputDepthRaw<td>output for Raw Data of Depth Stream of Camera<td>MEDIA_TYPE_STRUCTURED_DATA<td>MEDIA_SUBTYPE_STRUCT_STRUCTURED
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/camera/AADC_Realsense
* <tr><td>Filename<td>aadc_RealsenseCamera.plb
* <tr><td>Version<td>1.0.0
* </table>
*/

//!  Class of Realsense Camera Filter
/*!
* This is the main class of the Realsense Camera Filter
*/
class RealsenseCamera : public adtf::cFilter, adtf::IKernelThreadFunc {
    /*! set the filter id and version etc */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_REALSENSE_FILTER,
                                "frAIburg Realsense Camera",
                                OBJCAT_SensorDevice,
                                "librealsense", 1, 11, 0, "");

protected:
    /*! Output Pin for Raw Depth Data */
    cOutputPin  output_depth_raw_;
    /*! Output Pin for RGB Video */
    slim::VideoOutputPin video_out_rgb_;
    /*! Output Pin for Depth Visualization Video */
    slim::VideoOutputPin video_out_depthvis_;

public:
    RealsenseCamera(const tChar* __info);
    
    virtual ~RealsenseCamera();

    /*! enables Debug output to Console */
    static const cString PropEnableDebugName;
    /*! Default Value for enable Debug Output Property */
    static const tBool  PropEnableDebugDefault;
    /*! Frame rate for Depth Stream */
    static const cString PropFPSDepthName;
    /*! Frame rate for Color Stream */
    static const cString PropFPSColorName;
    /*! Default for Color and Depth Stream FPS */
    static const int PropFPSDefault;
    /*! Resolution of Depth Stream */
    static const cString PropResolutionDepthName;
    /*! Resolution of Color Stream */
    static const cString PropResolutionColorName;
    /*! Default Value for Color and Depth Resolution */
    static const cString PropResolutionDefault;
    /*! enables Color Stream */
    static const cString PropEnableColorName;
    /*! enables Depth Stream */
    static const cString PropEnableDepthName;
    /*! Default Value to enable Streams */
    static const tBool PropEnableStreamDefault;

    /*! the struct with all the properties*/
    struct filterProperties {
        /*! stores if debug output should be printed */
        tBool enableDebugOutput;
        /*! stores Frame rate for Depth Stream */
        int DepthFPS;
        /*! stores Framrate for Color Stream */
        int ColorFPS;
        /*! stores Resolution for Depth Stream */
        cString DepthResolution;
        /*! stores Resolution for Color Stream* */
        cString ColorResolution;
        /*! stores if Color Stream is Enabled */
        tBool Color;
        /*! stores if Depth Stream is Enabled */
        tBool Depth;
    }
    /*! the filter properties*/
    m_filterProperties;

    /*! context for librealsense */
    rs::context* context_;
    /*! member variable to save the Camera in */
    rs::device* device_;
    /*! True if a camera was found */
    tBool camera_found_;

 protected:

    /*! Init stage. */
    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Shutdown stage. */
    tResult Shutdown(tInitStage stage,
                     ucom::IException** __exception_ptr = NULL);

    /*! Start Stage. */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Stop Stage. */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*! Transmit Raw Depth Data over outputDepthRaw pin
    *   \param pData pointer to the Data to be transmitted
    *   \return Standard Result Code
    */
    tResult TransmitDepthRaw(const void* data);

    /*! Converts Raw Depth Data to Depth Visualization
    *   \param pData pointer to the Data to be transmitted
    *   \return Standard Result Code
    */
    tResult ConvertDepthAndTransmit(const void* data);

    /*! Creates Filter Property automatically from Camera Properties
    *   \param OptionNumber int to enum of Camera Properties
            (see Librealsense Documentation for the enum)
    *   \return void
    */
    void CreateProperty(int OptionNumber);

    /*! Returns Height of the Stream from Resolution
    *   \param Resolution as cString
    *   \return integer of Height
    */
    int HeightOutOfResolution(cString Resolution);

    /*! Returns Width of the Stream from Resolution
    *   \param Resolution as cString
    *   \return integer of Width
    */
    int WidthOutOfResolution(cString Resolution);

    /*! Function to be called in the Thread
    *   \param Thread pointer to the Thread belonging to the Function
    *   \param data pointer to the Data needed by the Thread
    *   \param size size of the Data needed by the Thread
    *   \return Standard Result Code
    */
    tResult ThreadFunc(adtf::cKernelThread* Thread, tVoid* data, tSize size);

private:
    /*! Thread to wait for Camera Picture */
    cKernelThread m_Thread;
    /*! Bitmap Format for RBG Output */
    tBitmapFormat m_BitmapFormatRGBOut;
    /*! Bitmap Format for Depth Visualization Output */
    tBitmapFormat m_BitmapFormatDepthOut;
    /*! Bitmap Format for Infrared Output */
    tBitmapFormat m_BitmapFormatInfraRedOut;

    /*! Is set to true if a display is connected to the depth vis output. */
    bool visualize_depth_;

    /*! Prints serial number, firmware, and properties in debug mode. */
    void PrintDeviceInfo();

    /*! Starts the color, depth, and if connected the infrared streams. */
    void StartStreams();

};


/*!
*@}
*/


//*************************************************************************************************
#endif // AADCUSER_FRAIBURG_REALSENSECAMERA_REALSENSECAMERA_H_
