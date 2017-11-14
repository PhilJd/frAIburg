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
#ifndef AADCUSER_FRAIBURG_LANEDETECTIONFILTER_LANEDETECTION_H_
#define AADCUSER_FRAIBURG_LANEDETECTIONFILTER_LANEDETECTION_H_

#include "stdafx.h"
#include "lane_finder.h"
#include "ADTF_OpenCV_helper.h"

 #include "line_points_detection.h"
#include "slim_pins.h"  // SimpleVideoPin
#include <boost/assign/list_of.hpp>
#include "global_map.hpp"
#include "map_element.hpp"
#include "map_types.h"
#include "map_helper.hpp"
#include "xml_helper.hpp"
#include "camera_transformations.h"

#define ADTF_PROPERTY_XML_CONFIG_FILE_NAME "frAIburg xml configuration"
#define ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT \
    "/home/aadc/ADTF/configuration_files/frAIburg_configuration.xml"

#define ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME \
        "frAIburg xml configuration camera sensor target"
#define ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_DEFAULT "camera basler cropped"

#define OID_ADTF_FILTER_DEF \
    "adtf.user.aadc_frAIburg_LaneDetection"  // unique for a filter
#define ADTF_FILTER_DESC \
    "frAIburg Lane Detection"  // this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME \
    "OpenCVLaneDetection"  // must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL \
    "accepted_version"                      // sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"  // version string
#define ADTF_FILTER_VERSION_Major \
    1  // this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0  // change will work but notice
#define ADTF_FILTER_VERSION_Build 0  // change will work but notice
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small Lane DetectionFilter \n$Rev:: 62948"


class LaneDetection : public adtf::cFilter {
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC,
                        adtf::OBJCAT_DataFilter, ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major, ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build, ADTF_FILTER_VERSION_LABEL);

 protected:
    /*! input for rgb image */
    slim::VideoInputPin video_input_pin_;

    /*! Output for rgb image */
    slim::VideoOutputPin video_output_pin_;

    /*! output for gcl */
    cOutputPin gcl_output_pin_;

    /*! Sends bool, enable 1 or disable 0 this filter */
    slim::InputPin enable_filter_pin_;

    /*! Send the detected lanes to the planner */
    slim::OutputPin tLaneElement_output_pin_;

    /*! output for one point on a detected lane */
    slim::OutputPin tCrossing_output_pin_;

    LinePointsDetection points_detector_;

    LaneFinder lane_finder_;

    tTimeStamp last_time_;

    tBool is_filter_enabled_;

 public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    explicit LaneDetection(const tChar* __info);

    /*! default destructor */
    virtual ~LaneDetection();

    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the
    *state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions
    *will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter
    *state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the
    * state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions
    * will be put when failed.
    *                                   If not using the cException smart
    * pointer,
    * the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter
    * state
    * changes as follows:\n   *
    *   \result Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage,
                     ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see
    * IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization
    * between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to
    * adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1,
                       tInt nParam2, IMediaSample* pMediaSample);

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready
    * -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions
    * will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a
    * configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the
    * Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state
    * IFilter::State_Running
    * -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions
    * will
    * be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a
    * configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the
    * Message
    * Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*! This Function is always called when any property has changed. This
    * should be the only place
    *    to read from the properties itself and store their values in a member.
    *
    *    \param [in] str_name the name of the property that has changed.
    *    \
    *    \return   Returns a standard result code.
    */
    tResult PropertyChanged(const tChar* str_name);

 private:  // private methods
    /*!     */
    tResult CreateInputPins(__exception);
    /*!     */
    tResult CreateOutputPins(__exception);
    /*!     */
    tResult SetPinIDs();

    /*! function to set the m_sProcessFormat and the  input_format_ variables
     *   \param pFormat the new format for the input pin
     *   \return Standard Result Code.
     */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! function to set the output image format
     *   \param outputImage the new format for the input pin
     *   \return Standard Result Code.
     */
    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);

    /*! function to process the mediasample
     *  1. detect points on horizontal and vertical lane marking
     *  2. fit lines to detected points
     *  3. detect crossing from fitted lines
     *  4. transmit crossing 
     *  5. transmit lines 
     *   \param pSample the new media sample
     *   \return Standard Result Code.
     */
    tResult ProcessVideo(IMediaSample* pSample);

    /*! Read from pin whether filter is enabled or not
     */
    tResult SetEnableBool(IMediaSample* media_sample);

    /*!
     * 1. display detected points and fitted lines
     * 2. display fitted lines in map
     */
    void DisplayDetections(
        const std::vector<tInt>& detection_lines,
        const std::vector<std::vector<tPoint> >& detected_lines,
        const std::vector<Lane>& vert_lanes,
        const std::vector<Lane>& horiz_lanes);

    /*! random number generator for displaying detected lane points*/
    std::vector<cv::Point3i> color_vec_;

    /*! enable debug mode */
    bool property_debug_enabled_;

    /*! the struct with all the properties*/
    struct FilterProperties {
        /*! Offset of the ROI in the Stream*/
        int roi_offset_x;
        /*! Offset of the ROI in the Stream*/
        int roi_offset_y;
        /*! Width of the ROI*/
        int roi_width;
        /*! Height of the ROI*/
        int roi_height;
        /*! Distance between detection lines searched in ROI */
        float detection_distance;
        /*! Minimum Line Width in Pixel */
        float min_line_width;
        /*! Maximum Line Width in Pixel */
        float max_line_width;
        /*! Minimum line contrast in gray Values */
        int min_line_contrast;
        /*! debug point x in meter */
        float debug_pnt_x;
        /*! debug point y in meter */
        float debug_pnt_y;
    }
    /*! the filter properties of this class */
    filter_properties_;

    frAIburg::map::GlobalMap* map_;

    CameraTransformations transform_helper_;

    /*!
     * Transmit gcl.
     *
     * \param   detectionLines  The detection lines.
     * \param   detectedLinePoints  The left lane pixels.
     *
     * \return  A tResult.
     */
    tResult TransmitGCL(
       const std::vector<tInt>& detectionLines,
       const std::vector<std::vector<tPoint> >& detectedLinePoints,
       const std::vector<std::vector<frAIburg::map::tMapPoint> > &sampled_lanes,
       bool has_lane);

    /*!
     * Transmit a vector of detected lanes.
     * This can be connected to the steering controller.
     *
     */
    tResult TransmitLaneVec(const std::vector<Lane>& lane_vec,
                            tTimeStamp sample_time);
    /*!
     * Transmit a tCrossing.
     */
    tResult TransmitCrossing(const tCrossing& crossing,
                            tTimeStamp sample_time);

    /*!
     * Sample lanes
     */
    void SampleLanes(const std::vector<Lane> &vert_lanes
        , const std::vector<Lane> &horiz_lanes
        , std::vector<std::vector<frAIburg::map::tMapPoint> > *sampled_lanes);

    /*!
     * Write the lane to the map
     */
    bool AddLanesToMap(std::vector<std::vector<frAIburg::map::tMapPoint> >
        &sampled_lanes);

    /*!
     * Load the frAIburg xml config file to get an object from CameraTransform
     */
    void LoadConfigFile();
    /*!
     * Call after config file is loaded so that ROI params have correct scale
     */
    void ReloadROIParams();

    /*! Check if roi exceeds image and reset roi to 0 if needed */
    void CheckFilterProperties(int rows, int cols);

    /*! Put corners into map for debugging */
    void AddCornersToMap(const tPath &corner_points);
};

/** @} */  // end of group

#endif  // AADCUSER_FRAIBURG_LANEDETECTION_LANE_DETECTION_H_
