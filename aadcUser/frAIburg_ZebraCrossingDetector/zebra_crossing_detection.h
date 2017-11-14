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
#ifndef AADCUSER_FRAIBURG_ZEBRACROSSINGDETECTIONFILTER_ZebraCrossingDetection_H_
#define AADCUSER_FRAIBURG_ZEBRACROSSINGDETECTIONFILTER_ZebraCrossingDetection_H_

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"
#include "customtypes.h"
#include "nonpin_types.h"
#include <opencv2/imgproc/imgproc.hpp>


//#include <boost/assign/list_of.hpp>

#include "global_map.hpp"
#include "map_element.hpp"
#include "map_types.h"
#include "map_helper.hpp"
//#include "xml_helper.hpp"
#include "slim_pins.h"  // SimpleVideoPin
#include "camera_transformations.h"

#define ADTF_PROPERTY_XML_CONFIG_FILE_NAME "frAIburg xml configuration"
#define ADTF_PROPERTY_XML_CONFIG_FILE_DEFAULT \
    "/home/aadc/ADTF/configuration_files/frAIburg_configuration.xml"
#define ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_NAME "frAIburg xml sensor"
#define ADTF_PROPERTY_XML_CONFIG_SENSOR_TARGET_DEFAULT "camera basler cropped"

#define ADTF_PROPERTY_DEBUG_ENABLED "Enbale filter debug modus"
#define ADTF_PROPERTY_DEBUG_ENABLED_DEFAULT false

#define OID_ADTF_FILTER_DEF \
    "adtf.user.aadc_frAIburg_ZebraCrossingDetection"  // unique for a filter
#define ADTF_FILTER_DESC \
    "frAIburg Zebra Crossing Detection"  // this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME \
    "OpenCVCrossingDetection"  // must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL \
    "accepted_version"                      // sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"  // version string
#define ADTF_FILTER_VERSION_Major \
    1  // this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0  // change will work but notice
#define ADTF_FILTER_VERSION_Build 0  // change will work but notice
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL \
    "A small Zebra Crossing Detection Filter \n$Rev:: 62948"
    

class ZebraCrossingDetection : public adtf::cFilter {
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC,
                        adtf::OBJCAT_DataFilter, ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major, ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build, ADTF_FILTER_VERSION_LABEL);

 protected:
    /*! input for rgb image */
    cVideoPin video_input_pin_;

    /*! Output for rgb image */
    slim::VideoOutputPin video_output_pin_;

   tTimeStamp last_time_;

 public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    explicit ZebraCrossingDetection(const tChar* __info);

    /*! default destructor */
    virtual ~ZebraCrossingDetection();

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

 private:
    struct Contour{
        std::vector<cv::Point> points;
        cv::Point center;
    };
     /*! bitmap format of input pin */
    tBitmapFormat video_input_format_;

    /*! the last received input image */
    cv::Mat input_image_;

    /* has information and shift functions for image_street to image */
    CameraTransformations transform_helper_;

    /*! Map pointer */
    frAIburg::map::GlobalMap* map_;

    /*! is debug mode enabled */
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
        /*! Width of the stripe */
        float zebra_crossing_element_width;
        /*! Height of the stripe */
        float zebra_crossing_element_height;
        /*! Threshold. If Elements further apart - zebra is not valid */
        float max_dist_between_elts;
        /*! tolerance */
        float area_tolerance;
    }
    /*! the filter properties of this class */
    filter_properties_;

    /*! function to set the m_sProcessFormat and the 
     *  video_input_format_ variables
     *   \param pFormat the new format for the input pin
     *   \return Standard Result Code.
     */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! Check if roi is is bigger than the image
     *  at UpdateInputImage and reset roi.
     */
    tResult CheckFilterProperties(int rows, int cols);



    /*! function to process the mediasample
     *   \param pSample the new media sample
     *   \return Standard Result Code.
     */
    tResult ProcessVideo(IMediaSample* pSample);


    /*!
     * Write the crossing to the map
     */
    bool AddToMap(const cv::Point2f &zebra_location,
                const frAIburg::map::tMapData orientation);

    /*!
     * Load the frAIburg xml config file and
     * use the information to create a camera_transformation object
     */
    void LoadConfigFile();

    /*!
     * Detect zebra crossings by using opencv contours.
     */
    bool Detect(const cv::Mat &img_transformed,
                                    cv::Point *return_match_location);
    /*!
     * Check the contours for some features like area, isConnected etc
     */
    void FindCandidates(const std::vector<std::vector<cv::Point> > &contours,
                        std::vector<Contour> *zebra_candidates);

    /*!
     * convert the given value in meter to the corresponding pixel value
     * using transformation parameters
     */
    int MeterToPixel(float meter);

    /*!
     * transform image point to car coordinate system
     */
    void ImagePointToCarCS(const cv::Point &image_point, cv::Point2f *car_point);

    void GetFixedGridOrientation(const frAIburg::map::tMapData &orientation,
                                frAIburg::map::tMapData *return_grid_orientation);

    void TransformToGlobal(const frAIburg::map::tMapData &rotation_angle,
                            const frAIburg::map::tMapData &rotation_point_x,
                            const frAIburg::map::tMapData &rotation_point_y,
                            const cv::Point2f &local_point,
                            cv::Point2f *zebra_location_global);

    /*! Reload the roi params after the config file was loaded */
    void ReloadROIParams();

};

/** @} */  // end of group

#endif  // AADCUSER_FRAIBURG_ZEBRACROSSINGDETECTIONFILTER_ZebraCrossingDetection_H_
