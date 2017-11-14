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
#ifndef AADCUSER_FRAIBURG_OBJECTDETECTION_OBJECT_DETECTION_H_
#define AADCUSER_FRAIBURG_OBJECTDETECTION_OBJECT_DETECTION_H_

#include "stdafx.h"
#include <boost/atomic.hpp>  // use atomic<bool> to control thread launch
#include "opencv2/core/core.hpp"
#include "ADTF_OpenCV_helper.h"
#include "tensorflow_controller.h"
#include "global_map.hpp"
#include "map_types.h"
#include "slim_pins.h"  // SimpleVideoPin
#include "camera_transformations.h"

namespace fmap = frAIburg::map;

const float MAXIMUM_OBJECT_DISTANCE = 2.0;
const float MAXIMUM_APPROX_OBJECT_HEIGHT = 0.4;
const int MINIMUM_BOUNDINGBOX_SIZE_PX = 4;

// unique filter name
#define OID_ADTF_FILTER_DEF "adtf.frAIburg_ObjectDetection"
// appears in the Component Tree in ADTF
#define ADTF_FILTER_DESC "Object Detection (Bounding Box) Filter"
// must match accepted_version_...
#define ADTF_FILTER_VERSION_SUB_NAME "ObjectDetectionFilter"
// sets the version entry
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
// the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "Filter to detect objects using a tf model."

// switch clang format off, to retain the markdown table
// clang-format off
/*! @defgroup ObjectDetectionFilter
*  @{
*    This filter runs an object detection network on the camera input and
*    outputs Bounding Boxes, labels and scores.
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
* | Pin          | Description                    | MajorType        | SubType |
* | ------------ | ------------------------------ | -----------------| -------------------------------- |
* | Video_Input  | Video Pin for data from camera | MEDIA_TYPE_VIDEO | MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED |
*
*
* **Plugin Details**
* | | |
* |-|-|
* | Path    | src/aadcUser/frAIburg_ObjectDetection  |
* | Filename| frAIburg_ObjectDetection.plb           |
* | Version | 1.0.0                                        |
*
*/
// clang-format on

// ! Filter to detect objects using tensorflow in the current image.
/*!
* This class detects objects (bounding boxes) in the current camera image.
*
*/
class ObjectDetection : public adtf::cFilter {
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF, ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary, ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major, ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build, ADTF_FILTER_VERSION_LABEL);

 protected:
    /*! Input for RGB image of the basler camera. */
    slim::VideoInputPin basler_input_pin_;

    /*! Output RGB image of the basler camera, including the bounding box. */
    slim::VideoOutputPin basler_output_pin_;

 public:
    /*! Default constructor for bounding box filter class. */
    explicit ObjectDetection(const tChar* __info);

    /*! Init stage. */
    tResult Init(tInitStage stage, ucom::IException** __exception_ptr);

    /*! Shutdown stage. */
    tResult Shutdown(tInitStage stage,
                     ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to. */
    tResult OnPinEvent(IPin* source, tInt event_code, tInt param1, tInt param2,
                       IMediaSample* media_sample);

    /*! Start Stage. */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Stop Stage. */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

 private:
    /*! Sets the ADTF Parameters for this filter, i.e. weights_path */
    void SetAdtfParameters(const tChar* __info);

    /*! Sets up the lookup map for the scores a label needs to achieve. */
    void SetupMinScoreMap();

    /*! Initalizes tensorflow, i.e. runs two forward passes. The first one
     *  triggers the graph optimization, the second one is timed to check
     *  whether we run on the GPU. */
    tResult InitTensorflow();

    /*! Routine that handles incoming media events, i.e. calls functions to run
     *  detection, visualize the output and add the objects to map. */
    void HandleMediaPinEvent(IMediaSample* media_sample);

    /*! The function to run in a thread, runs tensorflow, visualizes bounding
     *  boxes and transmits. Mat must be copied, as otherwise the image
     *  gets destroyed due to missing reference count increment. */
    void HandleMediaPinEventThreadFunc(cv::Mat img);

    /*! Runs the loaded model and saves bounding box predictions in the
        corresponding member variables. Returns true on success.*/
    bool DetectObjects(const cv::Mat& img);

    /*! Updates the valid detections. Iterates over the predicted boxes
     *  and compares the corresponding label to the minimum score for this
     *  label. */
    void UpdateValidDetectionsIndices();

    /*! Converts the two lower points of the valid_boxes_ to world
     *  coordinates. */
    bool RectsToMapPoints(std::vector<fmap::tMapPoint>* world_coordinates);

    /*! Removes real persons and real objects from the detections based on
     *  height estimation and distance to the car. */
    void RemoveNoisyDetections(std::vector<fmap::tMapPoint>* world_coordinates);

    /*! Gives an estimate of the height by taking the ratio of the bounding box
     *  in pixel space and multiplying it by the width in world coordinates. */
    float ApproximateHeight(const cv::Rect& bounding_box,
                            const fmap::tMapPoint& p1,
                            const fmap::tMapPoint& p2);

    /*! Converts bounding boxes stored in a tf::Tensor to cv::Rects if their
     *  index is in box_indices. Stores the rects in valid_boxes_. */
    void TfBoxesToCvRect(const tf::Tensor& boxes,
                         const cv::Size& img_size);

    /*! Post processes the computed bounding boxes and publish visualization. */
    void VisualizeOutput(cv::Mat* img);

    /*! Adds the objects to the map */
    void AddObjectsToMap(const cv::Size& img_size);

    /*! Fetches the ADTF Property and sets the ROI the detection is run on. */
    void SetRegionOfInterest();

    /*! Creates a map element from the given label and polygon. */
    fmap::tSptrMapElement LabelToMapElement(
        int label, std::vector<fmap::tMapPoint>* points,
        const cv::Rect& bounding_box);

    /*! Creates a pedestrian map element. */
    fmap::tSptrMapElement CreatePedestrian(
        int label, std::vector<fmap::tMapPoint>* points,
        const cv::Rect& bounding_box);

    /*! Creates a car map element. */
    fmap::tSptrMapElement CreateCar(
        int label, std::vector<fmap::tMapPoint>* points);

    /*! Is true if a thread is currently running a tf session.run. If true,
        incoming media samples are omitted.  */
    boost::atomic<bool> thread_is_active_;

    /*! Simple variable to prevent visualization when the member variables
     *  like valid_boxes_ are updated. */
    boost::atomic<bool> member_write_mutex_;

    /*! True if display is connected. */
    bool display_connected_;

    /*! The region of interest the perception is run on. */
    cv::Rect roi_;

    /*! Holds the transformation matrix of the Basler camera. */
    CameraTransformations basler_transform_;

    /*! Stores the pointer to the map singleton. */
    fmap::GlobalMap* map_;

    frAIburg::map::tMapCarPosition car_position_at_input_received_;

    frAIburg::map::tTimeMapStamp timestamp_at_input_received_;

    /*! The lowest minimum score over all classes a detection needs to be
     *  considered. min_score_ is the minimum value in min_score_for_label_. */
    float min_score_;

    /*! Contains the minimum scores for each label. */
    std::vector<float> min_score_for_label_;

    /*! The valid indices from the last detection. */
    std::vector<int> valid_indices_;

    /*! The valid boxes from the last detection. */
    std::vector<cv::Rect> valid_boxes_;

    /*! The active tensorflow controller, holding the graph and the session. */
    TensorflowController tf_controller_;
    /*! The fetch operations for bounding box object detection */
    std::vector<tf::OpAndTensor> fetch;
    /*! The tensor containing number of proposal bounding boxes */
    tf::Tensor detection_boxes_;
    /*! The scores corresponding to each box in detection_boxes_ */
    tf::Tensor detection_scores_;
    /*! The classes corresponding to each box in detection_boxes_ */
    tf::Tensor detection_classes_;
    /*! The session options used to construct the session. */
    tf::SessionOptions opts_;
    /*! The buffer for the session options. */
    tf::Buffer options_buf_;
};

/** @} */  // end of group

#endif  // AADCUSER_FRAIBURG_OBJECTDETECTION_OBJECT_DETECTION_H_
