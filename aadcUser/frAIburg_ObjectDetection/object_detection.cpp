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
#include "stdafx.h"
#include "object_detection.h"
#include <vector>
#include <sys/stat.h>
#include <unistd.h>   // usleep, wait for thread to finish
#include <algorithm>  // std::min
#include <boost/thread.hpp>
#include "bounding_box_visualization.h"
#include "camera_transformations.h"
#include "map_element.hpp"
#include "map_element_types.h"
#include "map_helper.hpp"
#include "opencv_tools.h"  // AdtfMediaSampleToCvMat
#include "tensorflow_opencv_bridge.h"  // cv::Mat to Tensor
#include "xml_helper.hpp"

// ToDo(phil): remove Debug
#include <sys/time.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace tf = tensorflow;
namespace fmap = frAIburg::map;

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, ObjectDetection)

ObjectDetection::ObjectDetection(const tChar* __info)
    : cFilter(__info) {
    // ToDo(Phil): set SessionOptions to GPU.allow growth
    SetAdtfParameters(__info);
    // ToDo(Phil): Check if it's necessary to initialize with correct size
    int tmp_boxes_shape_array[] = {1, 300, 4};
    std::vector<int64_t> shape_boxes(tmp_boxes_shape_array,
                                     tmp_boxes_shape_array + 3);
    detection_boxes_ = tf::Tensor(TF_FLOAT, shape_boxes);

    int tmp_scores_shape_array[] = {1, 300};
    std::vector<int64_t> shape_scores(tmp_scores_shape_array,
                                      tmp_scores_shape_array + 2);
    /*! The scrores corresponding to each box in detection_boxes */
    detection_scores_ = tf::Tensor(TF_FLOAT, shape_scores);

    int tmp_classes_shape_array[] = {1, 300};
    std::vector<int64_t> shape_classes(tmp_classes_shape_array,
                                       tmp_classes_shape_array + 2);
    /*! The classes corresponding to each box in detection_boxes */
    detection_classes_ = tf::Tensor(TF_FLOAT, shape_classes);
    fetch.reserve(3);
    fetch.push_back(tf::OpAndTensor("detection_boxes", &detection_boxes_));
    fetch.push_back(tf::OpAndTensor("detection_scores", &detection_scores_));
    fetch.push_back(tf::OpAndTensor("detection_classes", &detection_classes_));
    thread_is_active_ = false;
    member_write_mutex_ = false;
}

// ____________________________________________________________________________
tResult ObjectDetection::Start(__exception) {
    tf_controller_.ActivateSession();
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult ObjectDetection::Stop(__exception) {
    while (thread_is_active_) {
        usleep(100000);
    }
    tf_controller_.DeactivateSession();
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult ObjectDetection::Init(tInitStage stage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr));
    if (stage == StageFirst) {
        slim::register_pin_func func = &ObjectDetection::RegisterPin;
        RETURN_IF_FAILED(
            basler_input_pin_.StageFirst(this, func, "basler_rgb"));
        RETURN_IF_FAILED(
            basler_output_pin_.StageFirst(this, func, "basler_viz"));

        // Use session options to allow gpu growth
        // options_buf_ =
        // tf::Buffer("/home/aadc/ADTF/weights/faster_rcnn_resnet101_coco_11_"
        //                           "06_2017/session_config_gpugrowth.pb");
        // options_buf_ =
        // tf::Buffer("/home/aadc/ADTF/weights/faster_rcnn_resnet101_coco_11_"
        //                           "06_2017/session_config_gpugrowth.pb");

        // options_buf_ = tf::Buffer(
            // "/home/aadc/ADTF/session_config/"
            // "SessionConfig_XLA_on_allowgroth.pb");

        // options_buf_ =
        // tf::Buffer("/home/aadc/ADTF/src/models/session_configs/session_config_85percent_gpu_fraction.pb");
        options_buf_ =
        tf::Buffer("/home/aadc/ADTF/src/models/session_configs/session_config_allowgrowth_85percent_gpu_fraction.pb");
        opts_.SetConfig(options_buf_.GetDataPtr(), options_buf_.ByteSize());
        // tf_controller_ = TensorflowController(GetPropertyStr("weightspath"),
        // opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/faster_rcnn_resnet101_coco_"
        // "11_06_2017/frozen_inference_graph.pb", opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/rfcn_resnet101_coco_11_06_2017"
        // "/frozen_inference_graph.pb", opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/59286vanilla_ssd_mobilenet_v1_9class_inference_graph_optimized/frozen_inference_graph.pb",
        // opts_);
        //  tf_controller_ =
        //  TensorflowController("/home/aadc/ADTF/weights/340665_kitti_dolls_9class_rfcn_resnet101/frozen_inference_graph.pb",
        //  opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/662289vanilla_rfcn_resnet101_9class_inference_graph_optimized/frozen_inference_graph.pb",
        // opts_);
        // tf_controller_ = TensorflowController(
            // "/home/aadc/ADTF/weights/"
            // "1042585vanilla_rfcn_resnet101_9class_inference_graph_optimized/"
            // "frozen_inference_graph.pb",
            // opts_);


        tf_controller_ = TensorflowController("/home/aadc/ADTF/weights/1469616_kitti_dolls_9class_rfcn_resnet101/frozen_inference_graph.pb", opts_);
        
        // tf_controller_ = TensorflowController("/home/aadc/ADTF/weights/30973_rfcn_resnet101_nokittipeds/frozen_inference_graph.pb", opts_);
        // tf_controller_ = TensorflowController("/home/aadc/ADTF/weights/200000_rfcn_resnet101_nochild/frozen_inference_graph.pb", opts_);
        // tf_controller_ = TensorflowController("/home/aadc/ADTF/weights/403012_rfcn_resnet101_nochild/frozen_inference_graph.pb", opts_);
        

        // tf_controller_ = TensorflowController("/home/aadc/ADTF/weights/faster_rcnn_resnet101_lowproposals_coco_2017_11_08/frozen_inference_graph.pb", opts_);
        // tf_controller_ = TensorflowController("/home/aadc/ADTF/weights/faster_rcnn_resnet50_lowproposals_coco_2017_11_08/frozen_inference_graph.pb", opts_);
        // tf_controller_ = TensorflowController("/home/aadc/ADTF/weights/faster_rcnn_inception_v2_coco_2017_11_08/frozen_inference_graph.pb", opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/64275_ssd_mobilenet_v1_9class_3ratio_inference_graph_optimized/frozen_inference_graph.pb",
        // opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/79687_vanilla_ssd_mobilenet_v1_9class_padaugmented_inference_graph_optimized/frozen_inference_graph.pb",
        // opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/18019vanilla_rfcn_resnet101_9class_inference_graph_optimized/frozen_inference_graph.pb",
        // opts_);

        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/ssd_mobilenet_v1_coco_11_06_2017"
        // "/frozen_inference_graph.pb", opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/test_code/tensorflow/exported/early_vanilla_ssd_mobilenet_v1_9class_inference_graph"
        // "/frozen_inference_graph.pb", opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/test_code/tensorflow/exported/early_vanilla_ssd_mobilenet_v1_9class_inference_graph_optimized"
        // "/frozen_inference_graph.pb", opts_);
        // tf_controller_ =
        // TensorflowController("/home/aadc/ADTF/weights/ssd_inception_v2_coco_11_06_2017"
        // "/frozen_inference_graph.pb", opts_);
        // tf_controller_.ReadLabels("/home/aadc/ADTF/src/tests/cpp/tensorflow_tests/mscoco_labels.txt");
        tf_controller_.ReadLabels(
            "/home/aadc/ADTF/src/models/labels/audicup_label.txt");
        // tf_controller_.DeactivateSession();
    } else if (stage == StageGraphReady) {
        basler_input_pin_.StageGraphReady();
        SetRegionOfInterest();
        min_score_ = GetPropertyFloat("Minimum_detection_confidence");
        display_connected_ = basler_output_pin_.IsConnected();
        map_ = fmap::getInstance();
        SetupMinScoreMap();
        std::string xml_path = GetPropertyStr("fraiburgxmlconfigpath");
        basler_transform_ = CameraTransformations(xml_path, "camera basler cropped");
        RETURN_IF_FAILED(InitTensorflow());
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ObjectDetection::InitTensorflow() {
    // Try to setup tensorflow. First run is slow as the graph gets
    // optimized. If the second run is still slow, return an error.
    cv::Mat dummy(roi_.size(), CV_8UC3);
    cv::randn(dummy, cv::Scalar(0), cv::Scalar(255));
    cString error_msg = "Tensorflow Init failed. Check that nvidia driver "
                         "is installed. Close/Reopen ADTF.";
    if (!DetectObjects(dummy)) {
        LOG_ERROR(error_msg);
        RETURN_ERROR(-1);
    }
    // the second run should be fast. If it's not, we're likely running on
    // CPU only.
    struct timeval tv1, tv2;
    gettimeofday(&tv1, NULL);
    if (!DetectObjects(dummy)) {
        LOG_ERROR(error_msg);
        RETURN_ERROR(-1);
    }
    gettimeofday(&tv2, NULL);
    double span = static_cast<double>(tv2.tv_usec - tv1.tv_usec) / 1000000 +
                  static_cast<double>(tv2.tv_sec - tv1.tv_sec);
    if (span > 0.5) {
        LOG_ERROR(error_msg);
        RETURN_ERROR(-1);
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult ObjectDetection::Shutdown(tInitStage stage,
                                        ucom::IException** __exception_ptr) {
    return cFilter::Shutdown(stage, __exception_ptr);
}

// ____________________________________________________________________________
tResult ObjectDetection::OnPinEvent(IPin* source, tInt event_code,
                                          tInt param1, tInt param2,
                                          IMediaSample* media_sample) {
    RETURN_IF_POINTER_NULL(media_sample);
    RETURN_IF_POINTER_NULL(source);
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        HandleMediaPinEvent(media_sample);
    } else if (event_code == IPinEventSink::PE_MediaTypeChanged) {
        if (basler_input_pin_.IsSource(source)) {
            basler_input_pin_.UpdateInputFormat();
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ObjectDetection::HandleMediaPinEvent(IMediaSample* media_sample) {
    if (thread_is_active_) {
        return;
    }
    if (basler_input_pin_.PixelFormatIsUnknown()) {
        basler_input_pin_.UpdateInputFormat();
    }
    // Create  a cv::Mat from media sample without copying as this happens for
    // the roi later to get a contiguous buffer.
    // todo(phil): check if roi can be removed altogether when the image is
    // cropped in the camera filter directly
    const tVoid* sample_buffer;
    if (!IS_OK(media_sample->Lock(&sample_buffer))) {
        return;
    }
    tBitmapFormat format = basler_input_pin_.GetFormat();
    tVoid* buffer = const_cast<tVoid*>(sample_buffer);
    cv::Mat img = slim::cvtools::NoCopyAdtfMediaSampleToCvMat(buffer, format);
    img = img(roi_).clone();
    cv::cvtColor(img, img, CV_BGR2RGB);
    media_sample->Unlock(sample_buffer);
    if (img.rows == 0) {
        return;
    }
    thread_is_active_ = true;
    boost::thread(&ObjectDetection::HandleMediaPinEventThreadFunc, this,
                  img);
}

// ____________________________________________________________________________
void ObjectDetection::HandleMediaPinEventThreadFunc(cv::Mat img) {
    struct timeval tv1, tv2;
    timestamp_at_input_received_ = _clock->GetStreamTime();
    map_->GetGlobalCarPosition(&car_position_at_input_received_);
    gettimeofday(&tv1, NULL);
    bool detection_success = false;
    try {  // OpenCV rarely produces a transformation error. todo(phil)
           // find the cause of it...
        detection_success = DetectObjects(img);
    } catch (...) {
        LOG_ERROR(
            "ObjectDetection: Function DetectObjects(img) threw an"
            "exception.");
        thread_is_active_ = false;
        return;
    }
    if (detection_success) {
        member_write_mutex_ = true;
        AddObjectsToMap(img.size());
        member_write_mutex_ = false;
    }
    if (display_connected_) {
        VisualizeOutput(&img);
        basler_output_pin_.Transmit(img, _clock->GetStreamTime());
    }
    gettimeofday(&tv2, NULL);
    thread_is_active_ = false;
    printf("Total time for handle media pin event = %f seconds\n",
           static_cast<double>(tv2.tv_usec - tv1.tv_usec) / 1000000 +
               static_cast<double>(tv2.tv_sec - tv1.tv_sec));
}

// ____________________________________________________________________________
bool ObjectDetection::DetectObjects(const cv::Mat& img) {
    std::vector<tf::OpAndTensor> feed;
    tf::Tensor image_tensor;
    image_tensor = TfOpenCvBridge::NoCopyCvMatToTfTensor(img);
    if (image_tensor.NumDims() == 0) {  // tensor empty
        return false;
    }
    feed.push_back(tf::OpAndTensor("image_tensor", &image_tensor));
    if (!tf_controller_.ForwardPass(feed, &fetch)) {
        LOG_ERROR("TF forward pass failed.");
        return false;
    }
    return true;
}

// ____________________________________________________________________________
void ObjectDetection::VisualizeOutput(cv::Mat* img) {
    typedef BoundingBoxVisualization viz;
    FontStyle font_style(cv::Scalar(255, 255, 255), 1, 1);
    std::vector<std::string> labels;
    labels.reserve(valid_indices_.size());
    for (size_t i = 0; i < valid_indices_.size(); ++i) {
        int index = valid_indices_[i];
        int id = detection_classes_.get<float>(0, index, false);
        std::string label =
            viz::GetLabel(detection_scores_.get<float>(0, index, false), id,
                          tf_controller_.id_to_label_);
        labels.push_back(label);
    }
    viz::DrawBoundingBoxes(img, valid_boxes_, labels, cv::Scalar(0, 108, 239),
                           2, font_style);
}

// ____________________________________________________________________________
float ObjectDetection::ApproximateHeight(
        const cv::Rect& bounding_box, const fmap::tMapPoint& p1,
        const fmap::tMapPoint& p2) {
    float ratio = bounding_box.height / static_cast<float>(bounding_box.width);
    return ratio * std::abs((p1.y() - p2.y()));
}

// ____________________________________________________________________________
bool ObjectDetection::RectsToMapPoints(
    std::vector<fmap::tMapPoint>* world_coordinates) {
    // apply transformation to image
    if (!basler_transform_.Initialized()) {
        LOG_ERROR(
            "Transformation matrix for camera is empty. Most likely "
            "the file path to the xml config is invalid.");
        return false;
    }
    std::vector<cv::Point2i> source;
    std::vector<cv::Point2f> transformed;
    source.reserve(valid_boxes_.size() * 2);
    world_coordinates->reserve(valid_boxes_.size() * 2);
    cv::Point2i offset(roi_.tl());
    for (size_t i = 0; i < valid_boxes_.size(); ++i) {
        cv::Point2i box_width = cv::Point2i(valid_boxes_[i].width, 0);
        cv::Point2i bl = valid_boxes_[i].br() - box_width;
        source.push_back(bl + offset);
        source.push_back(valid_boxes_[i].br() + offset);
    }
    basler_transform_.PixelToStreet(source, &transformed);
    for (size_t i = 0; i < transformed.size(); ++i) {
        world_coordinates->push_back(
            fmap::tMapPoint(transformed[i].y, transformed[i].x));
    }
    return true;
}

// ____________________________________________________________________________
void ObjectDetection::RemoveNoisyDetections(
    std::vector<fmap::tMapPoint>* world_coordinates) {
    if (valid_boxes_.size() * 2 != world_coordinates->size()) {
        std::cout << "Error: world positions must be double the size of boxes.";
    }
    std::vector<cv::Rect> tmp_boxes;
    std::vector<fmap::tMapPoint> tmp_world_coordinates;
    std::vector<int> tmp_indices;
    tmp_indices.reserve(valid_indices_.size());
    tmp_boxes.reserve(valid_boxes_.size());
    tmp_world_coordinates.reserve(world_coordinates->size());
    for (size_t i = 0; i < valid_indices_.size(); ++i) {
        // If objects are too tall or too far away, continue
        if (((*world_coordinates)[(i * 2)].x() > MAXIMUM_OBJECT_DISTANCE) |
            ((*world_coordinates)[(i * 2) + 1].x() > MAXIMUM_OBJECT_DISTANCE)) {
            // (approx_height > MAXIMUM_APPROX_OBJECT_HEIGHT)) {
            continue;
        }
        // remove badly projected points
        if (((*world_coordinates)[(i * 2)].x() < 0.01) |
            ((*world_coordinates)[(i * 2) + 1].x() < 0.01)) {
            continue;
        }
        // if it's a person, check if height > width; not more, due to children
        if ((detection_classes_.get<float>(0, i, false) == 1.0) &&
            valid_boxes_[i].width > valid_boxes_[i].height) {
            std::cout << "width > height" << std::endl;
            continue;
        }
        tmp_indices.push_back(valid_indices_[i]);
        tmp_boxes.push_back(valid_boxes_[i]);
        tmp_world_coordinates.push_back((*world_coordinates)[i]);
    }
    valid_indices_ = tmp_indices;
    valid_boxes_ = tmp_boxes;
    *world_coordinates = tmp_world_coordinates;
}

// ____________________________________________________________________________
void ObjectDetection::AddObjectsToMap(const cv::Size& img_size) {
    typedef BoundingBoxVisualization viz;
    valid_indices_.clear();
    valid_boxes_.clear();
    UpdateValidDetectionsIndices();
    if (valid_indices_.size() == 0) {
        return;
    }
    TfBoxesToCvRect(detection_boxes_, img_size);
    std::vector<fmap::tMapPoint> world_coordinates;
    if (!RectsToMapPoints(&world_coordinates)) {
        return;
    }
    RemoveNoisyDetections(&world_coordinates);  // also updates indices
    for (size_t i = 0; i < valid_indices_.size(); ++i) {
        std::vector<fmap::tMapPoint> points;
        points.push_back(world_coordinates[(2 * i) + 1]);
        points.push_back(world_coordinates[(2 * i)]);
        int idx = valid_indices_[i];
        float score = detection_scores_.get<float>(0, idx, false);
        int id = static_cast<int>(detection_classes_.get<float>(0, idx, false));
        fmap::tSptrMapElement el = LabelToMapElement(id, &points,
                                                     valid_boxes_[i]);
        // don't enter real persons into the map
        if (id == 1 &&
            dynamic_cast<fmap::MapElementPedestrian*>(el.get())
                    ->GetPedestrianType() == fmap::REAL_PERSON) {
            continue;
        }
        el->LocalToGlobal(car_position_at_input_received_);
        std::string caption =
            viz::GetLabel(score, id, tf_controller_.id_to_label_);
        el->user_tag_ui_ = caption;
        // el->user_tag_ = caption;
        el->EnableTimeOfLife(1e6);     // in microseconds, rest with fuse
        map_->AddFuseElement(el, 0.4,  // max distance to fuse
                             -1,       // max area to fuse, -1 to deactivate
                             _clock->GetStreamTime(), fmap::MAP_FUSE_REPLACE);
    }
}

// ____________________________________________________________________________
void ObjectDetection::SetAdtfParameters(const tChar* __info) {
    // Path to the binary protobuf file containing the weights
    SetPropertyStr("weightspath",
                   "/home/aadc/ADTF/weights/faster_rcnn_resnet101_coco_"
                   "11_06_2017/frozen_inference_graph.pb");
    cString info =
        "Path to the weights file used to load the tensorflow graph.";
    SetPropertyStr("weightspath" NSSUBPROP_DESCRIPTION, info);
    // The minimum confidence for a detection
    SetPropertyFloat("Minimum_detection_confidence", 0.8);
    // The region of interest's dims
    SetPropertyInt("ROI_y", 0);
    SetPropertyStr("ROI_y" NSSUBPROP_DESCRIPTION,
                   "y value of the ROI's topleft corner.");
    SetPropertyInt("ROI_x", 0);
    SetPropertyStr("ROI_x" NSSUBPROP_DESCRIPTION,
                   "x value of the ROI's topleft corner.");
    SetPropertyInt("ROI_Height", 400);
    SetPropertyStr("ROI_Height" NSSUBPROP_DESCRIPTION, "Height of the ROI");
    SetPropertyInt("ROI_Width", 1280);
    SetPropertyStr("ROI_Width" NSSUBPROP_DESCRIPTION, "Width of the ROI");
    SetPropertyStr("fraiburgxmlconfigpath",
                   "/home/aadc/ADTF/configuration"
                   "_files/frAIburg_configuration");
    SetPropertyStr(
        "fraiburgxmlconfigpath" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER,
        "XML Files (*.xml)");
    // info = "Path to the xml config file containing the camera calibration.";
    // SetPropertyStr("fraiburgxmlconfigpath" NSSUBPROP_DESCRIPTION, info);
}

// ____________________________________________________________________________
void ObjectDetection::SetRegionOfInterest() {
    roi_.y = GetPropertyInt("ROI_y");
    roi_.x = GetPropertyInt("ROI_x");
    roi_.height = GetPropertyInt("ROI_Height");
    roi_.width = GetPropertyInt("ROI_Width");
}

// ____________________________________________________________________________
void ObjectDetection::SetupMinScoreMap() {
    // the last element in the map has the highest key
    if (tf_controller_.id_to_label_.size() == 0) {
        std::cout << "Error: SetupMinScoreMap: id_to_label has no entries."
                  << std::endl;
        return;
    }
    // Get the overall minimum score
    min_score_ = GetPropertyFloat("Minimum_detection_confidence");
    std::cout << "min score from adtf:" << min_score_ << std::endl;
    int num_labels = tf_controller_.id_to_label_.rbegin()->first;
    std::cout << "num_labels:" << num_labels << std::endl;
    min_score_for_label_.resize(num_labels);
    for (int i = 0; i < num_labels; ++i) {
        min_score_for_label_[i] = min_score_;
    }
    // min_score_for_label_[1] = 1.1;  // example how to set a custom min score
    for (size_t i = 0; i < tf_controller_.id_to_label_.size(); ++i) {
        min_score_ = std::min(min_score_for_label_[i], min_score_);
    }
}

// ____________________________________________________________________________
fmap::tSptrMapElement ObjectDetection::LabelToMapElement(
        int label, std::vector<fmap::tMapPoint>* points,
        const cv::Rect& bounding_box) {
    if (label <= 5) {  // is pedestrian
        return CreatePedestrian(label, points, bounding_box);
    } else {
        return CreateCar(label, points);
    }
}

// ____________________________________________________________________________
fmap::MapElementOrientation OrientationFromLabel(int label) {
    switch (label) {
        case 1:
        case 6:
            return fmap::LEFT;
            break;
        case 2:
        case 7:
            return fmap::RIGHT;
            break;
        case 3:
        case 8:
            return fmap::TOWARDS;
            break;
        case 4:
        case 9:
            return fmap::AWAY;
            break;
        default:
            return fmap::UNKNOWN_ORIENTATION;
    }
}

// ____________________________________________________________________________
fmap::tSptrMapElement ObjectDetection::CreatePedestrian(
        int label, std::vector<fmap::tMapPoint>* points,
        const cv::Rect& bounding_box) {
    fmap::MapPedestrianType type;
    // make square polygon
    float width = std::abs((*points)[1].y() - (*points)[0].y());
    points->push_back(
        fmap::tMapPoint((*points)[1].x() + width, (*points)[1].y()));
    points->push_back(
        fmap::tMapPoint((*points)[0].x() + width, (*points)[0].y()));
    type = label == 5 ? fmap::CHILD : fmap::ADULT;
    // To filter out all real persons, only use
    // the ones that have the right width and height
    float height = ApproximateHeight(bounding_box, (*points)[0], (*points)[1]);
    std::cout << "estimated height: " << height << std::endl;
    if ((width <= 0.02) | (width >= 0.2) || height > 0.38) {
        type = fmap::REAL_PERSON;
    }
    return fmap::tSptrMapElement(
        new fmap::MapElementPedestrian(type, OrientationFromLabel(label),
                                       *points, timestamp_at_input_received_));
}


// ____________________________________________________________________________
fmap::tSptrMapElement ObjectDetection::CreateCar(
    int label, std::vector<fmap::tMapPoint>* points) {
    // move points that are close to the car's front but on the very edges
    // a bit more towards the edges as these contain the most error from
    // cars that sit diagonally in their bounding boxes.
    // --> if the point is within 19 cm distance in the forward direction
    //     and only one of the points is close to the bumper
    float x0 = (*points)[0].x();
    float x1 = (*points)[1].x();
    float y0 = (*points)[0].y();
    float y1 = (*points)[1].y();
    if ((x0 < 0.19 || x1 < 0.19) &&
            ((std::abs(y0) < 0.15) || (std::abs(y1) < 0.15)) &&
            (y1*y0) > 0) {
        y0 = (y0 < 0 ? -1.0 : 1.0) * std::max(0.23f, std::abs(y0));
        y1 = (y1 < 0 ? -1.0 : 1.0) * std::max(0.23f, std::abs(y1));
        (*points)[0] = fmap::tMapPoint(x0, y0);
        (*points)[1] = fmap::tMapPoint(x1, y1);
    }
    // In a normal car you wouldn't reduce detected object's sizes, but as one
    // gets zero points for not finishing a sector, we prefer to touch objects
    // instead of waiting infinitely.
    // make a trapezoid polygon and make box 5 cm smaller in each direction
    y1 = std::max(y0 + 0.06f, y1 - 0.05f);
    y0 = std::min(y1 - 0.01f, y0 + 0.05f);
    (*points)[1] = fmap::tMapPoint(x1, y1);
    (*points)[0] = fmap::tMapPoint(x0, y0);
    float width = y1 - y0;
    float trapezoid_offset = 0.2f * width;
    points->push_back(fmap::tMapPoint(x1 + width, y1 - trapezoid_offset));
    points->push_back(fmap::tMapPoint(x0 + width, y0 + trapezoid_offset));
    return fmap::tSptrMapElement(new fmap::MapElementCar(
        OrientationFromLabel(label), *points, timestamp_at_input_received_));
}

// ____________________________________________________________________________
void ObjectDetection::UpdateValidDetectionsIndices() {
    for (int i = 0; i < detection_scores_.Shape()[1]; ++i) {
        float score = detection_scores_.get<float>(0, i, false);
        int label =
            static_cast<int>(detection_classes_.get<float>(0, i, false));
        if (score < min_score_) {
            return;  // no matches possible anymore
        }
        float min_x = detection_boxes_.get<float>(0, i, 1, false);
        float min_y = detection_boxes_.get<float>(0, i, 0, false);
        float max_x = detection_boxes_.get<float>(0, i, 3, false);
        float max_y = detection_boxes_.get<float>(0, i, 2, false);
        int x_left = roi_.width * min_x;
        int x_right = roi_.width * max_x;
        int y_bottom = roi_.height * max_y;
        int y_top = roi_.height * min_y;
        if (((x_right - x_left) < MINIMUM_BOUNDINGBOX_SIZE_PX) |
            ((y_bottom - y_top) < MINIMUM_BOUNDINGBOX_SIZE_PX)) {
            continue;
        }
        if (score >= min_score_for_label_[label]) {
            valid_indices_.push_back(i);
        }
    }
}

// ____________________________________________________________________________
void ObjectDetection::TfBoxesToCvRect(const tf::Tensor& boxes,
                                            const cv::Size& img_size) {
    for (size_t i = 0; i < valid_indices_.size(); ++i) {
        float width = static_cast<float>(img_size.width);
        float height = static_cast<float>(img_size.height);
        float min_x = boxes.get<float>(0, valid_indices_[i], 1, false);
        float min_y = boxes.get<float>(0, valid_indices_[i], 0, false);
        float max_x = boxes.get<float>(0, valid_indices_[i], 3, false);
        float max_y = boxes.get<float>(0, valid_indices_[i], 2, false);
        int x_left = width * min_x;
        int x_right = width * max_x;
        int y_bottom = height * max_y;
        int y_top = height * min_y;
        valid_boxes_.push_back(
            cv::Rect(cv::Point(x_left, y_top), cv::Point(x_right, y_bottom)));
    }
}
