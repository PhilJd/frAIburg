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

#include "thrift_client.h"
#include <boost/thread.hpp>
#include "map_element.hpp"
#include "map_element_types.h"
#include "opencv_tools.h"  // AdtfMediaSampleToCvMat
#include "r200_transformations.h"

#define CONSOLE_LOG(_text, _log_level)              \
    if (GetPropertyBool("EnableDebugLogConsole")) { \
        LOG_FN_OUTPUT((_text), _log_level);         \
    }
    //! < enables log function if console output is activated e
    //! CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text,
    //! adtf_util::LOG_LVL_INFO)                        //!< log info messages
#define CONSOLE_LOG_WARNING(_text) \
    CONSOLE_LOG(_text, adtf_util::LOG_LVL_WARNING)  //!< log warning messages
#define CONSOLE_LOG_DUMP(_text) \
    CONSOLE_LOG(_text, adtf_util::LOG_LVL_DUMP)  //!< log dump messages
#define CONSOLE_LOG_ERROR(_text) \
    CONSOLE_LOG(_text, adtf_util::LOG_LVL_ERROR)  //!< log error messages

// Before sending, the bitmap will be converted to the format specified here.
const char bitmap_send_extension[] = "*.jpg";  // alternatively: "*.bmp"

namespace fmap = frAIburg::map;
namespace thrift_protocol = apache::thrift::protocol;
namespace thrift_transport = apache::thrift::transport;

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, ThriftClient)

// ____________________________________________________________________________
ThriftClient::ThriftClient(const tChar* __info) : cFilter(__info) {
    SetPropertyInt("Thrift Port", 1833);
    SetPropertyStr("Thrift Port" NSSUBPROP_DESCRIPTION,
                   "Port number for Thrift Server");

    SetPropertyStr("Thrift IP Address", "localhost");
    SetPropertyStr("Thrift IP Address" NSSUBPROP_DESCRIPTION,
                   "The IPv4 address of the thrift RPC server.");

    SetPropertyBool("EnableDebugLogConsole", false);
    SetPropertyStr("EnableDebugLogConsole" NSSUBPROP_DESCRIPTION,
                   "If enabled additional debug information is printed to"
                   "the console (Warning: decreases performance).");
    SetPropertyBool("EnableDebugLogConsole" NSSUBPROP_ISCHANGEABLE, tFalse);
    depth_format_.nWidth = 320;
    depth_format_.nHeight = 240;
    depth_format_.nBitsPerPixel = 16;
    depth_format_.nPixelFormat = adtf_util::cImage::PF_GREYSCALE_16;
    depth_format_.nBytesPerLine = 640;    // nWidth * nBitsPerPixel / 8
    depth_format_.nSize = 320 * 240 * 2;  // in bytes
    depth_format_.nPaletteSize = 0;
}

// ____________________________________________________________________________
tResult ThriftClient::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst) {
        slim::register_pin_func func = &ThriftClient::RegisterPin;
        RETURN_IF_FAILED(video_input_.StageFirst(this, func, "r200_rgb_input"));
        IPinEventSink* event_sink = static_cast<IPinEventSink*>(this);
        RETURN_IF_FAILED(depth_raw_input_.Create(
            "raw_depth_input",
            new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA,
                                 MEDIA_SUBTYPE_STRUCT_STRUCTURED),
            event_sink));

        RETURN_IF_FAILED(RegisterPin(&depth_raw_input_));
        //output
        RETURN_IF_FAILED(pin_out_behaviour_planner_.FirstStageCreate(
            this, func, "PlannerBehaviour", "tBehaviour"));
        RETURN_IF_FAILED(pin_out_light_cmd_.FirstStageCreate(
        this, func, "light_command", "tLightCommand"));

    } else if (eStage == StageNormal) {
      //state var.
      id_selected_person_ = -1;
      start_wait_time_sec_ = -1;
      last_light_toggle_time_sec_ = -1;
      cnt_not_detected_selected_person_ = 0;
      light_toggle_state_ = false;
      current_state_ = demostate::STOPPED;

    } else if (eStage == StageGraphReady) {
        // init the socket, transport and the protocol from thrift
        boost::shared_ptr<thrift_transport::TTransport> socket(
            new thrift_transport::TSocket(GetPropertyStr("Thrift IP Address"),
                                          GetPropertyInt("Thrift Port")));
        boost::shared_ptr<thrift_transport::TTransport> transport(
            new thrift_transport::TBufferedTransport(socket));
        boost::shared_ptr<thrift_protocol::TProtocol> protocol(
            new thrift_protocol::TBinaryProtocol(transport));
        thrift_client_ =
            boost::make_shared<ext_iface::ExtServiceClient>(protocol);
        UpdateInputFormat();
        map_ = fmap::getInstance();

        // SendPing();

        //set pin ids
        const vector<string> ids_light =
            boost::assign::list_of("light_id")("switch_bool");
        RETURN_IF_FAILED(pin_out_light_cmd_
                         .StageGraphReadySetIDOrder(ids_light));
    }

    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ThriftClient::SendPing() {
  std::string return_msg;

  try {
      thrift_client_->ping(return_msg, "hello from adtf");
      LOG_INFO_PRINTF("msg ping from server: %s",return_msg.c_str());
  } catch (apache::thrift::TException except) {
      LOG_ERROR("Thrift client could not send ping to server");
      LOG_ERROR(cString::Format("EXCEPTION: %s", except.what()));
  }
}

// ____________________________________________________________________________
tResult ThriftClient::Start(__exception) {
    // Initialize parameters here, after the camera filter has activated
    // the stream with the parameters only the camera filter knows
    // (e.g. stream resolution).
    R200Transformations::InitializeIntelParameters();
    thread_is_active_ = false;
    update_depth_ = false;
    return cFilter::Start(__exception_ptr);
}

// ____________________________________________________________________________
tResult ThriftClient::Stop(__exception) {
    while (thread_is_active_) {
        usleep(100000);
    }
    return cFilter::Stop(__exception_ptr);
}

// ____________________________________________________________________________
tResult ThriftClient::Shutdown(tInitStage eStage,
                               ucom::IException** __exception_ptr) {
    return cFilter::Shutdown(eStage, __exception_ptr);
}

// ____________________________________________________________________________
tResult ThriftClient::OnPinEvent(IPin* source, tInt event_code, tInt param1,
                                 tInt param2, IMediaSample* media_sample) {
    RETURN_IF_POINTER_NULL(source);
    if (event_code == IPinEventSink::PE_MediaSampleReceived) {
        RETURN_IF_POINTER_NULL(media_sample);
        if (video_input_.IsSource(source)) {
            HandleMediaPinEventRgb(media_sample);
        } else if (source == &depth_raw_input_) {
            HandleMediaPinEventDepth(media_sample);
        }
    } else if (event_code == IPinEventSink::PE_MediaTypeChanged) {
        if (video_input_.IsSource(source)) {
            UpdateInputFormat();
        }
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
bool ThriftClient::OpenClient() {
    bool client_is_open =
        thrift_client_->getOutputProtocol()->getOutputTransport()->isOpen();
    try {                       // try to open if not opened
        if (!client_is_open) {  // if not open, open the port for the client
            thrift_client_->getOutputProtocol()->getOutputTransport()->open();
            client_is_open = thrift_client_->getOutputProtocol()
                                 ->getOutputTransport()
                                 ->isOpen();
        }
    } catch (apache::thrift::TException except) {
        LOG_ERROR(cString::Format("Exception (OpenClient): %s", except.what()));
    }
    if (!client_is_open) {
        LOG_ERROR("Thrift client could not send to server");
    }
    return client_is_open;
}

// ____________________________________________________________________________
void ThriftClient::SendWaitForRequest() {
    if (thrift_rawmessage_buffer_.raw_data.size() == 0 || !OpenClient()) {
        thread_is_active_ = false;
        return;
    }
    try {
        // send raw data to python server and receive response
        ext_iface::TObjectResultList response;
        thrift_client_->rawData(response, ext_iface::TransportDef::IMAGEDATA,
                                thrift_rawmessage_buffer_,
                                thrift_imageparams_buffer_);
        if (response.size() != 0) {
            AddDetectedPersonsToMap(response);
        }
        bool is_stopped = thrift_client_->IsStopped();
        UpdateState(is_stopped);
    } catch (apache::thrift::TException except) {
        LOG_ERROR("Thrift client could not send to server");
        LOG_ERROR(cString::Format("EXCEPTION: %s", except.what()));
        usleep(100000);
    }
    thread_is_active_ = false;
}

// ____________________________________________________________________________
void ThriftClient::AddDetectedPersonsToMap(
        const ext_iface::TObjectResultList& detections) {
    if (depth_img_.empty()) {
        LOG_WARNING_PRINTF("ThriftClient depth_img_ empty");
        return;
    }
    fmap::tMapID set_id = -1;
    std::vector<ext_iface::TObjectResult>::const_iterator it = detections.begin();
    for (; it != detections.end(); ++it) {
        // LOG_INFO_PRINTF("ThriftClient adding %s to map ",
        //                 it->classification.c_str());
        std::vector<fmap::tMapPoint> polygon = GetPolygon(*it);
        if (polygon.size() == 0) {
            continue;
        }
        fmap::tSptrMapElement el(new fmap::MapElementPedestrian(
            fmap::ADULT, fmap::UNKNOWN_ORIENTATION, polygon,
            timestamp_at_input_received_));
        el->user_tag_ui_ = it->classification;
        // todo(phil) : handle selected, show similarity?
        map_->AddFuseElement(el, 0.4,  // max distance to fuse
                             -1,       // max area to fuse, -1 to deactivate
                             _clock->GetStreamTime(), fmap::MAP_FUSE_REPLACE);
        el->EnableTimeOfLife(5e6);

        fmap::tMapID id = el->GetID();
        if (id == MAP_DEFAULT_ID){
         LOG_ERROR_PRINTF("ThriftClient el with default id after addfuse");
            set_id = 0;
        } else if(it->selected) {
            set_id = id;
        }
    }
    // set id only to not -1 if a person was selected
    // else to -1
    id_selected_person_ = set_id;
}

// ____________________________________________________________________________
void ThriftClient::UpdateState(bool is_stopped) {


    //change state based on current stat and is_stopped
    if (is_stopped && current_state_ != demostate::AT_PERSON_WAIT) {

        if (current_state_ != demostate::STOPPED){
          current_state_ = demostate::STOPPED;
          LOG_INFO_PRINTF("ThriftClient driving STOPPED");
          id_selected_person_ = -1;
          TransmitPlannerSignal(STOP);
        }
    }else if (current_state_ != demostate::AT_PERSON_WAIT
              && current_state_ != demostate::DRIVING_TO_PERSON)
    {
      // enable FOLLOW_LANE if start was send or a person was selected
      // if not driving to a person or not waiting at a person
        if (current_state_ != demostate::FOLLOW_LANE){
          LOG_INFO_PRINTF("ThriftClient FOLLOW_LANE");
          current_state_ = demostate::FOLLOW_LANE;
          TransmitPlannerSignal(FOLLOW_LANE);
          TransmitLightCommand(HAZZARD_LIGHTS, tFalse);
        }
    }

    // if the selected person is visible and selected
    // we drive untill we dont see the person until we're close
    // then we stop and toggle the lights

      if (current_state_ == demostate::STOPPED){
        //DO NOTHING
      } else if (id_selected_person_ != -1
                 && current_state_ != demostate::DRIVING_TO_PERSON)
      {
        TransmitPlannerSignal(FOLLOW_LANE);
        TransmitLightCommand(HAZZARD_LIGHTS, tFalse);


        // new person found and selected
        LOG_INFO_PRINTF("ThriftClient DRIVING_TO_PERSON");
        current_state_ = demostate::DRIVING_TO_PERSON;
        cnt_not_detected_selected_person_ = 0;

      } else if (current_state_ == demostate::DRIVING_TO_PERSON) {

        if (id_selected_person_ == -1){
          ++cnt_not_detected_selected_person_;
          LOG_INFO_PRINTF("ThriftClient driving to person, not detected cnt %d",
            cnt_not_detected_selected_person_);
          if (cnt_not_detected_selected_person_ >= 5){
            // the person was not detected 5 times in a row
            // and person at fixed postion
            // car drove close in lane following -> at person
            current_state_ = demostate::AT_PERSON_WAIT;
            LOG_INFO_PRINTF("ThriftClient AT_PERSON_WAIT");

            try {
                thrift_client_->GoalReached();
            } catch (apache::thrift::TException except) {
                LOG_ERROR_PRINTF("Sending GoalReached to server failed.");
            }
            TransmitPlannerSignal(STOP);
            TransmitLightSignal(true);
            light_toggle_state_ = true;
            start_wait_time_sec_ = _clock->GetStreamTime() / 1e6;
            last_light_toggle_time_sec_ = start_wait_time_sec_;
          }
        }else{
          //reset cnt if person was detected again
          cnt_not_detected_selected_person_ = 0;
        }

      } else if (current_state_ == demostate::AT_PERSON_WAIT) {

        TransmitLightCommand(HAZZARD_LIGHTS, tTrue);
        current_state_ = demostate::STOPPED;
        LOG_INFO_PRINTF("ThriftClient STOPPED");


        // float current_time_sec = _clock->GetStreamTime() / 1e6;
        // //light toggle
        // if (current_time_sec - last_light_toggle_time_sec_ > 0.2){

        //   light_toggle_state_ = !light_toggle_state_;
        //   LOG_INFO_PRINTF("ThriftClient toogle_light %d", light_toggle_state_);
        //   TransmitLightSignal(light_toggle_state_);
        //   last_light_toggle_time_sec_ = current_time_sec;
        // }

        // float wait_time = current_time_sec - start_wait_time_sec_;
        // if (wait_time > 2.0){
        //   start_wait_time_sec_ = -1;
        //   TransmitLightSignal(false);
        //   current_state_ = demostate::STOPPED;
        //   LOG_INFO_PRINTF("ThriftClient STOPPED");
        // }
      }
}

// ____________________________________________________________________________
std::vector<fmap::tMapPoint> ThriftClient::GetPolygon(
    const ext_iface::TObjectResult& data) {
    std::vector<fmap::tMapPoint> polygon;
    // create a square polygon
    tBitmapFormat rgb_format = video_input_.GetFormat();
    // the depth is fixed to 320x240, compute scale factor to find the depth
    // pixels that correspond to the rgb pixels
    float scale = 240.0 / rgb_format.nHeight;
    // pixel coordinates (bottom right and top left)
    cv::Point2f br(data.bbox_xmax, data.bbox_ymax);
    cv::Point2f bl(data.bbox_xmin, data.bbox_ymax);
    cv::Point2f tl(data.bbox_xmin, data.bbox_ymin);
    // as the edges of the bounding box might not be the face but background,
    // we use the non zero minimum as depth.
    cv::Point2f depth_rgb_offset(0, 0);  // (24, 0);
    cv::Point2i br_depth_roi(scale * br - depth_rgb_offset);
    cv::Point2i tl_depth_roi(scale * tl - depth_rgb_offset);
    br_depth_roi.x = std::min(std::max(tl_depth_roi.x, std::min(br_depth_roi.x, 318)), 0);
    br_depth_roi.y = std::min(std::max(tl_depth_roi.y, std::min(br_depth_roi.y, 238)), 0);
    tl_depth_roi.x = std::max(tl_depth_roi.x, 0);
    cv::Rect roi(tl_depth_roi, br_depth_roi);
    cv::Mat non_zero_mask = depth_img_(roi) > 0;
    double depth_value;
    cv::minMaxIdx(depth_img_(roi), &depth_value, NULL, NULL, NULL,
                  non_zero_mask);
    cv::Point2i center = br_depth_roi - (br_depth_roi - tl_depth_roi) / 2;
    depth_value = depth_img_.at<ushort>(center);
    // compute points with depth
    cv::Point3f br_depth(br.x, br.y, depth_value);
    cv::Point3f bl_depth(bl.x, bl.y, depth_value);
    // compute world coordinates
    cv::Point3f p1_world =
        R200Transformations::PixelToWorldCoordinate(bl_depth);
    cv::Point3f p2_world =
        R200Transformations::PixelToWorldCoordinate(br_depth);
    if (p1_world.z > 2 || p2_world.z > 2){  // likely a depth error
        return polygon;
    }
    // create polygon
    polygon.push_back(fmap::tMapPoint(p1_world.x, p1_world.y));
    polygon.push_back(fmap::tMapPoint(p2_world.x, p2_world.y));
    float width = std::abs(p1_world.y - p2_world.y);
    polygon.push_back(fmap::tMapPoint(p2_world.x + width, p2_world.y));
    polygon.push_back(fmap::tMapPoint(p1_world.x + width, p1_world.y));
    return polygon;
}

// ____________________________________________________________________________
tResult ThriftClient::UpdateInputFormat() {
    video_input_.UpdateInputFormat();
    tBitmapFormat format = video_input_.GetFormat();
    thrift_imageparams_buffer_.__set_bytesPerPixel(format.nBitsPerPixel);
    thrift_imageparams_buffer_.__set_height(int16_t(format.nHeight));
    if (format.nWidth == 1920) {
        thrift_imageparams_buffer_.__set_width(int16_t(1440));
    } else {
        thrift_imageparams_buffer_.__set_width(int16_t(format.nWidth));
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ThriftClient::HandleMediaPinEventDepth(IMediaSample* media_sample) {
    if (update_depth_) {  // only update depth if an rgb image arrived
        depth_img_ =
            slim::cvtools::AdtfMediaSampleToCvMat(media_sample, depth_format_);
        update_depth_ = false;
    }
}

// ____________________________________________________________________________
tResult ThriftClient::HandleMediaPinEventRgb(IMediaSample* media_sample) {
    if (thread_is_active_ || video_input_.GetFormat().nSize == 0) {
        RETURN_NOERROR;
    }
    if (video_input_.PixelFormatIsUnknown()) {
        UpdateInputFormat();
    }
    update_depth_ = true;
    tBitmapFormat format = video_input_.GetFormat();
    if (format.nWidth * format.nBitsPerPixel / 8 != format.nBytesPerLine) {
        RETURN_ERROR(ERR_NOT_SUPPORTED);
    }
    const tVoid* sample_buffer;
    if (!IS_OK(media_sample->Lock(&sample_buffer))) {
        RETURN_NOERROR;
    }
    timestamp_at_input_received_ = _clock->GetStreamTime();
    map_->GetGlobalCarPosition(&car_position_at_input_received_);
    tVoid* buffer = const_cast<tVoid*>(sample_buffer);
    cv::Mat img = slim::cvtools::NoCopyAdtfMediaSampleToCvMat(buffer, format);
    cv::cvtColor(img, img, CV_BGR2RGB);
    // cv::Mat rgb_img;
    // cv::cvtColor(img, rgb_img, CV_BGR2RGB);
    std::vector<unsigned char> output_buffer;
    // cut full hd resolution image to ratio of 640 x 480; offset was
    // determined manually
    if (img.rows ==  1080 && img.cols == 1920) {
        img = img(cv::Rect(250, 0, 1440, 1080));
    }
    if (cv::imencode(bitmap_send_extension, img, output_buffer)) {
        thrift_rawmessage_buffer_.raw_data.clear();
        thrift_rawmessage_buffer_.raw_data.assign(
            (const char*)(output_buffer.data()), output_buffer.size());
    }
    media_sample->Unlock(sample_buffer);
    thread_is_active_ = true;
    boost::thread(&ThriftClient::SendWaitForRequest, this);
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void ThriftClient::TransmitPlannerSignal(behaviour_type type) {
    tBehaviour planner_behaviour;
    planner_behaviour.behaviour_id = tInt32(type);
    planner_behaviour.behaviour_next_id = tInt32(BT_UNKNOWN);
    planner_behaviour.speed_id = type == STOP ? tInt32(ST_STOP)
                                              : tInt32(ST_SPEED_LOW);
    planner_behaviour.timestamp = _clock->GetStreamTime();
    planner_behaviour.is_finished = false;
    if (IS_FAILED(pin_out_behaviour_planner_.TransmitStruct<tBehaviour>(
            &planner_behaviour, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF(
            "DynamicEmergencyBreak planner_behaviour failed tansmit");
    }
}

// ____________________________________________________________________________
void ThriftClient::TransmitLightSignal(tBool enable) {

  tInt16 light_id = (tInt16) HEAD_LIGHTS;
  const vector<const void*> vals2 = boost::assign::list_of(
      (const void*)&light_id)((const void*)&enable);
  if (IS_FAILED(pin_out_light_cmd_.Transmit(
          vals2, _clock->GetStreamTime()))) {
      LOG_ERROR("failed sending light val ");
  }
}

tResult ThriftClient::TransmitLightCommand(tInt32 light_id, tBool switch_bool) {
    vector<const void*> vals1 = boost::assign::list_of((const void*)&light_id)(
        (const void*)&switch_bool);
    if (IS_FAILED(pin_out_light_cmd_.Transmit(
            vals1, _clock->GetStreamTime()))) {
        LOG_ERROR_PRINTF("failed sending light command ");
    }
    RETURN_NOERROR;
}
