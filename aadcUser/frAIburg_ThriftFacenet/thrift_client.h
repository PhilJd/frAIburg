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

#ifndef AADCUSER_FRAIBURG_THRIFTFACENET_THRIFT_CLIENT_H_
#define AADCUSER_FRAIBURG_THRIFTFACENET_THRIFT_CLIENT_H_

// thrift must be included before ADTF headers!!!
// these files are first available after the install script was performed once
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>
#include <thrift/transport/TSocket.h>
#include "stdafx.h"  // must com after thrift!

#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include <boost/atomic.hpp>  // use atomic<bool> to control thread launch

#include "ExtService.h"
#include "ExtIf_types.h"
#include "aadc_classification_structs.h"
#include "slim_pins.h"  // SimpleVideoPin
#include "global_map.hpp"  // fraiburg map!
#include "map_types.h"
#include "camera_transformations.h"
#include "nonpin_types.h" // planner signals

namespace fmap = frAIburg::map;

#define MINIMUM_DISTANCE_TO_PERSON 0.5

#define OID_ADTF_FILTER_DEF "frAIburg.aadc.ThriftClient"
#define ADTF_FILTER_DESC "frAIburg Thrift Client"
// ADTF_FILTER_VERSION_SUB_NAME must match accepted_version
#define ADTF_FILTER_VERSION_SUB_NAME "ThriftClient"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"
#define ADTF_FILTER_VERSION_STRING "1.0.0"
#define ADTF_FILTER_VERSION_Major 1
#define ADTF_FILTER_VERSION_Minor 0
#define ADTF_FILTER_VERSION_Build 0
#define ADTF_FILTER_VERSION_LABEL "this filter creates the THRIFT RPC Client which can receive image data and transmit face data.\n"


#undef GetObject  // necessary because WinGDI.h redefines it

// As it's only the demo task, we just put all stuff in here.
namespace demostate {
enum demostate {
    DRIVING_TO_PERSON,
    AT_PERSON_WAIT,
    STOPPED,
    FOLLOW_LANE
};
}  // namespace demostate

class ThriftClient : public adtf::cFilter {
 public:
    /*! This macro does all the plugin setup. */
    // and apparently sets the scope to protected -.-
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Tool,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL);

 public:
    explicit ThriftClient(const tChar* __info);

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
    /*! Updates the input format and the corresponding thrift buffers. */
    tResult UpdateInputFormat();

    /*! If not open, tries to open the thrift client. */
    bool OpenClient();

    /*! Sends a request to the server and waits for the answer. Should be run
     *  in a thread. */
    void SendWaitForRequest();

    void SendPing();

    void TransmitPlannerSignal(behaviour_type type);
    void TransmitLightSignal(tBool enable);

    tResult TransmitLightCommand(tInt32 light_id, tBool switch_bool); // transmitting the light command, (enum light_id, bool on or off)


    /*! Creates a polygon in map space from the detection's bounding box. */
    std::vector<fmap::tMapPoint> GetPolygon(const ext_iface::TObjectResult& data);

    /*! Input pin for the realsense RGB input. */
    slim::VideoInputPin video_input_;

    /*! Input for the raw depth. */
    cInputPin depth_raw_input_;

    slim::VideoInputPin android_image_input_;

    /* pin to send comands to the planner*/
    slim::OutputPin pin_out_behaviour_planner_;
    slim::OutputPin pin_out_light_cmd_;

    fmap::tMapCarPosition car_position_at_input_received_;

    fmap::tTimeMapStamp timestamp_at_input_received_;

    /*! the external interface thrift client */
    boost::shared_ptr<ext_iface::ExtServiceClient> thrift_client_;

    boost::atomic<bool> thread_is_active_;

    /*! Stores the last depth image. */
    cv::Mat depth_img_;

    /*! True if depth_img should be updated. Is set to true when a new RGB
     *  sample arrived. Simlpe way to keep these images roughly synced.*/
    bool update_depth_;

    /*! The format of the raw depth input. Is set in the constructor. */
    tBitmapFormat depth_format_;

    /*! Current state, used to implement a mini state machine. */
    demostate::demostate current_state_;

    /*! MapId of the selected person. */
    fmap::tMapID id_selected_person_;

 protected:
    /*! The OnPinEvent function of video_input_. */
    tResult HandleMediaPinEventRgb(IMediaSample* media_sample);

    /*! The OnPinEvent function of depth_raw_input_. */
    void HandleMediaPinEventDepth(IMediaSample* media_sample);

    /*! Enters detected persons into the map. */
    void AddDetectedPersonsToMap(
        const ext_iface::TObjectResultList& detections);

    /*! Updates the demo state current_state_ and sens commands to the
     *  planner. */
    void UpdateState(bool is_stopped);

    /*! Buffer for received data from inputPin and send to thrift. */
    ext_iface::TDataRaw thrift_rawmessage_buffer_;

    /*! Buffer for received data from inputPin and send to thrift. */
    ext_iface::TImageParams thrift_imageparams_buffer_;

    /*! Stores the pointer to the map singleton. */
    fmap::GlobalMap* map_;

    /*! Holds the transformation matrix of the Basler camera. */
    CameraTransformations basler_transform_;


    //state var.
    float start_wait_time_sec_;
    float last_light_toggle_time_sec_;
    int cnt_not_detected_selected_person_;
    bool light_toggle_state_;
};

/*!
*@}
*/

#endif  // AADCUSER_FRAIBURG_THRIFTFACENET_THRIFT_CLIENT_H_
