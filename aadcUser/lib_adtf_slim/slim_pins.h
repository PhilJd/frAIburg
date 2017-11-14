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
#ifndef AADCUSER_LIB_ADTF_SLIM_SLIM_PINS_H_
#define AADCUSER_LIB_ADTF_SLIM_SLIM_PINS_H_

#include "stdafx.h"
#include <boost/assign/list_of.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include "adtf_tools.h"
#include "adtf_log_macros.h"

namespace slim {

/*! @defgroup adtf_slim_pins
*  @{
* This classes includes various helper functions to simplify adtf
* pin creation
*
*/

// ----------------------------------------------------------------------------
//                                    PinBase
// ----------------------------------------------------------------------------
/*! General functionality needed by adc intput and output pins.
 *  Explicit Template Instantiation in the adtf_smart_pin.cpp for
 * cInputPin and cOutputPin for usage with InputPin and OutputPin
 */

template <class tPin>
class PinBase {
 public:
    /** MEMBER VAR **/
    std::vector<tBufferID> ids_;
    tInt media_sample_size_;
    cFilter* filter_ = NULL;
    cObjectPtr<IMediaTypeDescription> description_;
    tPin pin_;
    explicit PinBase(){};
    /** METHODS **/
    /*! defeines buffer ids and buffer sizes from given string ids for
     * needed for Transmit and ReadIDcopy in childen classes
    *  call in Filter Init stage StageGraphReady, test with cFilter
    *   EXAMPLE in Filter:
        vector<string>
    ids=boost::assign::list_of("f32Value")("ui32ArduinoTimestamp");
        speed_pinoutput_.StageGraphReadyInitBuffer(ids);
    *
    *   \param [ids]    strings of all the ids for the pMediaSample in the same
    order
    *                   for transmit and read
    */
    tResult StageGraphReadySetIDOrder(const std::vector<string>& ids);

    /*! Simpler read write without copy an locking the data
    *   call in filterstage Init StageFirst, test with cFilter
    *   EXAMPLE in Filter:
        register_pin_func func= &EmergencyFilter::RegisterPin;
        us_pinput_.FirstStageCreate(this,
    func,"UltrasonicStruct","tUltrasonicStruct");
    *
    *   \param [*filter]       *Filter which has the smart pin as member var
    *   \param [*pin_register] pointer to the protected function for
    *                          pin register of Transmita adtf Filter, see typedefs in
    *                             adtf_tools.h
    */
    tResult FirstStageCreate(cFilter* filter, register_pin_func func,
                             const tChar* strName, const tChar* strType);
};

// ----------------------------------------------------------------------------
//                                    InputPin
// ----------------------------------------------------------------------------

class InputPin : public PinBase<cInputPin> {
 public:
    explicit InputPin() : PinBase<cInputPin>() {}

    bool isSource(IPin* source);

    /*! Simpler adtf read lock without copying the data
    *   tested with cFilter
    *   EXAMPLE in Filter:
        tUltrasonicStruct* pSampleData = NULL;
        if (IS_OK(us_pinput_.ReadNoID_start(mediaSample,
                                           (const tVoid**) &pSampleData,
                                           sizeof(tUltrasonicStruct))))
        {work_with_pSampleData(pSampleData)
        us_pinput_.ReadNoID_end(mediaSample,(const tVoid**) pSampleData);
        }

    *
    *   \param [*buff] buffer
    *   \param [*size] size of buffer
    *   \note call ReadNoID_end before a return is called or the mediaSample
    *         mutex is not released!
    */
    tResult ReadNoID_start(IMediaSample* mediaSample, const tVoid** buff,
                           tInt size);
    tResult ReadNoID_end(IMediaSample* mediaSample, const tVoid** buff);

    /*! Simpler adtf read lock copying the data to given addresses
    *   EXAMPLE in Filter:
            vector<tVoid*> pos_rx_buff_;
            pos_rx_buff_=boost::assign::list_of((tVoid*)&current_car_pos_x_)
                                                   ((tVoid*)&current_car_pos_y_);
            if (car_pos_input_pin_.isSource(source))
                car_pos_input_pin_.ReadIDcopy(mediaSample, pos_rx_buff_);
    *   \param [val_address]    vektor with addresess in the same order set in
    *                           StageGraphReadySetIDOrder
    */
    tResult ReadIDcopy(IMediaSample* mediaSample,
                       const std::vector<void*> & val_address);

    /*! Read dynamic data
    *   based on function provided by aadc demo filter cDriverModule
    *   BETA NOT TESTED
    *   EXAMPLE in Filter:
            vector<char> test;
            uint32_t size=0;
            us_pinput_.ReadDynamicID<char>(mediaSample,test,&size);
    *
    *   \param [val_address] adresses to the send values, same order and size
    like
    in
    *               StageGraphReadyInitBuffer
    */
    template <typename Tdata>
    tResult ReadDynamicID(IMediaSample* mediaSample,
                          std::vector<Tdata> ret_data,
                          uint32_t* data_size);
};


// ----------------------------------------------------------------------------
//                                    OutputPin
// ----------------------------------------------------------------------------

class OutputPin : public PinBase<cOutputPin> {
    /** MEMBER VAR **/
    cCriticalSection mutex_;

 public:
    explicit OutputPin() : PinBase<cOutputPin>() {};
    /*! send function types are defined in StageGraphReadyInitBuffer
    *   StageGraphReadyInitBuffer must be called before usage
    *   tested with cFilter
    *   EXAMPLE in Filter:
         vector<const tVoid*> vals =
                        boost::assign::list_of((const tVoid*)&emergency_stop);
        if (IS_FAILED(pin_out_bool.Transmit(vals,_clock->GetStreamTime())))
        {LOG_ERROR("failed sending speed");}
    *
    *   \param [val_address] adresses to the send values, same order and size
    *                        like in StageGraphReadyInitBuffer
    */
    tResult Transmit(std::vector<const void*> val_address, tTimeStamp time);

    /*! Simpler send function for one type,
    *   StageGraphReadyInitBuffer must be called before usage, with one ID
    *   EXAMPLE in Filter:
    */
    template <typename Tdata>
    tResult Transmit(Tdata data, tTimeStamp timestamp = 0);

    /*! Simpler send function for two type,
    *   StageGraphReadyInitBuffer must be called before usage, with two ID
    *   the data types mus be in the same order as set in  InitBuffer
    *   EXAMPLE in Filter:
    RETURN_IF_FAILED(jury_obstacle_output_pin_.FirstStageCreate(
         this, func, "jury_obstacle", "tObstacle"));
    vector<string> id6s = boost::assign::list_of("f32x")("f32y");
    RETURN_IF_FAILED(jury_obstacle_output_pin_.StageGraphReadySetIDOrder(id6s));
        if (IS_FAILED(jury_obstacle_output_pin_.Transmit<float, float>(
                x_gloabal, y_gloabal, _clock->GetStreamTime()))) {
            LOG_ERROR("Failed sending enable jury_obstacle.");
        }
    */
    template <typename Tdata1, typename Tdata2>
    tResult Transmit(Tdata1 data1 ,Tdata2 data2, tTimeStamp timestamp);


    /** send a struct
    EXAMPLE:
    tBehaviour planner_behaviour;
    if (IS_FAILED(pin_out_behaviour_planner_
                         .TransmitStruct<tBehaviour>(&planner_behaviour,
                         _clock->GetStreamTime()))){}
   */
    template <typename Tstruct>
    tResult TransmitStruct(const Tstruct *data, tTimeStamp timestamp);
};

// ----------------------------------------------------------------------------
//                                    DynamicInputPin
// ----------------------------------------------------------------------------

enum DynamicConnectMode { FIXED_MEDIA_TYPE, ALL_MEDIA_TYPE };

/*! call in filter constructor "ConfigureConnectionPins(0);" to enable dynamic
MINIMAL EXAMPLE DynamicInputPin
*class MyDynamicfilter : public adtf::cFilter {
*    ADTF_FILTER(OID_ADTF_EMERGENCY_FILTER, ADTF_FILTER_EMERGENCY_FILTER_NAME,
*                adtf::OBJCAT_DataFilter);
*    DynamicInputPin dynamic_pin_in_test;
* public:
*
*    MyDynamicfilter(const tChar* __info):cFilter(__info)
*    {*...* ConfigureConnectionPins(0);}
*    *...*
*    tResult Init(tInitStage stage, __exception){
*        RETURN_IF_FAILED(cFilter::Init(stage, __exception_ptr))
*        if (stage == StageFirst)
*            {dynamic_pin_in_test.StageFirstCreate(this,func,
*                            FIXED_MEDIA_TYPE,"tSignalValue");}
*    }
*
*    // all the dynamic pins are added here
*    tResult Connect(IPin* pSource, const tChar* strDestName,
*                      __exception=NULL){
*        THROW_IF_POINTER_NULL(source);
*        if (dynamic_pin_in_test.IsEqualMediaType(source)){
*            if(IS_FAILED(dynamic_pin_in_test.StageConnect(source,strDestName)))
*                {LOG_ERROR("dyn pin failed");}
*        }else{
*            LOG_INFO(A_UTILS_NS::cString::Format("mediatype\
*             for %s is not supported",strDestName));
*        }
*        return(cFilter::Connect(source, strDestName, __exception_ptr));
*    }
*
*    tResult OnPinEvent(IPin* source, tInt event_code,
*                            tInt param1, tInt param2, IMediaSample*
*media_sample) {
*        RETURN_IF_POINTER_NULL(media_sample);
*        RETURN_IF_POINTER_NULL(source);
*        if (event_code == IPinEventSink::PE_MediaSampleReceived)
*        {
*            uint32_t index;
*            if (dynamic_pin_in_test.IsSource(source,&index)){
*
*                cObjectPtr<cDynamicInputPin> pin;
*                dynamic_pin_in_test.getDynamicPin(index,pin);
*                tSignalValue *data;
*                if
*(IS_OK(dynamic_pin_in_test.OnPinEventRead_start(media_sample,(const tVoid**)
*&data,
*                                                         sizeof(tSignalValue))))
*                {
*                    LOG_INFO(A_UTILS_NS::cString::Format("index: %d,name
*%s,data %f",
*                                        index,pin->GetName(),data->f32Value));
*                    dynamic_pin_in_test.OnPinEventRead_end(media_sample,(const
*tVoid**) &data);
*                }
*                else
*                        LOG_ERROR("read dynamic pin failed");
*            }
*        }
*        RETURN_NOERROR;
*    }
*}
*/
class DynamicInputPin {
    DynamicConnectMode mode_;
    cObjectPtr<IMediaType> description_;
    vector<cObjectPtr<cDynamicInputPin> > dynamic_pins_in_;
    register_pin_func reg_func_;
    cFilter* filter_;

 public:
    tResult StageFirstCreate(cFilter* filter, register_pin_func func,
                             DynamicConnectMode mode = ALL_MEDIA_TYPE,
                             const tChar* strType = NULL);

    // call in Connect stage of the cFilter
    // call  return(cFilter::Connect(source, strDestName, __exception_ptr)); at
    // the end
    tResult StageConnect(IPin* pin_source, const tChar* dest_str);

    bool IsSource(IPin* source, uint32_t* index = NULL);
    tResult getDynamicPin(uint32_t index,
                          cObjectPtr<cDynamicInputPin>& ret_pin);
    tResult getDynamicPinName(uint32_t index, std::string& ret_string);
    bool IsEqualMediaType(IPin* pin_source);
    uint32_t getDynamicPinCnt();

    tResult OnPinEventRead_start(IMediaSample* mediaSample, const tVoid** buff,
                                 tInt size);
    tResult OnPinEventRead_end(IMediaSample* mediaSample, const tVoid** buff);

    template <typename Tdata>
    tResult ReadDynamicID(IMediaSample* mediaSample,vector<Tdata> ret_data,
                                         uint32_t* return_data_size);
};

// ----------------------------------------------------------------------------
//                 TEMPLATE METHODS            OutputPin
// ----------------------------------------------------------------------------

// ____________________________________________________________________________
template <typename Tstruct>
tResult OutputPin::TransmitStruct(const Tstruct *data, tTimeStamp timestamp)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(cMediaAllocHelper::AllocMediaSample(
                      reinterpret_cast<tVoid**>(&pMediaSample)));

    pMediaSample->Update(timestamp, data, sizeof(Tstruct), 0);
    return pin_.Transmit(pMediaSample);
}

// ____________________________________________________________________________
template <typename Tdata>
tResult OutputPin::Transmit(Tdata data, tTimeStamp timestamp /*= 0*/){
  const std::vector<const tVoid*> val_adr =
                            boost::assign::list_of((const tVoid*)&data);
  return Transmit(val_adr, timestamp);
}

// ____________________________________________________________________________
template <typename Tdata1, typename Tdata2>
tResult OutputPin::Transmit(Tdata1 data1 ,Tdata2 data2,
                                 tTimeStamp timestamp){
  const std::vector<const tVoid*> val_adr =
    boost::assign::list_of((const tVoid*)&data1)((const tVoid*)&data2);
  return Transmit(val_adr, timestamp);
}


// ----------------------------------------------------------------------------
//                 TEMPLATE METHODS            InputPin
// ----------------------------------------------------------------------------

// ____________________________________________________________________________
template <typename Tdata>
tResult InputPin::ReadDynamicID(IMediaSample* mediaSample,
                                     vector<Tdata> ret_data,
                                     uint32_t* data_size) {
    __adtf_sample_read_lock_mediadescription(description_, mediaSample, pCoder);

    // retrieve number of elements by providing NULL as first paramter
    tSize szBufferSize = 0;
    tResult err = pCoder->GetDynamicBufferIDs(NULL, szBufferSize);
    if (IS_OK(err)) {
        *data_size = szBufferSize;
        // create a buffer depending on the size element
        std::vector<tSize> vecDynamicIDs(szBufferSize);
        if (sizeof(ret_data) < szBufferSize) ret_data.resize(szBufferSize);

        // get the dynamic ids (we already got the first "static" size element)
        if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()),
                                              szBufferSize))) {
            // iterate over all elements except the first
            for (tUInt32 nIdx = 0; nIdx < szBufferSize; ++nIdx) {
                // get the value and put it into the ret buffer
                pCoder->Get(vecDynamicIDs[nIdx],
                            static_cast<tVoid*>(&ret_data[nIdx]));
                if (IS_FAILED(err)) {
                    LOG_ERROR("InputPin:: ReadDynamicID read failed");
                    *data_size = 0;
                    RETURN_ERROR(err);
                }
            }
        }
    }
    RETURN_ERROR(err);
}


// ----------------------------------------------------------------------------
//                                    VideoPin
// ----------------------------------------------------------------------------
/*! This is the base class for the input and output pin. */
class VideoPin {
 public:
    tResult StageFirst(cFilter* filter, register_pin_func func,
                       const std::string& name, IPin::tPinDirection direction);

    /*! Returns true if the contained pin is the source. */
    inline bool IsSource(IPin* source) { return (&video_pin_ == source); }

    inline bool IsConnected() { return video_pin_.IsConnected(); }

    /*! Set the bitmap format. */
    void SetFormat(const tBitmapFormat& format);

    /*! Returns the bitmap format. */
    tBitmapFormat GetFormat() { return bitmap_format_; }

    bool PixelFormatIsUnknown();

 protected:
    cVideoPin video_pin_;

    /*! Bitmap format of the input pin */
    tBitmapFormat bitmap_format_;
};

// ------------- Input
/*! Class that handles a video input pin. */
class VideoInputPin : public VideoPin {
 public:
    tResult StageFirst(cFilter* filter, register_pin_func func,
                       const std::string& name);

    /*! To be called in stage graph ready.  Sets the input format. */
    tResult StageGraphReady();

    /*! Update the input bitmap format. */
    tResult UpdateInputFormat();

    /*! Returns the bitmap format. In addition to the base class function also
    *   tries to update the bitmap_format_ member if it is unknown. */
    tBitmapFormat GetFormat();
};

// ------------- Output
/*! Class that handles a video output pin. Only transmits if a pin is
 *  connected. */
class VideoOutputPin : public VideoPin {
 public:
    tResult StageFirst(cFilter* filter, register_pin_func func,
                       const std::string& name);

    /*! Transmit function for an image. */
    tResult Transmit(const void* data, tTimeStamp time);

    /*! Transmits the image in a cv::Mat. WARNING: Overwrites the output bitmap
     *  format!! */
    tResult Transmit(const cv::Mat& img, tTimeStamp time);
    /*! Same as TransmitRGB. Sets the PixelFormat to BGR before sending. */
    tResult TransmitBGR(const cv::Mat& img, tTimeStamp time);
};

}  // end namespace slim
#endif  //  AADCUSER_LIB_ADTF_SLIM_SLIM_PINS_H_
