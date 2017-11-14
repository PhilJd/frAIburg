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

#include "slim_pins.h"
#include "ADTF_OpenCV_helper.h"  // Mat2BmpFormat

namespace slim {

// ----------------------------------------------------------------------------
//                                    PinBase
// ----------------------------------------------------------------------------
// ____________________________________________________________________________
template <class tPin>
tResult PinBase<tPin>::StageGraphReadySetIDOrder(const vector<string>& ids) {
    cObjectPtr<IMediaSample> pMediaSample;

    if (IS_OK(cMediaAllocHelper::AllocMediaSample(
            reinterpret_cast<tVoid**>(&pMediaSample)))) {
        cObjectPtr<IMediaSerializer> pSerializer;
        RETURN_IF_FAILED(description_->GetMediaSampleSerializer(&pSerializer));
        media_sample_size_ = pSerializer->GetDeserializedSize();
        RETURN_IF_FAILED(pMediaSample->AllocBuffer(media_sample_size_));

        __adtf_sample_write_lock_mediadescription(description_, pMediaSample,
                                                  pCoderOutput);
        // get all id values for the stringss
        vector<string>::const_iterator it_str = ids.begin();
        tResult err = ERR_NOERROR;
        tBufferID id;
        for (; it_str != ids.end(); ++it_str) {
            err = (pCoderOutput->GetID((*it_str).c_str(), id));

            // DEBUG LOG
            if (IS_FAILED(err)) {
                ids_ = vector<tBufferID>();
                LOG_ERROR_PRINTF(
                    "PinBase::StageGraphReadyInitBuffer:\
                                buffer str ID (%s) is wrong",
                    (*it_str).c_str());
                RETURN_ERROR(err);
            } else {
                ids_.push_back(id);
                LOG_DUMP(
                    A_UTILS_NS::cString::Format("DEBUG IDs: str: %s ,\
                        tBufferID: %d",
                                                (*it_str).c_str(), *it_id));
            }
        }
        pMediaSample->FreeBuffer();
    }
    // NOTE the lock will be released on function exit
    RETURN_NOERROR;
}

// ____________________________________________________________________________
template <class tPin>
tResult PinBase<tPin>::FirstStageCreate(cFilter* filter,
                                        slim::register_pin_func func,
                                        const tChar* strName,
                                        const tChar* strType) {
    filter_ = filter;

    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(AdtfTools::GetDescManager(pDescManager, NULL));

    tResult err = AdtfTools::CreatePin<cFilter>(
        filter, pin_, strName, strType, description_, pDescManager, func);
    if (IS_FAILED(err)) {
        LOG_ERROR_PRINTF(
            "PinBase::create failed, check media "
            "description %s name, add in adtf, pin names be diffrent",
            strType);
    }
    RETURN_ERROR(err);
}

// PinBase Explicit Template Instantiation
template class PinBase<cInputPin>;
template class PinBase<cOutputPin>;

// ----------------------------------------------------------------------------
//                                    InputPin
// ----------------------------------------------------------------------------
// ____________________________________________________________________________
bool InputPin::isSource(IPin* source) { return (&pin_ == source); }

// ____________________________________________________________________________
tResult InputPin::ReadNoID_start(IMediaSample* mediaSample, const tVoid** buff,
                                 tInt size) {
    RETURN_IF_POINTER_NULL(mediaSample);
    if (!pin_.IsConnected()) {
        LOG_ERROR_PRINTF(
            "InputPin::ReadNoID_start:"
            " pin not connected but try to read data");
        RETURN_ERROR(ERR_IO_INCOMPLETE);
    } else if (mediaSample->GetSize() == size) {
        RETURN_IF_FAILED(mediaSample->Lock(buff));
    } else
        LOG_ERROR_PRINTF("InputPin::ReadNoID_start: ReadNoID_start size error");
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult InputPin::ReadNoID_end(IMediaSample* mediaSample, const tVoid** buff) {
    // RETURN_IF_POINTER_NULL(mediaSample);TODO

    RETURN_IF_FAILED(mediaSample->Unlock(buff));

    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult InputPin::ReadIDcopy(IMediaSample* mediaSample,
                             const vector<void*>& val_address) {
    RETURN_IF_POINTER_NULL(mediaSample);

    if (pin_.IsConnected()) {
        if (sizeof(val_address) != sizeof(ids_)) {
            LOG_ERROR_PRINTF(
                "InputPin::ReadIDcopy: size of vals add and\
                     ids must be the same,check if StageGraphReadySetIDOrder\
                     was called with the correct types and order!");
            RETURN_ERROR(ERR_INVALID_INDEX);
        }

        if (mediaSample->GetSize() == media_sample_size_) {
            __adtf_sample_read_lock_mediadescription(description_, mediaSample,
                                                     coderInput);

            std::vector<tBufferID>::iterator it_id = ids_.begin();
            vector<void*>::const_iterator it_val_p = val_address.begin();

            for (; it_id != ids_.end(); ++it_id, ++it_val_p) {
                RETURN_IF_POINTER_NULL(*it_val_p);
                RETURN_IF_FAILED(coderInput->Get(*it_id, *it_val_p));  // copied
                LOG_DUMP(A_UTILS_NS::cString::Format(
                    "DEBUG get val IDs: id: %d", *it_id));
            }
        } else {
            LOG_ERROR_PRINTF(
                "martPinInput::ReadIDcopy:pin read failed:\
                         check data types in create and SetIDOrder");
            RETURN_ERROR(ERR_IO_INCOMPLETE);
        }
    } else {
        LOG_ERROR_PRINTF(
            "martPinInput::ReadIDcopy:pin read failed:\
                     reading data but pin is not connected");
        RETURN_ERROR(ERR_IO_INCOMPLETE);
    }
    // NOTE the lock will be released on function exit
    RETURN_NOERROR;
}

// ----------------------------------------------------------------------------
//                             OutputPin
// ----------------------------------------------------------------------------

// ____________________________________________________________________________
tResult OutputPin::Transmit(vector<const void*> vals, tTimeStamp time) {
    if (sizeof(vals) != sizeof(ids_)) {
        LOG_ERROR_PRINTF(
            "OutputPin::Transmit: size of vals and ids must be the "
            "same, check if StageGraphReadyInitBuffer  was called with the "
            "correct types and order!");
        RETURN_ERROR(ERR_INVALID_INDEX);
    }

    tResult err = AdtfTools::Transmit(pin_, ids_, vals, mutex_, time,
                                      description_, media_sample_size_);

    if (IS_FAILED(err)) {
        LOG_ERROR_PRINTF(
            "OutputPin::Transmit: send failed, check id s and value "
            "order");
        if (ids_.empty()) {
            LOG_ERROR_PRINTF(
                "OutputPin::Transmit: id cnt is zero check if"
                " SetIDOrder was called before");
        }
    }

    RETURN_ERROR(err);
}

// ----------------------------------------------------------------------------
//                             DynamicInputPin
// ----------------------------------------------------------------------------
// ____________________________________________________________________________
tResult DynamicInputPin::StageFirstCreate(cFilter* filter,
                                          slim::register_pin_func func,
                                          DynamicConnectMode mode,
                                          const tChar* strType) {
    filter_ = filter;
    reg_func_ = func;
    mode_ = mode;

    if (mode_ == FIXED_MEDIA_TYPE)
        return (AdtfTools::CreateMediaType(description_, strType));
    else if (mode_ == ALL_MEDIA_TYPE && strType == NULL)
        RETURN_IF_POINTER_NULL(strType);
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DynamicInputPin::StageConnect(IPin* pin_source,
                                      const tChar* strDestName) {
    // based on
    // file:/ //
    // opt/adtf/2.14.0/doc/adtf_sdk_html_docs/page_demo_dynamicpin.html
    RETURN_IF_POINTER_NULL(pin_source);
    cString strDestPinname("");
    if (strDestName != NULL) {
        strDestPinname = cString(strDestName);
    } else {
        strDestPinname = pin_source->GetName();
    }

    cObjectPtr<IPin> pPinInput;

    if (IS_OK(filter_->FindPin(strDestPinname.GetPtr(), IPin::PD_Input,
                               &pPinInput))) {
        LOG_ERROR_PRINTF(
            "The destination Pin already exists, "
            "can not create new Pin \"%s\"",
            strDestPinname.GetPtr());
        RETURN_ERROR(ERR_INVALID_ARG);
    }

    cObjectPtr<IMediaType> pMediaType;
    tResult err = (pin_source->GetMediaType(&pMediaType));

    if (IS_FAILED(err)) {
        LOG_ERROR_PRINTF("DynamicInputPin get mdia type failed");
        RETURN_ERROR(err);
    }

    if (mode_ == ALL_MEDIA_TYPE || pMediaType->IsEqual(description_)) {
        // craete new pin for the media type and add it to the dynamic list
        cObjectPtr<cDynamicInputPin> pin_dynamic_input = new cDynamicInputPin();
        if (IS_OK(pin_dynamic_input->Create(
                strDestPinname.GetPtr(), pMediaType,
                static_cast<IPinEventSink*>(filter_)))) {
            // LOG_INFO(A_UTILS_NS::cString::Format(
            //     "DynamicInputPin create new pin: %s",
            //     strDestPinname.GetPtr()));
            RETURN_IF_FAILED((filter_->*reg_func_)(pin_dynamic_input));
            dynamic_pins_in_.push_back(pin_dynamic_input);
        } else
            LOG_ERROR_PRINTF("DynamicInputPin create new pin failed");
    } else if (mode_ == FIXED_MEDIA_TYPE &&
               !pMediaType->IsEqual(description_)) {  // we have a fix mode so
                                                      // just a mediadesripttion
                                                      // type is used
        LOG_WARNING(A_UTILS_NS::cString::Format(
            "DynamicInputPin pin has wrong media type: %s,\
                 but  %s is needed",
            pMediaType->GetIdentifier(), description_->GetIdentifier()));
    }

    RETURN_NOERROR;
}

bool DynamicInputPin::IsEqualMediaType(IPin* pin_source) {
    bool ret = false;
    if (mode_ == FIXED_MEDIA_TYPE) {
        cObjectPtr<IMediaType> pMediaType;
        tResult err = (pin_source->GetMediaType(&pMediaType));
        if (IS_FAILED(err)) {
            LOG_ERROR_PRINTF("DynamicInputPin get mdia type failed");
        } else {
            ret = pMediaType->IsEqual(description_);
        }
    } else
        ret = true;
    return ret;
}

// ____________________________________________________________________________
bool DynamicInputPin::IsSource(IPin* source, uint32_t* index) {
    for (uint i = 0; i < dynamic_pins_in_.size(); i++) {
        if (source == dynamic_pins_in_[i]) {
            if (index != NULL) {
                *index = i;
            }
            return true;
        }
    }
    return false;
}

// ____________________________________________________________________________
tResult DynamicInputPin::getDynamicPin(uint32_t index,
                                       cObjectPtr<cDynamicInputPin>& ret_pin) {
    if (index < dynamic_pins_in_.size()) {
        ret_pin = dynamic_pins_in_[index];
        RETURN_NOERROR;
    } else {
        RETURN_ERROR(ERR_IO_INCOMPLETE);
    }
}

// ____________________________________________________________________________
tResult DynamicInputPin::getDynamicPinName(uint32_t index,
                                           std::string& ret_string) {
    if (index < dynamic_pins_in_.size()) {
        ret_string = cString(dynamic_pins_in_[index]->GetName());
        RETURN_NOERROR;
    } else {
        RETURN_ERROR(ERR_IO_INCOMPLETE);
    }
}

// ____________________________________________________________________________
uint32_t DynamicInputPin::getDynamicPinCnt() { return dynamic_pins_in_.size(); }

// ____________________________________________________________________________
tResult DynamicInputPin::OnPinEventRead_start(IMediaSample* mediaSample,
                                              const tVoid** buff, tInt size) {
    RETURN_IF_POINTER_NULL(mediaSample);
    if (mediaSample->GetSize() == size) {
        RETURN_IF_FAILED(mediaSample->Lock(buff));
    } else {
        LOG_ERROR_PRINTF(
            "DynamicInputPin::Read_start: ReadNoID_start size error");
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult DynamicInputPin::OnPinEventRead_end(IMediaSample* mediaSample,
                                            const tVoid** buff) {
    RETURN_IF_POINTER_NULL(mediaSample);
    RETURN_IF_FAILED(mediaSample->Unlock(buff));
    RETURN_NOERROR;
}

// ----------------------------------------------------------------------------
//                                    VideoPin
// ----------------------------------------------------------------------------
// ____________________________________________________________________________
tResult VideoPin::StageFirst(cFilter* filter, slim::register_pin_func func,
                             const std::string& name,
                             IPin::tPinDirection direction) {
    RETURN_IF_FAILED(video_pin_.Create(name.c_str(), direction,
                                       static_cast<IPinEventSink*>(filter)));
    RETURN_IF_FAILED((filter->*func)(&video_pin_));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
void VideoPin::SetFormat(const tBitmapFormat& format) {
    bitmap_format_ = format;
    video_pin_.SetFormat(&bitmap_format_, NULL);
}

// ____________________________________________________________________________
bool VideoPin::PixelFormatIsUnknown() {
    return bitmap_format_.nPixelFormat == IImage::PF_UNKNOWN;
}

// ----------------------------------------------------------------------------
//                                     Input

// ____________________________________________________________________________
tResult VideoInputPin::StageGraphReady() { return UpdateInputFormat(); }

// ____________________________________________________________________________
tResult VideoInputPin::StageFirst(cFilter* filter,
                                  slim::register_pin_func func,
                                  const std::string& name) {
    return VideoPin::StageFirst(filter, func, name, IPin::PD_Input);
}

// ____________________________________________________________________________
tResult VideoInputPin::UpdateInputFormat() {
    cObjectPtr<IMediaType> pType;
    RETURN_IF_FAILED(video_pin_.GetMediaType(&pType));
    cObjectPtr<IMediaTypeVideo> type_video;
    RETURN_IF_FAILED(pType->GetInterface(
        IID_ADTF_MEDIA_TYPE_VIDEO, reinterpret_cast<tVoid**>(&type_video)));
    bitmap_format_ = *(type_video->GetFormat());
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tBitmapFormat VideoInputPin::GetFormat() {
    if (bitmap_format_.nPixelFormat == IImage::PF_UNKNOWN) {
        UpdateInputFormat();
    }
    return bitmap_format_;
}

// ----------------------------------------------------------------------------
//                                     Output

// ____________________________________________________________________________
tResult VideoOutputPin::StageFirst(cFilter* filter,
                                   slim::register_pin_func func,
                                   const std::string& name) {
    return VideoPin::StageFirst(filter, func, name, IPin::PD_Output);
}

// ____________________________________________________________________________
tResult VideoOutputPin::Transmit(const void* data, tTimeStamp time) {
    cObjectPtr<IMediaSample> sample;
    tVoid** void_sample = reinterpret_cast<tVoid**>(&sample);
    RETURN_IF_FAILED(cMediaAllocHelper::AllocMediaSample(void_sample));
    RETURN_IF_FAILED(sample->Update(time, data, bitmap_format_.nSize,
                                    IMediaSample::MSF_None));
    RETURN_IF_FAILED(video_pin_.Transmit(sample));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult VideoOutputPin::Transmit(const cv::Mat& img, tTimeStamp time) {
    if (img.empty()) {
        RETURN_NOERROR;
    }
    tBitmapFormat format;
    Mat2BmpFormat(img, format);
    SetFormat(format);
    cObjectPtr<IMediaSample> sample;
    tVoid** void_sample = reinterpret_cast<tVoid**>(&sample);
    RETURN_IF_FAILED(cMediaAllocHelper::AllocMediaSample(void_sample));
    RETURN_IF_FAILED(
        sample->Update(time, img.data, format.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(video_pin_.Transmit(sample));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult VideoOutputPin::TransmitBGR(const cv::Mat& img, tTimeStamp time) {
    if (img.empty()) {
        RETURN_NOERROR;
    }
    tBitmapFormat format;
    Mat2BmpFormat(img, format);
    format.nPixelFormat = adtf_util::IImage::PF_BGR_888;
    SetFormat(format);
    cObjectPtr<IMediaSample> sample;
    tVoid** void_sample = reinterpret_cast<tVoid**>(&sample);
    RETURN_IF_FAILED(cMediaAllocHelper::AllocMediaSample(void_sample));
    RETURN_IF_FAILED(
        sample->Update(time, img.data, format.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(video_pin_.Transmit(sample));
    RETURN_NOERROR;
}

}  // end namespace slim