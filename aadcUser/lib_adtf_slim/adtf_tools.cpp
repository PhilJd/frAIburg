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

#include "adtf_tools.h"
namespace slim {
// ____________________________________________________________________________
tResult AdtfTools::GetDescManager(
    cObjectPtr<IMediaDescriptionManager>& desc_manager, __exception) {
    return _runtime->GetObject(
        OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
        reinterpret_cast<tVoid**>(&desc_manager), __exception_ptr);
}

// ____________________________________________________________________________
template <typename Tfilter, typename pintype, typename pin_register_func>
tResult AdtfTools::CreatePin(Tfilter* filter, pintype& pin,
                             const tChar* strName, const tChar* strType,
                             cObjectPtr<IMediaTypeDescription>& desc_mediatype,
                             cObjectPtr<IMediaDescriptionManager>& pDescManager,
                             pin_register_func pin_register) {
    cObjectPtr<IMediaType> pType;
    RETURN_IF_FAILED(CreateMediaType(pType, strType, pDescManager));
    RETURN_IF_FAILED(
        pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
                            reinterpret_cast<tVoid**>(&desc_mediatype)));
    RETURN_IF_FAILED(
        pin.Create(strName, pType, static_cast<IPinEventSink*>(filter)));
    RETURN_IF_FAILED((filter->*pin_register)(&pin));
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult AdtfTools::CreateMediaType(
    cObjectPtr<IMediaType>& new_media_type, const tChar* strType,
    cObjectPtr<IMediaDescriptionManager>& pDescManager) {
    tChar const* strDesc = pDescManager->GetMediaDescription(strType);
    RETURN_IF_POINTER_NULL(strDesc);
    new_media_type = new cMediaType(0, 0, 0, strType, strDesc,
                                    IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_NOERROR;
}

// ____________________________________________________________________________
tResult AdtfTools::CreateMediaType(cObjectPtr<IMediaType>& new_media_type,
                                   const tChar* strType) {
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(AdtfTools::GetDescManager(pDescManager));
    RETURN_IF_FAILED(CreateMediaType(new_media_type, strType, pDescManager));
    RETURN_NOERROR;
}

// ____________________________________________________________________________

tResult AdtfTools::Transmit(cOutputPin& oPin, vector<tBufferID>& ids,
                            vector<const tVoid*>& vals,
                            cCriticalSection& mutex_critSec, tTimeStamp time,
                            cObjectPtr<IMediaTypeDescription>& description,
                            tInt media_sample_size) {
    // use mutex
    __synchronized_obj(mutex_critSec);

    // get size of media sample signal value
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(cMediaAllocHelper::AllocMediaSample(
        reinterpret_cast<tVoid**>(&pMediaSample)));

    if (ids.size() != vals.size()) {
        LOG_ERROR("AdtfTools::Transmit: size of ids and vals must be same!");
        RETURN_ERROR(ERR_INVALID_INDEX);
    }
    if (oPin.IsConnected()) {
        // get size of the data
        if (media_sample_size == 0) {
            cObjectPtr<IMediaSerializer> pSerializer;
            RETURN_IF_FAILED(
                description->GetMediaSampleSerializer(&pSerializer));
            media_sample_size = pSerializer->GetDeserializedSize();
        }
        //    LOG_ERROR(A_UTILS_NS::cString::Format("debug: media_sample_size %d
        //    ",media_sample_size));
        RETURN_IF_FAILED(pMediaSample->AllocBuffer(media_sample_size));

        // set timestamp
        RETURN_IF_FAILED(pMediaSample->SetTime(time));
        // pMediaSample->SetTime(_clock->GetStreamTime());

        {
            // pCoderOutput is the name of a new var created in the macro
            __adtf_sample_write_lock_mediadescription(description, pMediaSample,
                                                      pCoderOutput);

            // send values for id s
            vector<const tVoid*>::iterator it_val = vals.begin();
            vector<tBufferID>::const_iterator it_id = ids.begin();
            for (; it_id != ids.end(); ++it_id, ++it_val) {
                RETURN_IF_FAILED(pCoderOutput->Set(*it_id, *it_val));
                //           LOG_ERROR(A_UTILS_NS::cString::Format("send id %d ,
                //           send
                //           val %f",*it_id,*(float*)(*it_val)));
            }
        }
        // send data
        RETURN_IF_FAILED(oPin.Transmit(pMediaSample));
    }

    // NOTE the lock will be released on function exit
    RETURN_NOERROR;
}

// // Explicit Template Instantiation
// see: http: // www.cplusplus.com/articles/1C75fSEw/ and

// for templates usage

// cFilter ouputpin create
template tResult
AdtfTools::CreatePin<cFilter, cOutputPin, slim::register_pin_func>(
    cFilter* filter, cOutputPin& pin, const tChar* strName,
    const tChar* strType, cObjectPtr<IMediaTypeDescription>& desc_mediatype,
    cObjectPtr<IMediaDescriptionManager>& pDescManager,
    slim::register_pin_func pin_register);
// cFilter input pin create
template tResult
AdtfTools::CreatePin<cFilter, cInputPin, slim::register_pin_func>(
    cFilter* filter, cInputPin& pin, const tChar* strName, const tChar* strType,
    cObjectPtr<IMediaTypeDescription>& desc_mediatype,
    cObjectPtr<IMediaDescriptionManager>& pDescManager,
    slim::register_pin_func pin_register);

}  // namespace slim