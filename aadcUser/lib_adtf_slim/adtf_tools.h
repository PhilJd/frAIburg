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
#ifndef AADCUSER_LIB_ADTF_SLIM_ADTF_TOOLS_H_
#define AADCUSER_LIB_ADTF_SLIM_ADTF_TOOLS_H_

#include "stdafx.h"
#include <iostream>
#include <vector>

namespace slim {

// typedef register_func for the protected RegisterPin func
typedef tResult (cFilter::*register_pin_func)(IPin* pIPin);

// template<typename T>
// using cfilter_media_sample_func = tResult (T::*)(tVoid**,ucom::IException**);

// cfilter_media_sample_func<adtf::cFilter> mp = &foo::D;

/*! @defgroup adtf_tools
*  @{
* This static class includes various helper functions to simplify adtf
* related operations.
*
* This class exists mainly to reduce bulky function calls, e.g.
*  ~~~~~~~~~~~~~~~{.c}
*  _runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
*                      IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
*                      reinterpret_cast<tVoid**>(&desc_manager),
*                      __exception_ptr);
*  ~~~~~~~~~~~~~~~
*  turns into
*  ~~~~~~~~~~~~~~~{.c}
*  AdtfTools::GetDescManager(desc_manger, __exception_ptr)
*  ~~~~~~~~~~~~~~~
*
*/
class AdtfTools {
 public:
    /*! Saves the Description Manager in the given smart pointer */
    static tResult GetDescManager(
        cObjectPtr<IMediaDescriptionManager>& desc_manager,
        ucom::IException** __exception_ptr = NULL);

    static tResult CreateMediaType(cObjectPtr<IMediaType>& new_media_type,
                                   const tChar* strType);

    static tResult CreateMediaType(
        cObjectPtr<IMediaType>& new_media_type, const tChar* strType,
        cObjectPtr<IMediaDescriptionManager>& pDescManager);

    /*! template for simpler adtf input/outpin creation for all filters
    *   only call in child of cFilter or
    *   calls return if failed
    *   test with cFilter
    *   EXAMPLE in Filter:
    *   slim::register_pin_func func= &EmergencyFilter::RegisterPin;
    *   RETURN_IF_FAILED(AdtfTools::CreatePin<cFilter>(this,
    *                                   outputpin_emergencystop_speed_,
    *                                   "Name","tSignalValue",
    *                                   descriptionFloat_,pDescManager,func));
    *
    *   \param [*filter]   *Filter with pin was member
    *   \param [*pin_register] pointer to the protected function for
    *                          pin register of a adtf Filter, see typedefs above
    */
    template <typename Tfilter, typename pintype, typename pin_register_func>
    static tResult CreatePin(Tfilter* filter, pintype& pin,
                             const tChar* strName, const tChar* strType,
                             cObjectPtr<IMediaTypeDescription>& desc_mediatype,
                             cObjectPtr<IMediaDescriptionManager>& pDescManager,
                             pin_register_func pin_register);

    /*! template for simpler adtf opin transmit for all filters and types
    *   only call in child of cFilter or
    *   calls return if failed
    *   test with cFilter
    *   EXAMPLE in Filter:
    *   vector<tBufferID> ids=boost::assign::list_of(ID1)(ID2s);
        vector<tVoid*>
    vals=boost::assign::list_of((tVoid*)&value)((tVoid*)&timestamp);
        AdtfTools::Transmit<cFilter>(this,*oPin, ids,vals,
                                m_critSecTransmitControl,
                                _clock->GetStreamTime(),
                                descriptionFloat_);
    *
    *   \param [*ids_array] array to the send id's for pCoderOutput->set
    *   \param [*vals]   pointer array to the data
    *   \param [*pin_register] pointer to the protected function for
    *                          pin register of a adtf Filter, see typedefs above
    */
    static tResult Transmit(
        cOutputPin& oPin, vector<tBufferID>& ids, vector<const tVoid*>& vals,
        cCriticalSection& mutex_critSec, tTimeStamp time,
        cObjectPtr<IMediaTypeDescription>& m_pDescriptionFloat,
        tInt media_sample_size = 0);
};

/*! MACRO: for input/outpin creation
*   only call in child of cFilter
*   calls return if failed
*   EXAMPLE IN Filter: CREATE_PIN("EmergencyStopSpeed","tSignalValue",
*                                outputpin_emergencystop_speed_,
*                                 pDescriptionFloat_,pDescManager);
*
*    \param [strPinName]    name of the pin shown in adtf
*    \param [strType]   data type of the input or output
*    \param [pin]   cInputPin or cOutputPin
*    \param [desc_mediatype]  cObjectPtr<IMediaTypeDescription>
*    \param [pDescManager]  ccObjectPtr<IMediaDescriptionManager>
*/
#define CREATE_PIN(strPinName, strType, pin, desc_mediatype, pDescManager) \
    {                                                                      \
        tChar const* strDesc = pDescManager->GetMediaDescription(strType); \
        RETURN_IF_POINTER_NULL(strDesc);                                   \
        cObjectPtr<IMediaType> pType =                                     \
            new cMediaType(0, 0, 0, strType, strDesc,                      \
                           IMediaDescription::MDF_DDL_DEFAULT_VERSION);    \
        RETURN_IF_FAILED((pType)->GetInterface(                            \
            IID_ADTF_MEDIA_TYPE_DESCRIPTION,                               \
            reinterpret_cast<tVoid**>(&desc_mediatype)));                  \
        RETURN_IF_FAILED((pin).Create(strPinName, pType,                   \
                                      static_cast<IPinEventSink*>(this))); \
        RETURN_IF_FAILED(RegisterPin(&(pin)))                              \
    };

}  // namespace slim
#endif  //  AADCUSER_LIB_ADTF_SLIM_ADTF_TOOLS_H_

