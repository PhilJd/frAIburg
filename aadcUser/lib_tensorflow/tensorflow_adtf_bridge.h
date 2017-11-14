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

#ifndef AADCUSER_TENSORFLOW_TENSORFLOW_ADTF_BRIDGE_H_
#define AADCUSER_TENSORFLOW_TENSORFLOW_ADTF_BRIDGE_H_

#include <tensorflow/c/c_api.h>  // tf_dtypes
#include "stdafx.h"
#include "tensorflow_cpp_wrapper.h"

namespace tf = tensorflow;


/*! @defgroup tensorflow_adtf_bridge
*  @{
* This static class includes various helper functions to simplify tensorflow
* related operations in adtf, e.g. conversion functions to tensorflow tensors
* and types.
*/
class TfAdtfBridge {
 public:
    /*! Creates a tf::Tensor from an adtf_media_sample using the existing
        pointer. 
        Important!! Requires that the Media Sample is locked before and only
        unlocked when tensorflow has finished computation!!  */
    static tf::Tensor NoCopyAdtfMediaSampleToTfTensor(
        void* locked_sample_data, const tBitmapFormat& input_format);

    /*! Not implemented yet. Needs to do memcpy and define custom deallocator
     * function. */
    static tf::Tensor AdtfMediaSampleToTfTensor(
        IMediaSample* adtf_media_sample, const tBitmapFormat& input_format);

    /*! Conversion from adtf bmp datatype to tensorflow datatype
        Note that this loses channel information! */
    static tf::DataType AdtfBmpTypeToTensorType(tInt16 adtf_bmp_type,
                                                tf::Status* status);

    /*! Returns the number of channles of the adtf bmp type */
    static int NumberOfChannels(tInt16 adtf_bmp_type, tf::Status* status);

    /*! Conversion from adtf typedefs to tensorflow datatype */
    template <typename T>
    static tf::DataType AdtfTypeToTensorType(T);

};

#endif  //  AADCUSER_TENSORFLOW_TENSORFLOW_ADTF_BRIDGE_H_
