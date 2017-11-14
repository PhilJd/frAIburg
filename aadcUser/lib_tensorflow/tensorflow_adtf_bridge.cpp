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
#include "tensorflow_adtf_bridge.h"
#include <stdexcept>  // raise exceptions
#include <typeinfo>

// ____________________________________________________________________________
tf::Tensor TfAdtfBridge::NoCopyAdtfMediaSampleToTfTensor(
    void* locked_sample_data, const tBitmapFormat& input_format) {
    std::vector<int64_t> shape(4);  // shape here is always NHWC
    shape[0] = 1;                   // batch size
    shape[1] = input_format.nHeight;
    shape[2] = input_format.nWidth;
    tf::Status s;
    shape[3] = NumberOfChannels(input_format.nPixelFormat, &s);
    if (!s.CheckTF_OK(true)) {
        return tf::Tensor();
    }
    tf::DataType type = AdtfBmpTypeToTensorType(input_format.nPixelFormat, &s);
    if (!s.CheckTF_OK(true)) {
        return tf::Tensor();
    }
    return tf::Tensor(type, shape, locked_sample_data);
}

// ____________________________________________________________________________
tf::Tensor TfAdtfBridge::AdtfMediaSampleToTfTensor(
    IMediaSample* adtf_media_sample, const tBitmapFormat& input_format) {
    const tVoid* l_pSrcBuffer;
    if (!IS_OK(adtf_media_sample->Lock(&l_pSrcBuffer))) {
        return tf::Tensor();
    }
    LOG_ERROR("AdtfMediaSampleToTfTensor not implemented yet.");
    // convert to Tensor
    return tf::Tensor();
}

// ____________________________________________________________________________
tf::DataType TfAdtfBridge::AdtfBmpTypeToTensorType(tInt16 adtf_bmp_type,
                                                   tf::Status* status) {
    status->Set(TF_OK, "");
    switch (adtf_bmp_type) {
        case adtf_util::cImage::PF_GREYSCALE_8:
        case adtf_util::cImage::PF_RGB_888:
        case adtf_util::cImage::PF_BGR_888:
        case adtf_util::cImage::PF_RGBA_8888:
        case adtf_util::cImage::PF_BGRA_8888:
        case adtf_util::cImage::PF_ABGR_8888:
        case adtf_util::cImage::PF_ARGB_8888:
            return TF_UINT8;
        case adtf_util::cImage::PF_UNKNOWN:
            status->Set(TF_UNKNOWN, "ADTF Image type is cImage::UNKNOWN.");
            // Return resource type as this is rarely used and the most likely
            // type to throw an error.
            return TF_RESOURCE;
        default:
            status->Set(TF_UNKNOWN,
                        "No conversion to ADTF Image type"
                        "implemented.");
            throw std::invalid_argument("Error: Unknown Image type\n");
    }
}

// ____________________________________________________________________________
int TfAdtfBridge::NumberOfChannels(tInt16 adtf_bmp_type, tf::Status* status) {
    typedef adtf_util::cImage cImage;
    status->Set(TF_OK, "");
    switch (adtf_bmp_type) {
        case cImage::PF_GREYSCALE_8:
        case cImage::PF_GREYSCALE_16:
            return 1;
        case cImage::PF_RGB_888:
        case cImage::PF_BGR_888:
            return 3;
        case cImage::PF_RGBA_8888:
        case cImage::PF_BGRA_8888:
        case cImage::PF_ABGR_8888:
        case cImage::PF_ARGB_8888:
            return 4;
        case adtf_util::cImage::PF_UNKNOWN:
            status->Set(TF_UNKNOWN, "ADTF Image type is cImage::UNKNOWN.");
            return 0;
        default:
            status->Set(TF_UNKNOWN,
                        "No conversion to ADTF Image type"
                        "implemented.");
            throw std::invalid_argument("Error: Unknown Image type\n");
    }
}

// ____________________________________________________________________________
template <typename T>
tf::DataType TfAdtfBridge::AdtfTypeToTensorType(T) {
    // The following types have no corresponding tf type:
    // tUInt64, tUInt32, tSize, tFloat128
    if (std::type_info(T()) == std::type_info(tInt8())) return TF_INT8;
    if (std::type_info(T()) == std::type_info(tUInt8())) return TF_UINT8;
    if (std::type_info(T()) == std::type_info(tChar())) return TF_UINT8;
    if (std::type_info(T()) == std::type_info(tInt16())) return TF_INT16;
    if (std::type_info(T()) == std::type_info(tUInt16())) return TF_UINT16;
    if (std::type_info(T()) == std::type_info(tInt32())) return TF_INT32;
    if (std::type_info(T()) == std::type_info(tInt64())) return TF_INT64;
    if (std::type_info(T()) == std::type_info(tBool())) return TF_BOOL;
    if (std::type_info(T()) == std::type_info(tFloat32())) return TF_FLOAT;
    if (std::type_info(T()) == std::type_info(tFloat())) return TF_FLOAT;
    if (std::type_info(T()) == std::type_info(tFloat64())) return TF_DOUBLE;
    if (std::type_info(T()) == std::type_info(tInt())) return TF_INT32;
    if (std::type_info(T()) == std::type_info(tResult())) return TF_INT32;
    // !experimental! types I'm uncertain about and tf quantized types.
    LOG_INFO("Conversion of this type to tf type is experimental.\n");
    if (std::type_info(T()) == std::type_info(tWChar())) return TF_INT16;
    if (std::type_info(T()) == std::type_info(tUInt())) return TF_QINT32;
    if (std::type_info(T()) == std::type_info(tFileSize())) return TF_INT64;
    if (std::type_info(T()) == std::type_info(tFilePos())) return TF_INT64;
    if (std::type_info(T()) == std::type_info(tTimeStamp())) return TF_INT64;
    throw std::invalid_argument("Error: No conversion to tf type known.\n");
}