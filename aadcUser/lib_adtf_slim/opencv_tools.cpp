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
#include "opencv_tools.h"
#include <iostream>  // std::cout

namespace slim {
namespace cvtools {

// ____________________________________________________________________________
cv::Mat AdtfMediaSampleToCvMat(IMediaSample* adtf_media_sample,
                               const tBitmapFormat& input_format) {
    cv::Mat cv_img;
    const tVoid* buffer;
    if (!IS_OK(adtf_media_sample->Lock(&buffer))) {
        return cv_img;
    }
    UpdateCvMatFormat(input_format, &cv_img);
    // convert to mat, check that we use the right pixelformat
    tInt32 imgsize = tInt32(cv_img.total() * cv_img.elemSize());
    if (imgsize != input_format.nSize) {
        adtf_media_sample->Unlock(buffer);
        return cv_img;
    }
    // copy the data to matrix (make a copy, do not change the
    // adtf_media_sample content itself!)
    memcpy(cv_img.data, buffer, size_t(input_format.nSize));
    adtf_media_sample->Unlock(buffer);
    return cv_img;
}

// ____________________________________________________________________________
cv::Mat NoCopyAdtfMediaSampleToCvMat(tVoid* sample_data,
                                     const tBitmapFormat& input_format) {
    int cv_type = PixelFormat2CVType(input_format.nPixelFormat);
    cv::Mat cv_img(input_format.nHeight, input_format.nWidth, cv_type,
                   sample_data);
    // check that we use the right pixel format
    tInt32 imgsize = tInt32(cv_img.total() * cv_img.elemSize());
    if (imgsize != input_format.nSize) {
        std::cout << "Image size differs from the adtf input_format.nSize. "
                  << "Image Size: " << imgsize
                  << " bitmap format nSize:" << input_format.nSize << std::endl;
        return cv::Mat();
    }
    return cv_img;
}

// ____________________________________________________________________________
tResult UpdateCvMatFormat(const tBitmapFormat& new_format, cv::Mat* image,
                          bool verbose) {
    if (verbose) {
        LOG_INFO(adtf_util::cString::Format(
            "Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
            new_format.nWidth, new_format.nHeight, new_format.nBytesPerLine,
            new_format.nSize, new_format.nPixelFormat));
    }
    // (re)create the image matrix with the appropriate image format
    RETURN_IF_FAILED(BmpFormat2Mat(new_format, *image));
    RETURN_NOERROR;
}

}  // namespace cvtools
}  // namespace slim
