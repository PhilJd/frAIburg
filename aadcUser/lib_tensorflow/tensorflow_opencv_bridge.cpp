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
#include "tensorflow_opencv_bridge.h"
#include <opencv/cv.h>
#include <stdexcept>  // raise exceptions
#include <typeinfo>

// ____________________________________________________________________________
tf::Tensor TfOpenCvBridge::NoCopyCvMatToTfTensor(const cv::Mat& mat) {
    std::vector<int64_t> shape(4);  // shape here is always NHWC
    shape[0] = 1;                   // batch size
    shape[1] = mat.rows;
    shape[2] = mat.cols;
    shape[3] = mat.channels();
    tf::Status s;
    tf::DataType type = CVTypeToTensorType(mat, &s);
    if (!s.CheckTF_OK(true)) {
        return tf::Tensor();
    }
    // the actual dtype is not important as tf::Tensor takes void pointer
    return tf::Tensor(type, shape, (uchar*)mat.ptr());  // NOLINT
}

// ____________________________________________________________________________
tf::DataType TfOpenCvBridge::CVTypeToTensorType(const cv::Mat& mat,
                                                tf::Status* status) {
    return CVTypeToTensorType(mat.type(), status);
}

// ____________________________________________________________________________
tf::DataType TfOpenCvBridge::CVTypeToTensorType(int mat_type,
                                                tf::Status* status) {
    status->Set(TF_OK, "");
    switch (mat_type) {
        case CV_8U:
        case CV_8UC3:
        case CV_8UC4:
            return TF_UINT8;
        case CV_8S:
        case CV_8SC3:
        case CV_8SC4:
            return TF_INT8;
        case CV_16U:
        case CV_16UC3:
        case CV_16UC4:
            return TF_UINT16;
        case CV_16S:
        case CV_16SC3:
        case CV_16SC4:
            return TF_INT16;
        case CV_32S:
        case CV_32SC3:
        case CV_32SC4:
            return TF_INT32;
        case CV_32F:
        case CV_32FC3:
        case CV_32FC4:
            return TF_FLOAT;
        case CV_64F:
        case CV_64FC3:
        case CV_64FC4:
            return TF_DOUBLE;
        default:
            status->Set(TF_UNKNOWN,
                        "No conversion to OpenCV type implemented.");
            throw std::invalid_argument("Error: Unknown Image type\n");
    }
}
