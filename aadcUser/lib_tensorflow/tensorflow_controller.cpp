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

#include "tensorflow_controller.h"
#include <fstream>
#include <iostream>

// ____________________________________________________________________________
TensorflowController::TensorflowController() { ready_ = false; }

// ____________________________________________________________________________
TensorflowController::TensorflowController(const std::string& protobuf_path) {
    ready_ = LoadFromProtobuf(protobuf_path);
}

// ____________________________________________________________________________
TensorflowController::TensorflowController(const std::string& protobuf_path,
                                           const tf::SessionOptions& opts) {
    ready_ = LoadFromProtobuf(protobuf_path, opts);
}

// ____________________________________________________________________________
TensorflowController::~TensorflowController() {}

// ____________________________________________________________________________
bool TensorflowController::LoadFromProtobuf(const std::string& protobuf_path,
                                            const tf::SessionOptions& opts) {
    graph_.ImportGraphDef(protobuf_path);
    tf::Status s;
    session_ = tf::Session(&graph_, opts, &s);
    last_session_options_ = opts;
    return graph_.Initialized() & s.CheckTF_OK(true);
}

// ____________________________________________________________________________
bool TensorflowController::ForwardPass(const std::vector<tf::OpAndTensor>& feed,
                                       std::vector<tf::OpAndTensor>* fetch) {
    graph_.GetOperationByName((feed[0]).op_name);
    // return false;
    if (!ready_) {
        std::cout << "No Graph loaded or Session creation failed."
                     "Skipping forward pass."
                  << std::endl;
        return false;
    }
    tf::Status s;
    session_.Run(feed, fetch, NULL, NULL, &s);
    return s.CheckTF_OK(true);
}

void TensorflowController::CloseSession() { session_.Close(); }

// ____________________________________________________________________________
void TensorflowController::ActivateSession() {
    if (session_.Initialized()) {
        return;
    }
    tf::Status s;
    std::cout << "activate session" << std::endl;
    session_ = tf::Session(&graph_, last_session_options_, &s);
    s.CheckTF_OK(true);
}

// ____________________________________________________________________________
void TensorflowController::DeactivateSession() {
    std::cout << "In Deactive" << std::endl;
    if (!session_.Initialized()) {
        std::cout << "No Session initialized, doing nothing" << std::endl;
        return;
    }
    // overwrite active session, so it gets destructed and frees memory.
    std::cout << "deactive session" << std::endl;
    session_ = tf::Session();
}

// ____________________________________________________________________________
void TensorflowController::ReadLabels(const std::string& file_name) {
    std::ifstream file(file_name.c_str(), std::ifstream::in);
    if (!file) {
        std::cout << "Labels file '" << file_name.c_str() << "' not found."
                  << std::endl;
        return;
    }
    id_to_label_.clear();
    std::string label;
    for (int i = 0; std::getline(file, label); ++i) {
        id_to_label_[i + 1] = label;  // labels start at 1
    }
}
