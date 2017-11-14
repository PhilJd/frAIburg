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

#include "tensorflow_cpp_wrapper.h"
#include <stdio.h>  // read protobuf
#include <fstream>
#include <iostream>
#include <stdexcept>  // throw exception if tf type and template type mismatch

namespace tensorflow {

// ____________________________________________________________________________
std::string version() { return std::string(TF_Version()); }

// ____________________________________________________________________________
size_t DataTypeSize(DataType dtype) { return TF_DataTypeSize(dtype); }

// ____________________________________________________________________________
//! A default deallocator doing nothing. Useful when we use a cv::Mat's data
// as a tensor's / buffer's data pointer (cv::Mat handles memory management
// in this case)
void void_deallocator(void* data, size_t len, void* arg) {}
void void_deallocator(void* data, size_t len) {}

// ____________________________________________________________________________
void TfDestructor(TF_Session* session) {
    Status s;
    TF_CloseSession(session, s.c_status_.get());
    s.CheckTF_OK();
    TF_DeleteSession(session, s.c_status_.get());
    s.CheckTF_OK();
}

// -----------------------------------------------------------------------------
//                                    Status
// -----------------------------------------------------------------------------
// ____________________________________________________________________________
Status::Status() {
    c_status_ = boost::shared_ptr<TF_Status>(TF_NewStatus(), TF_DeleteStatus);
}

// ____________________________________________________________________________
void Status::Set(TF_Code code, const std::string& msg) {
    TF_SetStatus(c_status_.get(), code, msg.c_str());
}

// ____________________________________________________________________________
TF_Code Status::GetCode() { return TF_GetCode(c_status_.get()); }

// ____________________________________________________________________________
std::string Status::GetMessage() {
    return std::string(TF_Message(c_status_.get()));
}

// ____________________________________________________________________________
bool Status::CheckTF_OK(bool verbose) {
    bool is_tf_ok = GetCode() == TF_OK;
    if (!is_tf_ok && verbose) {
        std::cout << "Error:" << GetMessage() << std::endl;
    }
    return is_tf_ok;
}

// -----------------------------------------------------------------------------
//                                    Tensor
// -----------------------------------------------------------------------------
Tensor::Tensor() {
    std::vector<int64_t> shape;
    c_tensor_ = boost::shared_ptr<TF_Tensor>(
        TF_AllocateTensor(TF_FLOAT, &shape[0], 0, 0), TF_DeleteTensor);
    shape_ = shape;
}

// ____________________________________________________________________________
Tensor::Tensor(TF_DataType dtype, const std::vector<int64_t>& shape) {
    size_t tensor_size = ShapeToBufferLength(shape, dtype);
    c_tensor_ = boost::shared_ptr<TF_Tensor>(
        TF_AllocateTensor(dtype, &shape[0], shape.size(), tensor_size),
        TF_DeleteTensor);
    shape_ = shape;
}

// ____________________________________________________________________________
Tensor::Tensor(TF_DataType dtype, const std::vector<int64_t>& shape, void* data,
               void (*deallocator)(void* data, size_t len, void* arg) /*=NULL*/,
               void* deallocator_arg /*=NULL*/) {
    size_t tensor_size = ShapeToBufferLength(shape, dtype);
    if (deallocator == NULL) {
        deallocator = void_deallocator;
    }
    c_tensor_ = boost::shared_ptr<TF_Tensor>(
        TF_NewTensor(dtype, &shape[0], shape.size(), data, tensor_size,
                     deallocator, deallocator_arg),
        TF_DeleteTensor);
    shape_ = shape;
}

// ____________________________________________________________________________
TF_DataType Tensor::Dtype() const { return TF_TensorType(c_tensor_.get()); }

// ____________________________________________________________________________
int Tensor::NumDims() const { return TF_NumDims(c_tensor_.get()); }

// ____________________________________________________________________________
std::vector<int64_t> Tensor::Shape() const { return shape_; }

// ____________________________________________________________________________
size_t Tensor::ByteSize() const { return TF_TensorByteSize(c_tensor_.get()); }

// ____________________________________________________________________________
inline size_t Tensor::ShapeToBufferLength(const std::vector<int64_t>& shape,
                                          TF_DataType dtype) const {
    size_t tensor_size = 1;
    for (size_t i = 0; i < shape.size(); ++i) {
        tensor_size *= shape[i];
    }
    return tensor_size * TF_DataTypeSize(dtype);
}

// -----------------------------------------------------------------------------
//                                    Buffer
// -----------------------------------------------------------------------------
Buffer::Buffer() {
    c_buffer_ = boost::shared_ptr<TF_Buffer>(TF_NewBuffer(), TF_DeleteBuffer);
}

// ____________________________________________________________________________
Buffer::Buffer(const std::string& protobuf_path) {
    c_buffer_ = boost::shared_ptr<TF_Buffer>(TF_NewBuffer(), TF_DeleteBuffer);
    ReadFromProtobuf(protobuf_path);
}

// ____________________________________________________________________________
const void* Buffer::GetDataPtr() const { return c_buffer_.get()->data; }

// ____________________________________________________________________________
size_t Buffer::ByteSize() const { return c_buffer_.get()->length; }

// ____________________________________________________________________________
void Buffer::ReadFromProtobuf(const std::string& protobuf_path) {
    std::ifstream input(protobuf_path.c_str(), std::ios::binary);
    // Note: istream needs additional parenthesis due to `most vexing parse`
    data_ = std::vector<char>((std::istreambuf_iterator<char>(input)),
                              (std::istreambuf_iterator<char>()));
    c_buffer_.get()->data = &data_[0];
    c_buffer_.get()->length = data_.size();
    //  automatically deallocated when std::vector goes out of scope
    c_buffer_.get()->data_deallocator = void_deallocator;
}

// -----------------------------------------------------------------------------
//                                    Graph
// -----------------------------------------------------------------------------
Graph::Graph() {
    c_graph_ = boost::shared_ptr<TF_Graph>(TF_NewGraph(), TF_DeleteGraph);
    c_import_options_ = boost::shared_ptr<TF_ImportGraphDefOptions>(
        TF_NewImportGraphDefOptions(), TF_DeleteImportGraphDefOptions);
    TF_ImportGraphDefOptionsSetPrefix(c_import_options_.get(), "");
}

// ____________________________________________________________________________
Graph::Graph(const std::string& graph_def_path, Status* status) {
    c_graph_ = boost::shared_ptr<TF_Graph>(TF_NewGraph(), TF_DeleteGraph);
    c_import_options_ = boost::shared_ptr<TF_ImportGraphDefOptions>(
        TF_NewImportGraphDefOptions(), TF_DeleteImportGraphDefOptions);
    TF_ImportGraphDefOptionsSetPrefix(c_import_options_.get(), "");
    ImportGraphDef(graph_def_path, status);
}

// ____________________________________________________________________________
bool Graph::ImportGraphDef(const std::string& graph_def_path, Status* status) {
    default_buffer_.ReadFromProtobuf(graph_def_path);
    return ImportGraphDef(default_buffer_, status);
}

// ____________________________________________________________________________
bool Graph::ImportGraphDef(const Buffer& graph_def, Status* status) {
    if (status == NULL) {
        status = &default_status_;
    }
    TF_GraphImportGraphDef(c_graph_.get(), graph_def.c_buffer_.get(),
                           c_import_options_.get(), status->c_status_.get());
    return status->CheckTF_OK(true);
}

// ____________________________________________________________________________
void Graph::PrintGraphOps() const {
    size_t pos = 0;
    std::cout << "Graph Ops:" << std::endl;
    TF_Operation* op = TF_GraphNextOperation(c_graph_.get(), &pos);
    while ((op = TF_GraphNextOperation(c_graph_.get(), &pos)) != NULL) {
        std::cout << TF_OperationName(op) << "\n";
    }
    std::cout << std::endl;
}

// ____________________________________________________________________________
TF_Operation* Graph::OperationByName(const std::string& op_name) const {
    TF_Operation* result = NULL;
    try {
        result = TF_GraphOperationByName(c_graph_.get(), op_name.c_str());
    } catch (const std::exception& e) {
        std::cout << "Fetching operation `" << op_name
                  << "`from graph raised an exception: " << e.what()
                  << std::endl;
        return NULL;
    }
    if (result == NULL) {
        std::cout << "Error: No Operation with name '" << op_name
                  << "' in graph." << std::endl;
    }
    return result;
}

// -----------------------------------------------------------------------------
//                                SessionOptions
// -----------------------------------------------------------------------------
// ____________________________________________________________________________
SessionOptions::SessionOptions() {
    c_sessionoptions_ = boost::shared_ptr<TF_SessionOptions>(
        TF_NewSessionOptions(), TF_DeleteSessionOptions);
}

// ____________________________________________________________________________
void SessionOptions::SetTarget(const std::string& target) {
    TF_SetTarget(c_sessionoptions_.get(), target.c_str());
}

// ____________________________________________________________________________
void SessionOptions::SetConfig(const Buffer& buffer, Status* status) {
    if (status == NULL) {
        status = &default_status_;
    }
    SetConfig(buffer.GetDataPtr(), buffer.ByteSize(), status);
}

// ____________________________________________________________________________
void SessionOptions::SetConfig(const void* proto, size_t proto_len,
                               Status* status) {
    if (status == NULL) {
        status = &default_status_;
    }
    TF_SetConfig(c_sessionoptions_.get(), proto, proto_len,
                 status->c_status_.get());
    status->CheckTF_OK();
}

// -----------------------------------------------------------------------------
//                                    Session
// -----------------------------------------------------------------------------
// ____________________________________________________________________________
Session::Session() {}

// ____________________________________________________________________________
Session::Session(Graph* graph, const SessionOptions& opts, Status* status) {
    if (status == NULL) {
        status = &default_status_;
    }
    c_session_ = boost::shared_ptr<TF_Session>(
        TF_NewSession(graph->c_graph_.get(), opts.c_sessionoptions_.get(),
                      status->c_status_.get()),
        TfDestructor);
    graph_ = *graph;
    status->CheckTF_OK();
}

// ____________________________________________________________________________
/*Session::LoadFromSavedModel(Graph& graph, const SessionOptions& opts,
        Status* status, const Buffer& run_options,
        const std::string& export_dir, tags) {
    if (initialized_) {
        TF_CloseSession(session_, status);
        status-> CheckTF_OK();
        TF_DeleteSession(session_, status);
        status-> CheckTF_OK();
    }
    session_ = TF_LoadSessionFromSavedModel(opts.sessionoptions_,
                                            run_options.buffer_,
                                            export_dir.c_str(),
                                            tags, tags.size(),
                                            graph.graph_,
                                            meta_graph_def.buffer_,
                                            status->status_.get());
    initialized_ = status->CheckTF_OK();
}*/

// ____________________________________________________________________________
void Session::Run(const std::vector<OpAndTensor>& feed,
                  std::vector<OpAndTensor>* fetch, Buffer* run_options,
                  Buffer* run_metadata, Status* status) {
    if (!Initialized()) {
        std::cout << "Session not initialized" << std::endl;
        return;
    }
    if (status == NULL) {
        status = &default_status_;
    }
    // prepare the input c_api_operations
    std::vector<Output> input_nodes(feed.size());
    for (size_t i = 0; i < feed.size(); ++i) {
        TF_Operation* op = graph_.GetOperationByName((feed[i]).op_name);
        if (op == NULL) {
            status->Set(TF_INVALID_ARGUMENT, "Op not found.");
            return;
        }

        TF_Output output = {op, static_cast<int>(i)};
        input_nodes[i] = output;
    }
    // prepare the input c_api_tensors
    std::vector<TF_Tensor*> input_values(feed.size());
    for (size_t i = 0; i < feed.size(); ++i) {
        input_values[i] = (feed[i]).tensor->c_tensor_.get();
    }
    // prepare the output c_api_operations
    std::vector<Output> output_nodes(fetch->size());
    for (size_t i = 0; i < fetch->size(); ++i) {
        TF_Operation* op;
        op = graph_.GetOperationByName(fetch->at(i).op_name);
        if (op == NULL) {
            status->Set(TF_INVALID_ARGUMENT, "Op not found.");
            return;
        }
        TF_Output out = {op, 0};
        output_nodes[i] = out;
    }
    // prepare the output c_api_tensors
    std::vector<TF_Tensor*> output_values(fetch->size());
    for (size_t i = 0; i < fetch->size(); ++i) {
        output_values[i] = fetch->at(i).tensor->c_tensor_.get();
    }
    TF_Buffer* tf_runopts =
        run_options == NULL ? NULL : run_options->c_buffer_.get();
    TF_Buffer* tf_runmeta =
        run_metadata == NULL ? NULL : run_metadata->c_buffer_.get();
    TF_SessionRun(c_session_.get(), tf_runopts, &input_nodes[0],
                  &input_values[0], input_nodes.size(), &output_nodes[0],
                  &output_values[0], output_nodes.size(), NULL, 0, tf_runmeta,
                  status->c_status_.get());

    if (!status->CheckTF_OK()) {
        return;
    }
    // ToDo(phil): verify that this is no leak.
    // Reallocations may happen, so update the pointers.
    for (size_t i = 0; i < fetch->size(); ++i) {
        (*fetch)[i].tensor->c_tensor_ =
            boost::shared_ptr<TF_Tensor>(output_values[i], TF_DeleteTensor);
    }
}

// ____________________________________________________________________________
void Session::Close(Status* status) {
    if (c_session_.get() == NULL) {
        return;
    }
    if (status == NULL) {
        status = &default_status_;
    }
    TF_CloseSession(c_session_.get(), status->c_status_.get());
    status->CheckTF_OK();
}

}  // namespace tensorflow
