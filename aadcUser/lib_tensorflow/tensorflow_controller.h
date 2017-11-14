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

#ifndef AADCUSER_TENSORFLOW_TENSORFLOW_CONTROLLER_H_
#define AADCUSER_TENSORFLOW_TENSORFLOW_CONTROLLER_H_

#include <string>
#include <vector>
#include <map>
#include "tensorflow_cpp_wrapper.h"

namespace tf = tensorflow;


/*! A Controller for tensorflow that stores a session and a graph, simplifying
 *  repeated inference.
 */
class TensorflowController {
 public:
    /*! Creates an uninitalized tensorflow controller */
    TensorflowController();

    ~TensorflowController();

    /*! Creates a tensorflow controller and loads a tf graph from
        the given file */
    explicit TensorflowController(const std::string& protobuf_path);

    /*! Creates a tensorflow controller, loads a tf graph from the given file
     *  and initializes the session with the given session opts */
    TensorflowController(const std::string& protobuf_path,
                         const tf::SessionOptions & opts);
    
    /*! Loads a tf graph from the given (binary) protobuf file and activates
     *  the controller. */
    bool LoadFromProtobuf(const std::string& protobuf_path,
                          const tf::SessionOptions& opts =
                              tf::SessionOptions());

    /*! Executes a forward pass */
    bool ForwardPass(const std::vector<tf::OpAndTensor>& feed,
                     std::vector<tf::OpAndTensor>* fetch);
    
    /*! If not already activated, loads the previously loaded graph.
     *  This allocates memory on the GPU. */
    void ActivateSession();

    /*! If a session is active, overwrites this session with an empty one,
     *  such that the GPU memory is released. */
    void DeactivateSession();

    /*! Reads a label file and constructs a map[line_number] = class_name */
    void ReadLabels(const std::string& file_name);

    /*! Closes the tensorflow session. */
    void CloseSession();

    /*! Maps a class id to a label string. */
    std::map<int, std::string> id_to_label_;

    /*! Returns true if graph and session were loaded/created successfully */
    bool Ready() const { return ready_; }

 
 private:
    // This order must be preserved! The Session must always be destroyed
    // before the graph!! 
    tf::Session session_;
    tf::Graph graph_;
    tf::SessionOptions last_session_options_;
    bool ready_;
};

#endif  //  AADCUSER_TENSORFLOW_TENSORFLOW_CONTROLLER_H_
