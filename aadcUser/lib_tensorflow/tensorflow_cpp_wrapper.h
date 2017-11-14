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

#ifndef AADCUSER_TENSORFLOW_TENSORFLOW_CPP_WRAPPER_H_
#define AADCUSER_TENSORFLOW_TENSORFLOW_CPP_WRAPPER_H_

#include <string>
#include <vector>
#include <stdexcept>
#include <typeinfo>
#include <tensorflow/c/c_api.h>
#include <iostream>
// #if __cplusplus >= 201103L
//     #include <memory>
//     using std::shared_ptr;
// #else
#include <boost/shared_ptr.hpp>
using boost::shared_ptr;
// #endif


namespace tensorflow {

/*! @defgroup tensorflow_cpp_wrapper
*  @{
* This class wraps the tensorflow c api for cpp. This is necessary, as the
* tensorflow team currently only distributes a library file for this header.
* To use the cpp api directly we'd need to build it ourself, requiring an
* installation of bazel. This would also not allow us to use previously
* allocated memory.
*
* This wrapper handles memory allocation/deallocation of tensorflow objects
* and defines helper functions, e.g. to read a binary file into a buffer.
*
* In general, each `MyClass` is a wrapper for `TF_MyClass` in the cpp api
* and contains a member that is a TF_MyClass pointer. This member is only
* accessible to friend classes within this wrapper, as only the wrapper
* accesses the c_api.
*/

/*! Returns a string describing version information of TensorFlow. */
std::string version();

typedef TF_DataType DataType;
typedef TF_Output Output;  // an operation and the index within the operation
typedef TF_Input Input;  // an operation and the index within the operation

/*! Returns the byte size, i.e. sizeof() of the given tf dtype.*/
size_t DataTypeSize(DataType dtype);


class Session;  // forward declaration
class Graph;  // forward declaration
class Tensor;  // forward declaration
void TfDestructor(TF_Session* session);


/*! An OpAndTensor instance stores an operation name and a reference to a
 * tensor. This allows to specify to which operation in the graph a tensor
 * should be fed (resembling the feed_dict in python) or from which operation 
 * a tensor should be fetched.
 * 
 */

struct OpAndTensor {
    /*! The name of the operation the tensor should be fed to. */
    std::string op_name;
    /*! Reference to the tensor. */
    Tensor* tensor;

    // constructor
    OpAndTensor(const std::string& s, Tensor* t) : op_name(s), tensor(t) {}
};

// ToDo(phil) rename all TF_Object mmebers to tf_objects_

// -----------------------------------------------------------------------------
//                                    Status
// -----------------------------------------------------------------------------
class Status {
 public:
    Status();

    void Set(TF_Code code, const std::string& msg);

    TF_Code GetCode();

    std::string GetMessage();

    bool CheckTF_OK(bool verbose = true);

 private:
    friend class Session;
    friend class Graph;
    friend class Tensor;
    friend class SessionOptions;
    friend void TfDestructor(TF_Session* session);

    /* The TF_api object this class encapsulates. */
    boost::shared_ptr<TF_Status> c_status_;
};

// -----------------------------------------------------------------------------
//                                    Tensor
// -----------------------------------------------------------------------------
/*! The Tensor class is a wrapper for the c_api TF_Tensor */

class Tensor {
 public:
    /*! Constructor. Returns an empty tensor with shape (0) and
     * dtype=TF_FLOAT32. Useful for error handling. */
    Tensor();

    /*! Constructor that allocates memory using the tf Allocator. */
    Tensor(TF_DataType dtype, const std::vector<int64_t>& shape);

    /* Constructor that uses existing memory for a tensor. Must be row-major!
     * Currently there's no reason to specify the type of data, as cv::Mat is
     * always uchar, same holds for ADTF Media . So we're using void* here. */
    Tensor(TF_DataType dtype, const std::vector<int64_t>& shape, void* data,
           void (*deallocator)(void* data, size_t len, void* arg) = NULL,
           void* deallocator_arg = NULL);

    /*! Access tensor element. Currently only supports direct indexing. */
    template<typename T>
    inline T& get(const std::vector<size_t>& index, bool unsafe) const {
        if (!unsafe) DtypeCheck<T>();
        T* data = reinterpret_cast<T*>(TF_TensorData(c_tensor_.get()));
        if (shape_.size() != index.size()) {
            //ToDo(Phil) : add tensorslices
            throw std::invalid_argument("Currently only supports direct"
                                        "indexing, no slicing.");
        }
        // compute row-major index as: sum_{k=0}^n( ( Pi_{l=k+1}^d(N_l)) * n_k )
        size_t p = 0;
        for (size_t dim = 0; dim < shape_.size(); ++dim) {
            if (!unsafe && (index[dim] > shape_[dim] || index[dim] < 0)) {
                // ToDo(phil) Better error message, maybe separate
                // bounding check function
                throw std::out_of_range("Index is out of bounds");
            }
            p = index[dim] + p * shape_[dim];
        }
        return data[p];
    }

    /* Convenience function for accessing a 1D-tensor. */
    template<typename T> inline T& get(size_t a, bool unsafe) const {
       size_t idx[] = { a };
       return get<T>(std::vector<size_t>(idx, idx + 1), unsafe);
    }

    /* Convenience function for accessing a 2D-tensor. */
    template<typename T> inline T& get(size_t a, size_t b, bool unsafe) const {
       size_t idx[] = { a, b };
       return get<T>(std::vector<size_t>(idx, idx + 2), unsafe);
    }

    /* Convenience function for accessing a 3D-tensor. */
    template<typename T>
    inline T& get(size_t a, size_t b, size_t c, bool unsafe) const {
       size_t idx[] = { a, b, c };
       return get<T>(std::vector<size_t>(idx, idx + 3), unsafe);
    }

    /* Convenience function for accessing a 4D-tensor. */
    template<typename T>
    inline T& get(size_t a, size_t b, size_t c, size_t d, bool unsafe) const {
       size_t idx[] = { a, b, c, d };
       return get<T>(std::vector<size_t>(idx, idx + 4), unsafe);
    }

    /*! Returns the tf dtype of the tensor. */
    TF_DataType Dtype() const;

    /*! Returns the number of dimensions of the tensor. */
    int NumDims() const;

    /*! Returns the shape of the tensor. */
    std::vector<int64_t> Shape() const;

    /*! Returns the size in bytes of one element in the tensor. */
    size_t ByteSize() const;

    /*! Returns a pointer to the data with write access. */
    template <typename T> T* GetDataPtr() {
        return reinterpret_cast<T*>(TF_TensorData(c_tensor_.get()));
    }

    /*! Returns a readonly pointer to the data. */
    template <typename T> const T* GetReadOnlyPtr() const {
        return reinterpret_cast<T*>(TF_TensorData(c_tensor_.get()));
    }

    /* Computes the length of the flattened tensor (number of elements). */
    size_t ShapeToBufferLength(const std::vector<int64_t>& shape,
                               TF_DataType dtype) const;

 private:
    friend class Session;

    /*! The TF_api object this class encapsulates. */
    boost::shared_ptr<TF_Tensor> c_tensor_;

    /*! Shape of the tensor. */
    std::vector<int64_t> shape_;

    /*! Checks at least if the byte size of the template type T and the
     *  specified TF dtype match. For certain standard types also checks
     *  equality of type.
     */
    template <typename T>
    void DtypeCheck() const {
        typedef std::invalid_argument except;
    if (Dtype() == TF_FLOAT && typeid(T) != typeid(float))
         throw except("Dtype is TF_FLOAT, pointer type must be float");
    if (Dtype() == TF_INT32 && typeid(T) != typeid(int))
         throw except("Dtype is TF_INT32, pointer type must be int");
    if (Dtype() == TF_DOUBLE && typeid(T) != typeid(double))
         throw except("Dtype is TF_DOUBLE, pointer type must be double");
    if (Dtype() == TF_UINT8 && typeid(T) != typeid(char))
         throw except("Dtype is TF_UINT8, pointer type must be char");
    if (sizeof(T) != DataTypeSize(Dtype()))
        throw except("Template Type size does not match TF DType size");
}

};


// -----------------------------------------------------------------------------
//                                    Buffer
// -----------------------------------------------------------------------------
/*! The Buffer class is a wrapper for the c_api TF_Buffer */
class Buffer {
 public:
    Buffer();

    Buffer(const std::string& protobuf_path);

    /*! Reads a serialized protobuf into the buffer.
        \param protobuf_path   [in] The file path to the desired protobuf.
     */
    void ReadFromProtobuf(const std::string& protobuf_path);

    /*! Const pointer to the stored data */
    const void* GetDataPtr() const;

    /*! Returns the size in bytes. */
    size_t ByteSize() const;

 private:
    friend class Graph;
    friend class Session;
    /* The TF_api object this class encapsulates. */
    boost::shared_ptr<TF_Buffer> c_buffer_;
    std::vector<char> data_;
};


// -----------------------------------------------------------------------------
//                                    Graph
// -----------------------------------------------------------------------------
/*! The Graph class is a wrapper for the c_api TF_Graph */
class Graph {
 public:
    Graph();

    /*! Creates and loads a graph from a given path to a binary protobuf file */
    explicit Graph(const std::string& graph_def_path, Status* status = NULL);

    /*! Imports a graph from a binary protobuffer. Returns true on success */
    bool ImportGraphDef(const std::string& graph_def_path,
                        Status* status = NULL);

    /*! Imports a graph from a binary protobuffer. Returns true on success */
    bool ImportGraphDef(const Buffer& graph_def, Status* status = NULL);

    /*! Prints all Operations in the loaded graph. */
    void PrintGraphOps() const;

    /*! Returns true if a graph was previosuly successfully loaded. */
    bool Initialized() const { return c_graph_.get() != NULL; }

    // Todo(phil) remove
    TF_Operation* GetOperationByName(const std::string& op_name) const { return OperationByName(op_name); }

 private:
    friend class Session;

    TF_Operation* OperationByName(const std::string& op_name) const;

    /*! The TF_api object this class encapsulates. */
    boost::shared_ptr<TF_Graph> c_graph_;

    /*! Options used for importing. */
    boost::shared_ptr<TF_ImportGraphDefOptions> c_import_options_;

    /*! Default status used if no user-specified status is available. Needed
        to internally check if function call on tf api was successful. */
    Status default_status_;

    /*! Default buffer, used when graph is initialized from graph def path. */
    Buffer default_buffer_;
};

// -----------------------------------------------------------------------------
//                                    SessionOptions
// -----------------------------------------------------------------------------
/*! The SessionOptions class is a wrapper for the c_api TF_SessionOptions */
class SessionOptions {
 public:
    SessionOptions();

    /*! Set the target in SessionOptions.options. target can be empty,
     *  a single entry, or a comma separated list of entries.
     *  Each entry is in one of the following formats :
     * "local"       ip:port        host:port */
    void SetTarget(const std::string& target);

    /*! Sets a config from a binary prototxt. The easiest way to generate such
     *  a file is via python, e.g.:
     *  <pre><code> 
     *  config = tf.ConfigProto()
     *  config.gpu_options.allow_growth = True
     *  serialized = config.SerializeToString()
     *  with open("session_config.pb", 'wb') as f:
     *      f.write(serialized)
     *  </pre></code>
     */
    void SetConfig(const Buffer& buffer, Status* status = NULL);

    void SetConfig(const void* proto, size_t proto_len, Status* status = NULL);

 private:
    friend class Session;
    /*! The TF_api object this class encapsulates. */
    boost::shared_ptr<TF_SessionOptions> c_sessionoptions_;

    /*! used to internally check if TF function was successful if
     *  no user-specified status is available */
    Status default_status_;
};

// -----------------------------------------------------------------------------
//                                    Session
// -----------------------------------------------------------------------------
/*! The Session class is a wrapper for the c_api TF_Session */
class Session {
 public:
    Session();

    Session(Graph* graph, const SessionOptions& opts, Status* status = NULL);

    // LoadFromSavedModel();

    /*! Executes a forward pass using feed as input and fetches given output */
    void Run(const std::vector<OpAndTensor>& feed,
             std::vector<OpAndTensor>* fetch,
             Buffer* run_options = NULL,
             Buffer* run_metadata = NULL,
             Status* status = NULL);

    /*! Returns true if the session was created successfully */
    bool Initialized() const { return c_session_.get() != NULL; }

    // ToDo (phil) make private again
    /*! The TF_api object this class encapsulates. */
    boost::shared_ptr<TF_Session> c_session_;

    /*! Close a tf session and release the device */
    void Close(Status* status = NULL);

 private:

    /*! Closes and Destroys a Session. */


    /*! The graph associated with this session. Must outlive the session. */
    Graph graph_;

    // used to internally check if TF function was successful if
    // no user-specified status is available
    Status default_status_;
};

}  // namespace tensorflow

#endif  // AADCUSER_TENSORFLOW_TENSORFLOW_CPP_WRAPPER_H_

/*!
*@}
*/
