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
//
//  utils_circular_buffer.hpp
//  Circular Buffer based on boost circular_buffer
//     addition:  - observer pattern for event full and cycle compleat
//                - max min mean
//  EXAMPLE:
//          class TestCircularBufferEventListener
//                : public CircularBufferEventListener
//          {
//          public:
//            void EventBufferFilled(int id) const {
//              std::cout << "EventBufferFilled: "<<std::endl;
//            }
//            void EventCycleCompleat(int id) const{
//              std::cout << "EventCycleCompleat"<<std::endl;
//            }
//          };
//          TestCircularBufferEventListener l;
//          CircularBuffer<float> cb(3,&l);
//          std::cout << "--- PushBack 1,2,3 --- "<<std::endl;
//          cb.PushBack(1.);cb.PushBack(2.);cb.PushBack(3.);
//          std::cout << "size: "<<cb.Size()<<std::endl;
//          std::cout << "mean: "<<cb.Mean()<<std::endl;
//          std::cout << "max: "<<cb.Max()<<std::endl;
//          std::cout << "min: "<<cb.Min()<<std::endl;
//  Created by Markus on 05.09.17.
//

#ifndef AADCUSER_CIRCULAR_BUFFER_H_
#define AADCUSER_CIRCULAR_BUFFER_H_

#include <boost/circular_buffer.hpp>
#include <numeric>

namespace frAIburg {
namespace utils {

/*CircularBufferEventListener abstract methons are called by CircularBuffer */
class CircularBufferEventListener {
 public:
    CircularBufferEventListener() {}
    // id if event list is used with multiple CircularBuffers
    /*EventBufferFilled is called by CircularBuffer,
      if first filled since start or clear*/
    virtual void EventBufferFilled(int id) = 0;
    /*EventCycleCompleat called by CircularBuffer, if a cycle is completed,
      after capacity size elemetns are added  */
    virtual void EventCycleCompleat(int id) = 0;
};

template <typename T>
class CircularBuffer {
 public:
    /* \note if EventListener is used ensure that l exist while CircularBuffer*/
    CircularBuffer(int capacity = 0, CircularBufferEventListener *l = NULL,
                   int id = -1)
        : id_(id),
          add_cnt_event_(0U),
          max_min_set_(false),
          buffer_filled_(false),
          max_(T()),
          min_(T()),
          buffer_(capacity),
          listener_(l) {}

    void PushBack(T data) {
        if (buffer_.capacity()) {
            buffer_.push_back(data);
            // max min
            if (!max_min_set_) {
                max_min_set_ = true;
                max_ = data;
                min_ = data;
            }
            if (max_ < data) {
                max_ = data;
            }
            if (min_ > data) {
                min_ = data;
            }

            // events
            if (listener_) {
                ++add_cnt_event_;
                if (!buffer_filled_ && Size() == Capacity()) {
                    // buffer is full since start or clear
                    buffer_filled_ = true;
                    listener_->EventBufferFilled(id_);
                }
                if (add_cnt_event_ == Capacity()) {
                    add_cnt_event_ = 0;
                    listener_->EventCycleCompleat(id_);
                }
            }
        }
    }

    /*! comupte the mean */
    T Mean() const {
        if (!buffer_.empty())
            // 0.0 to to be not int
            return std::accumulate(buffer_.begin(), buffer_.end(), T()) /
                   Size();
        else
            return T();
    }

    /*! Clear buffer, max, min */
    void Clear() {
        buffer_filled_ = false;
        ResetMinMax();
        buffer_.clear();
    }

    void ResetMinMax() {
        max_min_set_ = false;
        max_ = T();
        min_ = T();
    }

    T Max() { return max_; }
    T Min() { return min_; }

    unsigned Size() const { return buffer_.size(); }

    unsigned Capacity() const { return buffer_.capacity(); }

 private:
    int id_;
    // seperate counter for event buffer full
    unsigned add_cnt_event_;
    bool max_min_set_;
    bool buffer_filled_;
    T max_;
    T min_;
    boost::circular_buffer<T> buffer_;
    CircularBufferEventListener *listener_;
};
}
} /*NAME SPACE frAIburg,utils*/

#endif  // AADCUSER_CIRCULAR_BUFFER_H_
