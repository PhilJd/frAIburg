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
// Ultrasonic ring buffer with clibration based on CircularBuffer

#ifndef AADCUSER_US_TO_MAP_US_CAL_BUF_H_
#define AADCUSER_US_TO_MAP_US_CAL_BUF_H_

#include <string>
#include "stdafx.h"
#include "circular_buffer.hpp"
#include "xml_helper.hpp"
// #include <boost/regex.hpp>
#include <boost/algorithm/string/replace.hpp>

// Circular buffer with clibration
template <typename T>
class USCalibratedRingBuffer : public frAIburg::utils::CircularBuffer<T> {
 public:
    USCalibratedRingBuffer(
        const char *calibration_file, const char *us_traget, int capacity = 0,
        frAIburg::utils::CircularBufferEventListener *l = NULL, int id = -1,
        T *min_calibrated_add_limit = NULL, T *max_calibrated_add_limit = NULL)
        : frAIburg::utils::CircularBuffer<T>(capacity, l, id),
          config_ok_(false) {
        //replace all numberic value
        us_name_ = us_traget;
        //TODO remove all alpha numeric character
        // us_name_ = boost::regex_replace(str,
        //               boost::regex("[:alnum:]"),// alpha numeric character.
        //               "");
        BOOST_FOREACH(char ch, std::string("0123456789"))
        {
          string s; s.push_back(ch);
          boost::replace_all(us_name_, s, "");
        }

        // LOG_ERROR(us_name_.c_str());
        // set lim fot adding vales
        if (max_calibrated_add_limit) {
            max_add_limit_ = *max_calibrated_add_limit;
            max_add_val_not_used_ = false;
        } else {
            max_add_val_not_used_ = true;
            max_add_limit_ = 0;
        }

        if (min_calibrated_add_limit) {
            min_add_limit_ = *min_calibrated_add_limit;
            min_add_val_not_used_ = false;
        } else {
            min_add_val_not_used_ = true;
            min_add_limit_ = 0;
        }

        // the the values form the xml config
        frAIburg::utils::XMLHelper config;
        if (config.ReadNameValue(calibration_file, "sensor", us_name_.c_str())) {
            if (config.GetValue<float>("offset_meter_x")) {
                trans_x_ = *(config.GetValue<float>("offset_meter_x"));
                trans_y_ = *(config.GetValue<float>("offset_meter_y"));
                rotation_rad_ = *(config.GetValue<float>("rotation_degree")) *
                                (M_PI / 180.);
                calibration_bias_ = *(config.GetValue<T>("calibration_bias"));
                calibration_factor_ =
                    *(config.GetValue<T>("calibration_factor"));
                field_of_view_angle_rad_ =
                    *(config.GetValue<T>("field_of_view_angle_degree")) *
                    (M_PI / 180.);
                ;
                config_ok_ = true;
            }
        } else {
            LOG_ERROR(
                "USCalibratedRingBuffer xml calibration read error, set the"
                "dynamic pin name to the aadc arduino communication block us "
                "pin name");
        }
    }

    /*! add data to ring buffer if in limits*/
    bool PushBackCalibrate(T data) {
        T data_calibrated = data * calibration_factor_ + calibration_bias_;
        if (config_ok_) {
            if ((min_add_val_not_used_ || data_calibrated >= min_add_limit_) &&
                (max_add_val_not_used_ || data_calibrated <= max_add_limit_)) {
                // add calibrated date to the parent ring buffer
                // if data is in min max limits
                this->PushBack(data_calibrated);
                return true;
            }
        }
        return false;
    }

    bool IsConfigOk() { return config_ok_; }

    std::string us_name_;
    // orientation of the us to the local car frame
    float trans_x_;
    float trans_y_;
    float rotation_rad_;  // roation to the local car frame
    float field_of_view_angle_rad_;

 private:
    T calibration_bias_;
    T calibration_factor_;
    T min_add_limit_;
    T max_add_limit_;
    bool min_add_val_not_used_;
    bool max_add_val_not_used_;
    bool config_ok_;
};

#endif  // AADCUSER_US_TO_MAP_US_CAL_BUF_H_
