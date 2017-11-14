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
//  xml_helper.hpp
//  XMLHelper
// - read a xml file and save vlaues in a unordered_map
// - all variable strings are trimmed
// - format:  <configuration/> and <value/> is fixed
//            <config_second/> , can cahnge for a different module
//  file.xml:
// <configuration datetime="02-Sep-2017" target="car_target_">
//   <config_second description="description_" target="target_second">
//       <value name="radii0" value="547.8809"/>
//   </config_second>
//   ... magnetometer example:
//    <sensor description="location-freiburg-03-Sep-2017" target="magnetometer">
//       <value name="radii0" value="547.8809"/>
//    </sensor>
// </configuration>
//

#ifndef AADCUSER_UTILS_XML_HELPER_H_
#define AADCUSER_UTILS_XML_HELPER_H_

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>
#include <string>
#include "adtf_log_macros.h"

namespace frAIburg {
namespace utils {

/*! helper to read a frAIburg xml configuration file*/
class XMLHelper {
 public:
    std::string car_target_;
    std::string config_second_description_;

    /*! store the wanted config in unordered map, get values with GetValue
     *  \param file_name xml file name, in adtf use the absolute path
     *  \param config_second name of the module config, is trimmed
     *  \param target_second target string at second level, is trimmed
     *   EXAMPLE:
           file.xml:
                 <configuration datetime="02-Sep-2017" target="car1">
                   <sensor description="freiburg" target="camera basler">
                     <value name="offset" value="547.8809"/>
                   </sensor>
                 </configuration>
            ->  file_name=  "file.xml"; config_second = "sensor";
                            target_second="camera basler";
           .cpp:
                 XMLHelper config;
                 if(config.ReadNameValue("file.xml","sensor","camera basler")){
                     float value=*(config.GetValue<float>("offset"));
                     std::cout<<"car target: "<<config.car_target_<<std::endl;
                 }
     */
    bool ReadNameValue(const char* file_name, const char* config_second,
                       const char* target_second) {
        std::string config_second_trimmed(config_second);
        boost::algorithm::trim(config_second_trimmed);
        std::string target_second_trimmed(target_second);
        boost::algorithm::trim(target_second_trimmed);

        return ReadfrAIburgXML(file_name, config_second_trimmed.c_str(),
                               target_second_trimmed.c_str(),
                               map_name_val_,  // return map
                               &car_target_, &config_second_description_);
    }

    /*! get the configuration value of the Read target
     *  \param value_name value name in the xml config, is trimmed
     *  \note call ReadNameValue first to read the config
     *   EXAMPLE for <value name="radii0" value="547.8809"/>:
           boost::optional<float> p = xml_helper.GetValue<float>("radii0");
           if (p) float value = *p;
     */
    template <typename tdata>
    boost::optional<tdata> GetValue(const char* value_name) {
        if (!value_name || !map_name_val_.count(value_name)) {
            LOG_ERROR_PRINTF(
                "ReadfrAIburgXML: GetValue %s"
                "NOT found",
                value_name);
            return boost::none;
        } else {
            std::string value_name_trimmed(value_name);
            boost::algorithm::trim(value_name_trimmed);
            return boost::lexical_cast<tdata>(
                map_name_val_.at(value_name_trimmed));
        }
    }

 private:
    // map with info for the sensror values and name:
    //<value name="radii0" value="547.8809"/>
    boost::unordered_map<std::string, std::string> map_name_val_;

    static bool ReadfrAIburgXML(
        const char* file_name, const char* config_second,
        const char* target_second,
        boost::unordered_map<std::string, std::string>& ret_map_name_val,
        std::string* ret_car_target = NULL,
        std::string* ret_sensor_description = NULL) {
        // read  the frAIburg xml with boost ptree
        // adding all target_second value and names to a unsorted map
        namespace bptree = boost::property_tree;

        bptree::ptree pt;

        if (!file_name || !config_second || !target_second) {
            LOG_ERROR_PRINTF("ReadfrAIburgXML: NULL pointer");
            return false;
        }

        // read config from xml file
        if (!ReadPTree(file_name, pt)) return false;

        //<calibration datetime="02-Sep-2017" target="car1">
        if (ret_car_target) {
            *ret_car_target =
                pt.get<std::string>("configuration.<xmlattr>.target");
            //  *ret_car_target=pt.get<std::string>("calibration.<xmlattr>.datetime");
        }

        // find config_second with the target_second
        // then add all vals and names to the hash map
        BOOST_FOREACH (bptree::ptree::value_type const& s,
                       pt.get_child("configuration")) {
            std::string target =
                s.second.get<std::string>("<xmlattr>.target", "empty");

            // find sensor target defined like:
            // <config_second description=" " target="target_second">
            if (s.first == config_second && target == target_second) {
                // check for if target is sensor_target
                // direclty with pt.get_child("calibration.sensor") cant check
                // the target
                if (ret_sensor_description) {
                    *ret_sensor_description = s.second.get<std::string>(
                        "<xmlattr>.description", "empty");
                }
                // loop over all names an vals for the sensor target:
                // set like: <value name="radii1" value="410.2406"/>
                bool found = false;
                BOOST_FOREACH (bptree::ptree::value_type const& child_v,
                               s.second) {
                    if (child_v.first == "value") {
                        std::string name = child_v.second.get<std::string>(
                            "<xmlattr>.name", "empty");
                        std::string str_val = child_v.second.get<std::string>(
                            "<xmlattr>.value", "0");
                        boost::algorithm::trim(str_val);
                        boost::algorithm::trim(name);
                        ret_map_name_val[name] = str_val;
                        if (!found) found = true;

                        //  LOG_INFO_PRINTF("ReadfrAIburgXML:"
                        //               "target: %s name: %s val: %s",
                        //               target_second,name.c_str(),str_val.c_str());
                    }
                }
                if (!found) {
                    LOG_ERROR_PRINTF(
                        "ReadfrAIburgXML: "
                        "second target %s no <value/> found",
                        target_second);
                }
                return found;
            }
        }

        LOG_ERROR_PRINTF(
            "ReadfrAIburgXML: second target %s "
            "not found",
            target_second);
        return false;
    }

    static bool ReadPTree(const char* file_name,
                          boost::property_tree::ptree& ret_ptree) {
        // try and catch because boost::filesystem::exists not working with
        // relativ path in adtf filter mask
        try {
            read_xml(file_name, ret_ptree);  // load tree
        } catch (std::exception const& e) {
            LOG_ERROR_PRINTF(
                "ReadfrAIburgXML: file %s not found"
                " or not a .xm or relativ path",
                file_name);
            return false;
        }
        return true;
    }
};
}
} /*NAME SPACE frAIburg,utils*/

#endif  //  AADCUSER_UTILS_XML_HELPER_H_
