/*****************************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must
   display the following acknowledgement: “This product includes software
   developed by the Audi AG and its contributors for Audi Autonomous Driving
   Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: spiesra,   $Date:: 2017-05-16 10:06:17#$ $Rev:: 63289   $
Author: Phil
**********************************************************************/

/*! get the option name string from a option id
*   \param option option it as integer
*   \return option as string
*/
cString OptionName(int Option)
{
    cString OptionName;

    switch(Option)
    {
    case(0):
        OptionName = "RS_OPTION_COLOR_BACKLIGHT_COMPENSATION";
        break;
    case(1):
        OptionName = "RS_OPTION_COLOR_BRIGHTNESS";
        break;
    case(2):
        OptionName = "RS_OPTION_COLOR_CONTRAST";
        break;
    case(3):
        OptionName = "RS_OPTION_COLOR_EXPOSURE";
        break;
    case(4):
        OptionName = "RS_OPTION_COLOR_GAIN";
        break;
    case(5):
        OptionName = "RS_OPTION_COLOR_GAMMA";
        break;
    case(6):
        OptionName = "RS_OPTION_COLOR_HUE";
        break;
    case(7):
        OptionName = "RS_OPTION_COLOR_SATURATION";
        break;
    case(8):
        OptionName = "RS_OPTION_COLOR_SHARPENSS";
        break;
    case(9):
        OptionName = "RS_OPTION_COLOR_WHITE_BALANCE";
        break;
    case(10):
        OptionName = "RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE";
        break;
    case(11):
        OptionName = "RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE";
        break;
    case(28):
        OptionName = "R200::RS_OPTION_R200_LR_AUTO_EYPOSURE_ENABLED";
        break;
    case(29):
        OptionName = "R200::RS_OPTION_R200_LR_GAIN";
        break;
    case(30):
        OptionName = "R200::RS_OPTION_R200_LR_EXPOSURE";
        break;
    case(31):
        OptionName = "R200::RS_OPTION_R200_EMITTER_ENABLED";
        break;
    case(32):
        OptionName = "R200::RS_OPTION_R200_DEPTH_UNITS";
        break;
    case(33):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CLAMP_MIN";
        break;
    case(34):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CLAMP_MAX";
        break;
    case(35):
        OptionName = "R200::RS_OPTION_R200_DISPARITY_MULTIPLIER";
        break;
    case(36):
        OptionName = "R200::RS_OPTION_R200_DISPARITY_SHIFT";
        break;
    case(37):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_MEAN_INTENSITY_SET_POINT";
        break;
    case(38):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_BRIGHT_RATIO_SET_POINT";
        break;
    case(39):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_KP_GAIN";
        break;
    case(40):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_KP_EXPOSURE";
        break;
    case(41):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_DARK_THRESHOLD";
        break;
    case(42):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_TOP_EDGE";
        break;
    case(43):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_BOTTOM_EDGE";
        break;
    case(44):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_LEFT_EDGE";
        break;
    case(45):
        OptionName = "R200::RS_OPTION_R200_AUTO_EXPOSURE_RIGHT_EDGE";
        break;
    case(46):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT";
        break;
    case(47):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT";
        break;
    case(48):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_MEDIAN_THRESHOLD";
        break;
    case(49):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD";
        break;
    case(50):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD";
        break;
    case(51):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD";
        break;
    case(52):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTORL_TEXTURE_DIFFERENCE_THRESHOLD";
        break;
    case(53):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD";
        break;
    case(54):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD";
        break;
    case(55):
        OptionName = "R200::RS_OPTION_R200_DEPTH_CONTROL_LR_THRESHOLD";
        break;
    case(65):
        OptionName = "RS_OPTION_FRAMES_QUEUE_SIZE";
        break;
    }
    return OptionName.GetPtr();
}
