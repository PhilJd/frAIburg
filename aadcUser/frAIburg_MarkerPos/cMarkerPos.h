/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: kuckal  $  $Date:: 2017-11-06 13:23:53#$ $Rev:: 69825    $
**********************************************************************/

#ifndef _MARKERPOS_H_
#define _MERKERPOS_H_

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace adtf;

#define RAD2DEG static_cast<tFloat32>(180.0/M_PI)
#define DEG2RAD static_cast<tFloat32>(M_PI/180.0)

#define OID_ADTF_FILTER_DEF                "adtf.frAIburg.MarkerPos"
#define ADTF_FILTER_DESC                   "frAIburg Marker Positioning"
#define ADTF_FILTER_VERSION_SUB_NAME       "Marker Positioning"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL   "frAIburg MarkerPos"
#define ADTF_FILTER_VERSION_STRING         "1.2.0"
#define ADTF_FILTER_VERSION_Major          1
#define ADTF_FILTER_VERSION_Minor          2
#define ADTF_FILTER_VERSION_Build          0
#define ADTF_FILTER_VERSION_LABEL          "Optical Marker Positioning for ADTF."
#define ADTF_CATEGORY OBJCAT_DataFilter

/*! Storage structure for the road sign data */
typedef struct _roadSign
    {
        /*! road sign */
        tInt16 u16Id;
        
        /*! init sign */
        tBool bInit; 

        /*! location */
        tFloat32 f32X;
        tFloat32 f32Y;

        /*! sign search radius */
        tFloat32 f32Radius;

        /*! direction (heading) of the road sign */
        tFloat32 f32Direction;

        tInt u16Cnt;

        tTimeStamp u32ticks;/*! measurement ticks*/

    } roadSign;

class cMarkerPos : public adtf::cFilter
{
    /*! This macro does all the plugin setup stuff
	* Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
	*/
	ADTF_FILTER_VERSION(
		OID_ADTF_FILTER_DEF,
		ADTF_FILTER_DESC,
		ADTF_CATEGORY,
		ADTF_FILTER_VERSION_SUB_NAME,
		ADTF_FILTER_VERSION_Major,
		ADTF_FILTER_VERSION_Minor,
		ADTF_FILTER_VERSION_Build,
		ADTF_FILTER_VERSION_LABEL);

    public:
        cMarkerPos(const tChar* __info);
        virtual ~cMarkerPos();

    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
        tResult PropertyChanged(const char* strProperty);

    private:
        /*! creates all the output pins
        @param __exception the exception pointer
        */
        tResult CreateOutputPins(__exception = NULL);

        /*! creates all the input pins
        @param __exception the exception pointer
        */
        tResult CreateInputPins(__exception = NULL);

        /*! processes the incoming InertialMeasUnit sample
        @param pMediaSample the incoming sample
        */
        tResult ProcessInerMeasUnitSample(IMediaSample* pMediaSampleIn);

        /*! processes the incoming RoadSignExt sample
        @param pMediaSample the incoming sample
        */
        tResult ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn);

        /*! reads and processes ADTF properties
        */
        tResult ReadProperties(const tChar* strPropertyName);

        /*! loads RoadSign configuration data
        */
        tResult LoadConfiguration();

        /*! function to transmit calculated position */
        tResult sendPositionStruct(const tTimeStamp &timeOfFix, const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius,
                                   const tFloat32 &f32Heading, const tFloat32 &f32Speed);

        /*! support functions */
        tTimeStamp GetTime();

        tFloat32 mod(tFloat32 x, tFloat32 y);
        tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);
        tFloat32 angleDiff(tFloat32 angle1, tFloat32 angle2);

        void transformRearPointToFront(float* x_res, float* y_res);

        /*! Input pin for the wheel speed data */
        cInputPin m_oInputSpeed;

        /*! Input pin for the road sign Ext data */
        cInputPin m_oInputRoadSignExt;

        /*! Input pin for the inertial measurement data */
        cInputPin m_oInputInerMeasUnit;

        /*! Output pin for the position data */
        cOutputPin m_oPinPosition;

        /*!.whether prints has to made to the console */
        tBool m_bDebugModeEnabled;

        /*! the critical section for the on pin events */
        cCriticalSection m_critSecOnPinEvent;
        cCriticalSection m_oSendPositionCritSection;

        /*! Descriptor */
        cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;
        /*! the id for the i16Identifier of the media description for output pin */
        tBufferID m_szIDRoadSignExtI16Identifier;
        /*! the id for the f32Imagesize of the media description for output pin */
        tBufferID m_szIDRoadSignExtF32Imagesize;
        /*! the id for the af32TVec of the media description for output pin */
        tBufferID m_szIDRoadSignExtAf32TVec;
        /*! the id for the af32RVec of the media description for output pin */
        tBufferID m_szIDRoadSignExtAf32RVec;
        /*! indicates if bufferIDs were set */
        tBool m_bIDsRoadSignExtSet;

        /*! media description for the input pin speed */
        cObjectPtr<IMediaTypeDescription> m_pDescMeasSpeed;
        /*! the id for the f32value of the media description for input pin for the speed */
        tBufferID m_buIDMeasSpeedF32Value;
        /*! the id for the arduino time stamp of the media description for input pin for thespeed */
        tBufferID m_buIDMeasSpeedArduinoTimestamp;
        /*! indicates of bufferIDs were set */
        tBool m_bIDsMeasWheelSpeedSet;

         /*! Descriptor */
        cObjectPtr<IMediaTypeDescription> m_pDescPosition;
        /*! the id for the f32x of the media description for output pin */
        tBufferID m_szIDPositionF32X;
        /*! the id for the f32y of the media description for output pin */
        tBufferID m_szIDPositionF32Y;
        /*! the id for the af32radius of the media description for output pin */
        tBufferID m_szIDPositionF32Radius;
        /*! the id for the af32speed of the media description for output pin */
        tBufferID m_szIDPositionF32Speed;
        /*! the id for the af32heading of the media description for output pin */
        tBufferID m_szIDPositionF32Heading;
        /*! indicates if bufferIDs were set */
        tBool m_bIDsPositionSet;

        /*! descriptor */
        cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;
        /*! the id for the f32x of the media description for output pin */
        tBufferID m_szIDInerMeasUnitF32G_x;
        tBufferID m_szIDInerMeasUnitF32G_y;
        tBufferID m_szIDInerMeasUnitF32G_z;

        tBufferID m_szIDInerMeasUnitF32A_x;
        tBufferID m_szIDInerMeasUnitF32A_y;
        tBufferID m_szIDInerMeasUnitF32A_z;

        tBufferID m_szIDInerMeasUnitF32M_x;
        tBufferID m_szIDInerMeasUnitF32M_y;
        tBufferID m_szIDInerMeasUnitF32M_z;
        tBufferID m_szIDInerMeasUnitArduinoTimestamp;
        /*! indicates if bufferIDs were set */
        tBool m_bIDsInerMeasUnitSet;

        /*! speed estimate */
        tFloat32 m_f32Speed;
        tFloat32 m_f32SpeedScale; /*! speed scalefactor */

        tUInt32 m_ui32ArduinoTimestamp;

        /*! camera offset parameters */
        tFloat32 m_f32CameraOffsetLat;
        tFloat32 m_f32CameraOffsetLon;

        /*! currently processed road-sign */
        tInt16 m_i16ID;
        tFloat32 m_f32MarkerSize;

        Mat m_Tvec; /*! translation vector */
        Mat m_Rvec; /*! rotation vector */

        tTimeStamp m_ticks;

        /*! EKF variables */
        Mat m_state; /*! filter state {X} */
        Mat m_errorCov; /*! error covariance matrix {P} */
        Mat m_processCov; /*! process covariance matrix {Q} */
        Mat m_transitionMatrix; /*! state transition matrix {F} */

        tBool m_isInitialized; /*! initialization state of the filter */

        /*! storage for the roadsign data */
        vector<roadSign> m_roadSigns;

        tInt m_ui32Cnt;

        FILE*m_log; // debug file

};

#endif // _MARKERPOS_H_
