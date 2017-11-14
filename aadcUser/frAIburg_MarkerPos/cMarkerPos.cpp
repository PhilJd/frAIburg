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
* $Author:: kuckal  $  $Date:: 2017-11-08 16:05:36#$ $Rev:: 69998    $
**********************************************************************/

#include "stdafx.h"
#include "cMarkerPos.h"

//#include <limits>

ADTF_FILTER_PLUGIN("AADC MarkerPos", OID_ADTF_FILTER_DEF, cMarkerPos)

/*! Configuration parameters */

// road sign distance and pose estimation
#define MP_LIMIT_ALPHA    60.0 // [degrees]
#define MP_LIMIT_YAW      15.0 // [degrees]
#define MP_LIMIT_YAW_INIT  8.0 // [degrees]
#define MP_LIMIT_DISTANCE  2.5 // [m]

// process covariances
#define MP_PROCESS_X                    1e-3
#define MP_PROCESS_Y                    1e-3
#define MP_PROCESS_HEADING              3e-4
#define MP_PROCESS_HEADING_DRIFT        5e-8
#define MP_PROCESS_SPEED                2e-3
#define MP_PROCESS_SPEED_SCALE          1e-6

// initial covariance values
#define MP_PROCESS_INIT_X               10.0
#define MP_PROCESS_INIT_Y               10.0
#define MP_PROCESS_INIT_HEADING         0.55
#define MP_PROCESS_INIT_HEADING_DRIFT   0.25
#define MP_PROCESS_INIT_SPEED           1.0
#define MP_PROCESS_INIT_SPEED_SCALE     0.5

// measurement covariances
#define MP_MEASUREMENT_X                0.5
#define MP_MEASUREMENT_Y                0.5
#define MP_MEASUREMENT_HEADING          1.0 // [radians]


#define MP_PROP_CAMERA_OFFSET_LAT "Camera Offset::Lateral"
#define MP_PROP_CAMERA_OFFSET_LON "Camera Offset::Longitudinal"

#define MP_PROP_SPEED_SCALE "Speed Scale"

cMarkerPos::cMarkerPos(const tChar* __info) : cFilter(__info)
{
    m_bDebugModeEnabled = tFalse;
    SetPropertyBool("Debug Output to Console", m_bDebugModeEnabled);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)");

    SetPropertyStr("Configuration","roadSign.xml");
    SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LAT, 0.0);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_DESCRIPTION, "Camera offset in lateral direction");

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LON, 0.26);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_DESCRIPTION, "Camera offset in longitudinal direction");

    SetPropertyFloat(MP_PROP_SPEED_SCALE, 1.0);
    SetPropertyBool(MP_PROP_SPEED_SCALE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_SPEED_SCALE NSSUBPROP_DESCRIPTION, "Initial scale value for the speed measurement");

    m_log = 0;

}

cMarkerPos::~cMarkerPos()
{
}

tResult cMarkerPos::CreateInputPins(__exception)
{
    //get the description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    //get description for sensor data pins
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);

    //get mediatype for data pins
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set the description for the speed pin
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescMeasSpeed));

    // create the speed measurement Input
    RETURN_IF_FAILED(m_oInputSpeed.Create("Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSpeed));

    // create the description for the road sign Ext pin
    tChar const * strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");
    RETURN_IF_POINTER_NULL(strDescExt);

    cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // create the extended road sign Input
    RETURN_IF_FAILED(m_oInputRoadSignExt.Create("RoadSign_ext", pTypeExt, static_cast<IPinEventSink*> (this)));

    // set the description for the extended road sign pin
    RETURN_IF_FAILED(pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSignExt));

    RETURN_IF_FAILED(RegisterPin(&m_oInputRoadSignExt));

    //get description for inertial measurement sensor data pin
    tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);

    //get mediatype for Inertial sensor data pins
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get mediatype description for inertial measurement unit sensor data type
    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));

    //create pin for inertial measurement unit data
    RETURN_IF_FAILED(m_oInputInerMeasUnit.Create("InerMeasUnit_Struct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputInerMeasUnit));

    RETURN_NOERROR;
}

tResult cMarkerPos::CreateOutputPins(__exception)
{
    //get the description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // create the description for the position pin
    tChar const * strDescPosition = pDescManager->GetMediaDescription("tPosition");
    RETURN_IF_POINTER_NULL(strDescPosition);
    cObjectPtr<IMediaType> pTypePosition = new cMediaType(0, 0, 0, "tPosition", strDescPosition, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // create the position OutputPin
    RETURN_IF_FAILED(m_oPinPosition.Create("Position", pTypePosition, this));
    RETURN_IF_FAILED(RegisterPin(&m_oPinPosition));
    // set the description for the extended marker pin
    RETURN_IF_FAILED(pTypePosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescPosition));

    RETURN_NOERROR;
}

tResult cMarkerPos::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        // no ids were set so far

        // input ids
        m_bIDsMeasWheelSpeedSet = tFalse;
        m_bIDsRoadSignExtSet    = tFalse;
        m_bIDsInerMeasUnitSet   = tFalse;

        // output ids
        m_bIDsPositionSet = tFalse;

        // create the input and output pins
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    }
    else if (eStage == StageNormal)
    {

       m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");

       if (m_bDebugModeEnabled)
        {
            m_log = fopen("markerLog.txt","w");
        }

        // read ADTF properties
        ReadProperties(NULL);

        // load roadsign configuration
        LoadConfiguration();

        // initialize EKF variables
        m_state = Mat(6,1,CV_64F,Scalar::all(0));
        m_errorCov = Mat(6,6,CV_64F,Scalar::all(0));

        tFloat64 T = 0.1;
        m_errorCov.at<double>(0,0) = MP_PROCESS_INIT_X;
        m_errorCov.at<double>(1,1) = MP_PROCESS_INIT_Y;
        m_errorCov.at<double>(2,2) = MP_PROCESS_INIT_HEADING;
        m_errorCov.at<double>(2,3) = MP_PROCESS_INIT_HEADING/T;
        m_errorCov.at<double>(3,3) = MP_PROCESS_INIT_HEADING_DRIFT;
        m_errorCov.at<double>(4,4) = MP_PROCESS_INIT_SPEED;
        m_errorCov.at<double>(5,5) = MP_PROCESS_INIT_SPEED_SCALE;

        m_transitionMatrix = Mat(6,6,CV_64F,Scalar::all(0));
        setIdentity(m_transitionMatrix);

        m_processCov = Mat(6,6,CV_64F,Scalar::all(0));
        m_processCov.at<double>(0,0) = MP_PROCESS_X;
        m_processCov.at<double>(1,1) = MP_PROCESS_Y;
        m_processCov.at<double>(2,2) = MP_PROCESS_HEADING;
        m_processCov.at<double>(3,3) = MP_PROCESS_HEADING_DRIFT;
        m_processCov.at<double>(4,4) = MP_PROCESS_SPEED;
        m_processCov.at<double>(5,5) = MP_PROCESS_SPEED_SCALE;

        m_isInitialized = tFalse;

        // initialize translation and rotation vectors
        m_Tvec = Mat(3,1,CV_32F,Scalar::all(0));
        m_Rvec = Mat(3,1,CV_32F,Scalar::all(0));

        // initialize other variables
        m_f32Speed     = 0;
        m_ui32ArduinoTimestamp = 0;

        m_ui32Cnt = 0;

        m_ticks = GetTime(); // init basetime

    }
    else if (eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cMarkerPos::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cMarkerPos::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cMarkerPos::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cMarkerPos::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    __synchronized_obj(m_critSecOnPinEvent);

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // process RoadSignExt sample
        if (pSource == &m_oInputRoadSignExt)
        {
            RETURN_IF_FAILED(ProcessRoadSignStructExt(pMediaSample));
        }
        // process Speed sample
        else if (pSource == &m_oInputSpeed)
        {

            // write values with zero
            tFloat32 f32Value = 0;
            tUInt32 Ui32TimeStamp = 0;
            {
                // focus for sample write lock
                // read data from the media sample with the coder of the descriptor
                __adtf_sample_read_lock_mediadescription(m_pDescMeasSpeed,pMediaSample,pCoderInput);

                if(!m_bIDsMeasWheelSpeedSet)
                {
                    pCoderInput->GetID("f32Value", m_buIDMeasSpeedF32Value);
                    pCoderInput->GetID("ui32ArduinoTimestamp", m_buIDMeasSpeedArduinoTimestamp);
                    m_bIDsMeasWheelSpeedSet = tTrue;
                }
                // get values from media sample
                pCoderInput->Get(m_buIDMeasSpeedF32Value, (tVoid*)&f32Value);
                pCoderInput->Get(m_buIDMeasSpeedArduinoTimestamp, (tVoid*)&Ui32TimeStamp);
            }


            m_f32Speed = f32Value;


        }

        // process InertialMeasurementUnit sample
        else if (pSource == &m_oInputInerMeasUnit)
        {
            RETURN_IF_FAILED(ProcessInerMeasUnitSample(pMediaSample));
        }

    }

    RETURN_NOERROR;
}

/*! calculates normalized angle difference */
tFloat32 cMarkerPos::angleDiff(tFloat32 angle1, tFloat32 angle2)
{
    // normalization
    angle1 = normalizeAngle(angle1, static_cast<tFloat32>(M_PI));
    angle2 = normalizeAngle(angle2, static_cast<tFloat32>(M_PI));

    // compute difference and normalize in [-pi pi]
    return normalizeAngle(angle2 - angle1, 0);
}

/*! calculates normalized angle */
tFloat32 cMarkerPos::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha-center+ static_cast<tFloat32>(M_PI), 2.0*static_cast<tFloat32>(M_PI)) + center- static_cast<tFloat32>(M_PI);
}

/*! calculates modulus after division */
tFloat32 cMarkerPos::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0f)
        {
            b_x = ceil(r - 0.5f);
        }
        else
        {
            b_x = floor(r + 0.5f);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16f * fabs(r))
        {
            return 0.0f;
        }
        else
        {
            return (r - floor(r)) * y;
        }
    }
}

/*! workaround for heading update due to pose estimation
    issues with current Aruco version
*/
#define MP_ARUCO_WORKAROUND

/*! Calculates orientation, distance and pose of the given road sign,
 * and updates the positioning filter accordingly */
tResult cMarkerPos::ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn)
{

    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionRoadSignExt,pMediaSampleIn,pCoderInput);

        // get IDs
        if(!m_bIDsRoadSignExtSet)
        {
            pCoderInput->GetID("i16Identifier",m_szIDRoadSignExtI16Identifier);
            pCoderInput->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoderInput->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoderInput->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            m_bIDsRoadSignExtSet = tTrue;
        }

        pCoderInput->Get(m_szIDRoadSignExtI16Identifier, (tVoid*)&m_i16ID);
        pCoderInput->Get(m_szIDRoadSignExtF32Imagesize, (tVoid*)&m_f32MarkerSize);
        pCoderInput->Get("af32TVec", (tVoid*)m_Tvec.data);
        pCoderInput->Get("af32RVec", (tVoid*)m_Rvec.data);

    }

    // ignore initial noisy markers
    if (m_ui32Cnt<50)
    {
        m_ui32Cnt++;
        RETURN_NOERROR;
    }

    cv::Mat R;
    cv::Rodrigues(m_Rvec, R); // rot is 3x3

    // calculate translation
    tFloat32 lateral = m_Tvec.at<float>(0);
    tFloat32 longitudinal = m_Tvec.at<float>(2);

    // add camera offset
    lateral += m_f32CameraOffsetLat;
    longitudinal += m_f32CameraOffsetLon;

    tFloat32 d0 = sqrt(lateral*lateral+longitudinal*longitudinal);

    tFloat32 a0 = atan2(lateral, longitudinal);
    a0 = (tFloat32)normalizeAngle(a0,0.0f)*RAD2DEG; // normalize angle -pi:pi

    a0 *= -1.0; // and change direction

    // calculate pose of the road sign
    tFloat32 yawE;

    tFloat32 pitch1  = -asin(R.at<float>(2,0));
    tFloat32 pitch2  = (tFloat32)normalizeAngle(static_cast<tFloat32>(CV_PI) - pitch1, 0.0f);

    tFloat32 yaw1  = atan2(R.at<float>(2,1) / cos(pitch1), R.at<float>(2,2) / cos(pitch1));
    tFloat32 yaw2  = atan2(R.at<float>(2,1) / cos(pitch2), R.at<float>(2,2) / cos(pitch2));

    // select shortest rotation for yaw
    if (abs(yaw1) <= abs(yaw2))
    {
        yawE   = yaw1*RAD2DEG;
    }
    else
    {
        yawE   = yaw2*RAD2DEG;
    }

    // check angle and distance limit
    if (fabs(a0) > MP_LIMIT_ALPHA || d0 > MP_LIMIT_DISTANCE)
    {
        RETURN_NOERROR;
    }

    if (m_bDebugModeEnabled) LOG_INFO(cString::Format("ID %d: d0 %f a0 %f yawE %f", m_i16ID, d0, a0, yawE));
	
    // wait for start-marker, and then initialize the filter
    if (m_isInitialized==tFalse)
    {
		for (unsigned int i=0;i<m_roadSigns.size();i++)
		{
			if ((m_roadSigns[i].u16Id==m_i16ID) && 
			    (m_roadSigns[i].bInit==tTrue))
			{
				 // calculate the vehicle position and heading based on
				// road-sign measurement

				// estimate heading
				#ifdef MP_ARUCO_WORKAROUND
				tFloat32 heading = m_roadSigns[i].f32Direction; // initialize directly with road sign heading
				#else
				// assume that car is aligned with the marker when initializing,
				// and allow only very accurate measurements
				if (fabs(yawE) > MP_LIMIT_YAW_INIT)
				{
					RETURN_NOERROR;
				}
				tFloat32 heading = m_roadSigns[i].f32Direction + yawE*DEG2RAD;
				#endif
				
				heading = normalizeAngle(heading,0.0f);

				tFloat32 shift = -1.0f*d0;

				tFloat32 correction = heading+a0*DEG2RAD;
				correction = normalizeAngle(correction,0.0f);

				// estimate location
				tFloat32 x = m_roadSigns[i].f32X+cos(correction)*shift;
				tFloat32 y = m_roadSigns[i].f32Y+sin(correction)*shift;

				if (m_bDebugModeEnabled) LOG_INFO(cString::Format("initialize e %f n %f h %f x %f y %f", x, y, heading*RAD2DEG,
					m_roadSigns[i].f32X, m_roadSigns[i].f32Y));

				// initialize filter state
				m_state.at<double>(0) = x;
				m_state.at<double>(1) = y;
				m_state.at<double>(2) = heading;
				m_state.at<double>(3) = 0;
				m_state.at<double>(4) = 0;
				m_state.at<double>(5) = 0;

				m_isInitialized = tTrue;

			}

		}
    
    RETURN_NOERROR;
    
    }

    // find a matching road sign

    tFloat64 dt = 0;

    tInt ind = -1;
    for (unsigned int i=0;i<m_roadSigns.size();i++)
    {
        if (m_roadSigns[i].u16Id == m_i16ID)
        {
            // calculate heading wrt marker
            tFloat32 heading = static_cast<tFloat32>(m_state.at<double>(2) + a0*DEG2RAD);
            heading = normalizeAngle(heading,0);

            // estimate marker location based on current vehicle location
            // and marker measurement
            tFloat32 x0 = static_cast<tFloat32>(m_state.at<double>(0)+cos(heading)*d0);
            tFloat32 y0 = static_cast<tFloat32>(m_state.at<double>(1)+sin(heading)*d0);

            // calculate error distance
            tFloat32 dx = x0-m_roadSigns[i].f32X;
            tFloat32 dy = y0-m_roadSigns[i].f32Y;

            tFloat32 distance = sqrt(dx*dx + dy*dy);
            
            tInt found = tFalse;
             
            // re-initialize with init-signs
            if (m_roadSigns[i].bInit==tTrue)
            {
                tFloat32 shift = -1.0*d0;

				// estimate location
				tFloat32 x = m_roadSigns[i].f32X+cos(heading)*shift;
				tFloat32 y = m_roadSigns[i].f32Y+sin(heading)*shift;
                
                // initialize filter but keep heading states
				m_state.at<double>(0) = x;
				m_state.at<double>(1) = y;
               
				m_state.at<double>(4) = 0;
				m_state.at<double>(5) = 0;
                
                found = tTrue;
				
            }
            // marker found within the radius
            else if (distance < m_roadSigns[i].f32Radius)
            {
               found = tTrue;
            }
            
            // found a suitable marker 
            if (found)
            {
                ind = i;

                // calculate time from previous marker measurement
                dt = (GetTime() - m_roadSigns[i].u32ticks)*1e-6;
                m_roadSigns[i].u32ticks = GetTime();

                // reset sample counter when marker reappears
                if (dt > 1.0)
                {
                    m_roadSigns[i].u16Cnt = 0;
                }

                break;
            }

        }

    }

    // update sample counter
    m_roadSigns[ind].u16Cnt ++;

    // conditions:
    // #1 no matching marker found
    // #2 too long time from previous marker input
    // #3 dropping samples when a new marker is found
    if (ind < 0 || dt > 0.3 || m_roadSigns[ind].u16Cnt < 10)
    {

        RETURN_NOERROR;
    }

    // EKF update step

    // calculate heading update
#ifdef MP_ARUCO_WORKAROUND
    // create pseudo measurement for heading update
    tFloat32 headingUpdate = static_cast<tFloat32>(m_state.at<double>(2));
#else
    // calculate heading update using marker direction and pose angle
    tFloat32 headingUpdate =  m_roadSigns[ind].f32Direction + yawE*DEG2RAD;
#endif

    tFloat32 shift = -1.0f*d0; // reversed translation direction

    // calculate translation direction
    tFloat32 correction = headingUpdate+a0*DEG2RAD;
    correction = normalizeAngle(correction,0);

    // update location estimate
    tFloat32 x = m_roadSigns[ind].f32X+cos(correction)*shift;
    tFloat32 y = m_roadSigns[ind].f32Y+sin(correction)*shift;

    Mat measCov = Mat(3,3,CV_64F,Scalar::all(0));
    measCov.at<double>(0,0) = MP_MEASUREMENT_X;
    measCov.at<double>(1,1) = MP_MEASUREMENT_Y;
    measCov.at<double>(2,2) = MP_MEASUREMENT_HEADING;

    Mat measurementMatrix = Mat(3,6,CV_64F,Scalar::all(0));

    measurementMatrix.at<double>(0,0) = 1.0;
    measurementMatrix.at<double>(1,1) = 1.0;
    measurementMatrix.at<double>(2,2) = 1.0;

    Mat identity = Mat(6,6,CV_64F,Scalar::all(0));
    setIdentity(identity);

    Mat measurement = Mat(3,1,CV_64F,Scalar::all(0));

    measurement.at<double>(0) = x;
    measurement.at<double>(1) = y;
    measurement.at<double>(2) = headingUpdate;

    if (m_log) fprintf(m_log,"markerPos: %.3f %.3f %.3f dt %f yawE %f a0 %f\n", x, y, headingUpdate, dt, yawE, a0);

    // propagate covariance
    m_errorCov = m_transitionMatrix*m_errorCov*m_transitionMatrix.t() + m_processCov;

    // calculate innovation
    Mat innovation = measurement - measurementMatrix*m_state;

    // modulo of the heading measurement
    innovation.at<double>(2) = static_cast<double>(angleDiff(mod(static_cast<tFloat32>(m_state.at<double>(2)),2.0f*static_cast<tFloat32>(M_PI))- static_cast<tFloat32>(M_PI),mod(headingUpdate,2.0f*static_cast<tFloat32>(M_PI))- static_cast<tFloat32>(M_PI)));

    Mat tmp = measurementMatrix*m_errorCov*measurementMatrix.t() + measCov;
    Mat gain = m_errorCov*measurementMatrix.t()*tmp.inv();

    // update state and covariance matrix
    m_state += gain*innovation;
    m_state.at<double>(2)= static_cast<double>(normalizeAngle(static_cast<tFloat32>(m_state.at<double>(2)) ,0.0f));
    m_errorCov = (identity-gain*measurementMatrix)*m_errorCov;

    RETURN_NOERROR;
}

/*! support function for getting time */
tTimeStamp cMarkerPos::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

/*! properties update handler */
tResult cMarkerPos::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

/*! reads ADTF properties */
tResult cMarkerPos::ReadProperties(const tChar* strPropertyName)
{

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MP_PROP_CAMERA_OFFSET_LAT))
    {
        m_f32CameraOffsetLat = static_cast<tFloat32> (GetPropertyFloat(MP_PROP_CAMERA_OFFSET_LAT));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MP_PROP_CAMERA_OFFSET_LON))
    {
        m_f32CameraOffsetLon = static_cast<tFloat32> (GetPropertyFloat(MP_PROP_CAMERA_OFFSET_LON));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MP_PROP_SPEED_SCALE))
    {
        m_f32SpeedScale = static_cast<tFloat32> (GetPropertyFloat(MP_PROP_SPEED_SCALE));
    }

    RETURN_NOERROR;
}

/*!
 * loads road-sign configuration from a file, and stores into memory
 * */
tResult cMarkerPos::LoadConfiguration()
{
    cFilename fileConfig = GetPropertyStr("Configuration");

    // create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");

    if (fileConfig.IsEmpty())
    {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    tInt i = 0;
    if (cFileSystem::Exists(fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(fileConfig);
        cDOMElementRefList oElems;

        if(IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                roadSign item;
                item.u16Id = tUInt16((*itElem)->GetAttribute("id","0").AsInt32());
                item.f32X = tFloat32((*itElem)->GetAttribute("x","0").AsFloat64());
                item.f32Y = tFloat32((*itElem)->GetAttribute("y","0").AsFloat64());
                item.f32Radius = tFloat32((*itElem)->GetAttribute("radius","0").AsFloat64());
                item.f32Direction = tFloat32((*itElem)->GetAttribute("direction","0").AsFloat64());

				item.bInit = tBool((*itElem)->GetAttribute("init","0").AsInt32());
				
                item.u16Cnt = 0;
                item.u32ticks = GetTime();

                item.f32Direction *= DEG2RAD; // convert to radians

                if (m_bDebugModeEnabled)
                {
                    LOG_INFO(cString::Format("LoadConfiguration::Id %d XY %f %f Radius %f Direction %f",
                        item.u16Id, item.f32X, item.f32Y, item.f32Radius, item.f32Direction));
                }

               m_roadSigns.push_back(item);

               i++;

            }
        }
     }
    else
    {
        LOG_ERROR("Configuration file does not exist");
        RETURN_ERROR(ERR_INVALID_FILE);
    }


    RETURN_NOERROR;
}

/*! sends position data out */
tResult cMarkerPos::sendPositionStruct(const tTimeStamp &timeOfFix, const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius,
                                   const tFloat32 &f32Heading, const tFloat32 &f32Speed)
{

    __synchronized_obj(m_oSendPositionCritSection);

    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescPosition->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescPosition,pMediaSample,pCoder);

        // get IDs
        if(!m_bIDsPositionSet)
        {
            pCoder->GetID("f32x", m_szIDPositionF32X);
            pCoder->GetID("f32y", m_szIDPositionF32Y);
            pCoder->GetID("f32radius", m_szIDPositionF32Radius);
            pCoder->GetID("f32speed", m_szIDPositionF32Speed);
            pCoder->GetID("f32heading", m_szIDPositionF32Heading);

            m_bIDsPositionSet = tTrue;
        }

        pCoder->Set(m_szIDPositionF32X, (tVoid*)&f32X);
        pCoder->Set(m_szIDPositionF32Y, (tVoid*)&f32Y);
        pCoder->Set(m_szIDPositionF32Radius, (tVoid*)&f32Radius);
        pCoder->Set(m_szIDPositionF32Speed, (tVoid*)&f32Speed);
        pCoder->Set(m_szIDPositionF32Heading, (tVoid*)&f32Heading);

        pMediaSample->SetTime(timeOfFix);
    }

    //  doing the transmit
    RETURN_IF_FAILED(m_oPinPosition.Transmit(pMediaSample));

    RETURN_NOERROR;
}

void cMarkerPos::transformRearPointToFront(float* x_res, float* y_res) {
    const float REARAXIS_BUMPER_LENGTH = 0.48;
    //const float REARAXIS_CAM_OFFSET = 0.25;
    *x_res = m_state.at<double>(0) + REARAXIS_BUMPER_LENGTH*cos(m_state.at<double>(2));
    *y_res = m_state.at<double>(1) + REARAXIS_BUMPER_LENGTH*sin(m_state.at<double>(2));
}


/*! processes inertial measurement data sample, and runs EKF prediction
 *  based on heading rate and speed measurements */
tResult cMarkerPos::ProcessInerMeasUnitSample(IMediaSample* pMediaSampleIn)
{
    // write values with zero
    tFloat32 f32G_x = 0;
    tFloat32 f32G_y = 0;
    tFloat32 f32G_z = 0;
    tFloat32 f32A_x = 0;
    tFloat32 f32A_y = 0;
    tFloat32 f32A_z = 0;
    tFloat32 f32M_x = 0;
    tFloat32 f32M_y = 0;
    tFloat32 f32M_z = 0;
    tUInt32 ui32ArduinoTimestamp = 0;

    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData,pMediaSampleIn,pCoderInput);

        // get the IDs for the items in the media sample
        if(!m_bIDsInerMeasUnitSet)
        {
            pCoderInput->GetID("f32G_x", m_szIDInerMeasUnitF32G_x);
            pCoderInput->GetID("f32G_y", m_szIDInerMeasUnitF32G_y);
            pCoderInput->GetID("f32G_z", m_szIDInerMeasUnitF32G_z);
            pCoderInput->GetID("f32A_x", m_szIDInerMeasUnitF32A_x);
            pCoderInput->GetID("f32A_y", m_szIDInerMeasUnitF32A_y);
            pCoderInput->GetID("f32A_z", m_szIDInerMeasUnitF32A_z);
            pCoderInput->GetID("f32M_x", m_szIDInerMeasUnitF32M_x);
            pCoderInput->GetID("f32M_y", m_szIDInerMeasUnitF32M_y);
            pCoderInput->GetID("f32M_z", m_szIDInerMeasUnitF32M_z);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDInerMeasUnitArduinoTimestamp);
            m_bIDsInerMeasUnitSet = tTrue;
        }

        //write date to the media sample with the coder of the descriptor
        pCoderInput->Get(m_szIDInerMeasUnitF32G_x, (tVoid*)&f32G_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32G_y, (tVoid*)&f32G_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32G_z, (tVoid*)&f32G_z);

        pCoderInput->Get(m_szIDInerMeasUnitF32A_x, (tVoid*)&f32A_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_y, (tVoid*)&f32A_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_z, (tVoid*)&f32A_z);

        pCoderInput->Get(m_szIDInerMeasUnitF32M_x, (tVoid*)&f32M_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32M_y, (tVoid*)&f32M_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32M_z, (tVoid*)&f32M_z);
        pCoderInput->Get(m_szIDInerMeasUnitArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);

    }

    tFloat64 dt = (tFloat64)(ui32ArduinoTimestamp-m_ui32ArduinoTimestamp)*1e-6;
    m_ui32ArduinoTimestamp = ui32ArduinoTimestamp;

    // filter not initialized
    if (m_isInitialized==tFalse)
    {
        RETURN_NOERROR;
    }

    // update heading
    tFloat32 hk = static_cast<tFloat32>(m_state.at<double>(2)) + (f32G_z*DEG2RAD + static_cast<tFloat32>(m_state.at<double>(3)))*static_cast<tFloat32>(dt);

    // normalize heading -pi:pi
    hk = normalizeAngle(hk,0);

    tFloat32 sc = m_f32SpeedScale;

    // update speed and scale
    tFloat32 ak = static_cast<tFloat32>(m_state.at<double>(5));
    tFloat32 vk = m_f32Speed*(sc-ak);

    // update transition matrix; F = I + Fc*dt
    m_transitionMatrix.at<double>(0,2) = -vk*sin(hk)*dt;
    m_transitionMatrix.at<double>(0,3) = -vk*sin(hk)*dt;
    m_transitionMatrix.at<double>(0,4) =     cos(hk)*dt;
    m_transitionMatrix.at<double>(0,5) = -vk/(sc-ak)*cos(hk)*dt;

    m_transitionMatrix.at<double>(1,2) =  vk*cos(hk)*dt;
    m_transitionMatrix.at<double>(1,3) =  vk*cos(hk)*dt;
    m_transitionMatrix.at<double>(1,4) =     sin(hk)*dt;
    m_transitionMatrix.at<double>(1,5) = -vk/(sc-ak)*sin(hk)*dt;

    m_transitionMatrix.at<double>(2,3) =  dt;

    // propagate state and covariance
    m_state.at<double>(0) += vk*cos(hk)*dt;
    m_state.at<double>(1) += vk*sin(hk)*dt;
    m_state.at<double>(2) = hk;
    m_state.at<double>(4) = vk;

    m_errorCov = m_transitionMatrix*m_errorCov*m_transitionMatrix.t() + m_processCov;

   //if (m_log) fprintf(m_log,"position: %.3f %.3f %.3f %.3f %.3f %.3f\n", m_state.at<double>(0), m_state.at<double>(1),
    //m_state.at<double>(2)*RAD2DEG, m_state.at<double>(3)*RAD2DEG, m_state.at<double>(4), m_state.at<double>(5));
    float x_new;
    float y_new;
    transformRearPointToFront(&x_new, &y_new);
    sendPositionStruct(GetTime(), 
		static_cast<tFloat32>(x_new), 
		static_cast<tFloat32>(y_new),
		static_cast<tFloat32>(sqrt(m_errorCov.at<double>(0,0) + m_errorCov.at<double>(1,1))),
		static_cast<tFloat32>(m_state.at<double>(2)), 
		static_cast<tFloat32>(m_state.at<double>(4))
	);
    if (m_log) {
        printf("Position Raw: x y %f %f\n", m_state.at<double>(0), m_state.at<double>(1));
        printf("Position Transformed: x y %f %f\n", x_new, y_new);

    }
    RETURN_NOERROR;
  }
