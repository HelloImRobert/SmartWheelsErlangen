#include "SWE_Odometry.h"

#define WHEELBASE 359.0f
#define CARWIDTH  260.0f //wheel center to wheel center
#define TIMESTAMP_RESOLUTION 1000.0f // for the Arduino timestamps
#define TIMESTAMP_RESOLUTION_ADTF 1000000.0f //ADTF uses microseconds
#define MAX_TICK_FILTER_VELOCITY 1000.0f //in mm/s
#define MY_PI 3.14159265358979f
#define MINIMUM_TURNING_RADIUS 600.0f // in mm -> actually ~ 680-ish ?
#define PITCH_COMPENSATION_STRENGTH 9810.0f //gravity in mm/s2
#define ACCELEROMETER_INPUT_SCALING 1000.0f //with this factor the values will be in mm/s2 -> further scaling with property



ADTF_FILTER_PLUGIN("SWE_Odometry", OID_ADTF_SWE_ODOMETRY, SWE_Odometry)


// ****************************************************************************************************************
//                                                   Init ADTF Filter
// ****************************************************************************************************************

SWE_Odometry::SWE_Odometry(const tChar* __info) : cFilter(__info), m_SlidingWindowCntLeftWheel(21, (0.4 * TIMESTAMP_RESOLUTION)), m_SlidingWindowCntRightWheel(21, (0.4 * TIMESTAMP_RESOLUTION)), m_yawSlidingWindow(3), m_mutex()
{

    m_velocityLeft = 0;
    m_velocityRight = 0;
    m_velocityFiltered = 0;
    m_velocityWheelSensors = 0;
    m_velocityAccelerometer = 0;
    m_velocityCombined = 0;

    m_lastPinEvent = GetTime();
    m_lastTriggerTime = 0;

    m_distanceX_sum = 0;
    m_distanceY_sum = 0;
    m_heading_sum = 0;
    m_distanceAllSum = 0;
    m_distanceAllSum_acc = 0;
    m_distanceAllSum_wheel = 0;
    m_current_turning_radius = 0;


    m_wheelDelta_left = 0;
    m_wheelDelta_right = 0;

    m_wheelCounter_left = 0;
    m_wheelCounter_right = 0;

    m_accelerometerValue_now = 0;
    m_accelerometerValue_old = 0;

    m_accelerometerTimestamp_last = 0;
    m_accelerometerTimestamp_now = 0;

    m_lastTimeStamp_wheels = 0;

    m_currentDirection = 0;

    debugvar = 0;

    m_wheelsync = false;

    m_heading_lastStep = 0.0;
    m_heading_now = 0.0;

    m_pitch_now = 0;

    SetPropertyFloat("Velocity Filter Strength 0-1",0.3);                                   //filter strength for the velocity smoothing filter
    SetPropertyFloat("Accelerometer Velocity weighting 0-1",0.95);
    SetPropertyFloat("Accelerometer Velocity drift compensation +- mm/s2",200.0);
    SetPropertyFloat("Wheel circumfence in mm",327);
    SetPropertyFloat("time and pulse resolution of the velocity calculation 2-40",5);
    SetPropertyFloat("Accelerometer value offset in mm/s2",0.0);
    SetPropertyFloat("Gravity for pitch compensation in mm/s2",9900.0);                     //strength of pitch compensation
    SetPropertyBool("Use high-res distance measurement (based on velocity)", true);
    SetPropertyBool("Use Accelerometer", true);                                             //calculate velocity based on accelerometer data? (you want to turn this off when testing on a bench)
    SetPropertyBool("Show compensated accelerometer value", false);                         //show the value of the accelerometer after pitch compensation for adjustment of the offset
    SetPropertyBool("SetDirection to 1", false);
    SetPropertyFloat("Accelerometer Scaling", 1.0);
    SetPropertyFloat("Pulses per wheel revolution", 8.6);                                   //it should be 8.0 but you have to compensate a little for all those errors
    SetPropertyFloat("HighresDistanceDriftCompensation", 0.001);
}

SWE_Odometry::~SWE_Odometry()
{
}

tResult SWE_Odometry::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
    
    m_SlidingWindowCntLeftWheel.Reset();
    m_SlidingWindowCntRightWheel.Reset();

    m_wheelCounter_left = 0;
    m_wheelCounter_right = 0;

    m_wheelDelta_left = 0;
    m_wheelDelta_right = 0;

    m_wheelsync = false;

    m_distanceAllSum = 0;
    m_distanceAllSum_acc = 0;
    m_distanceAllSum_wheel = 0;
    m_distanceX_sum = 0;
    m_distanceY_sum = 0;
    m_heading_sum = 0;

    m_lastPinEvent = GetTime();

    if(m_setdirectionone)
        m_currentDirection = 1;
    else
        m_currentDirection = 0;

    m_velocityLeft = 0;
    m_velocityRight = 0;
    m_velocityFiltered = 0;
    m_velocityWheelSensors = 0;
    m_velocityAccelerometer = 0;
    m_velocityCombined = 0;

    m_accelerometerValue_now = 0;
    m_accelerometerValue_old = 0;

}

tResult SWE_Odometry::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult SWE_Odometry::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult SWE_Odometry::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


        // ------- tSignalValue pins -----------
        // ----------------- create input pins ------------------
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

        RETURN_IF_FAILED(m_oInputWheelRight.Create("Sensor_left_Wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));
        RETURN_IF_FAILED(m_oInputWheelLeft.Create("Sensor_right_Wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));



        tChar const * strDescSignalValue_yaw = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue_yaw);
        cObjectPtr<IMediaType> pTypeSignalValue_yaw = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue_yaw,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue_yaw->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_yaw));

        RETURN_IF_FAILED(m_oInputYaw.Create("Gyro_Yaw", pTypeSignalValue_yaw, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputYaw));
        RETURN_IF_FAILED(m_oInputPitch.Create("Gyro_Pitch", pTypeSignalValue_yaw, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPitch));
        RETURN_IF_FAILED(m_oInputAccel.Create("Accelerometer_y_axis", pTypeSignalValue_yaw, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputAccel));


        tChar const * strDescSignalValue_direction = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue_direction);
        cObjectPtr<IMediaType> pTypeSignalValue_direction = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue_direction,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue_direction->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_direction));

        RETURN_IF_FAILED(m_oInputDirection.Create("Direction", pTypeSignalValue_direction, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputDirection));

        // ------- velocity output pin --------------

        tChar const * strDescSignalValue_Velocity = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue_Velocity);
        cObjectPtr<IMediaType> pTypeSignalValue_Velocity = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue_Velocity,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue_Velocity->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderVelocityOut));

        RETURN_IF_FAILED(m_oOutputVelocity.Create("Car_Velocity", pTypeSignalValue_Velocity, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputVelocity));



        // ---- tBoolSignal value ---------
        // ------ input pins --------------
        tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBool);
        cObjectPtr<IMediaType> pTypeBoolData = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBool,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeBoolData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderBool));


        RETURN_IF_FAILED(m_oInputTrigger.Create("Trigger", pTypeBoolData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTrigger));


        // ------------------ odometry output pins ---------------------
        tChar const * strDescOdometry = pDescManager->GetMediaDescription("tOdometry");
        RETURN_IF_POINTER_NULL(strDescOdometry);
        cObjectPtr<IMediaType> pTypeOdometry = new cMediaType(0, 0, 0, "tOdometry", strDescOdometry,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeOdometry->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescOdometryOut));


        RETURN_IF_FAILED(m_oOutputOdometry.Create("Odometry_Output", pTypeOdometry, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputOdometry));



        // ------------------ further init ---------------------

        m_filterStrength = (tFloat32)GetPropertyFloat("Velocity Filter Strength 0-1", 0.3);
        m_wheelCircumfence = (tFloat32)GetPropertyFloat("Wheel circumfence in mm",327);
        m_velocityResolution = (tFloat32)GetPropertyFloat("time and pulse resolution of the velocity calculation 2-40",5);
        m_acceleromter_weight = (tFloat32)GetPropertyFloat("Accelerometer Velocity weighting 0-1",0.9);
        m_accelerometer_compensation = (tFloat32)GetPropertyFloat("Accelerometer Velocity drift compensation +- mm/s2",200.0);
        m_useHighresDist = (tBool)GetPropertyBool("Use high-res distance measurement (based on velocity)", true);
        m_accelOffsetValue = (tFloat32)GetPropertyFloat("Accelerometer value offset in mm/s2",0.0);
        m_useAccelerometer = (tBool)GetPropertyBool("Use Accelerometer", true);
        m_showAccel = (tBool)GetPropertyBool("Show compensated accelerometer value", false);
        m_pitchCompensation = (tFloat32)GetPropertyFloat("Gravity for pitch compensation in mm/s2",9900.0);
        m_setdirectionone = (tBool)GetPropertyBool("SetDirection to 1", false);
        m_accelScaling = (tFloat32)GetPropertyFloat("Accelerometer Scaling", 1.0);
        m_tickPerTurn = (tFloat32)GetPropertyFloat("Pulses per wheel revolution", 8.6);
        m_distanceDriftCompensation = (tFloat32)GetPropertyFloat("HighresDistanceDriftCompensation", 0.001);

        if(m_setdirectionone)
            m_currentDirection = 1;
        else
            m_currentDirection = 0;

        //prevent stupid filter values
        if(m_filterStrength >= 1.0)
        {
            m_filterStrength = 0.99;
        }
        else if (m_filterStrength < 0.0)
        {
            m_filterStrength = 0.0;
        }

        if(m_acceleromter_weight > 1.0)
        {
            m_acceleromter_weight = 0.99;
        }
        else if (m_acceleromter_weight < 0.0)
        {
            m_acceleromter_weight = 0.0;
        }

        if(m_velocityResolution > 40)
        {
            m_velocityResolution = 40;
        }
        else if (m_velocityResolution < 2)
        {
            m_velocityResolution = 2;
        }

        if(!m_useAccelerometer)
            m_acceleromter_weight = 0.0;

        m_SlidingWindowCntLeftWheel.SetResolution(m_velocityResolution);
        m_SlidingWindowCntRightWheel.SetResolution(m_velocityResolution);


        RETURN_NOERROR;
    }
    else if (eStage == StageNormal)
    {

    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}


// ****************************************************************************************************************
//                                                   On Pin Event
// ****************************************************************************************************************


tResult SWE_Odometry::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {

        m_mutex.Enter(); //serialize the whole filter for data consistency

        if (pSource == &m_oInputWheelLeft)
        {
            tTimeStamp timeStamp;
            tFloat32 buffer;
            tUInt32 delta;

            if(!m_wheelsync)
                m_lastTimeStamp_wheels = timeStamp;

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&buffer);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            delta = buffer - m_wheelCounter_left;

            m_wheelCounter_left = buffer;
            m_wheelDelta_left = delta;

            //process data
            if (m_wheelsync) // has the sibling wheeldata arrived yet?
            {
                ProcessPulses(timeStamp);
                m_wheelsync = false;
            }
            else
            {
                m_wheelsync = true;
            }
        }
        else if (pSource == &m_oInputWheelRight)
        {
            tTimeStamp timeStamp;
            tFloat32 buffer;
            tUInt32 delta;

            if(!m_wheelsync)
                m_lastTimeStamp_wheels = timeStamp;

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&buffer);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            delta = buffer - m_wheelCounter_right;

            m_wheelCounter_right = buffer;
            m_wheelDelta_right = delta;


            //process data
            if (m_wheelsync) // has the sibling wheeldata arrived yet?
            {
                ProcessPulses(timeStamp);
                m_wheelsync = false;
            }
            else
            {
                m_wheelsync = true;
            }
        }
        else if (pSource == &m_oInputDirection)
        {
            tTimeStamp current_time;
            tTimeStamp timeStamp;
            current_time = GetTime();

            //do odometry step first
            CalcOdometryStep( current_time, m_lastPinEvent );
            m_lastPinEvent = current_time;


            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal_direction->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_currentDirection);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal_direction->Unlock(pCoderInput);

            if(m_setdirectionone)
                m_currentDirection = 1;

            //process input data
            if (m_currentDirection == 0) //on car stopped signal reset everything (we know we're standing still)
            {
                m_SlidingWindowCntLeftWheel.Reset();
                m_SlidingWindowCntRightWheel.Reset();
                m_velocityFiltered = 0;
                m_velocityLeft = 0;
                m_velocityRight = 0;
                m_velocityWheelSensors = 0;
                m_velocityCombined = 0;
                m_velocityAccelerometer = 0;
            }

        }
        else if (pSource == &m_oInputYaw)
        {
            tTimeStamp current_time;
            tTimeStamp timeStamp;
            current_time = GetTime();

            //save last yaw data for extrapolation
            m_yawSampleTime_now = current_time;

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal_yaw->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_heading_now);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal_yaw->Unlock(pCoderInput);

            //for extrapolation
            m_yawSlidingWindow.addNewValue(m_heading_now, timeStamp);

            //calc odometry step
            CalcOdometryStep(current_time,  m_lastPinEvent);
            m_lastPinEvent = current_time;

        }
        else if (pSource == &m_oInputPitch)
        {

            tTimeStamp timeStamp;

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal_yaw->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_pitch_now);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal_yaw->Unlock(pCoderInput);
        }
        else if (pSource == &m_oInputAccel)
        {
            tTimeStamp timeStamp;
            tTimeStamp current_time;
            tFloat32 buffer;

            current_time = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal_yaw->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&buffer);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal_yaw->Unlock(pCoderInput);

            //process the data
            m_accelerometerTimestamp_last = m_accelerometerTimestamp_now;
            m_accelerometerTimestamp_now = timeStamp;

            //DEBUG
            //LOG_ERROR(cString("OD:raw accelerometer value: " + cString::FromFloat64((tFloat64)buffer) + cString(" mm/s2")));

            m_accelerometerValue_old = m_accelerometerValue_now;
            m_accelerometerValue_now = CompensatePitchInfluence(buffer * ACCELEROMETER_INPUT_SCALING);

            CalcCombinedVelocity();

            //calc odometry step
            CalcOdometryStep(current_time,  m_lastPinEvent);

            SendVelocity();

            m_lastPinEvent = current_time;

        }
        else if (pSource == &m_oInputTrigger)
        {
            tTimeStamp current_time;
            tBool buffer;
            current_time = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderBool->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("bValue", (tVoid*)&buffer); // don't care
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_lastTriggerTime);
            m_pCoderBool->Unlock(pCoderInput);

            //process the data
            CalcOdometryStep( (current_time), m_lastPinEvent );
            m_lastPinEvent = current_time;

            SendOdometry(m_lastTriggerTime);

            // reset for next trigger interval
            m_distanceX_sum = 0;
            m_distanceY_sum = 0;
            m_heading_sum = 0;
        }

        m_mutex.Leave();
    }



    RETURN_NOERROR;
}

// ****************************************************************************************************************
//                                                   Process Input Data
// ****************************************************************************************************************

tFloat32  SWE_Odometry::CompensatePitchInfluence(tFloat32 accel_value)
{
    tFloat32 outputval;

    outputval = (accel_value - ((sin(m_pitch_now) * m_pitchCompensation)) + m_accelOffsetValue)*m_accelScaling;

    if(m_showAccel)
    {
        LOG_ERROR(cString("OD:raw accelerometer value: " + cString::FromFloat64((tFloat64)accel_value) + cString(" mm/s2")));
        LOG_ERROR(cString("OD: Pitch compensated + scaled accelerometer value: " + cString::FromFloat64((tFloat64)outputval) + cString(" mm/s2")));
    }

    return outputval;
}

tResult  SWE_Odometry::CalcCombinedVelocity()
{
    tFloat32 compensation_value;
    tFloat32 intervall;

    intervall = ((tFloat32)(m_accelerometerTimestamp_now - m_accelerometerTimestamp_last)/ TIMESTAMP_RESOLUTION); //time intervall in seconds

    if (intervall < 0)
        intervall = 0;

    m_velocityAccelerometer = intervall * ((m_accelerometerValue_now + m_accelerometerValue_old) / 2.0); //the delta in velocity since the last sample

    //DEBUG
    //debugvar = m_velocityAccelerometer;
    debugvar = m_velocityWheelSensors;
    //debugvar = debugvar + (CalcDistance(m_currentDirection, (tFloat32)m_wheelDelta_left ) / 2.0)   +  (CalcDistance(m_currentDirection, (tFloat32)m_wheelDelta_right) / 2.0);
    //SendVelocity();

    // apply new acceleration value
    m_velocityCombined = m_velocityCombined  +  m_velocityAccelerometer;

    //apply constant compensation
    tFloat32 difference;
    tFloat32 compensation_sum;

    difference = (m_velocityCombined - m_velocityWheelSensors);
    compensation_sum = intervall * m_accelerometer_compensation;

    if ( difference > 0)
    {
        if (difference < compensation_sum)
            compensation_value = -difference;
        else
            compensation_value = -compensation_sum;
    }
    else
    {
        if (difference > -compensation_sum)
            compensation_value = -difference;
        else
            compensation_value = compensation_sum;
    }

    m_velocityCombined += compensation_value;

    //apply recursive weighting compensation
    m_velocityCombined = (1.0-m_acceleromter_weight)*m_velocityWheelSensors + (m_acceleromter_weight*m_velocityCombined);

    //zero if standing still
    if (m_currentDirection == 0)
        m_velocityCombined = 0;

    m_distanceAllSum_acc = m_distanceAllSum + (intervall * m_velocityCombined);

    //calc filtered velocity
    m_velocityFiltered = FilterVelocity(m_filterStrength, m_velocityFiltered, m_velocityCombined );

    RETURN_NOERROR;
}

tResult SWE_Odometry::ProcessPulses(tTimeStamp timeStamp)
{

    FilterPulses();

    m_distanceAllSum_wheel = m_distanceAllSum_wheel + (CalcDistance(m_currentDirection, (tFloat32)m_wheelDelta_left ) / 2.0)   +  (CalcDistance(m_currentDirection, (tFloat32)m_wheelDelta_right) / 2.0);

    //calculate absolute distance moved
    if(m_useHighresDist)
    {
        tFloat compensation;
        //compensate the drift, but only when moving and more when moving fast (the wheelsensor distance is defined as ground truth -> that is, well our naive hope at least.... ;-)
        compensation = m_distanceDriftCompensation * fabs(m_velocityCombined) * ((tFloat32)(timeStamp - m_lastTimeStamp_wheels)/ TIMESTAMP_RESOLUTION);

        if ((m_distanceAllSum_wheel - m_distanceAllSum_acc) >= 0)
        {
            if((m_distanceAllSum_wheel - m_distanceAllSum_acc) < compensation) //when very close to each other
                compensation = (m_distanceAllSum_wheel - m_distanceAllSum_acc);
        }
        else
        {
            compensation = -compensation;

            if((m_distanceAllSum_wheel - m_distanceAllSum_acc) > compensation)
                compensation = (m_distanceAllSum_wheel - m_distanceAllSum_acc);
        }
        m_distanceAllSum_acc = m_distanceAllSum_acc + compensation;

        m_distanceAllSum = m_distanceAllSum_acc;
    }
    else
        m_distanceAllSum = m_distanceAllSum_wheel;

    m_SlidingWindowCntLeftWheel.AddNewValue(m_wheelDelta_left, timeStamp);
    m_SlidingWindowCntRightWheel.AddNewValue(m_wheelDelta_right, timeStamp);

    CalcVelocity();

    RETURN_NOERROR;
}

tResult SWE_Odometry::CalcVelocity()
{
    if (m_SlidingWindowCntLeftWheel.GetTime() > 0)
        m_velocityLeft = (((tFloat32)m_SlidingWindowCntLeftWheel.GetPulses() / m_SlidingWindowCntLeftWheel.GetTime() ) / m_tickPerTurn) * m_wheelCircumfence * TIMESTAMP_RESOLUTION * m_currentDirection;
    else
        m_velocityLeft = 0.0;

    if (m_SlidingWindowCntRightWheel.GetTime() > 0)
        m_velocityRight = (((tFloat32)m_SlidingWindowCntRightWheel.GetPulses() / m_SlidingWindowCntRightWheel.GetTime() ) / m_tickPerTurn) * m_wheelCircumfence * TIMESTAMP_RESOLUTION * m_currentDirection;
    else
        m_velocityRight = 0.0;

    m_velocityWheelSensors = ((m_velocityLeft + m_velocityRight) / 2.0);

    RETURN_NOERROR;
}

tResult SWE_Odometry::FilterPulses()
{
    tFloat32 relativeDelta;
    tFloat32 turningRadius;

    turningRadius = fabs(m_current_turning_radius); //positive turning radius = left wheel turns slower

    // keep turning radius within possible values (can be estimated to be lower due to quantization and other measurement errors)
    if (turningRadius < MINIMUM_TURNING_RADIUS)
        turningRadius = MINIMUM_TURNING_RADIUS;

    relativeDelta = (turningRadius / (turningRadius - CARWIDTH)); //the wheelpulses can differ by this much at this turning radius

    // if the delta of pulses between both wheels differs by more than the allowed relative delta + 1 pulse then its probably a pulse-burst (sensor error) and that usually happens when only one pulse should be registered.
    // if car is stopped don't accept pulses at all
    if(m_currentDirection == 0)
    {
        m_wheelDelta_left = 0;
        m_wheelDelta_right = 0;
    }
    else if((m_current_turning_radius < 0) && ( (m_wheelDelta_left) > (1 + ( ((relativeDelta) + 0.1) * m_wheelDelta_right )) )) //left wheel too fast?
    {
        m_wheelDelta_left = floor(1 + ( ((relativeDelta) + 0.1) * m_wheelDelta_right ));
    }
    else if((m_current_turning_radius < 0) && ( (1 + (m_wheelDelta_left / (relativeDelta - 0.1))) < (m_wheelDelta_right ) ))    //right wheel too fast?
    {
        m_wheelDelta_right = floor(1 + (m_wheelDelta_left / (relativeDelta - 0.1)));
    }
    else if ((m_current_turning_radius >= 0) && ( (m_wheelDelta_right) > (1 + ( (relativeDelta + 0.1) * m_wheelDelta_left )) )) //right wheel too fast?
    {
        m_wheelDelta_right = floor(1 + ( (relativeDelta + 0.1) * m_wheelDelta_left ));
    }
    else if ((m_current_turning_radius >= 0) && ( (1 + (m_wheelDelta_right / (relativeDelta - 0.1))) < ( m_wheelDelta_left ) )) //left wheel too fast?
    {
        m_wheelDelta_left = floor(1 + (m_wheelDelta_right / (relativeDelta - 0.1)));
    }

    RETURN_NOERROR;
}


// ****************************************************************************************************************
//                                          Odometry Main calculations
// ****************************************************************************************************************

tResult SWE_Odometry::CalcOdometryStep(tTimeStamp time_now, tTimeStamp time_last)
{
    // calculate the movement that happened since the last step and sum it up
    tFloat32 radius;
    tFloat32 X_distance;
    tFloat32 Y_distance;
    tFloat32 extrapol_heading;
    tFloat32 heading_diff;
    tFloat32 distance_driven;
    tFloat32 time_intervall;

    // ---------- get deltas -----------
    time_intervall = (tFloat32)(time_now - time_last) / TIMESTAMP_RESOLUTION_ADTF ;

    extrapol_heading = m_heading_now + GetExtrapolatedHeadingDiff(time_now);
    heading_diff = GetAngleDiff(extrapol_heading, m_heading_lastStep) ;

    distance_driven = time_intervall * m_velocityCombined;

    // corner cases
    if(m_currentDirection == 0) //if car stopped no movement (should have) occurred --> prevents impact of sensor drift
    {
        heading_diff = 0;
        distance_driven = 0;
    }

    // ------ calculate current turning radius ---------

    if (heading_diff != 0)
    {
        radius = (distance_driven / heading_diff);
    }
    else
    {
        radius = 100000;
    }

    // keep radius far enough from maxvalue of variables
    if (radius > 100000)
        radius = 100000;
    else if (radius < -100000)
        radius = -100000;

    //smoothed radius for pulse filter
    m_current_turning_radius = 0.25 * radius + 0.75 * m_current_turning_radius;

    //absolute value...
    radius = fabs(radius);


    // -------- calculate relative movement -------

    if( radius >= 10000)  // linear interpolation if infinite (or very big) radius
    {
        Y_distance = sin(heading_diff*0.5) * distance_driven;
        X_distance = cos(heading_diff*0.5) * distance_driven;
    }
    else //non-linear interpolation based on circle segments for bigger angular steps
    {
        if (distance_driven == 0)
        {
            X_distance = Y_distance = 0;
        }
        else
        {
            X_distance = sin(heading_diff) * radius;
            Y_distance = (1 - cos(heading_diff)) * radius;

            //negative quadrants
            if ((distance_driven > 0) && (heading_diff < 0))
            {
                X_distance *= (-1);
            }
            else if ((distance_driven < 0) && (heading_diff > 0))
            {
                X_distance *= (-1);
            }

            if ((heading_diff > 0) && (distance_driven < 0))
            {
                Y_distance *= (-1);
            }
            else if ((heading_diff < 0) && (distance_driven > 0))
            {
                Y_distance *= (-1);
            }
        }
    }

    // ----- sum everything up -----------

    m_distanceX_sum += X_distance;
    m_distanceY_sum += Y_distance;
    m_heading_sum   += heading_diff;

    // -------- update data for new step --------

    m_heading_lastStep = extrapol_heading;

    RETURN_NOERROR;
}


// ****************************************************************************************************************
//                                                   Helpers
// ****************************************************************************************************************

tFloat32 SWE_Odometry::GetAngleDiff(tFloat32 angle_new, tFloat32 angle_old)
{
    // get the smaller angle between two measurements + handle the corner cases when passing 0 and +/- PI
    tFloat32 angleDiff;


    angleDiff = angle_new - angle_old;

    if (angleDiff > (MY_PI))
    {
        angleDiff = angleDiff - (2*MY_PI);
    }
    else if (angleDiff < -(MY_PI)) //positive turn through -Pi / +Pi border
    {
        angleDiff = angleDiff + (2*MY_PI);
    }

    return angleDiff;
}

tFloat32 SWE_Odometry::CalcDistance(tInt32 direction, tFloat32 pulses)
{
    tFloat32 outputdata;

    outputdata = (pulses / m_tickPerTurn) * m_wheelCircumfence * direction;

    return outputdata;
}

tTimeStamp SWE_Odometry::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime(); // in microseconds
}

tFloat32  SWE_Odometry::FilterVelocity(tFloat32 filter_strength, tFloat32 old_velocity, tFloat32 new_velocity)
{
    tFloat32 outputval;
    outputval = new_velocity * (1.0 - filter_strength) + old_velocity* filter_strength;

    return outputval;
}

tFloat32 SWE_Odometry::GetExtrapolatedHeadingDiff(tTimeStamp time_now)
{
    // calculate the angular velocity between the last yaw samples and get the estimated/extrapolated difference in heading between the last sensor reading and now
    tFloat32 heading_diff_old;
    tFloat32 heading_diff_now;
    tFloat32 time_diff_old;
    tFloat32 time_diff_now;

    heading_diff_old =  GetAngleDiff(m_yawSlidingWindow.getEndValue(), m_yawSlidingWindow.getBeginValue());
    time_diff_old = (m_yawSlidingWindow.getEndTime() - m_yawSlidingWindow.getBeginTime()) * (TIMESTAMP_RESOLUTION_ADTF / TIMESTAMP_RESOLUTION); //is in ms

    time_diff_now = time_now - m_yawSampleTime_now; //how old is the current sample?

    if (time_diff_old > 0)
    {
        heading_diff_now = heading_diff_old * (time_diff_now / time_diff_old);
    }
    else
    {
        heading_diff_now = 0;
    }

    return heading_diff_now;
}



// ****************************************************************************************************************
//                                                   Transmit Data
// ****************************************************************************************************************


tResult SWE_Odometry::SendOdometry(tTimeStamp timestamp)
{

    //create new media sample
    cObjectPtr<IMediaCoder> pCoder;
    cObjectPtr<IMediaSample> pMediaSampleOutput;

    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescOdometryOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescOdometryOut->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&(timestamp));
    pCoder->Set("distance_x", (tVoid*)&(m_distanceX_sum));
    pCoder->Set("distance_y", (tVoid*)&(m_distanceY_sum));
    pCoder->Set("angle_heading", (tVoid*)&(m_heading_sum));
    pCoder->Set("velocity", (tVoid*)&(m_velocityCombined));
    pCoder->Set("distance_sum", (tVoid*)&(m_distanceAllSum));
    m_pCoderDescOdometryOut->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputOdometry.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}


tResult SWE_Odometry::SendVelocity()
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderVelocityOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    m_pCoderVelocityOut->WriteLock(pMediaSample, &pCoder);

    //pCoder->Set("f32Value", (tVoid*)&(m_velocityFiltered));
    pCoder->Set("f32Value", (tVoid*)&(debugvar));

    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&m_lastPinEvent);
    m_pCoderVelocityOut->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputVelocity.Transmit(pMediaSample));

    RETURN_NOERROR;
}
