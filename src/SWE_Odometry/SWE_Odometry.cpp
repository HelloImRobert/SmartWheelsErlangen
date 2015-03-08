#include "SWE_Odometry.h"
#include "SWE_cSmartSlidingWindow.h"

#define WHEELBASE 359.0f
#define WHEELPULSE_PER_TURN 16.0f
#define TIMESTAMP_RESOLUTION 1000.0f
#define MAX_TICK_FILTER_VELOCITY 1000.0f //in mm/s
#define MY_PI 3.14159265359f
#define MINIMUM_TURNING_RADIUS 600 // in mm -> actually ~ 680-ish ?


ADTF_FILTER_PLUGIN("SWE_Odometry", OID_ADTF_SWE_ODOMETRY, SWE_Odometry)


SWE_Odometry::SWE_Odometry(const tChar* __info) : cFilter(__info), m_SlidingWindowCntLeftWheel(21, (0.33 * TIMESTAMP_RESOLUTION)), m_SlidingWindowCntRightWheel(21, (0.33 * TIMESTAMP_RESOLUTION)), m_mutex()  //TODO
{

    m_steeringAngle = 0;
    //m_slippageAngle = 0;
    m_velocityLeft = 0;
    m_velocityRight = 0;
    m_velocityFiltered = 0;
    m_velocityUnfiltered = 0;

    m_oldTimeStamp = GetTime();
    m_lastPinEvent = GetTime();
    m_lastTriggerTime = GetTime();

    m_distanceX_sum = 0;
    m_distanceY_sum = 0;
    m_heading_sum = 0;
    m_distanceAllSum = 0;

    m_lastwheelCounter_left = 0;
    m_lastwheelCounter_right = 0;

    m_wheelCounter_left = 0;
    m_wheelCounter_right = 0;


    //DEBUG:
    //m_currentDirection = 0;
    m_currentDirection = 1;

    m_wheelsync = false;

    m_heading_lastStep = 0.0;
    m_heading_now = 0.0;
    m_heading_old_interpol = 0.0;

    m_errorPulses_left = 0;
    m_errorPulses_right = 0;

    SetPropertyFloat("Velocity Filter Strength",0.3);
    SetPropertyFloat("Wheel circumfence in mm",329);
    SetPropertyInt("time and pulse resolution of the velocity calculation 2-20",10);

}

SWE_Odometry::~SWE_Odometry()
{
}

tResult SWE_Odometry::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
    
    	m_SlidingWindowCntLeftWheel.Reset();
        m_SlidingWindowCntRightWheel.Reset();

        m_lastwheelCounter_left = 0;
        m_lastwheelCounter_right = 0;

        m_wheelCounter_left = 0;
        m_wheelCounter_right = 0;

        m_errorPulses_left = 0;
        m_errorPulses_right = 0;

        m_wheelsync = false;

        m_distanceAllSum = 0;
        m_distanceX_sum = 0;
        m_distanceY_sum = 0;
        m_heading_sum = 0;

        m_oldTimeStamp = GetTime();
        m_lastPinEvent = GetTime();
        m_lastTriggerTime = GetTime();


        //DEBUG:
        //m_currentDirection = 0;
        m_currentDirection = 1;

        m_velocityLeft = 0;
        m_velocityRight = 0;
        m_velocityFiltered = 0;
        m_velocityUnfiltered = 0;

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
        //RETURN_IF_FAILED(m_oInputSteeringAngle.Create("SteeringAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        //RETURN_IF_FAILED(RegisterPin(&m_oInputSteeringAngle));

        tChar const * strDescSignalValue_yaw = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue_yaw);
        cObjectPtr<IMediaType> pTypeSignalValue_yaw = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue_yaw,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue_yaw->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_yaw));

        RETURN_IF_FAILED(m_oInputYaw.Create("Gyro_Yaw", pTypeSignalValue_yaw, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputYaw));


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

        RETURN_IF_FAILED(m_oOutputVelocity.Create("Velocity", pTypeSignalValue_Velocity, static_cast<IPinEventSink*> (this)));
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

        m_filterStrength = (tFloat32)GetPropertyFloat("Velocity Filter Strength", 0.3);
        m_wheelCircumfence = (tFloat32)GetPropertyFloat("Wheel circumfence in mm",329);
        m_velocityResolution = (tFloat32)GetPropertyInt("time and pulse resolution of the velocity calculation 2-20",10);


        //prevent stupid filter values
        if(m_filterStrength >= 1.0)
        {
            m_filterStrength = 0.99;
        }
        else if (m_filterStrength < 0.0)
        {
            m_filterStrength = 0.0;
        }

        if(m_velocityResolution > 20)
        {
            m_velocityResolution = 20;
        }
        else if (m_velocityResolution < 2)
        {
            m_velocityResolution = 2;
        }

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

tResult SWE_Odometry::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {

        m_mutex.Enter(); //serialize this filter for data consistency

        /*!
        if (pSource == &m_oInputSteeringAngle) //TODO
        {
            //m_lastPinEvent = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_buffer);
            //pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&TimeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            //processing the data
            m_steeringAngle = m_buffer;

        }
        else
        */

        if (pSource == &m_oInputWheelLeft)
        {
            tTimeStamp timeStamp;

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_wheelCounter_left);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);


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

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_wheelCounter_right);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

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
            //tTimeStamp timestamp;
            current_time = GetTime();

            //do odometry step first
            CalcOdometryStep( current_time, m_lastPinEvent );
            m_lastPinEvent = current_time;


            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal_direction->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_currentDirection);
            //pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal_direction->Unlock(pCoderInput);

            //process input data
            if (m_currentDirection == 0) //on car stopped signal reset everything (we know we're standing still)
            {
                m_SlidingWindowCntLeftWheel.Reset();
                m_SlidingWindowCntRightWheel.Reset();
                m_velocityFiltered = 0;
                m_velocityLeft = 0;
                m_velocityRight = 0;
                m_velocityUnfiltered = 0;
            }

            //DEBUG
            debugvar = m_currentDirection;

        }
        else if (pSource == &m_oInputYaw)
        {
            tTimeStamp current_time;
            current_time = GetTime();

            m_yawSampleTime_old_interpol = m_yawSampleTime_now;
            m_yawSampleTime_now = current_time;
            m_heading_old_interpol = m_heading_now;

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal_yaw->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_heading_now);
            //pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal_yaw->Unlock(pCoderInput);

        }
        else if (pSource == &m_oInputTrigger)
        {
            tTimeStamp current_time;
            current_time = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderBool->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            //pCoderInput->Get("bValue", (tVoid*)&m_buffer); // don't care
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_lastTriggerTime);
            m_pCoderBool->Unlock(pCoderInput);

            //process the data
            CalcOdometryStep( current_time, m_lastPinEvent );
            m_lastPinEvent = current_time;

            SendOdometry(m_lastTriggerTime);

            // reset for next trigger interval
            m_distanceX_sum = 0;
            m_distanceY_sum = 0;
            m_heading_sum = 0;
        }
    }

    m_mutex.Leave();

    RETURN_NOERROR;
}

tResult SWE_Odometry::ProcessPulses(tTimeStamp timeStamp)
{

    tTimeStamp current_time;
    current_time = GetTime();

    FilterPulses();

    m_distanceAllSum = m_distanceAllSum + (CalcDistance(m_currentDirection, (m_wheelCounter_left - m_lastwheelCounter_left)) / 2.0) + ( CalcDistance(m_currentDirection, (m_wheelCounter_right - m_lastwheelCounter_right)) / 2.0);
    m_lastwheelCounter_left = m_wheelCounter_left;
    m_lastwheelCounter_right = m_wheelCounter_right;

    m_SlidingWindowCntLeftWheel.AddNewValue(m_wheelCounter_left, timeStamp);
    m_SlidingWindowCntRightWheel.AddNewValue(m_wheelCounter_right, timeStamp);

    CalcVelocity();

    CalcOdometryStep(current_time,  m_lastPinEvent);

    m_lastPinEvent = current_time;

    SendVelocity();

    RETURN_NOERROR;
}

tTimeStamp SWE_Odometry::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tResult SWE_Odometry::CalcVelocity()
{
    if (m_SlidingWindowCntLeftWheel.GetTime() > 0)
    {
        m_velocityLeft = (((tFloat32)m_SlidingWindowCntLeftWheel.GetPulses() / m_SlidingWindowCntLeftWheel.GetTime() ) / WHEELPULSE_PER_TURN) * m_wheelCircumfence * TIMESTAMP_RESOLUTION * m_currentDirection;
    }
    else
    {
        m_velocityLeft = 0.0;
    }

    if (m_SlidingWindowCntRightWheel.GetTime() > 0)
    {
        m_velocityRight = (((tFloat32)m_SlidingWindowCntRightWheel.GetPulses() / m_SlidingWindowCntRightWheel.GetTime() ) / WHEELPULSE_PER_TURN) * m_wheelCircumfence * TIMESTAMP_RESOLUTION * m_currentDirection;
    }
    else
    {
        m_velocityRight = 0.0;
    }

    m_velocityUnfiltered = ((m_velocityLeft + m_velocityRight) / 2.0);
    m_velocityFiltered = FilterVelocity(m_filterStrength, m_velocityFiltered, m_velocityUnfiltered );

    RETURN_NOERROR;
}

tResult  SWE_Odometry::FilterVelocity(tFloat32 filter_strength, tFloat32 old_velocity, tFloat32 new_velocity)
{
    tFloat32 outputval;
    outputval = new_velocity * (1.0 - filter_strength) + old_velocity* filter_strength;

    return outputval;
}

tResult SWE_Odometry::FilterPulses()
{
    tFloat32 delta_count_left;
    tFloat32 delta_count_right;
    tFloat32 tempCounter;

    //compensate for errors => IMPROVEMENT: better would be the registration of only the pulse delta right at the input pin
    m_wheelCounter_left = m_wheelCounter_left - m_errorPulses_left;
    m_wheelCounter_right = m_wheelCounter_right - m_errorPulses_right;

    delta_count_left = m_wheelCounter_left - m_lastwheelCounter_left;
    delta_count_right = m_wheelCounter_right - m_lastwheelCounter_right;

    //if the delta of pulses between both wheels differs by more than 1 pulse + 100% then its probably a pulse-burst (sensor error)
    // if car is stopped don't accept pulses at all
    // possible improvement: make the relative part (the worst-case 100%) dependend on steering angle
    if(m_currentDirection == 0)
    {
        m_errorPulses_left += (tInt32)(m_wheelCounter_left - m_lastwheelCounter_left);
        m_errorPulses_right += (tInt32)(m_wheelCounter_right - m_lastwheelCounter_right);

        m_wheelCounter_left = m_lastwheelCounter_left;
        m_wheelCounter_right = m_lastwheelCounter_right;
    }
    else if(delta_count_left > delta_count_right)
    {
        if( (delta_count_left - delta_count_right) > (1 + ( 1.0 * delta_count_right )) )
        {
            tempCounter = m_lastwheelCounter_left + delta_count_right; //guess the delta to be the same as the other wheel
            m_errorPulses_left = (tInt32)(m_wheelCounter_left - tempCounter); //those were too many
            m_wheelCounter_left = tempCounter;
        }
    }
    else if (delta_count_left < delta_count_right)
    {
        if( (delta_count_right - delta_count_left) > (1 + ( 1.0 * delta_count_left )) )
        {
            tempCounter = m_lastwheelCounter_right + delta_count_left; //guess the delta to be the same as the other wheel
            m_errorPulses_right += (tInt32)(m_wheelCounter_right - tempCounter); //those were too many
            m_wheelCounter_right = tempCounter;
        }
    }

    RETURN_NOERROR;
}

tFloat32 SWE_Odometry::CalcDistance(tInt32 direction, tFloat32 pulses)
{
    tFloat32 outputdata;

    outputdata = ((tFloat32)pulses / (tFloat32)WHEELPULSE_PER_TURN) * m_wheelCircumfence * direction;


    return outputdata;
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



    //DEBUG:
    pCoder->Set("f32Value", (tVoid*)&(m_velocityFiltered));
    //pCoder->Set("f32Value", (tVoid*)&(debugvar));



    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&m_lastPinEvent);
    m_pCoderVelocityOut->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputVelocity.Transmit(pMediaSample));

    RETURN_NOERROR;
}

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
    pCoder->Set("velocity", (tVoid*)&(m_velocityUnfiltered));
    pCoder->Set("distance_sum", (tVoid*)&(m_distanceAllSum));
    m_pCoderDescOdometryOut->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputOdometry.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}

tResult SWE_Odometry::CalcOdometryStep(tTimeStamp time_now, tTimeStamp time_last) //TODO
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
    time_intervall = (tTimeStamp)(time_now - time_last) / 10E6; //DEBUG

    extrapol_heading = m_heading_now + GetExtrapolatedHeadingDiff(time_now);
    heading_diff = GetAngleDiff(extrapol_heading, m_heading_lastStep) ;

    distance_driven = time_intervall * m_velocityUnfiltered;


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
        radius = 1000000;
    }

    //absolute value...
    if(radius < 0)
        radius *= (-1);


    // -------- calculate relative movement -------

    if(radius >= 10000)  // linear interpolation if infinite (or very big) radius
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

            if (distance_driven < 0)
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

tFloat32 SWE_Odometry::GetAngleDiff(tFloat32 angle_new, tFloat32 angle_old)
{
    // get the smaller angle between two measurements + handle the corner cases when passing 0 and +/- PI
    tFloat32 angleDiff;

    angleDiff = (angle_new + MY_PI) - (angle_old + MY_PI); //odometry sends them as +/-PI we want 0 - 2PI

    if((angleDiff < MY_PI) || (angleDiff > -MY_PI)) //if zero has not been passed....
    {
        angleDiff = angle_new - angle_old;
    }
    else if (angleDiff > MY_PI)
    {
        angleDiff = (angle_new - (2*MY_PI)) - angle_old;
    }
    else if (angleDiff < -MY_PI)
    {
        angleDiff = (angle_new + (2*MY_PI)) - angle_old;
    }
    else
    {
        angleDiff = MY_PI;
    }

    return angleDiff;
}

tFloat32 SWE_Odometry::GetExtrapolatedHeadingDiff(tTimeStamp time_now)
{
    // calculate the angular velocity between the last two yaw samples and get the difference in heading between the last sensor reading and now
    tFloat32 heading_diff_old;
    tFloat32 heading_diff_now;
    tFloat32 time_diff_old;
    tFloat32 time_diff_now;

    heading_diff_old = GetAngleDiff( m_heading_now, m_heading_old_interpol);
    time_diff_old = m_yawSampleTime_now - m_yawSampleTime_old_interpol;
    time_diff_now = time_now - m_yawSampleTime_now;

    heading_diff_now = heading_diff_old * (time_diff_now / time_diff_old);

    return heading_diff_now;
}
