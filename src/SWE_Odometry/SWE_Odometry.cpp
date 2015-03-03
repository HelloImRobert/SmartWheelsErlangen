#include "SWE_Odometry.h"
#include "SWE_cSmartSlidingWindow.h"


#define WHEELBASE 359.0f
#define WHEELTICK_PER_TURN 16.0f


ADTF_FILTER_PLUGIN("SWE_Odometry", OID_ADTF_SWE_ODOMETRY, SWE_Odometry)


SWE_Odometry::SWE_Odometry(const tChar* __info) : cFilter(__info), m_SlidingWindowCntLeftWheel(21, 500), m_SlidingWindowCntRightWheel(21, 500), m_mutex()  //TODO
{

    m_steeringAngle = 0;
    m_slippageAngle = 0;
    m_velocityLeft = 0;
    m_velocityRight = 0;
    m_velocityFiltered = 0;
    m_velocityUnfiltered = 0;
    m_velocityRight_last = 0;
    m_velocityLeft_last = 0;

    m_currTimeStamp = 0;
    m_oldTimeStamp = GetTime();
    m_lastPinEvent = GetTime();
    m_lastTriggerTime = GetTime();
    m_lastRightWheelTime = GetTime();
    m_lastLeftWheelTime = GetTime();

    m_distanceX_sum = 0;
    m_distanceY_sum = 0;
    m_heading_sum = 0;
    m_distanceAllSum = 0;

    m_lastwheelCounter_left = 0;
    m_lastwheelCounter_right = 0;

    m_wheelCounter_left = 0;
    m_wheelCounter_right = 0;

    m_currentDirection = 1;


    SetPropertyFloat("Velocity Filter Strength",0.5);
    SetPropertyFloat("Wheel circumfence in mm",329);
    SetPropertyInt("time and tick resolution of the velocity calculation 2-20",10);

}

SWE_Odometry::~SWE_Odometry()
{
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
        RETURN_IF_FAILED(m_oInputSteeringAngle.Create("SteeringAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSteeringAngle));
        RETURN_IF_FAILED(m_oInputYaw.Create("Gyro_Yaw", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputYaw));

        // ------- velocity output pin --------------
        RETURN_IF_FAILED(m_oOutputVelocity.Create("Velocity", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputVelocity));




        // ---- tBoolSignal value ---------
        // ------ input pins --------------
        tChar const * strDescBool = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBool);
        cObjectPtr<IMediaType> pTypeBoolData = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBool,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeBoolData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderBool));

        RETURN_IF_FAILED(m_oInputDirection.Create("Direction", pTypeBoolData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputDirection));
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

        m_filterStrength = (tFloat32)GetPropertyFloat("Velocity Filter Strength", 0.5);
        m_wheelCircumfence = (tFloat32)GetPropertyFloat("Wheel circumfence in mm",329);
        m_velocityResolution = (tFloat32)GetPropertyInt("time and tick resolution of the velocity calculation 2-20",10);

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

        m_mutex.Enter();

        if (pSource == &m_oInputSteeringAngle) //TODO
        {
            m_lastPinEvent = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_buffer);
            //pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_currTimeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            //processing the data
            m_steeringAngle = m_buffer;

            //CalcSingleOdometry( m_currTimeStamp - m_oldTimeStamp );
            //m_oldTimeStamp = m_currTimeStamp;



        }
        else if (pSource == &m_oInputWheelLeft) //TODO
        {

            m_lastPinEvent = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_buffer);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_currTimeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            //processing the data


            m_wheelCounter_left = FilterTicks(m_lastLeftWheelTime, m_currTimeStamp, m_lastwheelCounter_left, m_buffer);
            m_SlidingWindowCntLeftWheel.AddNewValue(m_wheelCounter_left, m_currTimeStamp);

            m_distanceAllSum = m_distanceAllSum + (CalcDistance(m_currentDirection, (m_wheelCounter_left - m_lastwheelCounter_left)) / 2.0);
            m_lastwheelCounter_left = m_wheelCounter_left;

            CalcVelocity();

            CalcSingleOdometry( m_currTimeStamp - m_oldTimeStamp );
            SendVelocity();

            m_oldTimeStamp = m_currTimeStamp;
            m_lastLeftWheelTime = m_currTimeStamp;


        }
        else if (pSource == &m_oInputWheelRight) //TODO
        {
            m_lastPinEvent = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_buffer);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_currTimeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            //processing the data

            m_wheelCounter_right = FilterTicks(m_lastRightWheelTime, m_currTimeStamp, m_lastwheelCounter_right, m_buffer);
            m_SlidingWindowCntRightWheel.AddNewValue(m_wheelCounter_left, m_currTimeStamp);

            m_distanceAllSum = m_distanceAllSum + ( CalcDistance(m_currentDirection, (m_wheelCounter_right - m_lastwheelCounter_right)) / 2.0);
            m_lastwheelCounter_right = m_wheelCounter_right;

            CalcVelocity();

            CalcSingleOdometry( m_currTimeStamp - m_oldTimeStamp );

            SendVelocity();

            m_oldTimeStamp = m_currTimeStamp;
            m_lastRightWheelTime = m_currTimeStamp;
        }
        else if (pSource == &m_oInputDirection)
        {

            m_lastPinEvent = GetTime();
            tBool mybuffer;
            mybuffer = true;

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderBool->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            pCoderInput->Get("bValue", (tVoid*)&mybuffer);
            //pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_currTimeStamp);
            m_pCoderBool->Unlock(pCoderInput);

            if (((mybuffer == true) && (m_currentDirection < 0)) || ((mybuffer == false) && (m_currentDirection > 0))) //when direction change, reset the speeds, (we know we're standing still)
            {
                m_SlidingWindowCntLeftWheel.Reset();
                m_SlidingWindowCntRightWheel.Reset();
                m_velocityFiltered = 0;
                m_velocityLeft = 0;
                m_velocityRight = 0;
                m_velocityUnfiltered = 0;
            }

            if (mybuffer)
            {
                m_currentDirection = 1;
            }
            else
            {
                m_currentDirection = -1;
            }

            /*
            //process the data
            timeIntervall = m_currTimeStamp - m_oldTimeStamp;

            CalcSingleOdometry(timeIntervall);

            m_oldTimeStamp = m_currTimeStamp;
            */

        }
        else if (pSource == &m_oInputYaw) //TODO
        {

        }
        else if (pSource == &m_oInputTrigger)
        {

            m_lastPinEvent = GetTime();

            // read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderBool->Lock(pMediaSample, &pCoderInput));

            //get values from media sample
            //pCoderInput->Get("bValue", (tVoid*)&m_buffer); // don't care
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_lastTriggerTime);
            m_pCoderBool->Unlock(pCoderInput);

            m_currTimeStamp = m_lastTriggerTime;

            //process the data
            CalcSingleOdometry(m_currTimeStamp - m_oldTimeStamp);
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

tTimeStamp SWE_Odometry::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tResult SWE_Odometry::CalcVelocity()
{
    if (m_SlidingWindowCntLeftWheel.GetTicks() != 0)
        m_velocityLeft = ((m_SlidingWindowCntLeftWheel.GetTicks() / (m_SlidingWindowCntLeftWheel.GetTime() )) / WHEELTICK_PER_TURN) * m_wheelCircumfence * 1000000;
    else
        m_velocityLeft = 0;

    if (m_SlidingWindowCntRightWheel.GetTicks() != 0)
        m_velocityRight = ((m_SlidingWindowCntRightWheel.GetTicks() / (m_SlidingWindowCntRightWheel.GetTime() )) / WHEELTICK_PER_TURN) * m_wheelCircumfence * 1000000;
    else
        m_velocityRight = 0;


    m_velocityUnfiltered = ((m_velocityLeft + m_velocityRight) / 2.0);


    //DEBUG
    //m_velocityUnfiltered = (tFloat32)m_SlidingWindowCntLeftWheel.GetTicks();


    m_velocityFiltered = FilterVelocity(m_filterStrength, m_velocityFiltered, m_velocityUnfiltered );

    RETURN_NOERROR;
}

tResult  SWE_Odometry::FilterVelocity(tFloat32 filter_strength, tFloat32 old_velocity, tFloat32 new_velocity)
{
    tFloat32 outputval;
    outputval = new_velocity * (1 - filter_strength) + old_velocity* filter_strength;

    return outputval;
}

tFloat32 SWE_Odometry::FilterTicks (tTimeStamp last_tick, tTimeStamp curr_tick, tFloat32 last_Count, tFloat32 curr_count) //TODO placeholder
{
    return curr_count;
}

tFloat32 SWE_Odometry::CalcDistance(tInt32 direction, tFloat32 ticks)
{
    tFloat32 outputdata;

    outputdata = (ticks / WHEELTICK_PER_TURN) * m_wheelCircumfence * direction;


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
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    m_pCoderDescSignal->WriteLock(pMediaSample, &pCoder);

    pCoder->Set("f32Value", (tVoid*)&(m_velocityFiltered));
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&m_lastPinEvent);
    m_pCoderDescSignal->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(pMediaSample->SetTime(m_lastPinEvent));
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
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(timestamp));
    RETURN_IF_FAILED(m_oOutputOdometry.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}

tResult SWE_Odometry::CalcSingleOdometry(tTimeStamp timeIntervall) //TODO
{
    tFloat32 radius;
    tFloat32 distance;
    tFloat32 angle;


    //calculate single movement step
    if ( (m_steeringAngle + m_slippageAngle) > 1.4) //prevent crazy steering angles
    {
        radius = WHEELBASE / ( tan( 1.4 ) );
    }
    else
    {
        radius = WHEELBASE / ( tan( m_steeringAngle + m_slippageAngle ) );
    }

    if ( timeIntervall < 0 ) timeIntervall = 0;

    angle = m_velocityUnfiltered * timeIntervall / radius;
    distance = 2 * radius * sin( angle * 0.5 );


    //divide vector into components + add to sum

    //m_distanceAllSum = m_distanceAllSum + distance;
    m_distanceX_sum = m_distanceX_sum + cos( angle + m_heading_sum ) * distance;
    m_distanceY_sum = m_distanceY_sum + sin( angle + m_heading_sum ) * distance;

    m_heading_sum = m_heading_sum + angle;

    // stuff



    RETURN_NOERROR;
}
