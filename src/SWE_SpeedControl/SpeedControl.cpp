#include "stdafx.h"
#include "SpeedControl.h"

ADTF_FILTER_PLUGIN("SWE Motor Speed Controller", OID_ADTF_SWE_SPEEDCONTROL, SpeedControl)

SpeedControl::SpeedControl(const tChar* __info) : cFilter(__info), m_velocity(0), m_setPoint(0), m_currentState(0), m_lastState(0), m_goingForwards(true), m_lastSampleTime(0), m_no_wait(true), m_last_pwm(90), m_last_brakeLights(0), m_last_reverseLights(0), m_last_goingForwards(1)
{
    SetPropertyFloat("Gear 3 PWM value",105); //the pwm value sent to the car motor when driving in this gear. In degrees 0-180 => 90 = stop/neutral
    SetPropertyFloat("Gear 3 speed threshold",150); // the speed in mm/s (measured) at which the controller decides it has reached the desired gear/speed

    SetPropertyFloat("Gear 2 PWM value",100);
    SetPropertyFloat("Gear 2 speed threshold",100);

    SetPropertyFloat("Gear 1 PWM value",95);
    SetPropertyFloat("Gear 1 speed threshold",50);

    SetPropertyFloat("Gear -1 PWM value", 85);
    SetPropertyFloat("Gear -1 speed threshold",-50);

    SetPropertyFloat("Gear -2 PWM value", 80);
    SetPropertyFloat("Gear -2 speed threshold",-100);

    SetPropertyFloat("light brake strength", 0.05); //pwm value for light braking
    SetPropertyFloat("strong brake strength",0.1);

    SetPropertyFloat("acceleration boost", 1.2); //pwm boost value for acceleration 1.0 = no boost

    SetPropertyFloat("Gear 0 speed window", 20); //threshold window for reliable stopping in mm/s

    SetPropertyFloat("PWM scaler", 1.0); //all pwm values are multiplied by this value

    SetPropertyInt("Stop Time in ms", 500); //minimum time the car stands still once it stops

}

SpeedControl::~SpeedControl()
{
}

tResult SpeedControl::CreateInputPins(__exception)
{	

    RETURN_IF_FAILED(m_oInputVelocity.Create("car velocity", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputVelocity));

    RETURN_IF_FAILED(m_oInputSetPoint.Create("set point gear_speed", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetPoint));
    RETURN_NOERROR;
}

tResult SpeedControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    // PWM output
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));


    RETURN_IF_FAILED(m_oOutputPWM.Create("PWMvalue", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputPWM));


    // Struct for brake
    tChar const * strDescLightOutput = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescLightOutput);
    cObjectPtr<IMediaType> pTypeLightData = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescLightOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLightData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCodeOutputbrakelight));

    RETURN_IF_FAILED(m_oOutputbrakelight.Create("Brakelight", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputbrakelight));

    RETURN_IF_FAILED(m_oOutputreverse.Create("ReverseLight", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputreverse));

    RETURN_IF_FAILED(m_oOutputDirection.Create("Direction", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputDirection));

    RETURN_NOERROR;
}

tResult SpeedControl::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {

    }
    else if(eStage == StageGraphReady)
    {

    }

    tFloat32 thresholdWindow_0 = (tFloat32)GetPropertyFloat("Gear 0 speed window", 20);

    tFloat32 boostValue = (tFloat32)GetPropertyFloat("acceleration boost", 1.2);

    m_pwm_p3 = (tFloat32)GetPropertyFloat("Gear 3 PWM value",105);
    m_pwm_boost_p3 = 90 + (m_pwm_p3 - 90) * boostValue;
    m_threshold_p3 = (tFloat32)GetPropertyFloat("Gear 3 speed threshold",150);

    m_pwm_p2 = (tFloat32)GetPropertyFloat("Gear 2 PWM value",100);
    m_pwm_boost_p2 = 90 + (m_pwm_p2 - 90) * boostValue;
    m_threshold_p2 = (tFloat32)GetPropertyFloat("Gear 2 speed threshold",100);

    m_pwm_p1 = (tFloat32)GetPropertyFloat("Gear 1 PWM value",95);
    m_pwm_boost_p1 = 90 + (m_pwm_p1 - 90) * boostValue;
    m_threshold_p1 = (tFloat32)GetPropertyFloat("Gear 1 speed threshold",50);


    m_pwm_0 = 90;
    m_threshold_p0 = 0 + thresholdWindow_0;
    m_threshold_n0 = 0 - thresholdWindow_0;


    m_pwm_n1 = (tFloat32)GetPropertyFloat("Gear -1 PWM value",85);
    m_pwm_boost_n1 = 90 + (m_pwm_n1 - 90) * boostValue;
    m_threshold_n1 = (tFloat32)GetPropertyFloat("Gear -1 speed threshold",-50);

    m_pwm_n2 = (tFloat32)GetPropertyFloat("Gear -2 PWM value",80);
    m_pwm_boost_n2 = 90 + (m_pwm_n2 - 90) * boostValue;
    m_threshold_n2 = (tFloat32)GetPropertyFloat("Gear -2 speed threshold",-100);



    m_lightBrake = 90 - 90*(tFloat32)GetPropertyFloat("light brake strength", 0.05);
    m_inv_lightBrake = 90 + 90*(tFloat32)GetPropertyFloat("light brake strength", 0.05);

    m_lightBrake = 90 - 90*(tFloat32)GetPropertyFloat("strong brake strength",0.1);
    m_inv_lightBrake = 90 + 90*(tFloat32)GetPropertyFloat("strong brake strength",0.1);

    m_pwmScaler = (tFloat32)GetPropertyFloat("PWM scaler", 1.0);

    m_stopTime =  (tInt32)GetPropertyInt("Stop Time in ms", 500) * 1000; //in us

    RETURN_NOERROR;
}

tResult SpeedControl::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult SpeedControl::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult SpeedControl::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult SpeedControl::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {

        RETURN_IF_POINTER_NULL( pMediaSample);

        if (pSource == &m_oInputVelocity)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;
            tFloat32 outputData = 0;

            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoder);


            //execute controller
            m_velocity = value;
            outputData = GetControllerValue();

            SetPWM(outputData);

        }
        else if (pSource == &m_oInputSetPoint)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //write values with zero
            tInt16 value = 0;
            tUInt32 timeStamp = 0;
            tFloat32 outputData = 0;

            //get values from media sample
            pCoder->Get("i8Value", (tVoid*)&value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoder);


            if ( value < -2 ) // prevent stupid values
            {
                value = -2;
            }
            else if ( value > 3)
            {
                value = 3;
            }

            m_setPoint = value;

            outputData = GetControllerValue();

            SetPWM(outputData);
        }
        else
            RETURN_NOERROR;
    }
    RETURN_NOERROR;
}

tResult SpeedControl::UpdateState()
{
    m_currentState = 0;

    if (m_velocity > m_threshold_p2) //is the car going forwards?
    {
        m_currentState = 3;
    }
    else if (m_velocity > m_threshold_p1)
    {
        m_currentState = 2;
    }
    else if (m_velocity > m_threshold_p0)
    {
        m_currentState = 1;
    }

    if ((m_velocity > m_threshold_n0) && (m_velocity < m_threshold_p0)) //has it stopped?
    {
        m_currentState = 0;
    }

    if (m_velocity < m_threshold_n1) //is it going backwards?
    {
        m_currentState = -1;
    }
    else if (m_velocity < m_threshold_n0)
    {
        m_currentState = -2;
    }

    RETURN_NOERROR;
}

tFloat32 SpeedControl::GetControllerValue()
{
    tFloat32 outputData = 0;

    //update current state
    UpdateState();


    //------------ make sure that the car really stops before driving again----------------
    if ( (( m_last_pwm <= m_pwm_0 ) && ( m_lastState > 0 ) && ( m_currentState == 0 ))
         ||  (( m_last_pwm >= m_pwm_0 ) && ( m_lastState < 0 ) && ( m_currentState == 0 )) ) //has car just stopped (willingly)?
    {
        SetPWM(m_pwm_0);
        m_no_wait = false;  //wait until standing completly still (=> to make sure that the direction sent to the odometry is only sent when standing completely still....)
        m_timerStart = GetTime();
    }
    else if (( !m_no_wait ) && (( GetTime() - m_timerStart ) >= m_stopTime ))
    {
        m_no_wait = true;
    }

    if (m_no_wait)
    {
        //-------------------- rules for acceleration and braking -------------------------

        if (m_currentState == 3)
        {
            SetReverseLights(false);

            switch(m_setPoint)
            {
            case 3:
                SetBrakeLights(false);
                outputData = m_pwm_p3;
                break;
            case 2:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                break;
            default:
                SetBrakeLights(true);
                outputData = m_strongBrake;
            }
        }
        else if (m_currentState == 2)
        {
            SetReverseLights(false);

            switch(m_setPoint)
            {
            case 3:
                SetBrakeLights(false);
                outputData = m_pwm_boost_p3;
                break;
            case 2:
                SetBrakeLights(false);
                outputData = m_pwm_p2;
                break;
            case 1:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                break;
            default:
                SetBrakeLights(true);
                outputData = m_strongBrake;
            }
        }
        else if (m_currentState == 1)
        {
            SetReverseLights(false);

            switch(m_setPoint)
            {
            case 3:
                SetBrakeLights(false);
                outputData = m_pwm_boost_p3;
                break;
            case 2:
                SetBrakeLights(false);
                outputData = m_pwm_boost_p2;
                break;
            case 1:
                SetBrakeLights(false);
                outputData = m_pwm_p1;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                break;
            default:
                SetBrakeLights(true);
                outputData = m_lightBrake;
            }
        }
        else if (m_currentState == 0)
        {
            switch(m_setPoint)
            {
            case 3:
                SetReverseLights(false);
                SetBrakeLights(false);
                SetDirection(true);
                outputData = m_pwm_boost_p3;
                break;
            case 2:
                SetReverseLights(false);
                SetBrakeLights(false);
                SetDirection(true);
                outputData = m_pwm_boost_p2;
                break;
            case 1:
                SetReverseLights(false);
                SetBrakeLights(false);
                SetDirection(true);
                outputData = m_pwm_boost_p1;
                break;
            case 0:
                SetReverseLights(false);
                SetBrakeLights(true);
                SetDirection(true);
                outputData = m_pwm_0;
                break;
            case -1:
                SetReverseLights(true);
                SetBrakeLights(false);
                SetDirection(false);
                outputData = m_pwm_boost_n1;
                break;
            case -2:
                SetReverseLights(true);
                SetBrakeLights(false);
                SetDirection(false);
                outputData = m_pwm_boost_n2;
                break;
            }
        }
        else if (m_currentState == -1)
        {
            SetReverseLights(true);

            switch(m_setPoint)
            {
            case 3:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                break;
            case 2:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                break;
            case 1:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;;
                break;
            case -1:
                SetBrakeLights(false);
                outputData = m_pwm_n1;
                break;
            case -2:
                SetBrakeLights(false);
                outputData = m_pwm_boost_n2;
                break;
            }

        }
        else if (m_currentState == -2)
        {
            SetReverseLights(true);

            switch(m_setPoint)
            {
            case 3:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 2:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 1:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;;
                break;
            case -1:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                break;
            case -2:
                SetBrakeLights(false);
                outputData = m_pwm_n2;
                break;
            }
        }
    }

    m_lastState = m_currentState;

    outputData = outputData * m_pwmScaler;

    // prevent erratic values (e.g. due to a stupid configuration)
    if ( outputData < 10.0 )
    {
        outputData = 10.0;
    }
    else if ( outputData > 170.0 )
    {
        outputData = 170.0;
    }

    return outputData;
}

tResult SpeedControl::SetBrakeLights(tBool state)
{

    if(state != m_last_brakeLights) //prevent unneccessary messages
    {
        cObjectPtr<IMediaCoder> pCoder;

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSampleOutput;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

        //allocate memory with the size given by the descriptor
        // ADAPT: m_pCoderDescPointLeft
        cObjectPtr<IMediaSerializer> pSerializer;

        m_pCodeOutputbrakelight->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSampleOutput->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        // ADAPT: m_pCoderDescPointLeft
        //cObjectPtr<IMediaCoder> pCoder;
        RETURN_IF_FAILED(m_pCodeOutputbrakelight->WriteLock(pMediaSampleOutput, &pCoder));
        pCoder->Set("bValue", (tVoid*)&(state));
        m_pCodeOutputbrakelight->Unlock(pCoder);

        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oOutputbrakelight.Transmit(pMediaSampleOutput));
    }

    m_last_brakeLights = state;

    RETURN_NOERROR;
}

tResult SpeedControl::SetReverseLights(tBool state)
{

    if(state != m_last_reverseLights)
    {
        cObjectPtr<IMediaCoder> pCoder;

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSampleOutput;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

        //allocate memory with the size given by the descriptor
        // ADAPT: m_pCoderDescPointLeft
        cObjectPtr<IMediaSerializer> pSerializer;

        m_pCodeOutputbrakelight->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSampleOutput->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        // ADAPT: m_pCoderDescPointLeft
        //cObjectPtr<IMediaCoder> pCoder;
        RETURN_IF_FAILED(m_pCodeOutputbrakelight->WriteLock(pMediaSampleOutput, &pCoder));
        pCoder->Set("bValue", (tVoid*)&(state));
        m_pCodeOutputbrakelight->Unlock(pCoder);

        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oOutputreverse.Transmit(pMediaSampleOutput));
    }

    m_last_reverseLights = state;

    RETURN_NOERROR;
}

tResult SpeedControl::SetDirection(tBool state)
{
    if(state != m_last_goingForwards)
    {
        cObjectPtr<IMediaCoder> pCoder;

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSampleOutput;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

        //allocate memory with the size given by the descriptor
        // ADAPT: m_pCoderDescPointLeft
        cObjectPtr<IMediaSerializer> pSerializer;

        m_pCodeOutputbrakelight->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSampleOutput->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        // ADAPT: m_pCoderDescPointLeft
        RETURN_IF_FAILED(m_pCodeOutputbrakelight->WriteLock(pMediaSampleOutput, &pCoder));
        pCoder->Set("bValue", (tVoid*)&(state));
        m_pCodeOutputbrakelight->Unlock(pCoder);

        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oOutputDirection.Transmit(pMediaSampleOutput));
    }

    m_last_goingForwards = state;

    RETURN_NOERROR;
}

tResult SpeedControl::SetPWM(tFloat32 pwm_value)
{
    if (pwm_value != m_last_pwm)
    {
        cObjectPtr<IMediaCoder> pCoder;
        tUInt32 timeStamp = 0;

        timeStamp = GetTime();

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        //allocate memory with the size given by the descriptor
        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSample->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        m_pCoderDescSignal->WriteLock(pMediaSample, &pCoder);

        pCoder->Set("f32Value", (tVoid*)&(pwm_value));
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignal->Unlock(pCoder);

        //transmit media sample over output pin
        RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oOutputPWM.Transmit(pMediaSample));
    }

    m_last_pwm = pwm_value;

    RETURN_NOERROR;
}

tResult SpeedControl::WaitIdle(tUInt32 idletime)
{

    tTimeStamp start_time;
    start_time = GetTime();

    while ((GetTime() - start_time) < idletime)
    {
        //do nothing
    }

    RETURN_NOERROR;
}

tTimeStamp SpeedControl::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}
