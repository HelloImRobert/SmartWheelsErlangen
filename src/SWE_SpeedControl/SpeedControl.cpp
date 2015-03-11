#include "SpeedControl.h"

ADTF_FILTER_PLUGIN("SWE Motor Speed Controller", OID_ADTF_SWE_SPEEDCONTROL, SpeedControl)

#define TIMER_RESOLUTION 1000000.0f
#define MAX_OUTPUT 100.0f
#define MIN_OUTPUT -100.0f

SpeedControl::SpeedControl(const tChar* __info) : cFilter(__info), m_velocity(0), m_gear(0), m_currentState(0), m_lastState(0), m_goingForwards(0), m_lastSampleTime(0),m_timerStart(0), m_no_wait(true), m_last_pwm(0), m_last_brakeLights(0), m_last_reverseLights(0), m_last_DirectionSent(0), m_initRun(0)
{
    SetPropertyFloat("Gear 3 PWM value",35); //the forward control pwm value sent to the car motor when driving in this gear. In percent 0 = stop/neutral
    SetPropertyFloat("Gear 3 speed threshold",1200); // the speed in mm/s (measured) at which the controller decides it has left this speed window => the upper limit of this speed window, the actual speed will be between this and the limit of the gear below this

    SetPropertyFloat("Gear 2 PWM value",25);
    SetPropertyFloat("Gear 2 speed threshold",600);

    SetPropertyFloat("Gear 1 PWM value",15);
    SetPropertyFloat("Gear 1 speed threshold",300);

    SetPropertyFloat("Gear -1 PWM value", -15); //reverse gear
    SetPropertyFloat("Gear -1 speed threshold",-300);

    SetPropertyFloat("Gear -2 PWM value", -25);
    SetPropertyFloat("Gear -2 speed threshold",-600);

    SetPropertyFloat("light brake strength", 0.03); //pwm value for light braking
    SetPropertyFloat("strong brake strength",0.1);  //pwm value for strong braking

    SetPropertyFloat("acceleration boost", 1.1); //pwm boost value for acceleration 1.0 = no boost

    SetPropertyFloat("Gear 0 speed window", 50); //threshold window around zero so the controller can stop right before 0 -> for reliable stopping in mm/s

    SetPropertyFloat("PWM scaler", 1.0); //all pwm values are multiplied by this value

    SetPropertyInt("Stop Time in ms", 500); //minimum time the car stands still once it stops

}

SpeedControl::~SpeedControl()
{
}

tResult SpeedControl::CreateInputPins(__exception)
{	
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

    RETURN_IF_FAILED(m_oInputVelocity.Create("car_velocity", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputVelocity));

    RETURN_IF_FAILED(m_oInputSetPoint.Create("set_gear", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetPoint));
    RETURN_NOERROR;
}

tResult SpeedControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    // PWM output
    tChar const * strDescSignalValue_pwm = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue_pwm);
    cObjectPtr<IMediaType> pTypeSignalValue_pwm = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue_pwm,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue_pwm->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_pwm));

    RETURN_IF_FAILED(m_oOutputPWM.Create("PWMvalue", pTypeSignalValue_pwm, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputPWM));


    tChar const * strDescSignalValue_direction = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue_direction);
    cObjectPtr<IMediaType> pTypeSignalValue_direction = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue_direction,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue_direction->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_direction));

    RETURN_IF_FAILED(m_oOutputDirection.Create("Direction_for_odometry", pTypeSignalValue_direction, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputDirection));

    RETURN_IF_FAILED(m_oOutputSetPoint.Create("SetPoint_Speed", pTypeSignalValue_direction, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputSetPoint));


    // Struct for brake
    tChar const * strDescLightOutput = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescLightOutput);
    cObjectPtr<IMediaType> pTypeLightData = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescLightOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLightData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCodeOutputbrakelight));

    RETURN_IF_FAILED(m_oOutputbrakelight.Create("Brakelight", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputbrakelight));

    RETURN_IF_FAILED(m_oOutputreverse.Create("ReverseLight", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputreverse));

    RETURN_IF_FAILED(m_oOutputCarStopped.Create("Car_Stopped_Flag", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputCarStopped));

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

    tFloat32 thresholdWindow_0 = (tFloat32)GetPropertyFloat("Gear 0 speed window", 50);

    if (thresholdWindow_0 < 10)
    {
        thresholdWindow_0 = 10;
        LOG_ERROR(cString("SC: SpeedControl: !!BAD PROPERTY CONFIG!! I tried to make it work but you better fix it"));
    }

    tFloat32 boostValue = (tFloat32)GetPropertyFloat("acceleration boost", 1.1);

    m_pwm_p3 = (tFloat32)GetPropertyFloat("Gear 3 PWM value",35);
    m_pwm_boost_p3 = (m_pwm_p3) * boostValue;
    m_threshold_p3 = (tFloat32)GetPropertyFloat("Gear 3 speed threshold",1200);

    m_pwm_p2 = (tFloat32)GetPropertyFloat("Gear 2 PWM value",25);
    m_pwm_boost_p2 = (m_pwm_p2 ) * boostValue;
    m_threshold_p2 = (tFloat32)GetPropertyFloat("Gear 2 speed threshold",600);

    m_pwm_p1 = (tFloat32)GetPropertyFloat("Gear 1 PWM value",15);
    m_pwm_boost_p1 = (m_pwm_p1 ) * boostValue;
    m_threshold_p1 = (tFloat32)GetPropertyFloat("Gear 1 speed threshold",300);

    m_pwm_0 = 0;
    m_threshold_p0 = 0 + thresholdWindow_0;
    m_threshold_n0 = 0 - thresholdWindow_0;


    m_pwm_n1 = (tFloat32)GetPropertyFloat("Gear -1 PWM value",-15);
    m_pwm_boost_n1 = (m_pwm_n1) * boostValue;
    m_threshold_n1 = (tFloat32)GetPropertyFloat("Gear -1 speed threshold",-300);

    m_pwm_n2 = (tFloat32)GetPropertyFloat("Gear -2 PWM value",-25);
    m_pwm_boost_n2 = (m_pwm_n2) * boostValue;
    m_threshold_n2 = (tFloat32)GetPropertyFloat("Gear -2 speed threshold",-600);

    // save yourself from bad configurations
    if(m_threshold_p1 <= m_threshold_p0)
    {
        m_threshold_p1 = m_threshold_p0 * 1.1;
        LOG_ERROR(cString("SC: SpeedControl: !!BAD PROPERTY CONFIG!! I tried to make it work but you better fix it"));
    }

    if(m_threshold_p2 <= m_threshold_p1)
    {
        m_threshold_p2 = m_threshold_p1 * 1.1;
        LOG_ERROR(cString("SC: SpeedControl: !!BAD PROPERTY CONFIG!! I tried to make it work but you better fix it"));
    }

    if(m_threshold_p3 <= m_threshold_p2)
    {
        m_threshold_p3 = m_threshold_p2 * 1.1;
        LOG_ERROR(cString("SC: SpeedControl: !!BAD PROPERTY CONFIG!! I tried to make it work but you better fix it"));
    }

    if(m_threshold_n1 >= m_threshold_n0)
    {
        m_threshold_n1 = m_threshold_n0 * 1.1;
        LOG_ERROR(cString("SC: SpeedControl: !!BAD PROPERTY CONFIG!! I tried to make it work but you better fix it"));
    }

    if(m_threshold_n2 >= m_threshold_n1)
    {
        m_threshold_n2 = m_threshold_n1 * 1.1;
        LOG_ERROR(cString("SC: SpeedControl: !!BAD PROPERTY CONFIG!! I tried to make it work but you better fix it"));
    }


    m_setPoint_p3 = (m_threshold_p3 - m_threshold_p2) * 0.5 + m_threshold_p2;
    m_setPoint_p2 = (m_threshold_p2 - m_threshold_p1) * 0.5 + m_threshold_p1;
    m_setPoint_p1 = (m_threshold_p1 - m_threshold_p0) * 0.5 + m_threshold_p0;
    m_setPoint_0  = 0;
    m_setPoint_n1 = (m_threshold_n1 - m_threshold_n0) * 0.5 + m_threshold_n0;
    m_setPoint_n2 = (m_threshold_n2 - m_threshold_n1) * 0.5 + m_threshold_n1;


    m_lightBrake = -100*(tFloat32)GetPropertyFloat("light brake strength", 0.05);
    m_inv_lightBrake = 100*(tFloat32)GetPropertyFloat("light brake strength", 0.05);

    m_strongBrake = -100*(tFloat32)GetPropertyFloat("strong brake strength",0.1);
    m_inv_strongBrake = 100*(tFloat32)GetPropertyFloat("strong brake strength",0.1);

    m_pwmScaler = (tFloat32)GetPropertyFloat("PWM scaler", 1.0);

    m_stopTime =  (tInt32)GetPropertyInt("Stop Time in ms", 500) * (TIMER_RESOLUTION / 1000.0) ; //in us

    m_initRun = 0;
    //m_timerStart_init = GetTime();

    RETURN_NOERROR;
}

tResult SpeedControl::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
    
    m_velocity = 0;
    m_gear = 0;
    m_currentState = 0;
    m_lastState = 1;
    m_goingForwards = 0;
    m_lastSampleTime = 0;
    m_timerStart = 0;
    m_no_wait = true;
    m_last_pwm = 0;
    //m_last_brakeLights = 0;
    //m_last_reverseLights = 0;
    //m_last_DirectionSent = 0;

    m_initRun = 0;

    SetPWM(0);
    //m_timerStart_init = GetTime();

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

        m_mutex.Enter(); //serialize the whole filter for data consistency

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

            if ( m_initRun > 80 )
            {
                outputData = GetControllerValue();
            }
            else
                m_initRun++;

            SetPWM(outputData);
        }
        else if (pSource == &m_oInputSetPoint)
        {

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;
            tFloat32 outputData = 0;

            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&value);
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

            if (m_gear != (tInt32)value) //keep from spamming the pwm output
            {
                m_gear = (tInt32)value;

                //DEBUG
                //LOG_ERROR(cString("SC: SpeedControl: gear set to " + cString::FromInt32(m_gear)));

                if ( m_initRun > 80 )
                {
                    outputData = GetControllerValue();
                }
                else
                    m_initRun++;

                SetPWM(outputData);

                //DEBUG
                //LOG_ERROR(cString("SC: SpeedControl: finished setting gear to " + cString::FromInt32(m_gear)));
            }

        }

        m_mutex.Leave();
    }
    RETURN_NOERROR;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++ Determine Current Driving State +++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

tResult SpeedControl::UpdateState()
{
    m_currentState = 0;

    if (m_velocity > m_threshold_p2) //is the car going forwards?
        m_currentState = 3;
    else if (m_velocity > m_threshold_p1)
        m_currentState = 2;
    else if (m_velocity > m_threshold_p0)
        m_currentState = 1;

    if ((m_velocity > m_threshold_n0) && (m_velocity < m_threshold_p0)) //has it stopped?
        m_currentState = 0;

    if (m_velocity < m_threshold_n0) //is it going backwards?
        m_currentState = -1;
    else if (m_velocity < m_threshold_n1)
        m_currentState = -2;

    RETURN_NOERROR;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++ Here the Magic Happens +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

tFloat32 SpeedControl::GetControllerValue()
{
    tFloat32 outputData = m_pwm_0;
    tFloat32 outputSetPoint = m_setPoint_0;

    //update current state
    UpdateState();


    //------------ make sure that the car really stops before driving again----------------
    if (( (( m_last_pwm <= m_pwm_0 ) && ( m_lastState > 0 ) && ( m_currentState <= 0 ))
          ||  (( m_last_pwm >= m_pwm_0 ) && ( m_lastState < 0 ) && ( m_currentState >= 0 )) ) && (m_no_wait)  )//has car just stopped (willingly)?
    {
        SetPWM(m_pwm_0);
        m_no_wait = false;   //wait until standing completly still (=> to make sure that the new direction sent to the odometry is only sent when standing completely still....)
        m_timerStart = GetTime();
    }
    else if (( !m_no_wait ) && (( GetTime() - m_timerStart ) >= m_stopTime )) //if stopping time passed...
    {
        m_no_wait = true;    //... allow new acceleration values...
        SetCarStopped(true); // ... and tell other filters that the car has just stopped
        SetDirection(0);
        //DEBUG
        //LOG_ERROR(cString("SC: SpeedControl: Car has stopped"));
    }



    //-------------------- decision tree for acceleration and braking + lights -------------------------

    if (m_no_wait)
    {
        if (m_currentState == 3)
        {
            SetReverseLights(false);
            SetDirection(1);

            switch(m_gear)
            {
            case 3:
                SetBrakeLights(false);
                outputData = m_pwm_p3;
                outputSetPoint = m_setPoint_p3;
                break;
            case 2:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                outputSetPoint = m_setPoint_p2;
                break;
            case 1:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                outputSetPoint = m_setPoint_p1;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_strongBrake;
                outputSetPoint = m_setPoint_0;
            default:
                SetBrakeLights(true);
                outputData = m_strongBrake;
                outputSetPoint = m_setPoint_0;
            }
        }
        else if (m_currentState == 2)
        {
            SetReverseLights(false);
            SetDirection(1);

            switch(m_gear)
            {
            case 3:
                SetBrakeLights(false);
                outputData = m_pwm_boost_p3;
                outputSetPoint = m_setPoint_p3;
                break;
            case 2:
                SetBrakeLights(false);
                outputData = m_pwm_p2;
                outputSetPoint = m_setPoint_p2;
                break;
            case 1:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                outputSetPoint = m_setPoint_p1;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_strongBrake;
                outputSetPoint = m_setPoint_0;
            default:
                SetBrakeLights(true);
                outputData = m_strongBrake;
                outputSetPoint = m_setPoint_0;
            }
        }
        else if (m_currentState == 1)
        {
            SetReverseLights(false);
            SetDirection(1);

            switch(m_gear)
            {
            case 3:
                SetBrakeLights(false);
                outputData = m_pwm_boost_p3;
                outputSetPoint = m_setPoint_p2;
                break;
            case 2:
                SetBrakeLights(false);
                outputData = m_pwm_boost_p2;
                outputSetPoint = m_setPoint_p2;
                break;
            case 1:
                SetBrakeLights(false);
                outputData = m_pwm_p1;
                outputSetPoint = m_setPoint_p1;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                outputSetPoint = m_setPoint_0;
                break;
            default:
                SetBrakeLights(true);
                outputData = m_lightBrake;
                outputSetPoint = m_setPoint_0;
            }
        }
        else if (m_currentState == 0)
        {
            switch(m_gear)
            {
            case 3:
                SetReverseLights(false);
                SetBrakeLights(false);
                SetDirection(1);
                outputData = m_pwm_boost_p3;
                outputSetPoint = m_setPoint_p3;
                break;
            case 2:
                SetReverseLights(false);
                SetBrakeLights(false);
                SetDirection(1);
                outputData = m_pwm_boost_p2;
                outputSetPoint = m_setPoint_p2;
                break;
            case 1:
                SetReverseLights(false);
                SetBrakeLights(false);
                SetDirection(1);
                outputData = m_pwm_boost_p1;
                outputSetPoint = m_setPoint_p1;
                break;
            case 0:
                SetReverseLights(false);
                SetBrakeLights(true);
                SetDirection(0);
                outputData = m_pwm_0;
                outputSetPoint = m_setPoint_0;
                break;
            case -1:
                SetReverseLights(true);
                SetBrakeLights(false);
                SetDirection(-1);
                outputData = m_pwm_boost_n1;
                outputSetPoint = m_setPoint_n1;
                break;
            case -2:
                SetReverseLights(true);
                SetBrakeLights(false);
                SetDirection(-1);
                outputData = m_pwm_boost_n2;
                outputSetPoint = m_setPoint_n2;
                break;
            }
        }
        else if (m_currentState == -1)
        {
            SetReverseLights(true);
            SetDirection(-1);

            switch(m_gear)
            {
            case 3:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case 2:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case 1:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case -1:
                SetBrakeLights(false);
                outputData = m_pwm_n1;
                outputSetPoint = m_setPoint_n1;
                break;
            case -2:
                SetBrakeLights(false);
                outputData = m_pwm_boost_n2;
                outputSetPoint = m_setPoint_n2;
                break;
            }
        }
        else if (m_currentState == -2)
        {
            SetReverseLights(true);
            SetDirection(-1);

            switch(m_gear)
            {
            case 3:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case 2:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case 1:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case 0:
                SetBrakeLights(true);
                outputData = m_inv_strongBrake;
                outputSetPoint = m_setPoint_0;
                break;
            case -1:
                SetBrakeLights(true);
                outputData = m_inv_lightBrake;
                outputSetPoint = m_setPoint_n1;
                break;
            case -2:
                SetBrakeLights(false);
                outputData = m_pwm_n2;
                outputSetPoint = m_setPoint_n2;
                break;
            }
        }
    }

    outputData = outputData * m_pwmScaler;
    outputSetPoint = outputSetPoint * m_pwmScaler;

    // prevent erratic values (e.g. due to a stupid configuration)
    if ( outputData < MIN_OUTPUT )
    {
        outputData = MIN_OUTPUT;
    }
    else if ( outputData > MAX_OUTPUT)
    {
        outputData = MAX_OUTPUT;
    }

    // remember current states
    m_lastState = m_currentState;
    m_last_pwm = outputData;

    SetSetPoint(outputSetPoint);

    return outputData;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++ Send Data +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

tResult SpeedControl::SetBrakeLights(tBool state)
{
    //DEBUG
    /*
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
    */

    RETURN_NOERROR;
}

tResult SpeedControl::SetReverseLights(tBool state)
{
    //DEBUG
    /*
     *
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

    */

    RETURN_NOERROR;
}

tResult SpeedControl::SetDirection(tFloat32 state)
{
    if(state != m_last_DirectionSent)
    {
        cObjectPtr<IMediaCoder> pCoder;
        //tUInt32 timeStamp = 0;

        //timeStamp = GetTime();

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        //allocate memory with the size given by the descriptor
        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescSignal_direction->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSample->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        m_pCoderDescSignal_direction->WriteLock(pMediaSample, &pCoder);

        pCoder->Set("f32Value", (tVoid*)&(state));
        //pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignal_direction->Unlock(pCoder);

        //transmit media sample over output pin
        RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oOutputDirection.Transmit(pMediaSample));
    }

    m_last_DirectionSent = state;

    RETURN_NOERROR;
}

tResult SpeedControl::SetCarStopped(tBool state)
{

    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptort
    cObjectPtr<IMediaSerializer> pSerializer;

    m_pCodeOutputbrakelight->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCodeOutputbrakelight->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("bValue", (tVoid*)&(state));
    m_pCodeOutputbrakelight->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputCarStopped.Transmit(pMediaSampleOutput));


    RETURN_NOERROR;
}

tResult SpeedControl::SetPWM(tFloat32 pwm_value)
{

    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal_pwm->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal_pwm, pMediaSample, pCoder);

        pCoder->Set("f32Value", (tVoid*)&(pwm_value));
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputPWM.Transmit(pMediaSample);


    //DEBUG
    //LOG_ERROR(cString("SC: SpeedControl: set PWM to " + cString::FromInt32(pwm_value)));

    RETURN_NOERROR;
}

tResult SpeedControl::SetSetPoint(tFloat32 value)
{

    cObjectPtr<IMediaCoder> pCoder;
    //tUInt32 timeStamp = 0;

    //timeStamp = GetTime();

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal_direction->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    m_pCoderDescSignal_direction->WriteLock(pMediaSample, &pCoder);

    pCoder->Set("f32Value", (tVoid*)&(value));
    //pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignal_direction->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputSetPoint.Transmit(pMediaSample));

    RETURN_NOERROR;
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Helpers ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

