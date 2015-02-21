#include "stdafx.h"
#include "SpeedControl.h"

ADTF_FILTER_PLUGIN("SWE Motor Speed Controller", OID_ADTF_SWE_SPEEDCONTROL, SpeedControl)

SpeedControl::SpeedControl(const tChar* __info) : cFilter(__info), m_velocity(0), m_setPoint(0), m_currentState(0), m_lastSampleTime(0)
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

}

SpeedControl::~SpeedControl()
{
}

tResult SpeedControl::CreateInputPins(__exception)
{	




    RETURN_IF_FAILED(m_oInputVelocity.Create("car velocity", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputVelocity));

    //--------------------------------
    //TODO: create an int input pin -->

    RETURN_IF_FAILED(m_oInputSetPoint.Create("set point (gear/speed)", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetPoint));
    RETURN_NOERROR;
}

tResult SpeedControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));


    RETURN_IF_FAILED(m_oOutputPWM.Create("PWM value", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputPWM));
    RETURN_NOERROR;

    //---------------------------------
    //TODO: output pins for brake/reverse lights

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
            outputData = getControllerValue(m_velocity);


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

            pCoder->Set("f32Value", (tVoid*)&(outputData));
            pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoder);

            //transmit media sample over output pin
            RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oOutputPWM.Transmit(pMediaSample));

        }
        else if (pSource == &m_oInputSetPoint)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //write values with zero
            tInt16 value = 0;
            tUInt32 timeStamp = 0;

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
        }
        else
        RETURN_NOERROR;
    }
    RETURN_NOERROR;
}

tResult SpeedControl::updateState()
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

tFloat32 SpeedControl::getControllerValue(tFloat32 measuredSpeed)
{
    tFloat32 outputData = 0;

    //update current state
    updateState();


    if (m_currentState == 3)
    {
        setReverseLights(false);

        switch(m_setPoint)
        {
            case 3:
                setBrakeLights(false);
                outputData = m_pwm_p3;
                break;
            case 2:
                setBrakeLights(true);
                outputData = m_lightBrake;
                break;
            default:
                setBrakeLights(true);
                outputData = m_strongBrake;
        }   
    }
    else if (m_currentState == 2)
    {
        setReverseLights(false);

        switch(m_setPoint)
        {
            case 3:
                setBrakeLights(false);
                outputData = m_pwm_boost_p3;
                break;
            case 2:
                setBrakeLights(false);
                outputData = m_pwm_p2;
                break;
            case 1:
                setBrakeLights(true);
                outputData = m_lightBrake;
                break;
            default:
                setBrakeLights(true);
                outputData = m_strongBrake;
        }
    }
    else if (m_currentState == 1)
    {
        setReverseLights(false);

        switch(m_setPoint)
        {
            case 3:
                setBrakeLights(false);
                outputData = m_pwm_boost_p3;
                break;
            case 2:
                setBrakeLights(false);
                outputData = m_pwm_boost_p2;
                break;
            case 1:
                setBrakeLights(false);
                outputData = m_pwm_p1;
                break;
            case 0:
                setBrakeLights(true);
                outputData = m_lightBrake;
                break;
            default:
                setBrakeLights(true);
                outputData = m_strongBrake;
        }
    }
    else if (m_currentState == 0)
    {
        switch(m_setPoint)
        {
            case 3:
                setReverseLights(false);
                setBrakeLights(false);
                outputData = m_pwm_boost_p3;
                break;
            case 2:
                setReverseLights(false);
                setBrakeLights(false);
                outputData = m_pwm_boost_p2;
                break;
            case 1:
                setReverseLights(false);
                setBrakeLights(false);
                outputData = m_pwm_boost_p1;
                break;
            case 0:
                setReverseLights(false);
                setBrakeLights(false);
                outputData = m_pwm_0;
                break;
            case -1:
                setReverseLights(true);
                setBrakeLights(false);
                outputData = m_pwm_boost_n1;
                break;
            case -2:
                setReverseLights(true);
                setBrakeLights(false);
                outputData = m_pwm_boost_n2;
                break;
        }
    }
    else if (m_currentState == -1)
    {
        setReverseLights(true);

        switch(m_setPoint)
        {
            case 3:
                setBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 2:
                setBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 1:
                setBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 0:
                setBrakeLights(true);
                outputData = m_inv_lightBrake;;
                break;
            case -1:
                setBrakeLights(false);
                outputData = m_pwm_n1;
                break;
            case -2:
                setBrakeLights(false);
                outputData = m_pwm_boost_n2;
                break;
        }

    }
    else if (m_currentState == -2)
    {
        setReverseLights(false);

        switch(m_setPoint)
        {
            case 3:
                setBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 2:
                setBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 1:
                setBrakeLights(true);
                outputData = m_inv_strongBrake;
                break;
            case 0:
                setBrakeLights(true);
                outputData = m_inv_strongBrake;;
                break;
            case -1:
                setBrakeLights(true);
                outputData = m_inv_lightBrake;
                break;
            case -2:
                setBrakeLights(false);
                outputData = m_pwm_n2;
                break;
        }
    }

    outputData = outputData * m_pwmScaler;

    // prevent erratic values (due to a stupid configuration)
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

tResult SpeedControl::setBrakeLights(tBool state)
{

    RETURN_NOERROR;
}

tResult SpeedControl::setReverseLights(tBool state)
{

    RETURN_NOERROR;
}

tTimeStamp SpeedControl::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}
