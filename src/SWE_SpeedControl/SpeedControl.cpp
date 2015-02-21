/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


// arduinofilter.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//
#include "stdafx.h"
#include "SpeedControl.h"

ADTF_FILTER_PLUGIN("SWE Motor Speed Controller", OID_ADTF_SWE_SPEEDCONTROL, SpeedControl)

SpeedControl::SpeedControl(const tChar* __info) : cFilter(__info), m_lastMeasuredError(0), m_setPoint(0), m_lastSampleTime(0)
{
    SetPropertyFloat("Gear 3 PWM value",90); //the pwm value sent to the car motor when driving in this gear. In degrees 0-180 => 90 = stop/neutral
    SetPropertyFloat("Gear 3 speed threshold",1); // the speed in mm/s (measured) at which the controller decides it has reached the desired gear/speed

    SetPropertyFloat("Gear 2 PWM value",90);
    SetPropertyFloat("Gear 2 speed threshold",1);

    SetPropertyFloat("Gear 1 PWM value",90);
    SetPropertyFloat("Gear 1 speed threshold",1);

    SetPropertyFloat("Gear 0 PWM value",90);
    SetPropertyFloat("Gear 0 speed threshold",1);

    SetPropertyFloat("Gear -1 PWM value", 90);
    SetPropertyFloat("Gear -1 speed threshold",1);

    SetPropertyFloat("Gear -2 PWM value", 90);
    SetPropertyFloat("Gear -2 speed threshold",1);

    SetPropertyFloat("light brake strength", 0.05); //pwm value for light braking
    SetPropertyFloat("strong brake strength",0.1);

    SetPropertyFloat("acceleration boost", 1.1); //pwm boost value for acceleration

    SetPropertyFloat("speed threshold window", 0.1); //threshold window for smooth transitions

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

    RETURN_IF_FAILED(m_oInputSetPoint.Create("set point (gear/speed)", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
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
    RETURN_IF_FAILED(RegisterPin(&m_oOutputManipulated));
    RETURN_NOERROR;

    //---------------------------------
    //TODO: output pin for brake lights

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


            m_measuredSensor1 = value;
            m_measuredSpeed = calcSpeed();



            //execute controller
            outputData = getControllerValue(m_measuredSpeed);





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
            RETURN_IF_FAILED(m_oOutputManipulated.Transmit(pMediaSample));

        }
        else if (pSource == &m_oInputSetPoint)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //write values with zero
            tInt16 value = 0;
            tUInt32 timeStamp = 0;

            //get values from media sample
            pCoder->Get("i16Value", (tVoid*)&value);
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


tFloat32 SpeedControl::calcSpeed()
{
    tFloat32 outputData = 0;

    return outputData;
}

tFloat32 SpeedControl::getControllerValue(tFloat32 measuredSpeed)
{
    tFloat32 outputData = 0;

    //update m_lastGear according to speed + m_setPoint

    //


    // prevent value overflow
    if ( outputData < 10.0 )
    {
        outputData = 10.0;
    }
    else if ( value > 170.0 )
    {
        outputData = 170.0;
    }

        return outputData;
}

tTimeStamp SpeedControl::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}
