/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: -This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.-
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS -AS IS- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "SWE_ControllerFilter.h"

#define TIME_RESOLUTION_ADTF 1000000.0f 
#define MINIMUM_TIME_BETWEEN_OUTPUTS 33333 //in microseconds

ADTF_FILTER_PLUGIN("SWE PID Controller", OID_ADTF_SWE_PIDCONTROLLER, SWE_ControllerFilter)

SWE_ControllerFilter::SWE_ControllerFilter(const tChar* __info) : cFilter(__info), m_lastMeasuredError(0), m_setPoint(0), m_feedForward(0), m_lastSampleTime(0), m_accumulatedVariable(0)
{
    SetPropertyFloat("Controller Kp value",0.1);
    SetPropertyFloat("Controller Ki value",0.1);
    SetPropertyFloat("Controller Kd value",0.1);

    SetPropertyFloat("max output",100);
    SetPropertyFloat("min output",-100);

    SetPropertyFloat("max controller influence upper",15);
    SetPropertyFloat("max controller influence lower",-15);

    SetPropertyBool("Use Feed Forward", true);
    SetPropertyBool("Feed-Forward = 0 => output = 0", true); //off means off e.g. when used as second/cascaded controller controlling the motor to ensure safe stopping

    SetPropertyInt("Sample Interval [msec]",1);
    SetPropertyBool("use automatically calculated sample interval",1);
    SetPropertyInt("Controller Type", 1);
    SetPropertyStr("Controller Type" NSSUBPROP_VALUELISTNOEDIT, "0@OFF|1@P|2@PI|3@PID");

    m_lastOutputTime = 0;
}

SWE_ControllerFilter::~SWE_ControllerFilter()
{
}

tResult SWE_ControllerFilter::CreateInputPins(__exception)
{    

    RETURN_IF_FAILED(m_oInputMeasured.Create("measured_variable", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputMeasured));
    RETURN_IF_FAILED(m_oInputSetPoint.Create("set_point", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetPoint));
    RETURN_IF_FAILED(m_oInputFeedForward.Create("feed_forward_value", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputFeedForward));
    RETURN_NOERROR;
}

tResult SWE_ControllerFilter::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

    RETURN_IF_FAILED(m_oOutputManipulated.Create("manipulated_variable", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputManipulated));
    RETURN_NOERROR;
}

tResult SWE_ControllerFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);

        m_Kp = (tFloat32)GetPropertyFloat("Controller Kp value",0.1);
        m_Ki = (tFloat32)GetPropertyFloat("Controller Ki value",0.1);
        m_Kd = (tFloat32)GetPropertyFloat("Controller Kd value",0.1);

        m_maxOutput = (tFloat32)GetPropertyFloat("max output",100);
        m_minOutput = (tFloat32)GetPropertyFloat("min output",-100);

        m_maxInfluence_upper = (tFloat32)GetPropertyFloat("max controller influence upper",15);
        m_maxInfluence_lower = (tFloat32)GetPropertyFloat("max controller influence lower",-15);

        m_useFF = (tBool)GetPropertyBool("Use Feed Forward", true);
        m_offMeansOff = (tBool)GetPropertyBool("Feed-Forward = 0 => output = 0", true);

        m_type = (tInt32)GetPropertyInt("Controller Type", 1);

        m_sampleIntervall = (tFloat32)GetPropertyInt("Sample Interval [msec]",1);
        m_useAutoSampleTime = (tBool)GetPropertyBool("use automatically calculated sample interval",1);
    }
    else if (eStage == StageNormal)
    {

    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult SWE_ControllerFilter::Start(__exception)
{
    return cFilter::Start(__exception_ptr);

    m_accumulatedVariable = 0;
    m_lastSampleTime = 0;
    m_feedForward = 0;
    m_measuredVariable = 0;
    m_setPoint = 0;
    m_lastMeasuredError = 0;
    m_lastOutputTime = 0;
}

tResult SWE_ControllerFilter::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult SWE_ControllerFilter::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult SWE_ControllerFilter::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {

        RETURN_IF_POINTER_NULL( pMediaSample);

        m_mutex.Enter(); //serialize the whole filter for data consistency
        
        if (pSource == &m_oInputMeasured)
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

            //calculation
            m_measuredVariable = value;
            outputData = getControllerValue(value);


            //DEBUG
            //LOG_ERROR(cString("PID: wait time " + cString::FromFloat64((GetTime() - m_lastOutputTime))));

           // if((GetTime() - m_lastOutputTime) >= MINIMUM_TIME_BETWEEN_OUTPUTS) //prevent output spamming wich can lead to crashes
           // {
                m_lastOutputTime = GetTime();

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
            //}

        }
        else if (pSource == &m_oInputSetPoint)
        {
            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;

            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoder);
            m_setPoint = value;
        }
        else if (pSource == &m_oInputFeedForward)
        {

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;

            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));
            
            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoder);
            m_feedForward = value;
        }

        m_mutex.Leave();
        

    }
    RETURN_NOERROR;
}


tFloat32 SWE_ControllerFilter::getControllerValue(tFloat32 measuredValue)
{
    
    tFloat32 sampleTime;
    tFloat32 returnvalue;

    //calculate the sample time in seconds if necessary
    if (m_useAutoSampleTime)
        sampleTime = (tFloat32)(GetTime() - m_lastSampleTime)/TIME_RESOLUTION_ADTF; //in seconds
    else
        sampleTime = m_sampleIntervall/1000.0;

    m_lastSampleTime = GetTime();

    if(sampleTime <= 0)
        sampleTime = 0.000001;

    //DEBUG
    //LOG_ERROR(cString("PID: sampletime " + cString::FromFloat64(returnvalue)));
    

    //the three controller algorithms
    if ((m_offMeansOff) && (m_useFF) && (m_feedForward == 0))
    {
        returnvalue = 0;
        m_accumulatedVariable = 0;
    }
    else if (m_type == 1)
    {
        //y = Kp * e
        returnvalue = m_Kp*(m_setPoint-measuredValue);
    }
    else if(m_type == 2) //PI- Regler
    {
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum
        m_accumulatedVariable += sampleTime*(m_setPoint-measuredValue);

        m_accumulatedVariable = LimitValue(m_accumulatedVariable, (m_maxInfluence_upper / m_Ki), (m_maxInfluence_lower / m_Ki)); //limit accumulation to ensure stability of controlled system

        returnvalue = m_Kp*(m_setPoint-measuredValue) + m_Ki*m_accumulatedVariable;
    }
    else if(m_type == 3)
    {
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum + Kd * (e - ealt)/Ta
        //ealt = e
        m_accumulatedVariable += sampleTime*(m_setPoint-measuredValue);

        m_accumulatedVariable = LimitValue(m_accumulatedVariable, (m_maxInfluence_upper / m_Ki), (m_maxInfluence_lower / m_Ki)); //limit accumulation to ensure stability of controlled system

        returnvalue =  m_Kp*(m_setPoint-measuredValue) + m_Ki*m_accumulatedVariable + m_Kd*((m_setPoint-measuredValue)-m_lastMeasuredError)/sampleTime;

        m_lastMeasuredError = (m_setPoint-measuredValue);
    }
    else //off
        returnvalue = 0;

    returnvalue = LimitValue(returnvalue, m_maxInfluence_upper, m_maxInfluence_lower);

    if(m_useFF)
        returnvalue += m_feedForward;

    // keep within boundries
    returnvalue = LimitValue(returnvalue, m_maxOutput, m_minOutput);

    //DEBUG
    //LOG_ERROR(cString("PID: controller output " + cString::FromFloat64(returnvalue)));

    return returnvalue;
}

tTimeStamp SWE_ControllerFilter::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 SWE_ControllerFilter::LimitValue(tFloat32 inputVar, tFloat32 upperBound, tFloat32 lowerBound)
{
    tFloat32 outputVar;

    if(inputVar > upperBound)
        outputVar = upperBound;
    else if (inputVar < lowerBound)
        outputVar = lowerBound;
    else
        outputVar = inputVar;

    return outputVar;
}
