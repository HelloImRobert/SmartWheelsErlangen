#include <cmath>
#include "stdafx.h"
#include "SWE_Dummy.h"

#include <iostream>
#include <fstream>

ADTF_FILTER_PLUGIN("SWE Dummy", OID_ADTF_SWE_DUMMY, cSWE_Dummy)

cSWE_Dummy::cSWE_Dummy(const tChar* __info) : cFilter(__info)
{


}

cSWE_Dummy::~cSWE_Dummy()
{
}

tResult cSWE_Dummy::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescInput = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescInput);
    cObjectPtr<IMediaType> pTypeInput = new cMediaType(0, 0, 0, "tSignalValue", strDescInput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeInput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderInput));

    RETURN_IF_FAILED(m_input.Create("Input", pTypeInput, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_input));
    RETURN_NOERROR;
}

tResult cSWE_Dummy::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // TCP
    tChar const * strDescTCP = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescTCP);
    cObjectPtr<IMediaType> pTypeTCP = new cMediaType(0, 0, 0, "tSignalValue", strDescTCP,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeTCP->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescTCP));

    RETURN_IF_FAILED(m_output.Create("Output", pTypeTCP, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_output));


    RETURN_NOERROR;
}

tResult cSWE_Dummy::Init(tInitStage eStage, __exception)
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
           sendOutput(m_value);
    }

    RETURN_NOERROR;
}

tResult cSWE_Dummy::Start(__exception)
{

    m_value = 0;
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_Dummy::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_Dummy::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_Dummy::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
    // ADAPT: pMediaTypeDescInputMeasured, m_pCoderDescInputMeasured !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // NOCH SO BAUEN, DASS IN FKT CREATE_INPUT_PINS EINGEFUEGT WERDEN KANN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);
    if (pType != NULL)
    {
        cObjectPtr<IMediaTypeDescription> pMediaTypeDescInputMeasured;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescInputMeasured));
        m_pCoderDescInputMeasured = pMediaTypeDescInputMeasured;
    }

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescInputMeasured != NULL)
    {

        RETURN_IF_POINTER_NULL( pMediaSample);

        if (pSource == &m_input)
        {

            tTimeStamp timeStamp;
            cObjectPtr<IMediaCoder> pCoder;

            RETURN_IF_FAILED(m_pCoderInput->Lock(pMediaSample, &pCoder));
            pCoder->Get("f32Value", (tVoid*)&m_value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderInput->Unlock(pCoder);

            LOG_ERROR(cString("Received: " + cString::FromFloat64(m_value) ));

           sendOutput(m_value + 1.0);

        }

            RETURN_NOERROR;
        // -------------------------------------------------------------------

    }
    RETURN_NOERROR;
}



tResult cSWE_Dummy::sendOutput(tFloat32 value)
{
    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescTCP->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescTCP, pMediaSample, pCoder);

        pCoder->Set("f32Value", (tVoid*)&(value));
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_output.Transmit(pMediaSample);

    LOG_ERROR(cString("Sent: " + cString::FromFloat64(m_value) ));

    RETURN_NOERROR;
}

/*
computes intersetion points and indicator for steering angle calculation
indicator values:
1: two intersection points found
0: less than two intersection points found
else: computation error
*/
