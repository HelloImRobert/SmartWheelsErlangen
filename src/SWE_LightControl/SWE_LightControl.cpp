#include <cmath>
#include "SWE_LightControl.h"

#include <iostream>
#include <fstream>

ADTF_FILTER_PLUGIN("SWE LightControl", OID_ADTF_SWE_LIGHTCONTROL, cSWE_LightControl)

cSWE_LightControl::cSWE_LightControl(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("Controller Kp value",1);
    SetPropertyFloat("Controller Ki value",1);
    SetPropertyFloat("Controller Kd value",1);
    SetPropertyFloat("Controller Precontrol Value", 1);

    SetPropertyInt("Sample Interval [msec]",1);
    SetPropertyBool("use automatically calculated sample interval",1);
    SetPropertyInt("Controller Typ", 1);
    SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID");


}

cSWE_LightControl::~cSWE_LightControl()
{
}

tResult cSWE_LightControl::CreateInputPins(__exception)
{
    //MB Neue PinstInt8SignalValue
       RETURN_IF_FAILED(m_oInputLightData.Create("LightData", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
      RETURN_IF_FAILED(RegisterPin(&m_oInputLightData));


    RETURN_NOERROR;
}

tResult cSWE_LightControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


    // Struct for Headlight
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescLightOutput = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescLightOutput);
    cObjectPtr<IMediaType> pTypeLightData = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescLightOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLightData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLightOutput));

    RETURN_IF_FAILED(m_oOutputheadlight.Create("Headlight", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputheadlight));
    //--------------------------------------------------------------
  //MB Turn Left
    RETURN_IF_FAILED(m_oOutputturnleft.Create("LeftBlink", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputturnleft));

    //MB Turn right
    RETURN_IF_FAILED(m_oOutputturnright.Create("rightBlink", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputturnright));
    //MB brake
    RETURN_IF_FAILED(m_oOutputbrake.Create("brakeLight", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputbrake));
    //MB Reverse
    RETURN_IF_FAILED(m_oOutputreverse.Create("ReverseLight", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputreverse));


    RETURN_NOERROR;
}

tResult cSWE_LightControl::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);


    }
    else if (eStage == StageNormal)
    {
        LOG_INFO(cString::Format( "MB:Licht hat StageNormal erreicht"));

    }
    else if(eStage == StageGraphReady)
    {
        LOG_INFO(cString::Format( "MB:Licht hat StageGraphReady erreicht"));

    }

    RETURN_NOERROR;
}

tResult cSWE_LightControl::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_LightControl::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_LightControl::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_LightControl::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
    // ADAPT: pMediaTypeDescInputMeasured, m_pCoderDescInputMeasured !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // NOCH SO BAUEN, DASS IN FKT CREATE_INPUT_PINS EINGEFUEGT WERDEN KANN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);
    if (pType != NULL)
    {
        cObjectPtr<IMediaTypeDescription> pMediaTypeDescLightData;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescLightData));
        m_pCoderDescLightData = pMediaTypeDescLightData;
    }

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescLightData != NULL)
    {

        RETURN_IF_POINTER_NULL( pMediaSample);



        //MB hier die OnPinEventsRein

        /*Hier kommen daten als Zahl an, wobei dies als Byte interpretiert wird mit:
        *Bits:
        *1=Headlight
        *2=Turnleft
        *4=Turnright
        *8=Brake
        *16=reverse
        *
        *bsp 24=brake und reverse
        *
        *
        */

        if(pSource == &m_oInputLightData)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescLightData->Lock(pMediaSample, &pCoder));
           int value=0;
           pCoder->Get("int8Value", (tVoid*)&value);
           m_pCoderDescLightData->Unlock(pCoder);

            LichtAn(value);
        }



    }
    RETURN_NOERROR;
}




tResult cSWE_LightControl::LichtAn(tInt8 value)
{
    tBool head=false;
    tBool left=false;
    tBool right=false;
    tBool brake=false;
    tBool reverse=false;


    if(value>=16)
    {
        reverse=true;
        value-=16;
    }
    if(value>=8)
    {
        brake=true;
        value-=8;
    }
    if(value>=4)
    {
        right=true;
        value-=4;
    }
    if(value>=2)
    {
        left=true;
        value-=2;
    }
    if(value>=1)
    {
        head=true;
        value-=1;
    }

    if(value!=0)
    {
        //Fehler in der Logik wenn wir hier ankommen
    }


    //Ausgabe als Media Sample




    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    // ADAPT: m_pCoderDescPointLeft
    cObjectPtr<IMediaSerializer> pSerializer;

    m_pCoderDescLightOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    // ADAPT: m_pCoderDescPointLeft
    //cObjectPtr<IMediaCoder> pCoder;
    //---------------------------------------Front scheinwerfer-----------------------------------------------------------------------------------------
    RETURN_IF_FAILED(m_pCoderDescLightOutput->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("bValue", (tVoid*)&(head));
    m_pCoderDescLightOutput->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputheadlight.Transmit(pMediaSampleOutput));


//---------------------------------------Blinken rechts-----------------------------------------------------------------------------------------
    RETURN_IF_FAILED(m_pCoderDescLightOutput->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("bValue", (tVoid*)&(right));
    m_pCoderDescLightOutput->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputturnright.Transmit(pMediaSampleOutput));

    //---------------------------------------Blinken links-----------------------------------------------------------------------------------------
        RETURN_IF_FAILED(m_pCoderDescLightOutput->WriteLock(pMediaSampleOutput, &pCoder));
        pCoder->Set("bValue", (tVoid*)&(left));
        m_pCoderDescLightOutput->Unlock(pCoder);

        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oOutputturnleft.Transmit(pMediaSampleOutput));

//---------------------------------------Brems Licht-----------------------------------------------------------------------------------------



    RETURN_IF_FAILED(m_pCoderDescLightOutput->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("bValue", (tVoid*)&(brake));
    m_pCoderDescLightOutput->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputbrake.Transmit(pMediaSampleOutput));
//---------------------------------------Reverse Licht-----------------------------------------------------------------------------------------
    RETURN_IF_FAILED(m_pCoderDescLightOutput->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("bValue", (tVoid*)&(reverse));
    m_pCoderDescLightOutput->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputreverse.Transmit(pMediaSampleOutput));





    RETURN_NOERROR;







    //Die Bools auf die jeweiligen Pins legen

}

