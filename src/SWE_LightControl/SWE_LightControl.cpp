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
    //MB Neue Pins
    RETURN_IF_FAILED(m_oInputLightData.Create("Light Data", new cMediaType(0, 0, 0, "tInt8"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputLightData));
    RETURN_NOERROR;
}

tResult cSWE_LightControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    //MB headlight
    tChar const * strheadlight = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strheadlight);
    cObjectPtr<IMediaType> ptrstrheadlight = new cMediaType(0, 0, 0, "tBoolSignalValue", strheadlight,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(ptrstrheadlight->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverDATA));
    RETURN_IF_FAILED(m_oOutputheadlight.Create("HeadLight", ptrstrheadlight, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputheadlight));


    //MB Turn Left
    tChar const * strheadlight2 = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strheadlight2);
    cObjectPtr<IMediaType> ptrstrheadlight2 = new cMediaType(0, 0, 0, "tBoolSignalValue", strheadlight2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(ptrstrheadlight2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverDATA));
    RETURN_IF_FAILED(m_oOutputturnleft.Create("TurnLeft", ptrstrheadlight2, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputturnleft));

    //MB Turn right
    tChar const * strheadlight3 = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strheadlight3);
    cObjectPtr<IMediaType> ptrstrheadlight3 = new cMediaType(0, 0, 0, "tBoolSignalValue", strheadlight3,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(ptrstrheadlight3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverDATA));
    RETURN_IF_FAILED(m_oOutputturnright.Create("TurnRight", ptrstrheadlight3, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputturnright));

    //MB brake
    tChar const * strheadlight4 = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strheadlight4);
    cObjectPtr<IMediaType> ptrstrheadlight4 = new cMediaType(0, 0, 0, "tBoolSignalValue", strheadlight4,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(ptrstrheadlight4->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverDATA));
    RETURN_IF_FAILED(m_oOutputbrake.Create("Brake", ptrstrheadlight4, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputbrake));


    //MB Reverse
    tChar const * strheadlight5 = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strheadlight5);
    cObjectPtr<IMediaType> ptrstrheadlight5 = new cMediaType(0, 0, 0, "tBoolSignalValue", strheadlight5,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(ptrstrheadlight5->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverDATA));
    RETURN_IF_FAILED(m_oOutputreverse.Create("Reverse", ptrstrheadlight5, static_cast<IPinEventSink*> (this)));
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
        LichtAn(1);
    }
    else if(eStage == StageGraphReady)
    {

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
        cObjectPtr<IMediaTypeDescription> pMediaTypeDescInputMeasured;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescInputMeasured));
        m_pCoderDescInputMeasured = pMediaTypeDescInputMeasured;
    }

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescInputMeasured != NULL)
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
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

                int value=0;
                pCoder->Get("i8Value", (tVoid*)&value);
                m_pCoderDescInputMeasured->Unlock(pCoder);

                LichtAn(value);
        }



    }
    RETURN_NOERROR;
}




void cSWE_LightControl::LichtAn(tInt8 value)
{
     bool head=false;
     bool left=false;
     bool right=false;
     bool brake=false;
     bool reverse=false;


   if(value>=16)
   {
        reverse=true;
        value=-16;
   }
   if(value>=8)
   {
        brake=true;
        value=-8;
   }
   if(value>=4)
   {
        right=true;
        value=-4;
   }
   if(value>=2)
   {
        left=true;
        value=-2;
   }
   if(value>=1)
   {
        head=true;
        value=-1;
   }

   if(value!=0)
   {
       //Fehler in der Logik
   }














   //Die Bools auf die jeweiligen Pins legen

}

