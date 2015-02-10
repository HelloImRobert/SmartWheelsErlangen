#include <cmath>
#include "stdafx.h"
#include "SWE_KIControl.h"

#include <iostream>
#include <fstream>

ADTF_FILTER_PLUGIN("SWE KIControl", OID_ADTF_SWE_KICONTROL, cSWE_KIControl)

cSWE_KIControl::cSWE_KIControl(const tChar* __info) : cFilter(__info)
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

cSWE_KIControl::~cSWE_KIControl()
{
}

tResult cSWE_KIControl::CreateInputPins(__exception)
{
    //MB Neue Pins
    RETURN_IF_FAILED(m_oInputRoadData.Create("Road Data", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputRoadData));
    RETURN_IF_FAILED(m_oInputObjectData.Create("Object Data", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputObjectData));

    RETURN_NOERROR;
}

tResult cSWE_KIControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


    //MB Outputpins
    tChar const * strDriverDATA = pDescManager->GetMediaDescription("ArrayRasterPoint");

    RETURN_IF_POINTER_NULL(strDriverDATA);

    cObjectPtr<IMediaType> pTypeDriverDATA = new cMediaType(0, 0, 0, "ArrayRasterPoint", strDriverDATA,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeDriverDATA->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverDATA));

    RETURN_IF_FAILED(m_oOutputDriverCourse.Create("DriveData", pTypeDriverDATA, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputDriverCourse));

    RETURN_NOERROR;
}

tResult cSWE_KIControl::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);

        //Hier XML Datei einlesen etc
    }
    else if (eStage == StageNormal)
    {

    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_KIControl::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_KIControl::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_KIControl::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_KIControl::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
        if(pSource == &m_oInputObjectData)
        {
                ObjectAvoidance();
        }
        else if(pSource == &m_oInputRoadData)
        {
                DriverCalc();
        }





/*
        if (pSource == &m_oInputMeasured)
        {

            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));


            tFloat32 outputValue = 0;
            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&(outputValue));
            //pCoder->Get("f32Value", (tVoid*)&(m_IntersectionPointLeft.y));
            //pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescInputMeasured->Unlock(pCoder);

            //-------------- do caculations and assign output -------------------------



            //-------------- write output to output pin -------------------------

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSample;
            RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

            //allocate memory with the size given by the descriptor
            // ADAPT: m_pCoderDescPointLeft
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescPointLeft->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSample->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            // ADAPT: m_pCoderDescPointLeft
            //cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescPointLeft->WriteLock(pMediaSample, &pCoder));

            //pCoder->Set("f32Value", (tVoid*)&(outputValue));


            //pCoderOutput->Set("ui32Point2dTimestamp", 7);
            pCoder->Set("xCoord", (tVoid*)&(m_IntersectionPointLeft.x));
            pCoder->Set("yCoord", (tVoid*)&(m_IntersectionPointLeft.y));
            m_pCoderDescPointLeft->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPointLeft.Transmit(pMediaSample));

            //RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
            //RETURN_IF_FAILED(m_oOutputManipulated.Transmit(pMediaSample));

        }
        else if (pSource == &m_oInputSetPoint)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;

            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescInputMeasured->Unlock(pCoder);

        }
        else
            RETURN_NOERROR;
        // -------------------------------------------------------------------
*/
    }
    RETURN_NOERROR;
}



//Modul fuer Juroren einbauen
void cSWE_KIControl::ObjectAvoidance()
{
    /*
     * 1. Pruefen ob Emergency Stopp, falls Ja einleiten
     * 2. Situation analysieren:
     *      a) Gerade ausfahren oder Abbiegen/Kreuzung/Vorfahrt
     *      b) Gerade aus, Geschwindigkeit runter und Hinterherfahren
     *      c) Unbewegtes Objekt ueberholen
     *      d) Bewegtes Objekt ueberholen
     *      e) Vorfahrt, Warten bis Objekt vorbei 3 Varianten
     *              1) Wir biegen rechts ab, dann nur nach links achten
     *              2) Wir biegen links ab, dann auf Rechts, Links und gerade aus verkehr achten
     *              3) Wir fahren gerade aus, dann auf Rechts und Links achten
     *      f)Objekt auf Gegenfahrbahn ignorieren
     *
     */
}

void cSWE_KIControl::DriverCalc()
{
/*
 *  1. Pruefen der Aufgaben
 *  2. Aktualliesieren der DriverDaten
 *  3. Anhand der Aufgaben pruefen was als neachstes getan wird
 *  4. Daraus folgend, die beste Route berrechnen
 *      a) Dabei Schilder beachten
 *      b) Ueberholen mit einplannen
 *      c) Geschwindigkeit anpassen
 *      d) Auf Kreuzungen reagieren
 *  5. Wenn gesucht, Parkplatz anfahren
 *      a)Parkmodul starten
 *  6. Pruefen welcher datensatz aktueller ist
 *          a)Entweder jedesmal komplett neu berrechnen
 *          b)Oder die Datenanpassen(Aufweandiger)
 * 7.Erfuellung der Aufgaben Pruefen
 * 8. Lichtanlage einstellen vlt eigenes Modul das zwischen geschalten wird
 *
 *
 *
 *
 *
 *
 *
 */
}


