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
        //Dummmy funktion bis xml einlesen steht
        CommandCounter=6;
        int dummycount=CommandCounter;
        int Commands[]=new int[CommandCounter];
        while (dummycount>=0)
        {
            Commands[dummycount]=3;
            dummycount--;
        }
      /*
       *Int werte in Commands:
       * 1=left
       * 2=right
       * 3=straigth
       * 4=einparken1
       * 5=einparken2
       * 6=ausparken1
       * 7=ausparken2
       *
       */
    }
    else if (eStage == StageNormal)
    {
        halteLinie=false;
        hlsearch=false;
        abgebogen=false;
        roadfree=true;
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
        else if(pSource== &m_oInputSignData)
        {
            Signtype=1;
            /*Hier den  ausgelesenen Wert aus dem Schilder modul rein
            Es gibt folgende Schildtypen:

            */

        }
        else if(pSource==&m_oInputParkData)
        {
            Parkroutine();
        }
        else if(pSource==&m_oInputTC)
        {
            //Hier warten, bis wir eine Nachticht bekommen die uns sagt, das der Abbiege vorgang abgeschlossen ist.
            abgebogen=true;
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

void cSWE_KIControl::Parkroutine()
{

}

void cSWE_KIControl::ObjectAvoidance()
{
    /*
     * Relevante Objekte bestimmen.
     *Distanz auslesen
     * Werte anpassen
     * Objektliste anlegen zum durchlaufen
     * an Kreuzung neues system
     */
    double distanz=12;


    if(halteLinie)
    {
        //pruefen ob Kreuzung frei

        roadfree=false;
    }
    else
    {
        switch(distanz)
        {
            //Notbremsung
            case distanz<5:
                            SpeedControl=0;
                            sendTC(0,0);
                            ControlLight(9);
                            break;
            //langsames Hinterherfahren
            case 10>distanz>5:
                            SpeedControl=1;
                            sendTC(1,1);
                            break;
            //Objekte weit genug weg
            default:
                            SpeedControl=2;
                            sendTC(2,1);
                            break;
        }
    }
}
void cSWE_KIControl::sendTC(int speed, int type)
{

   /*Hier das senden an den TC rein(Speed, Punkt und Typ)
    Typen:
    0=Notbremsung
    1=normales fahren
    2=Links abbiegen
    3=rechtsabbiegen
    4=ueberholen
    5=Kreuzung gerade aus
    Speed:
    Stufen: 2,1,0,-1,-2
    Punkt auslesen aus der gespeicherten Punkt liste.
    Punktx;
    Punkty;
    */
}

void cSWE_KIControl::DriverCalc()
{

/*
 *
 * Daten in Punkte array schreiben
 *
 * Bestimmt nicht die Geschwindikeit, die wird allein durch das immer aktive Objektmodul angegeben.
 *
 *
 *
 *
 *
       *Int werte in Commands:
       * 1=left
       * 2=right
       * 3=straigth
       * 4=einparken1
       * 5=einparken2
       * 6=ausparken1
       * 7=ausparken2
       *

 *
 */
    switch (Commands[CommandCounter])
    {
        //left-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 1:
            if(Signtype<=1)
            {
                 sendTC(SpeedControl,1);
                  ControlLight(1);
            }
            else if(Signtype>1)
            {

                //an Schildtyp anpassen:
                //auch im Objectavoidance Modul zb. Rechts vor links anders als vorfahrt gewaehren
              //  if(Signtype==2)
                 //   roadfree=true;

                //pruefen ob wir schon an der HalteLinie stehen.
                if(halteLinie)
                {
                    if(!hlsearch)
                       ControlHL();



                    if(abgebogen)
                    {
                        if(CommandCounter!=0)
                            CommandCounter--;
                        else
                        {
                            //Game Over
                        }
                        Signtype=0;
                        abgebogen=false;
                        halteLinie=false;
                        roadfree=true;
                    }
                    else
                    {
                        if(roadfree)
                        {
                            sendTC(SpeedControl,2);
                            ControlLight(3);
                        }
                    }



                }
                else
                {
                    if(hlsearch)
                        ControlHL();

                    sendTC(SpeedControl,1);
                     ControlLight(1);
                }
            }
                    break;
        //right-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 2:
                if(Signtype<=1)
                {
                     sendTC(SpeedControl,1);
                      ControlLight(1);
                }
                else if(Signtype>1)
                {
                    //pruefen ob wir schon an der HalteLinie stehen.
                    if(halteLinie)
                    {
                        if(!hlsearch)
                           ControlHL();



                        if(abgebogen)
                        {
                            if(CommandCounter!=0)
                                CommandCounter--;
                            else
                            {
                                //Game Over
                            }
                            Signtype=0;
                            abgebogen=false;
                            halteLinie=false;
                        }
                        else
                        {
                            if(roadfree)
                            {
                                sendTC(SpeedControl,3);
                                ControlLight(5);
                            }
                        }



                    }
                    else
                    {
                        if(hlsearch)
                            ControlHL();

                        sendTC(SpeedControl,1);
                         ControlLight(1);
                    }
                }
                            break;
        //straigth-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 3:
                    if(Signtype<=1)
                    {
                         sendTC(SpeedControl,1);
                          ControlLight(1);
                    }
                    else if(Signtype>1)
                    {
                        //pruefen ob wir schon an der HalteLinie stehen.
                        if(halteLinie)
                        {
                            if(!hlsearch)
                               ControlHL();



                            if(abgebogen)
                            {
                                if(CommandCounter!=0)
                                    CommandCounter--;
                                else
                                {
                                    //Game Over
                                }
                                Signtype=0;
                                abgebogen=false;
                                halteLinie=false;
                            }
                            else
                            {
                                if(roadfree)
                                {
                                    sendTC(SpeedControl,5);
                                    ControlLight(1);
                                }
                            }



                        }
                        else
                        {
                            if(hlsearch)
                                ControlHL();

                            sendTC(SpeedControl,1);
                             ControlLight(1);
                        }
                    }
                    break;
        //Einparken1-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 4:
                    break;
        //einparken2-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 5:
                    break;
        //ausparken1-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 6:
                    break;
        //ausparken2-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 7:
                    break;

    }





}
void cSWE_KIControl::ControlLight(int lights)
{
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
}

void cSWE_KIControl::ControlHL()
{
    hlsearch=!hlsearch;
    //Enable or Disable HalteLinien
}


