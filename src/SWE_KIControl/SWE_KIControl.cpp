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


    RETURN_IF_FAILED(m_oInputObjectData.Create("ObjectData", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputObjectData));

    RETURN_IF_FAILED(m_oInputRoadData.Create("RoadData", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputRoadData));

    RETURN_IF_FAILED(m_oInputSignData.Create("SignData", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSignData));



    RETURN_IF_FAILED(m_oInputParkData.Create("ParkData", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputParkData));

    RETURN_IF_FAILED(m_oInputTC.Create("TCData", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputTC));
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
 
        while (dummycount>=0)
        {
             Commands.push_back(3);
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
		parking=false;
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

                DriverCalc();
        }
        else if(pSource == &m_oInputRoadData)
        {

            //Punkte liste fuellen


        }
        else if(pSource== &m_oInputSignData)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputRoadSign->Lock(pMediaSample, &pCoder));
           int value=0;
           pCoder->Get("int8Value", (tVoid*)&value);
           m_pCoderDescInputRoadSign->Unlock(pCoder);

            if(parking&&value==1)//und wenn schildtyp parken.
            {
				SecondSigntype=1;
			}
            else if(value==3)//wenn Nur gerade aus Schild
			{
				SecondSigntype=3;
			}
            else if(value!=1)//wenn schildtyp nicht parken
			{
                 Signtype=value;
			}
            /*Hier den  ausgelesenen Wert aus dem Schilder modul rein
            Es gibt folgende Schildtypen:
			1=Parken
			2=Vorfahrt
			3=Nur gerade aus
			4=geweahren
			5=halt
			6=Vorfahrt rechts
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
		kreuzungstypen:
		1=geradeaus und links
		2=geradeaus und rechts
		3=gerade aus und beides
		4=links und rechts



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

	float carwidth=450;
	float site=carwidth/2;
	
	if(objecte.size()>0)
	{
		int t=objecte.size();
        for(int i=0;i<t;i++)
		{
			if(halteLinie)
			{
				/*pruefen ob Kreuzung frei
				anhand des Schildes und der Kreuzungstypen bestimmen
				kreuzungstypen:
				1=geradeaus und links
				2=geradeaus und rechts
				3=gerade aus und beides
				4=links und rechts
				  Es gibt folgende Schildtypen:
			1=Parken
			2=Vorfahrt
			3=Nur gerade aus
			4=geweahren
			5=halt
			6=Vorfahrt rechts
				*/
				switch(Signtype)
				{
					case 2:
						SpeedControl=1;
						roadfree=true;
						break;
						break;
                    case 4||5:
						//immmer, die Kreuzung an sich prüfen
                        if(kreuzungstyp==1)
						{
							//links und gerad aus prüfen
						}
                        else if(kreuzungstyp==2)
						{
							//rechts und gerade aus prüfen
						}
                        else if(kreuzungstyp==3)
						{
							//alles Prüfen
						}
                        else if(kreuzungstyp==4)
						{
							//rechts und links prüfen
						}
							roadfree=false;
						break;
						break;
					case 6:
							//immmer, die Kreuzung an sich prüfen
                        if(kreuzungstyp==1)
						{
							// gerad aus prüfen
						}
                        else if(kreuzungstyp==2||kreuzungstyp==3)
						{
							//rechts und gerade aus prüfen
						}
                        else if(kreuzungstyp==4)
						{
							//rechts prüfen
						}
							roadfree=false;
						break;
						break;
				}
			
			}
			else
			{
				/*
				*Punkte liste durchlaufen
				hierbei geraden zwischen den punkten ziehen angefangen am Ursprung,wenn in Gefahrenberreich
				dies wird bestimmt durch gerade und distanz zur geraden 
				Distanz Zum Auto berrechnen und wenn Innerhalb der Spec bremsen oder langsamer werden
				 */

			
				//wenn objekt relevant, distanz prüfen
				double distanz=sqrt(objecte[i].first*objecte[i].first+objecte[i].second*objecte[i].second);
			


			
                if(distanz<=5)
                {
                        //Notbremsung
                    SpeedControl=0;
                    sendTC(0,0);
                    ControlLight(9);

                }
                else if(10>=distanz>5)
                {
                    //langsames Hinterherfahren
                        SpeedControl=1;
                }
                else
                {
                    //Objekte weit genug weg
                        SpeedControl=2;
                }



			}
		}
	}
	else
	{
		SpeedControl=2;
	}
}
void cSWE_KIControl::sendTC(int speed, int type)
{
	//wenn einmal typ notbremsung, dann speed immer auf 0 und typ immer auf notbremsung setzen, bis speed wieder hochgesetzt wird.
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
            if(Signtype<=3)
            {
                 sendTC(SpeedControl,1);
                  ControlLight(1);
            }
            else if(Signtype>3)
            {

                //an Schildtyp anpassen:
                // rechts vor links, gewähren und stop, stop und gewähren gleich
             
                //pruefen ob wir schon an der HalteLinie stehen.
                if(halteLinie)
                {
					if(!hlsearch)
						   ControlHL();
					//Hier muss Kreuzungstyp feststehen
					if(SecondSigntype!=3 && kreuzungstyp!=2 )//alle typen bei dennen ein links abbiegen möglich ist.
					{
						
						if(abgebogen)
						{
							if(CommandCounter!=0)
								CommandCounter--;
							else
							{
								//Game Over sieg
							}
							Signtype=0;
							SecondSigntype=0;
							abgebogen=false;
							halteLinie=false;
							roadfree=true;
							kreuzungstyp=0;
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
						if(abgebogen)
						{
							
							Signtype=0;
							SecondSigntype=0;
							abgebogen=false;
							halteLinie=false;
							roadfree=true;
							kreuzungstyp=0;
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
                    if(Signtype<3)
					{
					      sendTC(SpeedControl,1);
						   ControlLight(1);
					}
					else if(Signtype>=3)
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


