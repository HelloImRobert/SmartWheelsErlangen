#include <cmath>
#include "stdafx.h"
#include "SWE_KIControl.h"

#include <iostream>
#include <fstream>

ADTF_FILTER_PLUGIN("SWE KIControl", OID_ADTF_SWE_KICONTROL, cSWE_KIControl)
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Constructor-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

cSWE_KIControl::cSWE_KIControl(const tChar* __info) : cFilter(__info)
{
    SetPropertyStr("ManeuverFile", "");
        SetPropertyBool("ManeuverFile" NSSUBPROP_FILENAME, tTrue);
        SetPropertyStr("ManeuverFile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");



}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Destructor-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

cSWE_KIControl::~cSWE_KIControl()
{
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------InputPins Erstellen-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

tResult cSWE_KIControl::CreateInputPins(__exception)
{
    //MB NeuetInt8SignalValue PinstPointstPointstPoints






//whyyy dont you gooo
    RETURN_IF_FAILED(m_oInputParkData.Create("ParkData", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputParkData));

    RETURN_IF_FAILED(m_oInputTC.Create("TCData", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputTC));





    RETURN_IF_FAILED(m_oInputSignData.Create("SignData", new cMediaType(0, 0, 0, "tRoadSign"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSignData));


    RETURN_IF_FAILED(m_oInputObjectData.Create("ObjectData", new cMediaType(0, 0, 0, "tPointArray"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputObjectData));

    RETURN_IF_FAILED(m_oInputRoadData.Create("RoadData", new cMediaType(0, 0, 0, "tPointArray"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputRoadData));


    RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", new cMediaType(0, 0, 0, "tJuryStruct"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));



    RETURN_NOERROR;
}


 
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------OutputPins Erstellen-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


tResult cSWE_KIControl::CreateOutputPins(__exception)
{        
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));



    // TO ADAPT tInt8SignalValuefor new Pin/Dadatype: strDescPointLeft, "tPoint2dtInt8SignalValue", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescLightOutput = pDescManager->GetMediaDescription("tInt8SignalValue");
    RETURN_IF_POINTER_NULL(strDescLightOutput);
    cObjectPtr<IMediaType> pTypeLightData = new cMediaType(0, 0, 0, "tInt8SignalValue", strDescLightOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLightData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLightOutput));

    RETURN_IF_FAILED(m_oOutputLightControl.Create("LightControl", pTypeLightData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputLightControl));

    //----------------------------------------------------tKITC----tKITC------

    tChar const * strDescTCOutput = pDescManager->GetMediaDescription("tKITC");
    RETURN_IF_POINTER_NULL(strDescTCOutput);
    cObjectPtr<IMediaType> pTypeTCData = new cMediaType(0, 0, 0, "tKITC", strDescTCOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeTCData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescTCOutput));

    RETURN_IF_FAILED(m_oOutputTC.Create("TCtoKIData", pTypeTCData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputTC));
    //--------------------------------------------------------------
    //----------------------------------------------------Parkingloot----Parkingloot------

    tChar const * strDescTCOutput2 = pDescManager->GetMediaDescription("tInt8SignalValue");
    RETURN_IF_POINTER_NULL(strDescTCOutput2);
    cObjectPtr<IMediaType> pTypeParkData = new cMediaType(0, 0, 0, "tInt8SignalValue", strDescTCOutput2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeParkData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescParkOutput));

    RETURN_IF_FAILED(m_oOutputParking.Create("Parkingloot", pTypeParkData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputParking));
    //--------------------------------------------------------------

    //----------------------------------------------------Driverstruct----Driverstruct------

    tChar const * strDescTCOutput3 = pDescManager->GetMediaDescription("tDriverStruct");
    RETURN_IF_POINTER_NULL(strDescTCOutput3);
    cObjectPtr<IMediaType> pTypedriverstruct = new cMediaType(0, 0, 0, "tDriverStruct", strDescTCOutput3,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypedriverstruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverStruct));

    RETURN_IF_FAILED(m_oOutputDriverStruct.Create("DriverStruct", pTypedriverstruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputDriverStruct));
    //--------------------------------------------------------------









    RETURN_NOERROR;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------init-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

tResult cSWE_KIControl::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);





    }
    else if (eStage == StageNormal)
    {
        halteLinie=false;
        hlsearch=false;
        abgebogen=false;
        roadfree=true;
        Signtype=0;
		parking=false;
        adminstopp=false;
        for(int a=0;a<10;a++)
        {
            points[a].x=0;
            points[a].y=2000+a;
            objecte[a].x=0;
            objecte[a].y=2000+a;
        }

        signsize=0;
    }
    else if(eStage == StageGraphReady)
    {


        m_maneuverListFile = GetPropertyStr("ManeuverFile");

          if (m_maneuverListFile.IsEmpty())
          {
              LOG_ERROR("DriverFilter: Maneuver file not found");
              RETURN_ERROR(ERR_INVALID_FILE);
          }
          ADTF_GET_CONFIG_FILENAME(m_maneuverListFile);

              m_maneuverListFile = m_maneuverListFile.CreateAbsolutePath(".");


              if (cFileSystem::Exists(m_maneuverListFile))
                 {
                     cDOM oDOM;
                     oDOM.Load(m_maneuverListFile);
                     cDOMElementRefList oSectorElems;
                     cDOMElementRefList oManeuverElems;

                     //read first Sector Elem
                     if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
                     {
                         //iterate through sectors
                         for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
                         {
                             //if sector found
                             tSector sector;
                             sector.id = (*itSectorElem)->GetAttributeUInt32("id");

                             if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
                             {
                                 //iterate through maneuvers
                                 for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                                 {
                                     tAADC_Maneuver man;
                                     man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                                     man.action = (*itManeuverElem)->GetAttribute("action");
                                     sector.maneuverList.push_back(man);
                                     if(man.action =="left")
                                     {
                                            Commands.push_back(1);
                                     }
                                     else if(man.action =="right")
                                     {
                                             Commands.push_back(2);
                                     }
                                     else if(man.action =="straight")
                                     {
                                             Commands.push_back(3);
                                     }
                                     else if(man.action =="parallel_parking")
                                     {
                                             Commands.push_back(4);
                                     }
                                     else if(man.action =="cross_parking")
                                     {
                                             Commands.push_back(5);
                                     }
                                     else if(man.action =="pull_out_left")
                                     {
                                             Commands.push_back(6);
                                     }
                                     else if(man.action =="pull_out_right")
                                     {
                                             Commands.push_back(7);
                                     }


                                 }
                             }

                             m_sectorList.push_back(sector);
                         }
                     }
                     if (oSectorElems.size() > 0)
                     {
                         LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
                     }
                     else
                     {
                         LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
                         RETURN_ERROR(ERR_INVALID_FILE);
                     }
                 }
                 else
                 {
                     LOG_ERROR("DriverFilter: no valid Maneuver File found!");
                     RETURN_ERROR(ERR_INVALID_FILE);
                 }

              /*
               * Man?ver
               * in m_sectorList stehen alle abschnitte drin, jeder abschnitt hat eine Id.
               * in den abschnitten stehen die verschiedenen commandos, mit fortlaufender id

               *
               */
              Commandsector=0;
              Commandsectormax=m_sectorList.size()-1;
              CommandCountermax=m_sectorList[Commandsector].maneuverList.size()-1;
             // CommandCountermax=m_sectorList[0].maneuverList.size()-1;
              CommandCounter=0;

        //CommandCounter=6;
//    for(int ab=0;ab<m_sectorList[0].maneuverList.size();ab++)
//    {

//        if(m_sectorList[0].maneuverList[ab].action=="left")
//        {
//               Commands.push_back(1);
//        }
//        else if(m_sectorList[0].maneuverList[ab].action=="right")
//        {
//                Commands.push_back(2);
//        }
//        else if(m_sectorList[0].maneuverList[ab].action=="straight")
//        {
//                Commands.push_back(3);
//        }
//        else if(m_sectorList[0].maneuverList[ab].action=="parallel_parking")
//        {
//                Commands.push_back(4);
//        }
//        else if(m_sectorList[0].maneuverList[ab].action=="cross_parking")
//        {
//                Commands.push_back(5);
//        }
//        else if(m_sectorList[0].maneuverList[ab].action=="pull_out_left")
//        {
//                Commands.push_back(6);
//        }
//        else if(m_sectorList[0].maneuverList[ab].action=="pull_out_right")
//        {
//                Commands.push_back(7);
//        }
//      //  LOG_ERROR("MB: Found ");
//    }
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

    RETURN_NOERROR;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Start Stop etc-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Onpin Event-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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
        //--------------------------------------------------------------Objecte erkennen und FahrRoutine----------------------------------------------------------------------------------------
        if(pSource == &m_oInputObjectData)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

           pCoder->Get("tPoint", (tVoid*)&objecte); //Werte auslesen
            m_pCoderDescInputMeasured->Unlock(pCoder);
            //Hier die Objekte auslesen, und in eigene Vecor bauen
            if( adminstopp!=true)
            {
                ObjectAvoidance();

                DriverCalc();
            }
            else
            {
                SpeedControl=0;
                sendTC(0,0);
            }
        }
        //--------------------------------------------------------------RoadData----------------------------------------------------------------------------------------
        else if(pSource == &m_oInputRoadData)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));
            pCoder->Get("tPoint", (tVoid*)&objecte); //Werte auslesen
            m_pCoderDescInputMeasured->Unlock(pCoder);
            //Punkte liste fuellen


        }
        //--------------------------------------------------------------Schilder einlesen----------------------------------------------------------------------------------------
        else if(pSource== &m_oInputSignData)
        {

            tInt8 value = 0;
            tFloat32 area = 0;
           cObjectPtr<IMediaCoder> pCoder;
           RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

           pCoder->Get("i8Identifier", (tVoid*)&value);
           pCoder->Get("fl32Imagesize", (tVoid*)&area);


           m_pCoderDescInputMeasured->Unlock(pCoder);




// So bekommen wir die Daten
           //Vorfahrt gewaehren 1

           //Vorfahrt an naechster Kreuzung 2

           //Halt! Vorfahrt gewaehren (Stop) 3

           //Parken 4

           //Vorgeschriebene Fahrtrichtung geradeaus 5

           //Kreuzung 6

           //erstmal unwichtig
           //Fussgaengerueberweg 7

           //Kreisverkehr 8

           //Ueberholverbot 9

           //Verbot der Einfahrt 10

           //Einbahnstrasse 11


            if(parking&&value==4)//und wenn schildtyp parken.
            {
                if(signsize>area)
                {
                    signsize=area;
                }
                else
                {
                    SecondSigntype=4;
                }
			}
            else if(value==5)//wenn Nur gerade aus Schild
			{
                SecondSigntype=5;
			}
            else if(value<=3 || value==6)//wenn schildtyp nicht parken
			{
                if(signsize>area)
                {
                }
                else
                {
                   signsize=area;
                   Signtype=value;
                }
			}                 

        }
        //--------------------------------------------------------------Parkdaten einlesen----------------------------------------------------------------------------------------

        else if(pSource==&m_oInputParkData)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));
            int value=0;
            pCoder->Get("int8Value", (tVoid*)&value);
            m_pCoderDescInputMeasured->Unlock(pCoder);

            if(value==1 && (Commands[CommandCounter]==4 ||Commands[CommandCounter]==5))
            {
                if(CommandCounter!=CommandCountermax)
                {
                   CommandCounter++;
                }
                else
                {

                }
            }
            else if(value==2 && (Commands[CommandCounter]==6 ||Commands[CommandCounter]==7))
            {
                if(CommandCounter!=CommandCountermax)
                {
                   CommandCounter++;
                }
                else
                {

                }
             }
        }

        //--------------------------------------------------------------Daten vom JuryModul einlesen----------------------------------------------------------------------------------------

        else if(pSource==&m_JuryStructInputPin)
        {
                       tInt8 i8ActionID = -2;
                       tInt16 i16entry = -1;
                       cObjectPtr<IMediaCoder> pCoder;
                       RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

                           pCoder->Get("i8ActionID", (tVoid*)&i8ActionID);
                           pCoder->Get("i16ManeuverEntry", (tVoid*)&i16entry);


                       if (i8ActionID==-1)
                       {
                           //scheinbar stopp befehl
                        adminstopp=true;
                        //SendtoJury(tInt8 i8StateID, tInt16 i16ManeuverEntry)
                          // if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received: Stop with maneuver ID %d",i16entry));
                           //emit sendStop((int)i16entry);
                       }
                       else if (i8ActionID==0)
                       {
                           //Ready anforderung

                          // if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received: Request Ready with maneuver ID %d",i16entry));
                           //emit sendRequestReady((int)i16entry);
                       }
                       else if (i8ActionID==1)
                       {
                              adminstopp=false;
                           //Start
                          // if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received: Run with maneuver ID %d",i16entry));
                           //emit sendRun((int)i16entry);
                       }
        }



        //--------------------------------------------------------------Daten vom TC einlesen----------------------------------------------------------------------------------------

        else if(pSource==&m_oInputTC)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));
            int value=0;
            pCoder->Get("int8Value", (tVoid*)&value);
            m_pCoderDescInputMeasured->Unlock(pCoder);
           if(value==0)
           {

           }
           else if(value==1)
           {
            abgebogen=true;
           }
           else if(value==2)
           {
            halteLinie=true;
           }

           /*
            * TC rueckmeldungen:
            0=nichts besonderes
            1=bin abgebogen
            2=stehe an haltelinie
           */

        }


    }
    RETURN_NOERROR;
}



//Modul fuer Juroren einbauen
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Parken-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

tResult cSWE_KIControl::Parkroutine(int value)
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    // ADAPT: m_pCoderDescPointLeft
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescParkOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    // ADAPT: m_pCoderDescPointLeft
    //cObjectPtr<IMediaCoder> pCoder;
    //---------------------------------------Front scheinwerfer-----------------------------------------------------------------------------------------
    RETURN_IF_FAILED(m_pCoderDescParkOutput->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("int8Value", (tVoid*)&(value));
    m_pCoderDescParkOutput->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputParking.Transmit(pMediaSampleOutput));
   RETURN_NOERROR;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------ObjectAvoidance-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void cSWE_KIControl::ObjectAvoidance()
{

	float carwidth=450;
	float site=carwidth/2;
    float Notbrems= 500;
    float Slowdist=1000;
    double distline=0;

    cv::Point2d pointtocheck;

//Die werte für das Objekt array
        int t=10;
        int i=0;

//Alle Objekte durchlaufen
        for( i=0;i<t;i++)
		{
           LOG_INFO(cString::Format( "MB:Ki Object wert eingelesen"));

//Wenn objekte gleich 0,0 oder 9999,9999 nicht beachten
          if((objecte[i].x!=0 && objecte[i].y!=0) ||(objecte[i].x!=9999 && objecte[i].y!=9999))
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

            1=geweahren
			2=Vorfahrt
            3=halt
            4=Parken
            5=Nur gerade aus
			6=Vorfahrt rechts



				*/
				switch(Signtype)
				{
					case 2:
						SpeedControl=1;
						roadfree=true;
						break;
						break;
                    case 1||3:
						//immmer, die Kreuzung an sich prüfen
                        if(kreuzungstyp==1)
						{
							//links und gerad aus prüfen
                            m_boundary.first.x=0;
                            m_boundary.first.y=0;

                            m_boundary.second.x=0;//zweiter wert auf der anderen Seite der KReuzung
                            m_boundary.second.y=0;



                            pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                            pointtocheck.y=objecte[i].y;

                            distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts
                            if(distline>0)
                            {

                                roadfree=false;
                                break;
                                break;
                            }


						}
                        else if(kreuzungstyp==2)
						{
							//rechts und gerade aus prüfen
                            m_boundary.first.x=0;
                            m_boundary.first.y=0;

                            m_boundary.second.x=0;//zweiter wert auf der anderen Seite der KReuzung
                            m_boundary.second.y=0;



                            pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                            pointtocheck.y=objecte[i].y;

                            distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts
                            if(distline<0)
                            {

                                roadfree=false;
                                break;
                                break;
                            }
						}
                        else if(kreuzungstyp==3)
						{
							//alles Prüfen
                            m_boundary.first.x=0;
                            m_boundary.first.y=0;

                            m_boundary.second.x=0;//zweiter wert auf der anderen Seite der KReuzung
                            m_boundary.second.y=0;



                            pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                            pointtocheck.y=objecte[i].y;

                            distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts
                            if(distline>0 ||distline<0)
                            {

                                roadfree=false;
                                break;
                                break;
                            }
						}
                        else if(kreuzungstyp==4)
						{
							//rechts und links prüfen
                            m_boundary.first.x=0;
                            m_boundary.first.y=0;

                            m_boundary.second.x=0;//zweiter wert auf der anderen Seite der KReuzung
                            m_boundary.second.y=0;



                            pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                            pointtocheck.y=objecte[i].y;

                            distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts
                            if(distline>0 ||distline<0)
                            {

                                roadfree=false;
                                break;
                                break;
                            }
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
                            m_boundary.first.x=0;
                            m_boundary.first.y=0;

                            m_boundary.second.x=0;//zweiter wert auf der anderen Seite der KReuzung
                            m_boundary.second.y=0;



                            pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                            pointtocheck.y=objecte[i].y;

                            distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts
                            if(distline<0)
                            {

                                roadfree=false;
                                break;
                                break;
                            }
						}
                        else if(kreuzungstyp==4)
						{
							//rechts prüfen
                            m_boundary.first.x=0;
                            m_boundary.first.y=0;

                            m_boundary.second.x=0;//zweiter wert auf der anderen Seite der KReuzung
                            m_boundary.second.y=0;



                            pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                            pointtocheck.y=objecte[i].y;

                            distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts
                            if(distline<0)
                            {

                                roadfree=false;
                                break;
                                break;
                            }
						}
							roadfree=false;
						break;
						break;
				}
			
			}
			else
			{
                //Straßenpunkte  auf 10 setzen
                    int ti=10;
                    int ia=0;
                    //Alle Straßenpunkte durchlaufen
                    for( ia=0;ia<ti;ia++)
                    {
                        if(ia==0)
                        {
                            m_boundary.first.x=0;
                            m_boundary.first.y=0;
                        }
                        else
                        {
                            m_boundary.first.x=points[ia-1].x;
                            m_boundary.first.y=points[ia-1].y;

                        }
                        m_boundary.second.x=points[ia].x;
                        m_boundary.second.y=points[ia].y;

                        pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                        pointtocheck.y=objecte[i].y;
                      distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts


                        if(distline<0)
                            distline*=-1;

                        if(distline<=site)
                        {

                            double distanz=sqrt(objecte[i].x*objecte[i].x+objecte[i].y*objecte[i].y);

                            if(distanz<=Notbrems)
                            {
                                 //Notbremsung
                                SpeedControl=0;
                                sendTC(0,0);
                                ControlLight(9);
                                LOG_INFO(cString::Format( "MB:Ki Notbremsung"));

                            }
                            else if(Slowdist>=distanz && distanz>Notbrems)
                            {
                                //langsames Hinterherfahren
                                ControlLight(6);
                                    SpeedControl=1;
                            }
                            else
                            {
                                //Objekte weit genug weg
                                ControlLight(1);
                                    SpeedControl=2;
                            }


                        }

                    }

			}
          }
        }

    return ;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Daten an TC senden-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

tResult cSWE_KIControl::sendTC(int speed, int type)
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
    
    */

    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    // ADAPT: m_pCoderDescPointLeft
    cObjectPtr<IMediaSerializer> pSerializer;

    m_pCoderDescTCOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    // ADAPT: m_pCoderDescPointLeft
    //cObjectPtr<IMediaCoder> pCoder;
    //---------------------------------------Tc Daten-----------------------------------------------------------------------------------------
    RETURN_IF_FAILED(m_pCoderDescTCOutput->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("tInt8ValueSpeed", (tVoid*)&(speed));
    pCoder->Set("tInt8ValueCommand", (tVoid*)&(type));
    m_pCoderDescTCOutput->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputTC.Transmit(pMediaSampleOutput));

RETURN_NOERROR;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------DriverCalc, das Gehirn-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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

       Schildtypen:
            1=geweahren
            2=Vorfahrt
            3=halt
            4=Parken
            5=Nur gerade aus
            6=Vorfahrt rechts

 */

// TODO: Parken richtig einbauen, und die Kreuzungsgeschichte Korrekteinbinden
    switch (Commands[CommandCounter])
    {
        //left-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 1:
          LOG_ERROR("MB: Links abbiegen jetzte aber richtig ");
            if(Signtype==4 ||Signtype==0 ) // wenn wir im Parken modus oder kein Schild haben
            {
                 sendTC(SpeedControl,1);

                  ControlLight(1);
            }
            else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
            {

                //an Schildtyp anpassen:
                // rechts vor links, gewähren und stop, stop und gewähren gleich
             
                //pruefen ob wir schon an der HalteLinie stehen.
                if(halteLinie)
                {
					if(!hlsearch)
						   ControlHL();

                    //immer bisschen warten am besten aber nur 1 mal
                        tTimeStamp m_currTimeStamp= cSystem::GetTime();
                        while((cSystem::GetTime()-m_currTimeStamp)<5);

					//Hier muss Kreuzungstyp feststehen
					if(SecondSigntype!=3 && kreuzungstyp!=2 )//alle typen bei dennen ein links abbiegen möglich ist.
					{
						
						if(abgebogen)
						{
                            if(CommandCounter!=CommandCountermax)
                                CommandCounter++;
							else
							{
                                Commandsector++;
                                 CommandCountermax+=m_sectorList[Commandsector].maneuverList.size()-1;

                                //Game Over sieg hier rein was passieren soll, wenn ziel erreicht
							}
							Signtype=0;
							SecondSigntype=0;
                            signsize=0;
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
                            signsize=0;
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
        if(Signtype==4 ||Signtype==0 ) // wenn wir im Parken modus oder kein Schild haben
        {
             sendTC(SpeedControl,1);
             LOG_INFO(cString::Format( "MB:KiLicht an "));
              ControlLight(1);
        }
        else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
        {

            //an Schildtyp anpassen:
            // rechts vor links, gewähren und stop, stop und gewähren gleich

            //pruefen ob wir schon an der HalteLinie stehen.
            if(halteLinie)
            {
                if(!hlsearch)
                       ControlHL();
                if(Signtype==5)
                {
                    tTimeStamp m_currTimeStamp= cSystem::GetTime();
                    while((cSystem::GetTime()-m_currTimeStamp)<5);
                }
                //Hier muss Kreuzungstyp feststehen

                if(SecondSigntype!=3 && kreuzungstyp!=1 )//alle typen bei dennen ein links abbiegen möglich ist.
                {

                    if(abgebogen)
                    {
                        if(CommandCounter!=CommandCountermax)
                            CommandCounter++;
                        else
                        {
                             Commandsector++;
                               CommandCountermax+=m_sectorList[Commandsector].maneuverList.size()-1;
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
                            sendTC(SpeedControl,3);
                            ControlLight(5);
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
        //straigth-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 3:
        if(Signtype==4 ||Signtype==0 ) // wenn wir im Parken modus oder kein Schild haben
        {
             sendTC(SpeedControl,1);
             LOG_INFO(cString::Format( "MB:KiLicht an "));
              ControlLight(1);
        }
        else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
        {

            //an Schildtyp anpassen:
            // rechts vor links, gewähren und stop, stop und gewähren gleich

            //pruefen ob wir schon an der HalteLinie stehen.
            if(halteLinie)
            {
                if(!hlsearch)
                       ControlHL();
                if(Signtype==5)
                {
                    tTimeStamp m_currTimeStamp= cSystem::GetTime();
                    while((cSystem::GetTime()-m_currTimeStamp)<5);
                }
                //Hier muss Kreuzungstyp feststehen

                if(kreuzungstyp!=4 )//alle typen bei dennen ein gerade aus möglich ist.
                {

                    if(abgebogen)
                    {
                        if(CommandCounter!=CommandCountermax)
                            CommandCounter++;
                        else
                        {
                             Commandsector++;
                               CommandCountermax+=m_sectorList[Commandsector].maneuverList.size()-1;
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
                            sendTC(SpeedControl,5);
                            ControlLight(1);
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
                            sendTC(SpeedControl,0);
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
        //Einparken1-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 4:

                    parking=true;
                    //wennn Parkschild erkannt, sende Parken jetzt wenn parkschild das größte schild ist das erkannt wird
                    if(SecondSigntype==4)
                    {
                         Parkroutine(1);
                    }


                    break;
        //einparken2-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 5:

             parking=true;
             //wennn Parkschild erkannt, sende Parken jetzt wenn parkschild das größte schild ist das erkannt wird
                 if(SecondSigntype==4)
                 {
                      Parkroutine(2);
                 }
                    break;
        //ausparken1-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 6:

                    SecondSigntype=0;
                    Parkroutine(3);
                    break;
        //ausparken2-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 7:

                    SecondSigntype=0;
                    Parkroutine(4);
                    break;

    }





}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Licht Steuerung-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

tResult cSWE_KIControl::ControlLight(int lights)
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
    if(lights==9)
    {
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
    pCoder->Set("int8Value", (tVoid*)&(lights));
    m_pCoderDescLightOutput->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputLightControl.Transmit(pMediaSampleOutput));
}
RETURN_NOERROR;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------HalteLinien Steuerung-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void cSWE_KIControl::ControlHL()
{
    hlsearch=!hlsearch;
    //Enable or Disable HalteLinien
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Distanz zu geraden Funktion Mat-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

double cSWE_KIControl::getPerpendicDistance(const cv::Point2d& referencePoint)
{
    // calculate line directions
    cv::Point2d x = m_boundary.first - referencePoint;
    cv::Point2d directionVector = m_boundary.second - m_boundary.first;
    cv::Point2d orthogonalDirection;
    orthogonalDirection.x = -directionVector.y;
    orthogonalDirection.y = directionVector.x;

    // calculate intersection point
    double cross = orthogonalDirection.x*directionVector.y - orthogonalDirection.y*directionVector.x;
    if (fabs(cross) < 1e-8)
    {
        //throw std::domain_error("Point is on the line!");
        return 0;
    }

    double t1 = (x.x * directionVector.y - x.y * directionVector.x)/cross;
    cv::Point2d intersecPoint = (referencePoint + orthogonalDirection * t1);

    // calculate distance
    double distance = cv::norm(intersecPoint - referencePoint);

    // calculate sign of position of intersetion point (+ left of vehicle; - right of vehicle)
    if(intersecPoint.y <= 0){
        distance = distance*(-1);
    }


    return distance;
}
tResult cSWE_KIControl::SendtoJury(tInt8 i8StateID, tInt16 i16ManeuverEntry)
{
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescDriverStruct->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();

        RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
        {   // focus for sample write lock
            __adtf_sample_write_lock_mediadescription(m_pCoderDescDriverStruct,pMediaSample,pCoder);


            pCoder->Set("i8StateID", (tVoid*)&i8StateID);
            pCoder->Set("i16ManeuverEntry", (tVoid*)&i16ManeuverEntry);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oOutputDriverStruct.Transmit(pMediaSample);
        RETURN_NOERROR;
}
