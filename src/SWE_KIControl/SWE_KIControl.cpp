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
    //MB NeuetInt8SignalValue PinstPointstPointstPoints das MAgische es geht alles wieder Kommentarfeld







    RETURN_IF_FAILED(m_oInputParkData.Create("ParkData", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputParkData));

    RETURN_IF_FAILED(m_oInputTC.Create("TCData", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputTC));





    RETURN_IF_FAILED(m_oInputSignData.Create("SignData", new cMediaType(0, 0, 0, "tRoadSign"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSignData));


    RETURN_IF_FAILED(m_oInputObjectData.Create("ObjectData", new cMediaType(0, 0, 0, "tPointArray"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputObjectData));

    RETURN_IF_FAILED(m_oInputRoadData.Create("RoadData", new cMediaType(0, 0, 0, "tTrajectory"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputRoadData));


    RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", new cMediaType(0, 0, 0, "tJuryStruct"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));

    RETURN_IF_FAILED(m_oCrossingIndicator.Create("Crossing_Indicator", new cMediaType(0, 0, 0, "tCrossingIndicator"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oCrossingIndicator));






    //TODO: Pin fuer einlesen der Kreuzungsdaten



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
 //----------------------------------------------------Lanetrigger----------

    tChar const * lanestr = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(lanestr);

    cObjectPtr<IMediaType> pTypelaneData = new cMediaType(0, 0, 0, "tBoolSignalValue", lanestr,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(pTypelaneData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLane));

    RETURN_IF_FAILED(m_oOutputLane.Create("Lanetrigger", pTypelaneData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputLane));



     //----------------------------------------------------tclane------
    tChar const * lanestr2 = pDescManager->GetMediaDescription("tCrossingIndicator");
    RETURN_IF_POINTER_NULL(lanestr2);

    cObjectPtr<IMediaType> pTypelaneData2 = new cMediaType(0, 0, 0, "tCrossingIndicator", lanestr2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(pTypelaneData2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDesctclane));

    RETURN_IF_FAILED(m_oOutputtclane.Create("Crossing_Indicatortotc", pTypelaneData2, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputtclane));










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
        crosscall=false;
        adminstopp=false; //Wenn Wettkampf, auf True setzen!!!!!!! GANNNZ WICHTIG SONST GEHT GAR NIX MIT JURY
        status=0;
        Parksteuerung=0;
        blinking=0;
        crosscalldone=false;
        for(int a=0;a<10;a++)
        {
        cv::Point2d dumm;
            dumm.x=2000+a;
            dumm.y=0;
            trajectory.push_back(dumm);


          //  trajectory.at(a).x=2000+a;
           // trajectory.at(a).y=0;
          //  points[a].x=2000+a;
           // points[a].y=0;
            objecte[a].x=2000+a;
            objecte[a].y=0;
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
        CommandCountermax=m_sectorList[m_sectorList.size()-1].maneuverList.size()-1;
        // CommandCountermax=m_sectorList[0].maneuverList.size()-1;
        CommandCounter=0;


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

   // m_mutex.Enter(); //(Robert)

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

            stringstream elementGetter;
            for( size_t j = 0; j < 10; j++)
            {
                elementGetter << "tPoint[" << j << "].xCoord";
                pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(objecte[j].x));
                elementGetter.str(std::string());

                elementGetter << "tPoint[" << j << "].yCoord";
                pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(objecte[j].y));
                elementGetter.str(std::string());
            }
            m_pCoderDescInputMeasured->Unlock(pCoder);

            //      pCoder->Get("tPoint", (tVoid*)&objecte); //Werte auslesen
            // pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            //           for(int a=0;a<10;a++)
            //           {

            //              // if(objecte[a].x==0 && objecte[a].y==2000+a)
            //                    //  LOG_ERROR("Standard wert nicht gut");
            //               string currentDestinationName , currentSourceName;
            //               {
            //                   // convoluted way to concatenate the arrayName with an integer, but standard C++
            //                   std::stringstream stringStream;
            //                   stringStream << "DestinationPointsiii" << a;
            //                   currentDestinationName = stringStream.str();
            //               }
            //               {
            //                   std::stringstream stringStream;
            //                   stringStream << "Sensor " << a << "xwert: "<<objecte[a].x <<" ywert: "<<objecte[a].y;
            //                   currentSourceName = stringStream.str();
            //               }
            //               LOG_ERROR(currentSourceName.c_str());

            //     }



            if( adminstopp==false)
            {
                if(parking==false)
                {
                    if(status==3)
                        SendtoJury(2, CommandCounter);
                    ObjectAvoidance();


                    //testfalle

                }

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
            // pCoder->Get("tPoint", (tVoid*)&points); //Werte auslesen

            //Punkte liste fuellen

           // std::vector< cv::Point2d > trajectory;
            {
                tInt8 BoundaryArrayCountTemp = 0;
                pCoder->Get("TrajectoryPoints.Count", (tVoid*)&(BoundaryArrayCountTemp));
                trajectory.resize( BoundaryArrayCountTemp );

                stringstream elementGetter;
                for( int j = 0; j < BoundaryArrayCountTemp; j++)
                {
                    elementGetter << "TrajectoryPoints.Points[" << j << "].xCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(trajectory.at(j).x));
                    elementGetter.str(std::string());

                    elementGetter << "TrajectoryPoints.Points[" << j << "].yCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(trajectory.at(j).y));
                    elementGetter.str(std::string());
                }
            }
        m_pCoderDescInputMeasured->Unlock(pCoder);




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

            // value
            // 1= einparken fertig
            //2= ausparken fertig
            //3= bin bereit Steuerung zu uebernehmen
            if(value==1 && (Commands[CommandCounter]==4 ||Commands[CommandCounter]==5))
            {
                if(CommandCounter!=CommandCountermax)
                {
                    CommandCounter++;
                }
                else
                {
                    status=3;
                }
            }
            else if(value==2 && (Commands[CommandCounter]==6 ||Commands[CommandCounter]==7))
            {
                if(CommandCounter!=CommandCountermax)
                {
                    CommandCounter++;
                    Parksteuerung=0;
                }
                else
                {
                    status=3;
                }
            }
            else if(value==3)
            {
                Parksteuerung=3;
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

            m_pCoderDescInputMeasured->Unlock(pCoder);

            if(i16entry>=0)
                CommandCounter= i16entry;

            if (i8ActionID==-1)
            {
                //scheinbar stopp befehl
                adminstopp=true;

                //Parkroutine für Markus das er weiß jetzt ist stopp
                Parkroutine(0);


                //Variablen neu Initialisieren
                halteLinie=false;
                hlsearch=false;
                blinking=0;
                abgebogen=false;
                roadfree=true;
                Signtype=0;
                parking=false;
                crosscall=false;
                status=0;
                Parksteuerung=0;
                for(int a=0;a<10;a++)
                {
                    trajectory.at(a).x=2000+a;
                    trajectory.at(a).y=0;
                   // points[a].x=2000+a;
                   // points[a].y=0;
                    objecte[a].x=2000+a;
                    objecte[a].y=0;
                }

                signsize=0;


            }
            else if (i8ActionID==0)
            {

                CommandCounter=i8ActionID;

                //Ready anforderung
                //TODO:: Ready an Jury senden
                //0 ready
                //-1 error
                //1 running
                //2 complete
                //
                //
                SendtoJury(0, CommandCounter);
            }
            else if (i8ActionID==1)
            {
                adminstopp=false;
                CommandCounter=i16entry;
                //Start

            }
        }
        //--------------------------------------------------------------Daten von kreuzungserkennung einlesen----------------------------------------------------------------------------------------

        else if(pSource == &m_oCrossingIndicator)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("isRealStopLine", (tVoid*)&(m_stoplineData.isRealStopLine));
            pCoder->Get("crossingType", (tVoid*)&(m_stoplineData.crossingType));
            pCoder->Get("StopLinePoint1.xCoord", (tVoid*)&(m_stoplineData.StopLinePoint1.x));
            pCoder->Get("StopLinePoint1.yCoord", (tVoid*)&(m_stoplineData.StopLinePoint1.y));
            pCoder->Get("StopLinePoint2.xCoord", (tVoid*)&(m_stoplineData.StopLinePoint2.x));
            pCoder->Get("StopLinePoint2.yCoord", (tVoid*)&(m_stoplineData.StopLinePoint2.y));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------
//            1=geradeaus und links
//            2=geradeaus und rechts
//            3=gerade aus und beides
//            4=links und rechts
            if(Commands[CommandCounter]>5)
            {

                if(m_stoplineData.crossingType == 4)
                    parkbefehl=1;
                else
                      parkbefehl=2;

            }
            else if(Commands[CommandCounter]==3)
            {

               transmitCrossingIndicator(  m_stoplineData.isRealStopLine, m_stoplineData.crossingType , m_stoplineData.StopLinePoint1 , m_stoplineData.StopLinePoint2 );

            }
            else
            {
                if(crosscall)
                {
                    kreuzungstyp=m_stoplineData.crossingType ;

                    crosscall=false;
                    crosscalldone=true;
                }
                else
                {
                    if(!m_stoplineData.isRealStopLine)
                        m_stoplineData.isRealStopLine=true;

                    transmitCrossingIndicator(  m_stoplineData.isRealStopLine, m_stoplineData.crossingType , m_stoplineData.StopLinePoint1 , m_stoplineData.StopLinePoint2 );
                }
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
            2=stehe an haltelinieelementGetter
           */

        }


    }

   // m_mutex.Leave(); //(Robert)

    RETURN_NOERROR;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Send to Lane-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


 tResult cSWE_KIControl::sendtoLane(bool value)
 {
     cObjectPtr<IMediaCoder> pCoder;

     //create new media sample
     cObjectPtr<IMediaSample> pMediaSampleOutput;
     RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

     //allocate memory with the size given by the descriptor
     // ADAPT: m_pCoderDescPointLeft
     cObjectPtr<IMediaSerializer> pSerializer;
     m_pCoderDescLane->GetMediaSampleSerializer(&pSerializer);
     tInt nSize = pSerializer->GetDeserializedSize();
     pMediaSampleOutput->AllocBuffer(nSize);

     //write date to the media sample with the coder of the descriptor
     // ADAPT: m_pCoderDescPointLeft
     //cObjectPtr<IMediaCoder> pCoder;
     //---------------------------------------parkwert----------------------------------------------------------------------------------------
     RETURN_IF_FAILED(m_pCoderDescLane->WriteLock(pMediaSampleOutput, &pCoder));
     pCoder->Set("bValue", (tVoid*)&(value));
     m_pCoderDescLane->Unlock(pCoder);

     //transmit media sample over output pin
     // ADAPT: m_oIntersectionPointLeft
     RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
     RETURN_IF_FAILED(m_oOutputLane.Transmit(pMediaSampleOutput));
     RETURN_NOERROR;
 }





//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------Parken-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

tResult cSWE_KIControl::Parkroutine(tInt8 value)
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
    //---------------------------------------parkwert----------------------------------------------------------------------------------------
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
    double site=carwidth/2;
    float Notbrems= 700;
    float Slowdist=1500;
    double distline=0;

    cv::Point2d pointtocheck;

    //Die werte f\FCr das Objekt array
    int t=10;
    int i=0;
    int noObject=10;
    //Alle Objekte durchlaufen
    for( i=0;i<t;i++)
    {

        //Wenn objekte gleich 0,0 oder 9999,9999 nicht beachten
        if((objecte[i].x!=0 && objecte[i].y!=0) &&(objecte[i].x!=9999 && objecte[i].y!=9999) && i!=9 && i!=8 && i!=7 && i!=3 && i!=2)
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
                    //immmer, die Kreuzung an sich pruefen
                    if(kreuzungstyp==1)
                    {
                        //links und gerad aus pruefen
                        m_boundary.first.x=0;
                        m_boundary.first.y=0;
                        //TODO:: richtige werte fuer zweiten Punkt
                        m_boundary.second.x=200;//zweiter wert auf der anderen Seite der KReuzung
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
                        //rechts und gerade aus pruefen
                        m_boundary.first.x=0;
                        m_boundary.first.y=0;

                        m_boundary.second.x=200;//zweiter wert auf der anderen Seite der KReuzung
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
                        //alles Pruefen
                        m_boundary.first.x=0;
                        m_boundary.first.y=0;

                        m_boundary.second.x=200;//zweiter wert auf der anderen Seite der KReuzung
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
                        //rechts und links pruefen
                        m_boundary.first.x=0;
                        m_boundary.first.y=0;

                        m_boundary.second.x=200;//zweiter wert auf der anderen Seite der KReuzung
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
                    //immmer, die Kreuzung an sich pruefen
                    if(kreuzungstyp==1)
                    {
                        // gerad aus pr\FCfen
                    }
                    else if(kreuzungstyp==2||kreuzungstyp==3)
                    {
                        //rechts und gerade aus pruefen
                        m_boundary.first.x=0;
                        m_boundary.first.y=0;

                        m_boundary.second.x=200;//zweiter wert auf der anderen Seite der KReuzung
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
                        //rechts pruefen
                        m_boundary.first.x=0;
                        m_boundary.first.y=0;

                        m_boundary.second.x=200;//zweiter wert auf der anderen Seite der KReuzung
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


                int ti=trajectory.size();//10;
                int ia=0;
                //Alle Strassenpunkte durchlaufen
                for( ia=0;ia<ti;ia++)
                {
                    if(ia==0)
                    {
                        m_boundary.first.x=0;
                        m_boundary.first.y=0;
                    }
                    else
                    {
                        m_boundary.first.x=trajectory[ia-1].x;
                        m_boundary.first.y=trajectory[ia-1].y;

                    }
                    m_boundary.second.x=trajectory[ia].x;
                    m_boundary.second.y=trajectory[ia].y;

                    pointtocheck.x=objecte[i].x;//Punkt aus der Objekte liste
                    pointtocheck.y=objecte[i].y;
                    distline=getPerpendicDistance(pointtocheck);//negativer wert ist rechts


                    if(distline<0)
                        distline*=-1;




                    if(distline<=site)
                    {

                        double distanz;
                        distanz=cv::norm(objecte[i]);

                        if(distanz<=Notbrems)
                        {

                            //Notbremsung
                            SpeedControl=0;
                            sendTC(0,0);
                            ControlLight(9);

                            //                               //  LOG_ERROR("MB:Ki Notbremsung");

                            string currentDestinationName , currentSourceName;
                            {
                                // convoluted way to concatenate the arrayName with an integer, but standard C++
                                std::stringstream stringStream;
                                stringStream << "DestinationPointsiii" << i;
                                currentDestinationName = stringStream.str();
                            }
                            {
                                std::stringstream stringStream;
                                stringStream << "Sensor " << i << " Distanz "<< distanz << "xwert: "<<objecte[i].x <<" ywert: "<<objecte[i].y ;
                                currentSourceName = stringStream.str();
                            }


                            // LOG_ERROR(currentSourceName.c_str());


                            return;

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
                            SpeedControl=3;
                        }


                    }
                    else
                    {

                        SpeedControl=3;
                    }

                }

            }
        }
        else
        {
            noObject--;
            if(noObject==0)
                SpeedControl=3;
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
    6=Parking( Steurung aus / idle)
    7= speed signal off    (Robert)
    Speed:
    Stufen: 3,2,1,0,-1,-2      (Robert) -> Stufe 3 ist implementiert und sollte auch genutzt werden da 2 noch recht langsam ist
    
    */
    if(speed==0)
    {
        type=0; //(Robert) besser echten emergency stop und stop aufgrund von hindernissen trennen => normaler stop nur speed = 0, damit kann ein gestopptes manoever wieder aufgenommen werden. Bei echtem emergency stop vllt. warnblinker an?
    }



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

    // TODO: die Kreuzungsgeschichte Korrekteinbinden
    switch (Commands[CommandCounter])
    {
    //left-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    case 1:
        // LOG_ERROR("MB: Links abbiegen jetzte aber richtig ");
        if(Signtype==4 ||(Signtype==0 && kreuzungstyp==0)) // wenn wir im Parken modus oder kein Schild haben und keine Kreuzung
        {
            sendTC(SpeedControl,1);

            ControlLight(1);
        }
        else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
        {


            //pruefen ob wir schon an der HalteLinie stehen.
            if(halteLinie)
            {
                if(!hlsearch)
                    ControlHL();

                //immer bisschen warten am besten aber nur 1 mal
                tTimeStamp m_currTimeStamp= cSystem::GetTime();
                while((cSystem::GetTime()-m_currTimeStamp)<5); //(Robert)


                crosscall=true;
                sendtoLane(true);
                if(crosscalldone)
                {
                    //Hier muss Kreuzungstyp feststehen
                    if(SecondSigntype!=3 && kreuzungstyp!=2 )//alle typen bei dennen ein links abbiegen moeglich ist.
                    {

                        if(abgebogen)
                        {
                            if(CommandCounter!=CommandCountermax)
                                CommandCounter++;
                            else
                            {
                                status=3;

                                //Game Over sieg hier rein was passieren soll, wenn ziel erreicht
                            }
                            Signtype=0;
                            SecondSigntype=0;
                            signsize=0;
                            abgebogen=false;
                            halteLinie=false;
                            roadfree=true;
                            crosscalldone=false;
                            kreuzungstyp=0;
                             crosscalldone=false;
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
                             crosscalldone=false;
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
            }
            else
            {
                if(hlsearch)
                    ControlHL();

                sendTC(SpeedControl,1);//1
                ControlLight(1);
            }
        }
        break;
        //right-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    case 2:
         if(Signtype==4 ||(Signtype==0 && kreuzungstyp==0))// wenn wir im Parken modus oder kein Schild haben
        {
            sendTC(SpeedControl,1);
            LOG_INFO(cString::Format( "MB:KiLicht an "));
            ControlLight(1);
        }
        else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
        {

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
                crosscall=true;
                sendtoLane(true);
                if(crosscalldone)
                {
                if(SecondSigntype!=3 && kreuzungstyp!=1 )//alle typen bei dennen ein links abbiegen moeglich ist.
                {

                    if(abgebogen)
                    {
                        if(CommandCounter!=CommandCountermax)
                            CommandCounter++;
                        else
                        {
                            status=3;
                        }
                        Signtype=0;
                        SecondSigntype=0;
                        abgebogen=false;
                        halteLinie=false;
                        crosscalldone=false;
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
                         crosscalldone=false;
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
        if(Signtype==4 ||(Signtype==0 && kreuzungstyp==0)) // wenn wir im Parken modus oder kein Schild haben
        {
            sendTC(SpeedControl,1);

            ControlLight(1);
        }
        else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
        {

            //an Schildtyp anpassen:
            // rechts vor links, gew\E4hren und stop, stop und gewaehren gleich

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
                crosscall=true;
                sendtoLane(true);
                if(crosscalldone)
                {
                if(kreuzungstyp!=4 )//alle typen bei dennen ein gerade aus moeglich ist.
                {

                    if(abgebogen)
                    {
                        if(CommandCounter!=CommandCountermax)
                            CommandCounter++;
                        else
                        {
                            status=3;
                        }
                        Signtype=0;
                        SecondSigntype=0;
                        abgebogen=false;
                        halteLinie=false;
                        roadfree=true;
                        crosscalldone=false;
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
                       crosscalldone=false;
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

        //parallel parken-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    case 4:

        parking=true;
        //wennn Parkschild erkannt, sende Parken jetzt wenn parkschild das groesste schild ist das erkannt wird
        if(Parksteuerung==3)
        {
            Parkroutine(7);
            sendTC(SpeedControl,6);
        }
        else
        {
            if(SecondSigntype==4)
            {
                      Parkroutine(1);
                         parkbefehl=1;
            }                    

             if(Signtype==4 ||(Signtype==0 && kreuzungstyp==0)) // wenn wir im Parken modus oder kein Schild haben
            {
                if(SpeedControl!=0)
                sendTC(1,1);
                else
                sendTC(SpeedControl,1);

                ControlLight(1);
            }
            else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
            {

                //an Schildtyp anpassen:
                // rechts vor links, gewaehren und stop, stop und gewaehren gleich

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
                    crosscall=true;
                    sendtoLane(true);
                    if(crosscalldone)
                    {
                    if(kreuzungstyp!=4 )//alle typen bei dennen ein gerade aus moeglich ist.
                    {

                        if(abgebogen)
                        {
crosscalldone=false;
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
                           crosscalldone=false;
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
                }
                else
                {
                    if(hlsearch)
                        ControlHL();

                    sendTC(SpeedControl,1);
                    ControlLight(1);
                }
            }
        }


        break;
        //querparken-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    case 5:

        parking=true;
        //wennn Parkschild erkannt, sende Parken jetzt wenn parkschild das groesste schild ist das erkannt wird
        if(Parksteuerung==3)
        {
            Parkroutine(7);
            sendTC(SpeedControl,6);
        }
        else
        {
            if(SecondSigntype==4)
            {
                Parkroutine(2);
                parkbefehl=2;
            }
             if(Signtype==4 ||(Signtype==0 && kreuzungstyp==0)) // wenn wir im Parken modus oder kein Schild haben
            {
                if(SpeedControl!=0)
                sendTC(1,1);
                else
                sendTC(SpeedControl,1);

                ControlLight(1);
            }
            else if(Signtype>2 || Signtype==1) //wenn das Schild auswirkungen auf die Fahrregeln hat
            {

                //an Schildtyp anpassen:
                // rechts vor links, gewaehren und stop, stop und gewaeren gleich

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
                    crosscall=true;
                    sendtoLane(true);
                    if(crosscalldone)
                    {
                    if(kreuzungstyp!=4 )//alle typen bei dennen ein gerade aus moeglich ist.
                    {

                        if(abgebogen)
                        {
 crosscalldone=false;
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
  crosscalldone=false;
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
                }
                else
                {
                    if(hlsearch)
                        ControlHL();

                    sendTC(SpeedControl,1);
                    ControlLight(1);
                }
            }
        }
        break;


        //pull_out_left-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    case 6:

        SecondSigntype=0;

        if(parkbefehl==0)
         sendtoLane(true);
        if(parkbefehl==1)
             Parkroutine(3);
        else
            Parkroutine(5);
        break;
        //pull_out_right-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    case 7:

        SecondSigntype=0;

         if(parkbefehl==0)
              sendtoLane(true);

           if(parkbefehl==1)
                  Parkroutine(4);
           else
                    Parkroutine(6);
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
    if(blinking!=lights && (lights==3 ||lights==5 || lights==1))
       {

        blinking=lights;

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
//--------------------------------------------------------------Crossingindicator-------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



tResult cSWE_KIControl::transmitCrossingIndicator( const tBool isRealStopLine , const tInt8 crossingType , const cv::Point2d& StopLinePoint1 , const cv::Point2d& StopLinePoint2 )
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleCrossIndicator;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleCrossIndicator));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDesctclane->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleCrossIndicator->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDesctclane->WriteLock(pMediaSampleCrossIndicator, &pCoder));

    pCoder->Set("isRealStopLine", (tVoid*)&(isRealStopLine));
    pCoder->Set("crossingType", (tVoid*)&(crossingType));
    pCoder->Set("StopLinePoint1.xCoord", (tVoid*)&(StopLinePoint1.x));
    pCoder->Set("StopLinePoint1.yCoord", (tVoid*)&(StopLinePoint1.y));
    pCoder->Set("StopLinePoint2.xCoord", (tVoid*)&(StopLinePoint2.x));
    pCoder->Set("StopLinePoint2.yCoord", (tVoid*)&(StopLinePoint2.y));

    m_pCoderDesctclane->Unlock(pCoder);

    //transmit media sample over output pin

    RETURN_IF_FAILED(pMediaSampleCrossIndicator->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oOutputtclane.Transmit(pMediaSampleCrossIndicator));

    RETURN_NOERROR;


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
