#include <cmath>
#include "stdafx.h"
#include "SWE_ParkPilot.h"

#include <iostream>
#include <fstream>

// +++ begin_defines +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define POS_IR_SIDE_RIGHT -150.0    //y-Value of IR sensors
#define TH_SHORT 140.0
#define TH_LONG_ALONGSIDE 590.0
#define TH_LONG_CROSS 590.0

#define PARK_ALONGSIDE 1            // park style
#define PARK_CROSS 2                // park style

#define ALONGSIDE_SIZE 900          // minimum size of parking lot
#define CROSS_SIZE 500              // minimum size of parking lot
#define EASY_ALONGSIDE 100          // buffer for easy S-curve maneuver

#define INVALIDE_LOW 0.0
#define INVALIDE_HIGH 9999.0

#define POS_IR_FRONT_SIDE 450.0     // x-Pos of front IR sensor


// +++ end_defines ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


ADTF_FILTER_PLUGIN("SWE ParkPilot", OID_ADTF_SWE_PARKPILOT, cSWE_ParkPilot)

cSWE_ParkPilot::cSWE_ParkPilot(const tChar* __info) : cFilter(__info)
{
        m_IRFrontRightCur = INVALIDE_HIGH;
        m_IRRearRightCur = INVALIDE_HIGH;
        m_IRFrontRightOld = INVALIDE_HIGH;
        m_IRRearRightOld = INVALIDE_HIGH;
        m_distCur = 0.0;
        m_distEntry = 0.0;
        m_parkTrigger = 0;
        m_lastIRshort = 0.0;
        m_searchActive = false;
        m_entry = false;
        m_entrySaved = false;
        m_minDistReached = false;
        m_odometryData.distance_x = 0.0;
        m_odometryData.distance_y = 0.0;
        m_odometryData.angle_heading = 0.0;
        m_odometryData.distance_sum = 0.0;
        m_odometryData.velocity = 0.0;

}

cSWE_ParkPilot::~cSWE_ParkPilot()
{
}

tResult cSWE_ParkPilot::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_inputParkTrigger.Create("Park Trigger", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputParkTrigger));

    RETURN_IF_FAILED(m_inputOdometry.Create("Odometry", new cMediaType(0, 0, 0, "tOdometry"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputOdometry));

    RETURN_NOERROR;
}

tResult cSWE_ParkPilot::CreateOutputPins(__exception)
{

    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    RETURN_IF_FAILED(m_outputVelocity.Create("Speed Signal", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputVelocity));

    RETURN_NOERROR;
}

tResult cSWE_ParkPilot::Init(tInitStage eStage, __exception)
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

tResult cSWE_ParkPilot::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_ParkPilot::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_ParkPilot::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_ParkPilot::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);


//    if (pType != NULL)
//    {
//        cObjectPtr<IMediaTypeDescription> pMediaTypeDescParkBool;
//        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescParkBool));
//        m_pCoderDescParkBool = pMediaTypeDescParkBool;

//    }

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDesc != NULL)
    {

        RETURN_IF_POINTER_NULL( pMediaSample);

        if(pSource == &m_inputParkTrigger)
        {
           // save the park trigger: alongside or cross
           cObjectPtr<IMediaCoder> pCoder;
           RETURN_IF_FAILED(m_pCoderDesc->Lock(pMediaSample, &pCoder));
           pCoder->Get("tInt", (tVoid*)&m_parkTrigger);
           m_pCoderDesc->Unlock(pCoder);

        }
        else if(pSource == &m_ObjectData)
        {
            // leave if we have not received a park trigger
            if( m_parkTrigger != PARK_ALONGSIDE && m_parkTrigger != PARK_CROSS )
            {
                RETURN_NOERROR;
            }

            // if we have received a park trigger: take a look at the IR data
            cv::Point2d objectData[10];
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDesc->Lock(pMediaSample, &pCoder));
            pCoder->Get("PointArray", (tVoid*)&objectData);
            m_pCoderDesc->Unlock(pCoder);

            m_IRFrontRightCur = (-1) * (objectData[1].y - POS_IR_SIDE_RIGHT);
            m_IRRearRightCur = (-1) * (objectData[4].y - POS_IR_SIDE_RIGHT);

            // save latest "short" value
            if( m_IRFrontRightCur < TH_SHORT )
            {
                m_lastIRshort = m_IRFrontRightCur;
            }

            // Check whether we have reached the first car; if yes, activate the search

            //TODO: fuer eine kurze mindestlaenge
            if(m_IRFrontRightCur > TH_SHORT && m_searchActive == false)
            {
                RETURN_NOERROR;
            }
            else
            {
                m_searchActive = true;

                if(m_parkTrigger == PARK_ALONGSIDE)
                {
                    searchRoutineAlongside();
                }
                else if(m_parkTrigger == PARK_CROSS)
                {
                    searchRoutineCross();
                }
                else
                {
                    RETURN_NOERROR;
                }
            }

        }
        else if(pSource == &m_inputOdometry)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDesc->Lock(pMediaSample, &pCoder));
            pCoder->Get("distance_x", (tVoid*)&m_odometryData.distance_x);
            pCoder->Get("distance_y", (tVoid*)&m_odometryData.distance_y);
            pCoder->Get("angle_heading", (tVoid*)&m_odometryData.angle_heading);
            pCoder->Get("velocity", (tVoid*)&m_odometryData.velocity);
            pCoder->Get("distance_sum", (tVoid*)&m_odometryData.distance_sum);
            m_pCoderDesc->Unlock(pCoder);
        }



    }
    RETURN_NOERROR;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ ALONGSIDE ROUTINE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

tResult cSWE_ParkPilot::searchRoutineAlongside()
{
    tFloat32 distTravelled = 0;
    tFloat32 distStartPark = 0;

    //...now search for possible lot beginning and save the distance at this point
    if( m_IRFrontRightCur >= TH_LONG_ALONGSIDE )
    {
        if(m_entrySaved == false)
        {
            m_entry = true;
        }
        else
        {
            m_entry = false;
        }

        if(m_entry == true)
        {
            // save data at entry point
            m_distEntry = m_odometryData.distance_sum;
            m_entrySaved = true;
        }

        if(m_entrySaved == true)
        {
            distTravelled = m_distCur - m_distEntry;

            // check if space is already sufficient
            if( distTravelled >= ALONGSIDE_SIZE )
            {
                m_minDistReached = true;
            }
            else
            {
                m_minDistReached = false;
            }
        }
    }
    else if( m_IRRearRightCur <= TH_SHORT && m_entrySaved == true )
    {
        // groesse muss hier nicht gespeichert werden, da m_minDistReached vorherzugeschlagen haette
        // dient nur dazu die Suche nach einer neuen Parkluecke zu initalisieren
        m_entrySaved = false;
        RETURN_NOERROR;
    }

    // minimum distance is reached....
    // .....and we reached easy parking situation
    if( m_minDistReached == true && distTravelled >= (ALONGSIDE_SIZE + EASY_ALONGSIDE) )
    {
        //easy
        distStartPark = m_distCur;
        parkRoutineAlongside(distTravelled, distStartPark);
    }
    // ....and we take as much space as we can get
    else if( m_minDistReached == true && m_IRFrontRightCur <= TH_SHORT )
    {
        //normal
        distStartPark = m_distCur;
        parkRoutineAlongside(distTravelled, distStartPark);
    }

    RETURN_NOERROR;
}


tResult cSWE_ParkPilot::parkRoutineAlongside(tFloat32 lotSize, tFloat32 distStartPark)
{

    tFloat32 centralAngle = atan( 1 + ( (150 + m_lastIRshort) / (-949) ) );
    tFloat32 distToGo = POS_IR_FRONT_SIDE + ( 800 * sin( centralAngle ) );

    if( m_distCur - distStartPark >= distToGo)
    {
        //VOLLBREMSUNG

        //MAXIMALER LENKEINSCHLAG RECHTS

        //RÜCKWÄRTS BIS ROT = centralAngle + ausgangswinkel

        //MAXIMALER LENKEINLAG LINKS

        //RÜCKWÄRTS BIS ROT = ausgangswinkel


    }


    //easy maneuver
    if( lotSize > (ALONGSIDE_SIZE + EASY_ALONGSIDE) )
    {

    }
    //normal maneuver
    else
    {

    }


    // at the end....
    m_entry = false;
    m_entrySaved = false;
    m_minDistReached = false;
    RETURN_NOERROR;
}




// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ CROSS ROUTINE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




tResult cSWE_ParkPilot::searchRoutineCross()
{
    RETURN_NOERROR;
}

tResult cSWE_ParkPilot::parkRoutineCross(tFloat32 lotSize, tFloat32 distStartPark)
{
    RETURN_NOERROR;
}






tResult cSWE_ParkPilot::sendSpeed(tInt8 speed)
{

    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSpeedOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescSpeedOut->WriteLock(pMediaSample, &pCoder));
    pCoder->Set("tInt8SignalValue", (tVoid*)&(speed));
    m_pCoderDescSpeedOut->Unlock(pCoder);

    //transmit media sample over output pin
    //RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_outputVelocity.Transmit(pMediaSample));

    RETURN_NOERROR;

}







