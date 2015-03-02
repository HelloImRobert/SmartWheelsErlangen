#include "SWE_ParkPilot.h"

// +++ begin_defines +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*! Sensor positions */
#define POS_IR_SIDE_RIGHT -150.0
#define POS_IR_FRONT_SIDE 450.0

/*! Thresholds */
#define TH_SHORT 140.0
#define TH_LONG_ALONGSIDE 590.0
#define TH_LONG_CROSS 590.0

/*! Park styles */
#define PARK_ALONGSIDE 1
#define PARK_CROSS 2

/*! Lot sizes */
#define ALONGSIDE_SIZE 765         // minimum size of parking lot
#define CROSS_SIZE 500             // ??minimum size of parking lot
#define EASY_ALONGSIDE 40          // buffer for easy S-curve maneuver

/*! Helpers to calculate central angle */
#define CALC_QUOTIENT -799
#define CALC_CARHALF 150
#define CALC_RADIUS 400
#define DEG_TO_RAD 0.017453292

/*! Invalide sensor values */
#define INVALIDE_LOW 0.0
#define INVALIDE_HIGH 9999.0

/*! Steering angles */
#define STEER_RIGHT_MAX -30.0             // Maximaler Lenkwinkel rechts
#define STEER_LEFT_MAX 30.0               // Maximaler Lenkwinkel links
#define STEER_NEUTRAL 0.0                 // Lenkwinkel = 0

/*! Adjustment for normal parking maneuver */
// TODO: in filter properties schreiben
#define ANGLE_ADJUSTMENT 3                  // in degree !
#define DIST_ADJUSTMENT 30
#define COUNT_MANEUVERS 4

// +++ end_defines ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


ADTF_FILTER_PLUGIN("SWE ParkPilot", OID_ADTF_SWE_PARKPILOT, cSWE_ParkPilot)

cSWE_ParkPilot::cSWE_ParkPilot(const tChar* __info) : cFilter(__info)
{
        m_IRFrontRightCur = INVALIDE_HIGH;
        m_IRRearRightCur = INVALIDE_HIGH;
        m_IRFrontRightOld = INVALIDE_HIGH;
        m_IRRearRightOld = INVALIDE_HIGH;
        m_distEntry = 0.0;
        m_parkTrigger = 0;
        m_lastIRshort = 0.0;

        m_searchActive = false;
        m_entry = false;
        m_entrySaved = false;
        m_minDistReached = false;
        m_carStopped = false;

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
    RETURN_IF_FAILED(m_inputParkTrigger.Create("Park_Trigger", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputParkTrigger));

    RETURN_IF_FAILED(m_inputObjectData.Create("Object_Data", new cMediaType(0, 0, 0, "tPoints"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputObjectData));

    RETURN_IF_FAILED(m_inputOdometry.Create("Odometry_Data", new cMediaType(0, 0, 0, "tOdometry"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputOdometry));

    RETURN_NOERROR;
}

tResult cSWE_ParkPilot::CreateOutputPins(__exception)
{

    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    RETURN_IF_FAILED(m_outputVelocity.Create("Speed_Signal", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputVelocity));

    RETURN_IF_FAILED(m_outputSteering.Create("Steering_Signal", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputSteering));

    RETURN_NOERROR;

    // oder so?? :P
    /*
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSteeringOut));

        RETURN_IF_FAILED(m_oOutput.Create("output_value", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutput));
        RETURN_NOERROR;
     */
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
        else if(pSource == &m_inputObjectData)
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

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ ALONGSIDE ROUTINE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
            distTravelled = m_odometryData.distance_sum - m_distEntry;

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
// park style
    // minimum distance is reached....
    // .....and we reached easy parking situation
    if( m_minDistReached == true && distTravelled >= (ALONGSIDE_SIZE + EASY_ALONGSIDE) )
    {
        //easy
        distStartPark = m_odometryData.distance_sum;
        parkRoutineAlongsideEasy(distStartPark);
    }
    // ....and we take as much space as we can get
    else if( m_minDistReached == true && m_IRFrontRightCur <= TH_SHORT )
    {
        //normal
        distStartPark = m_odometryData.distance_sum;
        parkRoutineAlongsideNormal(distStartPark);
    }

    RETURN_NOERROR;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ALONGSIDE EASY +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::parkRoutineAlongsideEasy(tFloat32 distStartPark)
{

    tFloat32 centralAngle = atan( 1 + ( (CALC_CARHALF + m_lastIRshort) / (CALC_QUOTIENT) ) );
    tFloat32 distToGo = POS_IR_FRONT_SIDE + ( 2 * CALC_RADIUS * sin( centralAngle ) );
    tFloat32 actualDistGone = 0.0;
    tFloat32 overshoot = 0.0;
    tFloat32 headingAtStart = 0.0;


    if( m_odometryData.distance_sum - distStartPark >= distToGo)
    {
        //VOLLBREMSUNG
        sendSpeed( 0 );
    }

    if( m_carStopped == true )
    {
        actualDistGone = m_odometryData.distance_sum - distStartPark;
        overshoot = actualDistGone - distToGo;

        // Blink right

        // Go backwards....
        sendSpeed( -1 );
    }


    //...until "distToGo" is reached again
    // maximum steering angle right (als servo signal rausgeben!! +-30)
    if( (actualDistGone - m_odometryData.distance_sum) >= overshoot )
    {
        headingAtStart = m_odometryData.angle_heading;
        sendSteeringAngle(STEER_RIGHT_MAX);
    }

    // backwards until heading = headingAtStart + centralAngle....
    //....maximum steering angle left
    if( (m_odometryData.angle_heading - headingAtStart) >= centralAngle )
    {
        sendSteeringAngle(STEER_LEFT_MAX);
    }

    // backwards until headingAtStart is reached again
    if( m_odometryData.angle_heading <= headingAtStart )
    {
        sendSpeed( 0 );
        // Stop blinking
    }



    // at the end....
    m_entry = false;
    m_entrySaved = false;
    m_minDistReached = false;
    RETURN_NOERROR;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ALONGSIDE NORMAL +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::parkRoutineAlongsideNormal(tFloat32 distStartPark)
{

    tFloat32 centralAngle = atan( 1 + ( (CALC_CARHALF + m_lastIRshort) / (CALC_QUOTIENT) ) ) + ( ANGLE_ADJUSTMENT * DEG_TO_RAD );
    tFloat32 distToGo = POS_IR_FRONT_SIDE + ( 2 * CALC_RADIUS * sin( centralAngle ) ) - DIST_ADJUSTMENT;
    tFloat32 actualDistGone = 0.0;
    tFloat32 overshoot = 0.0;
    tFloat32 headingAtStart = 0.0;


    if( m_odometryData.distance_sum - distStartPark >= distToGo)
    {
        //VOLLBREMSUNG
        sendSpeed( 0 );
    }

    while(m_carStopped != true){}

    if( m_carStopped == true )
    {
        actualDistGone = m_odometryData.distance_sum - distStartPark;
        overshoot = actualDistGone - distToGo;

        // Blink right

        // Go backwards....
        sendSpeed( -1 );
    }

    //...until "distToGo" is reached again
    // maximum steering angle right (als servo signal rausgeben!! +-30)
    while( (actualDistGone - m_odometryData.distance_sum) >= overshoot ){}
    if( (actualDistGone - m_odometryData.distance_sum) >= overshoot )
    {
        headingAtStart = m_odometryData.angle_heading;
        sendSteeringAngle(STEER_RIGHT_MAX);
    }

    // backwards until heading = headingAtStart + centralAngle....
    //....maximum steering angle left
    if( (m_odometryData.angle_heading - headingAtStart) >= centralAngle )
    {
        sendSteeringAngle(STEER_LEFT_MAX);
    }

    // backwards until headingAtStart is reached again
    if( m_odometryData.angle_heading <= headingAtStart + ( ANGLE_ADJUSTMENT * DEG_TO_RAD ) )
    {
        sendSpeed( 0 );
        // Stop blinking
    }


    // Rangieren bis er drin is
    // hart coden oder algo?
    while(m_odometryData.angle_heading < headingAtStart)
    {
        sendSteeringAngle(STEER_RIGHT_MAX);
        sendSpeed( 1 );

        if( m_odometryData.angle_heading >= ( ANGLE_ADJUSTMENT * DEG_TO_RAD / COUNT_MANEUVERS) )
        {
            sendSpeed( 0 );
            sendSteeringAngle(STEER_LEFT_MAX);
            sendSpeed( -1 );
        }

        // NACHDENKEN!
//        if( m_odometryData.angle_heading )
//        {
//            sendSpeed( 0 );
//        }

    }



    // at the end....
    m_entry = false;
    m_entrySaved = false;
    m_minDistReached = false;
    RETURN_NOERROR;
}




// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ CROSS ROUTINE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




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


tResult cSWE_ParkPilot::sendSteeringAngle(tFloat32 steeringAngle)
{

    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSteeringOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescSteeringOut->WriteLock(pMediaSample, &pCoder));
    pCoder->Set("tSignalValue", (tVoid*)&(steeringAngle));
    m_pCoderDescSteeringOut->Unlock(pCoder);

    //transmit media sample over output pin
    //RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_outputSteering.Transmit(pMediaSample));

    RETURN_NOERROR;

}






