#include "SWE_ParkPilot.h"

// +++ begin_defines +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*! Sensor positions */
#define POS_IR_SIDE_RIGHT -150.0f
#define POS_IR_SIDE_LEFT 150.0f
#define POS_IR_FRONT_SIDE 450.0f

/*! Thresholds */
#define TH_SHORT 140.0f
#define TH_LONG_ALONGSIDE 590.0f
#define TH_LONG_CROSS 590.0f

/*! Park styles */
#define PARK_ALONGSIDE 1
#define PARK_CROSS 2
#define PULL_LEFT 3
#define PULL_RIGHT 4

/*! Lot sizes */
#define ALONGSIDE_SIZE 765.0f         // minimum size of parking lot
#define CROSS_SIZE 450.0f             // minimum size of parking lot
#define EASY_ALONGSIDE 40.0f          // buffer for easy S-curve maneuver

/*! Helpers to calculate central angle */
#define CALC_QUOTIENT -799.0f
#define CALC_CARHALF 150.0f
#define CALC_RADIUS 400.0f

/*! Other math helpers */
#define DEG_TO_RAD 0.017453292f
#define STEERING_TO_SERVO 0.71428571f

/*! Invalid sensor values */
#define INVALID_LOW 0.0f
#define INVALID_HIGH 9999.0f

/*! Steering angles */
#define STEER_RIGHT_MAX 30.0f             // Maximaler Lenkwinkel rechts
#define STEER_LEFT_MAX -30.0f               // Maximaler Lenkwinkel links
#define STEER_NEUTRAL 0.0f                 // Lenkwinkel = 0

// +++ end_defines ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


ADTF_FILTER_PLUGIN("SWE ParkPilot", OID_ADTF_SWE_PARKPILOT, cSWE_ParkPilot)

cSWE_ParkPilot::cSWE_ParkPilot(const tChar* __info) : cFilter(__info)
{
        // initialise member variables
        m_searchState = 0;
        m_parkState = 0;
        m_pulloutState = 0;

        m_IRFrontRightCur = INVALID_HIGH;
        m_IRRearRightCur = INVALID_HIGH;
        m_IRFrontLeftCur = INVALID_HIGH;
        m_distEntry = 0.0;
        m_distExit = 0.0;	// maybe kill
        m_parkTrigger = 0;
        m_lastIRshort = 0.0;

        m_distStartPark = 0.0;
        m_centralAngle = 0.0;
        m_counterAngle = 0.0;
        m_distStart = 0.0;
        m_rememberDist = 0.0;

        m_searchActive = false;
        m_minDistReached = false;
        m_carStopped = false;

        m_odometryData.distance_x = 0.0;
        m_odometryData.distance_y = 0.0;
        m_odometryData.angle_heading = 0.0;
        m_odometryData.distance_sum = 0.0;
        m_odometryData.velocity = 0.0;

        // set filter properties
        SetPropertyFloat("A_HeadingAngleAdjustment",3.0);
        SetPropertyFloat("A_DistanceAdjustment",40.0);
        SetPropertyFloat("A_SecurityBuffer", 20.0);

        SetPropertyFloat("C_StraightForward", 50.0);
        SetPropertyFloat("C_HeadingAngleLeftForward", 35.0);
        SetPropertyFloat("C_SteerAngleRightBackward", 31.8);
        SetPropertyFloat("C_PerpendicularBackward", 470.0);

        SetPropertyFloat("C_PullLeftStraight", 720.0);
        SetPropertyFloat("C_PullRightStraight", 300.0);


}

cSWE_ParkPilot::~cSWE_ParkPilot()
{
}

tResult cSWE_ParkPilot::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


    // PARK TRIGGER INPUT
    tChar const * strDescParkTrigger = pDescManager->GetMediaDescription("tInt8SignalValue");
    RETURN_IF_POINTER_NULL(strDescParkTrigger);
    cObjectPtr<IMediaType> pTypeParkTrigger = new cMediaType(0, 0, 0, "tInt8SignalValue", strDescParkTrigger,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeParkTrigger->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderParkTrigger));

    RETURN_IF_FAILED(m_inputParkTrigger.Create("Park_Trigger", pTypeParkTrigger, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputParkTrigger));


    // OBJECT DATA INPUT
    tChar const * strDescObjectData = pDescManager->GetMediaDescription("tPointArray");
    RETURN_IF_POINTER_NULL(strDescObjectData);
    cObjectPtr<IMediaType> pTypeObjectData = new cMediaType(0, 0, 0, "tPointArray", strDescObjectData,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeObjectData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescObjectData));

    RETURN_IF_FAILED(m_inputObjectData.Create("Object_Data", pTypeObjectData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputObjectData));


    // ODOMETRY INPUT
    tChar const * strDescOdometry = pDescManager->GetMediaDescription("tOdometry");
    RETURN_IF_POINTER_NULL(strDescOdometry);
    cObjectPtr<IMediaType> pTypeOdometry = new cMediaType(0, 0, 0, "tOdometry", strDescOdometry,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeOdometry->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescOdometry));

    RETURN_IF_FAILED(m_inputOdometry.Create("Odometry_Data", pTypeOdometry, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputOdometry));


    RETURN_NOERROR;
}

tResult cSWE_ParkPilot::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


    // SPEED OUTPUT
    tChar const * strDescSpeed = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSpeed);
    cObjectPtr<IMediaType> pTypeSpeed = new cMediaType(0, 0, 0, "tSignalValue", strDescSpeed,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSpeedOut));

    RETURN_IF_FAILED(m_outputSpeed.Create("Speed_Signal", pTypeSpeed, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputSpeed));


    // STEERING OUTPUT
    tChar const * strDescSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSteeringAngle);
    cObjectPtr<IMediaType> pTypeSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strDescSteeringAngle,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSteeringOut));

    RETURN_IF_FAILED(m_outputSteering.Create("Steering_Signal", pTypeSteeringAngle, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputSteering));



//    RETURN_IF_FAILED(m_outputParkState.Create("ParkState", new cMediaType(0, 0, 0, "tInt8SignalValue"), static_cast<IPinEventSink*> (this)));
//    RETURN_IF_FAILED(RegisterPin(&m_outputParkState));

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

        m_AngleAdjustment = GetPropertyFloat("A_HeadingAngleAdjustment");
        m_DistanceAdjustment = GetPropertyFloat("A_DistanceAdjustment");
        m_securityBuffer = GetPropertyFloat("A_SecurityBuffer");

        m_straightForward = GetPropertyFloat("C_StraightForward");
        m_headingAngleForward = GetPropertyFloat("C_HeadingAngleLeftForward");
        m_steeringAnlgeBackward = GetPropertyFloat("C_SteerAngleRightBackward");
        m_perpendicularBackward = GetPropertyFloat("C_PerpendicularBackward");
        m_pullLeftStraight = GetPropertyFloat("C_PullLeftStraight");
        m_pullRightStraight = GetPropertyFloat("C_PullRightStraight");
    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_ParkPilot::Start(__exception)
{
    // initialise member variables
    m_searchState = 0;
    m_parkState = 0;
    m_pulloutState = 0;

    m_IRFrontRightCur = INVALID_HIGH;
    m_IRRearRightCur = INVALID_HIGH;
    m_IRFrontLeftCur = INVALID_HIGH;
    m_distEntry = 0.0;
    m_distExit = 0.0;	// maybe kill
    m_parkTrigger = 0;
    m_lastIRshort = 0.0;

    m_distStartPark = 0.0;
    m_centralAngle = 0.0;
    m_counterAngle = 0.0;
    m_distStart = 0.0;
    m_rememberDist = 0.0;

    m_searchActive = false;
    m_minDistReached = false;
    m_carStopped = false;

    m_odometryData.distance_x = 0.0;
    m_odometryData.distance_y = 0.0;
    m_odometryData.angle_heading = 0.0;
    m_odometryData.distance_sum = 0.0;
    m_odometryData.velocity = 0.0;


    //FOR TEST START
    LOG_ERROR(cString("PP: Trigger set to 1" ));
    m_parkTrigger = 1;
    //FOR TEST ENDE
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
    RETURN_IF_POINTER_NULL(pSource);


//    if (pType != NULL)
//    {
//        cObjectPtr<IMediaTypeDescription> pMediaTypeDescParkBool;
//        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescParkBool));
//        m_pCoderDescParkBool = pMediaTypeDescParkBool;

//    }

    //if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDesc != NULL)
    if( nEventCode == IPinEventSink::PE_MediaSampleReceived )
    {

        RETURN_IF_POINTER_NULL( pMediaSample);
//        if(pSource == &m_inputParkTrigger)
//        {
//           // save the park trigger: alongside or cross
//           cObjectPtr<IMediaCoder> pCoder;
//           RETURN_IF_FAILED(m_pCoderParkTrigger->Lock(pMediaSample, &pCoder));
//           pCoder->Get("tInt8SignalValue", (tVoid*)&m_parkTrigger);
//           m_pCoderParkTrigger->Unlock(pCoder);

//        }
        if(pSource == &m_inputObjectData)
        {
            //LOG_ERROR(cString("Data from OBJECT" ));
            // leave if we have not received a park trigger
            if( m_parkTrigger > 4 || m_parkTrigger == 0)
            {
                RETURN_NOERROR;
            }

            //FOR TEST START
            sendSteeringAngle( STEER_NEUTRAL );
            LOG_ERROR(cString("PP: Send Speed" ));
            tFloat32 test = 1;
            sendSpeed( test );
            //FOR TEST ENDE

            // if we have received a park trigger: take a look at the IR data
            cv::Point2d objectData[10];
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescObjectData->Lock(pMediaSample, &pCoder));
            pCoder->Get("tPointArray", (tVoid*)&(objectData));
            m_pCoderDescObjectData->Unlock(pCoder);

            m_IRFrontRightCur = (-1) * (objectData[5].y - POS_IR_SIDE_RIGHT);
            m_IRRearRightCur = (-1) * (objectData[7].y - POS_IR_SIDE_RIGHT);
            m_IRFrontLeftCur = objectData[4].y - POS_IR_SIDE_LEFT;

            // Check whether we have reached the first car; if yes, activate the search
            // Check whether 10 samples are < TH_SHORT; if yes, active search
            if(m_searchActive == false)
            {
                tFloat32 nextElem;

                if(m_initTest_vect.size() < 10)
                {
                    m_initTest_vect.push_back(nextElem);
                    m_initTest_vect.back() = m_IRFrontRightCur;
                }
                else
                {
                    m_initTest_vect.erase( m_initTest_vect.begin() );
                    m_initTest_vect.push_back(nextElem);
                    m_initTest_vect.back() = m_IRFrontRightCur;
                }


                for(uint i = 0; i < m_initTest_vect.size(); ++i)
                {
                    if(m_initTest_vect.at(i) > TH_SHORT)
                    {
                        m_searchActive = false;
                    }
                    else
                    {
                        m_searchActive = true;
                    }
                }
            }

        }
        else if(pSource == &m_inputOdometry)
        {
            //LOG_ERROR(cString("PP: Data from ODOMETRY" ));
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescOdometry->Lock(pMediaSample, &pCoder));
            pCoder->Get("distance_x", (tVoid*)&(m_odometryData.distance_x));
            pCoder->Get("distance_y", (tVoid*)&(m_odometryData.distance_y));
            pCoder->Get("angle_heading", (tVoid*)&(m_odometryData.angle_heading));
            pCoder->Get("velocity", (tVoid*)&(m_odometryData.velocity));
            pCoder->Get("distance_sum", (tVoid*)&(m_odometryData.distance_sum));
            m_pCoderDescOdometry->Unlock(pCoder);
        }
    }

    /* INDICES:
     * 101... search Alongside
     * 201... park Alongside easy
     * 301... park Alongside normal
     * 401... pull out Alongside
     *
     * 501... search Cross
     * 601... park Cross
     * 701... pull out Cross right
     * 801... pull out Cross left
     */


    // Suche Aktivieren
    if(m_searchActive == true && m_parkTrigger == PARK_ALONGSIDE)
    {
        m_searchState = 101;
    }
    else if(m_searchActive == true && m_parkTrigger == PARK_CROSS)
    {
        m_searchState = 501;
    }




    // Was machen wir?
    if( m_searchState > 100 && m_searchState < 200 )
    {
        searchRoutineAlongside();
    }
    else if( m_searchState > 500 && m_searchState < 600 )
    {
        searchRoutineCross();
    }
    else if( m_searchState == 104 && m_parkState > 200 && m_parkState < 300 )
    {
        parkRoutineAlongsideEasy();
    }
    else if( m_searchState == 104 && m_parkState > 300 && m_parkState < 400 )
    {
        parkRoutineAlongsideNormal();
    }
    else if( m_searchState == 503 && m_parkState > 600 && m_parkState < 700 )
    {
        parkRoutineCross();
    }
    else if( m_parkTrigger == PULL_LEFT && m_parkState == 605 )
    {
        pullOutCrossLeft();
    }
    else if( m_parkTrigger == PULL_RIGHT && m_parkState == 605 )
    {
        pullOutCrossRight();
    }
    else if( (m_parkTrigger == PULL_LEFT && m_parkState == 206) || (m_parkTrigger == PULL_LEFT && m_parkState == 307) )
    {
        // TODO
        pullOutAlongsideLeft();
    }
    else if( (m_parkTrigger == PULL_RIGHT && m_parkState == 206) || (m_parkTrigger == PULL_RIGHT && m_parkState == 307) )
    {
        // TODO
        pullOutAlongsideRight();
    }

    RETURN_NOERROR;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ ALONGSIDE ROUTINE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ++++++++++++++++++++++++++++++++++++++++ SEARCH ALONGSIDE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::searchRoutineAlongside()
{
    tFloat32 distTravelled = 0;

    switch( m_searchState )
    {
        case 101: // search for possible beginning
            if( m_IRFrontRightCur >= TH_LONG_ALONGSIDE )
            {
                m_distEntry = m_odometryData.distance_sum;
                m_searchState = 102;
            }
            break;


        case 102: // check the size
            distTravelled = m_odometryData.distance_sum - m_distEntry;

            // too small, return to state 1
            if( m_IRFrontRightCur <= TH_SHORT && distTravelled < ALONGSIDE_SIZE )
            {
                m_searchState = 101;
            }

            // min size reached, proceed with state 3
            if( distTravelled >= ALONGSIDE_SIZE )
            {
                m_searchState = 103;
            }
            break;

        case 103: // min size reached, try to get as much space as possible
            distTravelled = m_odometryData.distance_sum - m_distEntry;
            m_distExit = m_odometryData.distance_sum;

            if( distTravelled >= ALONGSIDE_SIZE + EASY_ALONGSIDE )
            {
                m_searchState = 104; // 0 = search ended
                // TODO: Start blinking
                LOG_INFO(cString("PP: LOT DETECTED" ));
                m_parkState = 201;
            }

            if( m_IRFrontRightCur <= TH_SHORT )
            {
                m_searchState = 104; // 0 = search ended
                // TODO: Start blinking
                LOG_INFO(cString("PP: LOT DETECTED" ));
                m_parkState = 301;
            }
            break;

        default:
            break;
    }

    RETURN_NOERROR;

}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ALONGSIDE EASY +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::parkRoutineAlongsideEasy()
{

    switch(m_parkState)
    {
        case 201: // drive forward until position for starting the park maneuver is reached and break
            m_centralAngle = atan( 1 + ( (CALC_CARHALF + m_lastIRshort) / (CALC_QUOTIENT) ) );
            m_distStart = POS_IR_FRONT_SIDE + ( 2 * CALC_RADIUS * sin( m_centralAngle ) );

            if( m_odometryData.distance_sum - m_distEntry >= m_distStart )
            {
                sendSteeringAngle( STEER_NEUTRAL );
                // UEBERLEGEN, OB GLEICH RUECKWAERTSGANG REINBALLERN
                sendSpeed( 0 );
                m_headingAtStart = m_odometryData.angle_heading;
                m_parkState = 202;
            }
            break;

        case 202: // wait until we have stopped and drive backwards...
            if( m_carStopped == true )
            {
                //Denkhilfe
                //distStop = m_odometryData.distance_sum;
                //overshoot = distStop - m_distStart;
                m_parkState = 203;
                sendSpeed( -1 );
            }
            break;

        case 203: // ...drive backwards until overshoot is killed and steer max right...
            //Denkhilfe
            // fahre rueckwaerts bis distStop - overshoot = m_distStart
            if( m_odometryData.distance_sum <= m_distStart )
            {
                m_parkState = 204;
                sendSteeringAngle(STEER_RIGHT_MAX);
            }
            break;

        case 204: // ... steer max right until central angle is reached and steer max left...
            if( m_odometryData.angle_heading >= (m_headingAtStart + m_centralAngle) )
            {
                m_parkState = 205;
                sendSteeringAngle(STEER_LEFT_MAX);
            }
            break;

        case 205: // ... steer max left until m_headingAtStart is reached again
            if( m_odometryData.angle_heading <= m_headingAtStart )
            {
                m_parkState = 206;
                sendSpeed( 0 );
                sendParkState ( 1 );
                m_rememberDist = m_odometryData.distance_sum;
                //Stop blinking
            }

        default:
            break;

    }
    RETURN_NOERROR;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ALONGSIDE NORMAL +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::parkRoutineAlongsideNormal()
{

    switch(m_parkState)
    {
        case 301: // drive forward until position for starting the park maneuver is reached and break
            m_centralAngle = atan( 1 + ( (CALC_CARHALF + m_lastIRshort) / (CALC_QUOTIENT) ) );
            m_distStart = POS_IR_FRONT_SIDE + ( 2 * CALC_RADIUS * sin( m_centralAngle ) );

            if( m_odometryData.distance_sum - m_distEntry >= m_distStart )
            {
                sendSteeringAngle( STEER_NEUTRAL );
                sendSpeed( 0 );
                m_headingAtStart = m_odometryData.angle_heading;
                m_parkState = 302;
            }
            break;

        case 302: // wait until we have stopped and drive backwards...
            if( m_carStopped == true )
            {
                //Denkhilfe
                //distStop = m_odometryData.distance_sum;
                //overshoot = distStop - m_distStart;
                m_parkState = 3;
                sendSpeed( -1 );
            }
            break;

        case 303: // ...drive backwards until overshoot is killed and steer max right...
            //Denkhilfe
            // fahre rueckwaerts bis distStop - overshoot = m_distStart
            if( m_odometryData.distance_sum <= m_distStart )
            {
                m_parkState = 304;
                sendSteeringAngle(STEER_RIGHT_MAX);
            }
            break;

        ///// AB HIER NEUER CODE FUER 'NORMAL' /////
        case 304: // ... steer max right until central angle + adjustment is reached and steer max left...
            if( m_odometryData.angle_heading >= (m_headingAtStart + m_centralAngle + (m_AngleAdjustment * DEG_TO_RAD)  ) )
            {

                tFloat32 distLeft = m_distStart - (CALC_RADIUS * sin( m_centralAngle + (m_AngleAdjustment * DEG_TO_RAD) )) - m_securityBuffer;
                m_counterAngle = asin(distLeft);
                m_parkState = 305;
                sendSteeringAngle(STEER_LEFT_MAX);
            }
            break;

        case 305: // ... drive counterAngle backwards
            if( m_odometryData.angle_heading <= m_headingAtStart + m_counterAngle )
            {
                m_parkState = 306;
                sendSpeed( 1 );
            }
            break;

        case 306: // pull straight
            if( m_odometryData.angle_heading <= m_headingAtStart )
            {
                m_parkState = 307;
                sendSpeed( 0 );
                sendParkState ( 1 );
                m_rememberDist = m_odometryData.distance_sum;
            }
            break;

        default:
            break;

    }
    RETURN_NOERROR;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ PULLOUT ALONGSIDE RIGHT +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::pullOutAlongsideRight()
{
    RETURN_NOERROR;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ PULLOUT ALONGSIDE LEFT +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::pullOutAlongsideLeft()
{
    RETURN_NOERROR;
}



// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++ CROSS ROUTINE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ++++++++++++++++++++++++++++++++++++++++ SEARCH CROSS  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::searchRoutineCross()
{
    tFloat32 distTravelled = 0;

    switch( m_searchState )
    {
        case 501: // search for possible beginning
            if( m_IRFrontRightCur >= TH_LONG_CROSS )
            {
                m_distEntry = m_odometryData.distance_sum;
                m_searchState = 502;
            }
            break;


        case 502: // check the size
            distTravelled = m_odometryData.distance_sum - m_distEntry;

            // too small, return to state 1
            if( m_IRFrontRightCur <= TH_SHORT && distTravelled < CROSS_SIZE )
            {
                m_searchState = 1;
            }

            // min size reached
            if( distTravelled >= CROSS_SIZE )
            {
                m_searchState = 503;
                m_distExit = m_odometryData.distance_sum;
                m_parkState = 601;

            }
            break;

        default:
            break;
    }

    RETURN_NOERROR;
}


// ++++++++++++++++++++++++++++++++++++++++ PARK CROSS  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::parkRoutineCross()
{
    switch( m_parkState )
    {
        case 601: // go left for easier return
            if( m_odometryData.distance_sum >= ( m_distEntry + (CROSS_SIZE/2) + m_straightForward ) )
            {
                m_headingAtStart = m_odometryData.angle_heading;
                sendSteeringAngle(STEER_LEFT_MAX);
                m_parkState = 602;
            }
            break;

        case 602:
            if( m_odometryData.angle_heading >= m_headingAtStart + m_headingAngleForward )
            {
                sendSpeed( 0 );
                sendSteeringAngle( STEERING_TO_SERVO * m_steeringAnlgeBackward );
                sendSpeed( -1 );
                m_parkState = 603;
            }
            break;

        case 603: //drive backwards until perpendicular to starting position = parallel to parking lot
            if( m_odometryData.angle_heading >= m_headingAtStart + M_PI_2 )
            {
                sendSteeringAngle( STEER_NEUTRAL );
                m_rememberDist = m_odometryData.distance_sum;
                m_parkState = 604;
            }
            break;

        case 604: // drive backwards until final parking position reached
            if( m_odometryData.distance_sum <= (m_rememberDist - m_perpendicularBackward)  )
            {
                sendSpeed( 0 );
                sendParkState ( 1 );
                m_parkState = 605;
            }
            break;

        default:
            break;

    }
    RETURN_NOERROR;
}


// ++++++++++++++++++++++++++++++++++++++++ CROSS PULL OUT LEFT  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::pullOutCrossLeft()
{
    switch( m_pulloutState )
    {
        case 801:
            sendSpeed( 1 );
            if( m_IRFrontRightCur > 500 && m_IRFrontLeftCur > 500 )
            {
                m_rememberDist = m_odometryData.distance_sum;
                m_headingAtStart = m_odometryData.angle_heading;
            }
            m_pulloutState = 802;
            break;

        case 802:
            if( m_odometryData.distance_sum >= (m_rememberDist + m_pullLeftStraight) )
            {
                sendSteeringAngle( STEER_LEFT_MAX );
                m_pulloutState = 803;
            }
            break;

        case 803:
            if( m_odometryData.angle_heading >= (m_headingAtStart + M_PI_2) )
            {
                sendSteeringAngle( STEER_NEUTRAL );
                // send FINISHED PIN TO KI

                // set back all the stuff
                m_parkTrigger = 0;
                m_searchState = 0;
                m_parkState = 0;
                m_pulloutState = 0;
                m_distEntry = 0;
                m_headingAtStart = 0;
                m_distStartPark = 0.0;
                m_centralAngle = 0.0;
                m_distStart = 0.0;
                m_rememberDist = 0.0;
                m_searchActive = false;
                m_minDistReached = false;
                m_carStopped = false;

            }
            break;

        default:
            break;

    }
    RETURN_NOERROR;
}



// ++++++++++++++++++++++++++++++++++++++++ CROSS PULL OUT RIGHT  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::pullOutCrossRight()
{
    switch( m_pulloutState )
    {
        case 701:
            sendSpeed( 1 );
            if( m_IRFrontRightCur > 500 && m_IRFrontLeftCur > 500 )
            {
                m_rememberDist = m_odometryData.distance_sum;
                m_headingAtStart = m_odometryData.angle_heading;
            }
            m_pulloutState = 702;
            break;

        case 702:
            if( m_odometryData.distance_sum >= (m_rememberDist + m_pullRightStraight) )
            {
                sendSteeringAngle( STEER_RIGHT_MAX );
                m_pulloutState = 703;
            }
            break;

        case 703:
            if( m_odometryData.angle_heading >= (m_headingAtStart - M_PI_2) )
            {
                sendSteeringAngle( STEER_NEUTRAL );
                // send FINISHED PIN TO KI

                // set back all the stuff
                m_parkTrigger = 0;
                m_searchState = 0;
                m_parkState = 0;
                m_pulloutState = 0;
                m_distEntry = 0;
                m_headingAtStart = 0;
                m_distStartPark = 0.0;
                m_centralAngle = 0.0;
                m_distStart = 0.0;
                m_rememberDist = 0.0;
                m_searchActive = false;
                m_minDistReached = false;
                m_carStopped = false;

            }
            break;

        default:
            break;

    }
    RETURN_NOERROR;
}


tResult cSWE_ParkPilot::sendSpeed(tFloat32 speed)
{

    //create new media sample
    cObjectPtr<IMediaCoder> pCoder;
    cObjectPtr<IMediaSample> pMediaSampleOutput;

    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSpeedOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescSpeedOut->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("f32Value", (tVoid*)&(speed));
    m_pCoderDescSpeedOut->Unlock(pCoder);

    //transmit media sample over output pin
    RETURN_IF_FAILED(m_outputSpeed.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;

}


tResult cSWE_ParkPilot::sendSteeringAngle(tFloat32 steeringAngle)
{
    //create new media sample
    cObjectPtr<IMediaCoder> pCoder;
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSteeringOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    RETURN_IF_FAILED(m_pCoderDescSteeringOut->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("f32Value", (tVoid*)&(steeringAngle));
    m_pCoderDescSteeringOut->Unlock(pCoder);

    RETURN_IF_FAILED(m_outputSteering.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;

}


tResult cSWE_ParkPilot::sendParkState(tInt8 parkState)
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescParkStateOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescParkStateOut->WriteLock(pMediaSample, &pCoder));
    pCoder->Set("tInt8SignalValue", (tVoid*)&(parkState));
    m_pCoderDescParkStateOut->Unlock(pCoder);

    //transmit media sample over output pin
    //RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_outputParkState.Transmit(pMediaSample));

    RETURN_NOERROR;
}



