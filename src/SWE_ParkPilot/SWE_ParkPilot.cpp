#include "SWE_ParkPilot.h"
#include <iostream>
#include <fstream>

// +++ begin_defines +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*! Sensor positions */
#define POS_IR_SIDE_RIGHT -150.0f   // Abstand der IR Sensoren zur x-Achse
#define POS_IR_SIDE_LEFT 150.0f     // Abstand der IR Sensoren zur x-Achse
#define POS_IR_FRONT_SIDE 450.0f   // Abstand der vorderen IR Sensoren zur Hinterachse (x-Richtung)

/*! Thresholds */
#define TH_SHORT 150.0f
#define TH_LONG 450.0f
#define TH_LONG_ALONGSIDE 350.0f  //besser eigtl 590 !!
#define TH_LONG_CROSS 450.0f      //besser eigtl 590 !!

/*! Park styles */
#define PARK_ALONGSIDE 1
#define PARK_CROSS 2
#define PULL_LEFT_FROM_A 3
#define PULL_RIGHT_FROM_A 4
#define PULL_LEFT_FROM_C 5
#define PULL_RIGHT_FROM_C 6
#define GOT_CONTROL 7

/*! Blink left/right */
#define BLINK_STOP 0
#define BLINK_LEFT 2
#define BLINK_RIGHT 4

/*! Park States */
#define PARKING_FINISHED 1
#define PULLOUT_FINISHED 2
#define LOT_FOUND 3

/*! Lot sizes */
#define ALONGSIDE_SIZE 765.0f   // DEBUG, true value = 765.0f         // minimum size of parking lot
#define CROSS_SIZE 450.0f               // minimum size of parking lot
#define EASY_ALONGSIDE 40.0f            // buffer for easy S-curve maneuver

/*! Helpers to calculate central angle
#define CALC_QUOTIENT -799.0f
#define CALC_CARHALF 150.0f
#define CALC_RADIUS 400.0f
*/

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

#define TIMER_RESOLUTION        1000000.0f

// +++ end_defines ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//
// for TESTING set m_parkTrigger to specific value and set the bools under TESTING
// in constuctor and stark method

ADTF_FILTER_PLUGIN("SWE ParkPilot", OID_ADTF_SWE_PARKPILOT, cSWE_ParkPilot)

cSWE_ParkPilot::cSWE_ParkPilot(const tChar* __info) : cFilter(__info)
{
        // initialise member variables
        m_searchState = 0;
        m_pulloutState = 0;

        //TESTING START
        m_parkState = 0;
        m_parkTrigger = 1;  // always initialise with 10 !

        m_searchActivated = false;
        m_minDistReached = false;
        m_carStopped = false;
        m_parkAlongside = false;
        m_parkCross = false;
        m_pullLeft = false;
        m_pullRight = false;
        m_gotControl = true;
        m_outOfLot = false;
        //TESTING END



        m_firstIR = false;

        m_IRFrontRightCur = INVALID_HIGH;
        m_IRRearRightCur = INVALID_HIGH;
        m_IRFrontLeftCur = INVALID_HIGH;
        m_distEntry = 0.0;
        m_distExit = 0.0;	// maybe kill
        m_lastIRshort = 0.0;

        m_angleAbs = 0.0;
        m_rememberDist = 0.0;

        m_odometryData.distance_x = 0.0;
        m_odometryData.distance_y = 0.0;
        m_odometryData.angle_heading = 0.0;
        m_odometryData.distance_sum = 0.0;
        m_odometryData.velocity = 0.0;

        // set filter properties

        SetPropertyBool("Logging", false);
        SetPropertyBool("Testing", false);
        SetPropertyInt("Stop Time in ms", 15000);

        SetPropertyFloat("A_LotSize", 765.0);
        SetPropertyFloat("A_StraightForward(mm)", 650.0);
        SetPropertyFloat("A_CentralAngle(DEG)",50.0);
        SetPropertyFloat("A_CounterAngle(DEG)",43.0);
        SetPropertyFloat("A_CentralAngleSteering(0-1max)", 1.0);
        SetPropertyBool("A_ActivateManeuvering", true);
        SetPropertyFloat("A_ManeuverAngleOne(DEG)", 35.0);
        SetPropertyFloat("A_ManeuverAngleTwo(DEG)", 17.0);
        SetPropertyFloat("A_DriftCompensation", 0.0);
        SetPropertyFloat("AP_firstAnlge", 2.0);
        SetPropertyFloat("AP_secondAnlge", 15.0);
        SetPropertyFloat("AP_thirdAnlge", 22.0);
        SetPropertyFloat("AP_fourthAnlge", 37.0);
        SetPropertyFloat("AP_DriftComp(DEG)", 0.0);


        SetPropertyFloat("C_LotSize", 400.0);
        SetPropertyFloat("C_StraightForward(mm)", 60.0);
        SetPropertyFloat("C_HeadingAngleLeftForward(DEG)", 35.0);
        SetPropertyFloat("C_PerpendicularBackward(mm)", 250.0);

        SetPropertyFloat("CP_PullLeftStraight(mm)", 300.0);
        SetPropertyFloat("CP_PullLeftStraightBlind(mm)", 300.0);
        SetPropertyFloat("CP_PullRightStraightBlind(mm)", 250.0);
        SetPropertyFloat("CP_PullRightStraight(mm)", 230.0);
        SetPropertyFloat("CP_PullLeftAngle(DEG)", 50.0);
        SetPropertyFloat("CP_PullRightAngle(DEG)", 90.0);




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


    // IR INPUT RIGHT
    tChar const * strDescIRR = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescIRR);
    cObjectPtr<IMediaType> pTypeIRR = new cMediaType(0, 0, 0, "tSignalValue", strDescIRR,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeIRR->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderIRR));

    RETURN_IF_FAILED(m_pin_input_ir_right.Create("ir_front_right", pTypeIRR, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_right));

    // IR INPUT LEFT
    tChar const * strDescIRL = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescIRL);
    cObjectPtr<IMediaType> pTypeIRL = new cMediaType(0, 0, 0, "tSignalValue", strDescIRL,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeIRL->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderIRL));

    RETURN_IF_FAILED(m_pin_input_ir_left.Create("ir_front_left", pTypeIRL, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_left));


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

    // LIGHT OUTPUT
    tChar const * strDescLightOutput = pDescManager->GetMediaDescription("tInt8SignalValue");
    RETURN_IF_POINTER_NULL(strDescLightOutput);
    cObjectPtr<IMediaType> pTypeLightOutput = new cMediaType(0, 0, 0, "tInt8SignalValue", strDescLightOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLightOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLightOutput));

    RETURN_IF_FAILED(m_outputBlink.Create("Blink_Signal", pTypeLightOutput, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputBlink));

    // PARKSTATE OUTPUT
    tChar const * strDescTCOutput2 = pDescManager->GetMediaDescription("tInt8SignalValue");
    RETURN_IF_POINTER_NULL(strDescTCOutput2);
    cObjectPtr<IMediaType> pTypeParkData = new cMediaType(0, 0, 0, "tInt8SignalValue", strDescTCOutput2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeParkData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescParkOutput));

    RETURN_IF_FAILED(m_oOutputParking.Create("Parkinglooti", pTypeParkData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputParking));



 RETURN_NOERROR;


}

tResult cSWE_ParkPilot::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {

        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);

        // test and debugging
        m_logging = (tBool)GetPropertyBool("Logging", false);
        m_testing = (tBool)GetPropertyBool("Testing", false);
        m_stopTime =  (tUInt32)GetPropertyInt("Stop Time in ms", 15000) * (TIMER_RESOLUTION / 1000.0);

        // lot sizes
        m_AlongsideSize = (tFloat32)GetPropertyFloat("A_LotSize", 765.0);
        m_CrossSize = (tFloat32)GetPropertyFloat("C_LotSize", 400.0);

        // alongside park and pull
        m_centralAngleSteering = (tFloat32)GetPropertyFloat("A_CentralAngleSteering(0-1max)", 1.0);
        m_centralAngle = DEG_TO_RAD * (tFloat32)GetPropertyFloat("A_CentralAngle(DEG)", 48.0);
        m_counterAngle = DEG_TO_RAD * (tFloat32)GetPropertyFloat("A_CounterAngle(DEG)", 43.0);
        m_manAngleOne = DEG_TO_RAD * (tFloat32)GetPropertyFloat("A_ManeuverAngleOne(DEG)");
        m_manAngleTwo = DEG_TO_RAD * (tFloat32)GetPropertyFloat("A_ManeuverAngleTwo(DEG)");
        m_activeManeuvering = (tBool)GetPropertyBool("A_ActivateManeuvering", false);
        m_distStartPark = GetPropertyFloat("A_StraightForward(mm)", 650.0);
        m_driftComp = DEG_TO_RAD * (tFloat32)GetPropertyFloat("A_DriftCompensation", 0.0);
        m_pullFirst = DEG_TO_RAD * GetPropertyFloat("AP_firstAnlge", 2.0);
        m_pullSecond = DEG_TO_RAD * GetPropertyFloat("AP_secondAnlge", 15.0);
        m_pullThird = DEG_TO_RAD * GetPropertyFloat("AP_thirdAnlge", 22.0);
        m_pullFourth = DEG_TO_RAD * GetPropertyFloat("AP_fourthAnlge", 39.0);
        m_pullDriftComp = DEG_TO_RAD * GetPropertyFloat("AP_DriftComp(DEG)", 0.0);

        // cross park and pull
        m_straightForward = GetPropertyFloat("C_StraightForward(mm)", 60.0);
        m_headingAngleForward = DEG_TO_RAD * (tFloat32)GetPropertyFloat("C_HeadingAngleLeftForward(DEG)");
        m_perpendicularBackward = GetPropertyFloat("C_PerpendicularBackward(mm)");
        m_pullLeftStraightBlind = (tFloat32)GetPropertyFloat("CP_PullLeftStraightBlind(mm)", 300.0);
        m_pullRightStraightBlind = (tFloat32)GetPropertyFloat("CP_PullRightStraightBlind(mm)", 250.0);
        m_pullLeftStraight = GetPropertyFloat("CP_PullLeftStraight(mm)", 300.0);
        m_pullRightStraight = GetPropertyFloat("CP_PullRightStraight(mm)", 230.0);
        m_pullLeftAngle = DEG_TO_RAD * (tFloat32)GetPropertyFloat("CP_PullLeftAngle(DEG)", 50.0);
        m_pullRightAngle = DEG_TO_RAD * (tFloat32)GetPropertyFloat("CP_PullRightAngle(DEG)", 90.0);


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
{   // initialise member variables
    m_searchState = 0;
    m_pulloutState = 0;

    //TESTING START
    m_parkState = 0;
    m_parkTrigger = 1;  //always initialise with 10

    m_searchActivated = false;
    m_minDistReached = false;
    m_carStopped = false;
    m_parkAlongside = false;
    m_parkCross = false;
    m_pullLeft = false;
    m_pullRight = false;
    m_gotControl = true;
    m_outOfLot = false;
    //TESTING END
    m_firstIR = false;

    m_IRFrontRightCur = INVALID_HIGH;
    m_IRRearRightCur = INVALID_HIGH;
    m_IRFrontLeftCur = INVALID_HIGH;
    m_distEntry = 0.0;
    m_distExit = 0.0;	// maybe kill
    m_lastIRshort = 0.0;

    m_angleAbs = 0.0;
    m_rememberDist = 0.0;

    m_startTimer = GetTime();

    m_odometryData.distance_x = 0.0;
    m_odometryData.distance_y = 0.0;
    m_odometryData.angle_heading = 0.0;
    m_odometryData.distance_sum = 0.0;
    m_odometryData.velocity = 0.0;


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

tTimeStamp cSWE_ParkPilot::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tResult cSWE_ParkPilot::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    // TIMER LOOP FOR TESTING ONLY! DEBUG
    if((GetTime() - m_startTimer) < m_stopTime)
    {
        RETURN_NOERROR;
    }

    m_mutex.Enter(); //serialize the whole filter for data consistency

    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);
    RETURN_IF_POINTER_NULL(pSource);


    if( nEventCode == IPinEventSink::PE_MediaSampleReceived )
    {

        RETURN_IF_POINTER_NULL( pMediaSample);


        if(pSource == &m_inputParkTrigger)
        {
           // save the park trigger: alongside or cross
           cObjectPtr<IMediaCoder> pCoder;
           RETURN_IF_FAILED(m_pCoderParkTrigger->Lock(pMediaSample, &pCoder));
           pCoder->Get("int8Value", (tVoid*)&m_parkTrigger);
           m_pCoderParkTrigger->Unlock(pCoder);

           if( m_parkTrigger == GOT_CONTROL )
           {
               m_gotControl = true;
           }
           else if( m_parkTrigger == 0 )
           {
               sendSpeed( 0 );
           }

        }
        else if(pSource == &m_pin_input_ir_right)
        {
            tTimeStamp timeStamp;
            cObjectPtr<IMediaCoder> pCoder;

            RETURN_IF_FAILED(m_pCoderIRR->Lock(pMediaSample, &pCoder));
            pCoder->Get("f32Value", (tVoid*)&m_IRFrontRightCur);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderIRR->Unlock(pCoder);

            m_IRFrontRightCur = 10 * m_IRFrontRightCur;

            ///FOR TESTING
            if(m_testing == true && (GetTime() - m_startTimer) >= m_stopTime)
            {
                sendSteeringAngle( STEER_NEUTRAL );
                sendSpeed( 1.0 );
                LOG_ERROR(cString("PP: Trigger set, going forward" ));
                m_parkTrigger = 1;
                m_testing = false;
            }
            ///FOR TESTING

            if( m_IRFrontRightCur <= TH_SHORT && m_firstIR == false)
            {
                m_checkPointOne = m_odometryData.distance_sum;
                m_firstIR = true;
                //LOG_ERROR(cString("PP: SHORT!, IRcur: " + cString::FromFloat64(m_IRFrontRightCur)  ));
            }

            if( m_IRFrontRightCur > TH_LONG && m_firstIR == true && m_searchState == 0 && m_parkTrigger <= 2)
            {
                if( m_odometryData.distance_sum - m_checkPointOne >= 180 )
                {
                    LOG_ERROR(cString("PP: 1 Search Active, ObjectLength: " + cString::FromFloat64(m_odometryData.distance_sum - m_checkPointOne) ));
                    m_searchState = 1;
                }
                else
                {
                    LOG_ERROR(cString("PP: 2 Search NOT Active, ObjectLength: " + cString::FromFloat64(m_odometryData.distance_sum - m_checkPointOne) ));
                    m_firstIR = false;
                    m_searchState = 0;
                }
            }
            else if(m_firstIR == true && m_searchState == 0 && (m_odometryData.distance_sum - m_checkPointOne >= 180) && m_parkTrigger <= 2 )
            {
                m_searchState = 1;
                LOG_ERROR(cString("PP: 3 Search Active, ObjectLength: " + cString::FromFloat64(m_odometryData.distance_sum - m_checkPointOne) ));
            }

           jumpIntoStates(); //(Robert)

        }
        else if(pSource == &m_pin_input_ir_left)
        {
            tTimeStamp timeStamp;
            cObjectPtr<IMediaCoder> pCoder;

            RETURN_IF_FAILED(m_pCoderIRL->Lock(pMediaSample, &pCoder));
            pCoder->Get("f32Value", (tVoid*)&m_IRFrontLeftCur);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderIRL->Unlock(pCoder);

            m_IRFrontLeftCur = 10 * m_IRFrontLeftCur;
        }
        else if(pSource == &m_inputOdometry)
        {

            tTimeStamp timestamp;
            cObjectPtr<IMediaCoder> pCoder;

            RETURN_IF_FAILED(m_pCoderDescOdometry->Lock(pMediaSample, &pCoder));
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&(timestamp));
            pCoder->Get("distance_x", (tVoid*)&(m_odometryData.distance_x));
            pCoder->Get("distance_y", (tVoid*)&(m_odometryData.distance_y));
            pCoder->Get("angle_heading", (tVoid*)&(m_odometryData.angle_heading));
            pCoder->Get("velocity", (tVoid*)&(m_odometryData.velocity));
            pCoder->Get("distance_sum", (tVoid*)&(m_odometryData.distance_sum));
            m_pCoderDescOdometry->Unlock(pCoder);

            m_angleAbs = m_angleAbs + m_odometryData.angle_heading;
            // ignore jumping angles
//            if( abs(m_odometryData.angle_heading) <= (3 * DEG_TO_RAD) )
//            {
//                m_angleAbs = m_angleAbs + m_odometryData.angle_heading;
//            }

        }

    }


    m_mutex.Leave();

    RETURN_NOERROR;
}


tResult cSWE_ParkPilot::jumpIntoStates()
{
    /* INDICES:
 * 101... search Alongside
 * 201... park Alongside
 * 301... pull out Alongside right
 * 401... pull out Alongside left
 *
 * 501... search Cross
 * 601... park Cross
 * 701... pull out Cross right
 * 801... pull out Cross left
 */

    // Suche Aktivieren
    if(m_searchState == 1 && m_parkTrigger == PARK_ALONGSIDE)
    {
        m_searchState = 101;
    }
    else if(m_searchState == 1 && m_parkTrigger == PARK_CROSS)
    {
        m_searchState = 501;
    }

    // DEBUG
    //LOG_ERROR(cString("PP: On Pin: Search State" + cString::FromInt(m_searchState) + "; ParkState" + cString::FromInt(m_parkState) + "; PullState" + cString::FromInt(m_pulloutState) + "; Trigger: " + cString::FromInt(m_parkTrigger)));
    // DEBUG

    // Was machen wir?
    if( m_searchState > 100 && m_searchState < 200 && m_parkState == 0)
    {
        searchRoutineAlongside();
    }
    else if( m_searchState > 500 && m_searchState < 600 && m_parkState == 0)
    {
        searchRoutineCross();
    }
    else if( m_searchState == 103 && m_parkState > 200 && m_parkState < 300 )
    {
        parkRoutineAlongside();
    }
    else if( m_searchState == 503 && m_parkState > 600 && m_parkState < 700 )
    {
        parkRoutineCross();
    }
    else if( m_parkTrigger == PULL_LEFT_FROM_C )
    {
        if( m_pulloutState == 0 )
        {
            m_pulloutState = 801;
        }

        //TESTING
//        if((GetTime() - m_startTimer) >= m_stopTime )
//        {
//            pullOutCrossLeft();
//        }

        pullOutCrossLeft();

    }
    else if( m_parkTrigger == PULL_RIGHT_FROM_C )
    {
        if( m_pulloutState == 0 )
        {
            m_pulloutState = 701;
        }
        pullOutCrossRight();
    }
    else if( m_parkTrigger == PULL_LEFT_FROM_A)
    {
        if( m_pulloutState == 0 )
        {
            m_pulloutState = 401;
        }
        pullOutAlongsideLeft();
    }
    else if( m_parkTrigger == PULL_RIGHT_FROM_A )
    {
        if( m_pulloutState == 0 )
        {
            m_pulloutState = 301;
        }
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
            if( m_IRFrontRightCur >= TH_SHORT )    //TH_LONG_ALONGSIDE
            {
                m_distEntry = m_odometryData.distance_sum;
                m_searchState = 102;
            }
            //LOG_ERROR(cString("PP: 101 Next State" + cString::FromInt(m_searchState) + "IR: " + cString::FromFloat64(m_IRFrontRightCur)));
            break;


        case 102: // check the size
            distTravelled = m_odometryData.distance_sum - m_distEntry;

            // too small, return to state 1
            if( m_IRFrontRightCur <= TH_SHORT && distTravelled < m_AlongsideSize )
            {
                m_searchState = 101;
            }

            // min size reached, proceed with state 3
            if( distTravelled >= m_AlongsideSize )
            {
                m_searchState = 103;
                m_parkState = 201;
                m_distExit = m_odometryData.distance_sum;
                m_angleAbs = 0.0;
                LOG_ERROR(cString("PP: LOT DETECTED with size: " + cString::FromFloat64(distTravelled) ));
                sendParkState( LOT_FOUND );
                sendBlink( BLINK_RIGHT );
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP:102 Next " + cString::FromInt(m_searchState) + "; Measured: " + cString::FromFloat64(distTravelled) ));
            break;

        default:
            break;
    }

    RETURN_NOERROR;

}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ALONGSIDE NORMAL +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::parkRoutineAlongside()
{
    if(m_gotControl == false)
    {
        RETURN_NOERROR;
    }

    switch(m_parkState)
    {
        case 201: // drive forward until position for starting the park maneuver is reached and break

            if( m_odometryData.distance_sum >= (m_distStartPark + m_distExit) )
            {
                m_headingAtStart = m_angleAbs;
                sendSpeed( -1.0 );
                sendSteeringAngle(STEER_RIGHT_MAX * m_centralAngleSteering);
                m_parkState = 202;
            }
            //LOG_ERROR(cString("PP: 201 Next " + cString::FromInt(m_parkState) + "; still to go: " + cString::FromFloat64(m_distStartPark + m_distExit - m_odometryData.distance_sum) + "; Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 202: // ... steer max right until central angle + adjustment is reached and steer max left...
            if( m_angleAbs >= (m_headingAtStart + m_centralAngle  ) )
            {
                sendSteeringAngle(STEER_LEFT_MAX);
                m_parkState = 203;

            }
            //LOG_ERROR(cString("PP: 202 Next " + cString::FromInt(m_parkState) + "; CenA: " + cString::FromFloat64((m_headingAtStart + m_centralAngle)/DEG_TO_RAD) + "; Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 203: // ... drive counterAngle backwards
            if( m_angleAbs <= (m_headingAtStart + m_counterAngle) )
            {
                m_parkState = 204;
                sendSteeringAngle(STEER_RIGHT_MAX);
                sendSpeed( 1.0 );
                if( m_activeManeuvering == true )
                {
                    m_parkState = 204;
                }
                else
                {
                    m_parkState = 206;
                }
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 203 Next " + cString::FromInt(m_parkState) + "; CouA: " + cString::FromFloat64((m_headingAtStart + m_counterAngle)/DEG_TO_RAD) + "; Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;


        case 204:
            if( m_angleAbs <= (m_headingAtStart + m_manAngleOne) )
            {
                m_parkState = 205;
                sendSteeringAngle( STEER_LEFT_MAX );
                sendSpeed( -1.0 );
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 204 Next " + cString::FromInt(m_parkState) + "; ManOne: " + cString::FromFloat64((m_headingAtStart + m_manAngleOne)/DEG_TO_RAD) + "; Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 205:
            if( m_angleAbs <= (m_headingAtStart + m_manAngleTwo) )
            {
                m_parkState = 206;
                sendSteeringAngle(STEER_RIGHT_MAX);
                sendSpeed( 1.0 );
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 205 Next " + cString::FromInt(m_parkState) + "; ManTwo: " + cString::FromFloat64((m_headingAtStart + m_manAngleTwo)/DEG_TO_RAD) + "; Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 206:
            if( m_angleAbs <= (m_headingAtStart + m_driftComp) )
            {
                m_parkState = 208;
                sendSpeed( 0 );
                sendSteeringAngle( STEER_NEUTRAL );
                sendParkState ( 1 );
                sendBlink( BLINK_STOP );
                m_activeManeuvering = false;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 206 Next " + cString::FromInt(m_parkState) + "; HeadStart: " + cString::FromFloat64(m_headingAtStart/DEG_TO_RAD) + "; Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 207: // pull straight
            if( m_angleAbs <= m_headingAtStart )
            {
                m_parkState = 208;
                sendSpeed( 0 );
                sendSteeringAngle( STEER_NEUTRAL );
                sendParkState ( PARKING_FINISHED );
                sendBlink( BLINK_STOP );
                m_rememberDist = m_odometryData.distance_sum;
            }
            LOG_ERROR(cString("PP: 206 Next State" + cString::FromInt(m_parkState) ));
            break;

        default:
            break;

    }
    RETURN_NOERROR;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ PULLOUT ALONGSIDE RIGHT +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::pullOutAlongsideRight()
{
    switch( m_pulloutState )
    {
        case 301:
            sendBlink( BLINK_LEFT );
            m_headingAtStart = m_angleAbs;
            sendSteeringAngle(STEER_RIGHT_MAX);
            sendSpeed( -1 );
            m_pulloutState = 302;
            break;

        case 302:
            if( m_angleAbs >= m_headingAtStart + m_pullFirst )
            {
                sendSpeed( 1 );
                sendSteeringAngle( STEER_LEFT_MAX );
                m_pulloutState = 303;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 302: Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 303:
            if( m_angleAbs >= m_headingAtStart + m_pullSecond )
            {
                sendSpeed( -1 );
                sendSteeringAngle( STEER_RIGHT_MAX );
                m_pulloutState = 304;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 303: Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 304:
            if( m_angleAbs >= m_headingAtStart + m_pullThird )
            {
                sendSpeed( 1 );
                sendSteeringAngle( STEER_LEFT_MAX );
                m_pulloutState = 305;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 304: Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 305:
            if( m_angleAbs >= m_headingAtStart + m_pullFourth )
            {
                sendSteeringAngle( STEER_RIGHT_MAX );
                m_pulloutState = 306;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 305: Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 306:
            if( m_angleAbs <= m_headingAtStart + m_pullDriftComp)
            {
                sendSteeringAngle( STEER_NEUTRAL );
                m_pulloutState = 0;
                m_parkTrigger = 99;   // fuer testcase sonst wieder 0 setzten
                sendParkState( PULLOUT_FINISHED );
                sendBlink( BLINK_STOP );
                //reset all the bullshit
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 306: Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD) + "; HeadStart: " + cString::FromFloat64((m_headingAtStart + m_pullDriftComp)/DEG_TO_RAD)));
            break;

        default:
            break;
    }
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
            if( m_IRFrontRightCur >= TH_SHORT )
            {
                m_distEntry = m_odometryData.distance_sum;
                m_searchState = 502;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 501: Next: " + cString::FromInt(m_searchState)));
            break;


        case 502: // check the size
            distTravelled = m_odometryData.distance_sum - m_distEntry;

            // too small, return to state 1
            if( m_IRFrontRightCur <= TH_SHORT && distTravelled < m_CrossSize )
            {
                m_searchState = 501;
            }

            // min size reached
            if( distTravelled >= m_CrossSize )
            {
                if(m_logging == true)
                    LOG_ERROR(cString("PP: 502: LOT FOUND"));
                m_searchState = 503;
                m_distExit = m_odometryData.distance_sum;
                m_parkState = 601;
                sendParkState( LOT_FOUND );
                sendBlink( BLINK_RIGHT );

            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 502: Next: " + cString::FromInt(m_searchState) + " Size: " + cString::FromFloat64(distTravelled)));
            break;

        default:
            break;
    }

    RETURN_NOERROR;
}


// ++++++++++++++++++++++++++++++++++++++++ PARK CROSS  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tResult cSWE_ParkPilot::parkRoutineCross()
{
    if(m_gotControl == false)
    {
        RETURN_NOERROR;
    }

    switch( m_parkState )
    {
        case 601: // go left for easier return
            if( m_odometryData.distance_sum >= ( m_distExit + m_straightForward ) )
            {
                m_headingAtStart = m_angleAbs;
                sendSteeringAngle(STEER_LEFT_MAX);
                m_parkState = 602;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 601: Next: " + cString::FromInt(m_parkState)));
            break;

        case 602:
            if( m_angleAbs >= m_headingAtStart + m_headingAngleForward )
            {
                sendSteeringAngle( STEER_RIGHT_MAX );
                sendSpeed( -1 );
                m_parkState = 603;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 602: Next: " + cString::FromInt(m_parkState) + " Abs: " + cString::FromFloat64(m_angleAbs/DEG_TO_RAD)));
            break;

        case 603: //drive backwards until perpendicular to starting position = parallel to parking lot
            if( m_angleAbs >= m_headingAtStart + M_PI_2 )
            {
                sendSteeringAngle( STEER_NEUTRAL );
                m_rememberDist = m_odometryData.distance_sum;
                m_parkState = 604;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 603: Next: " + cString::FromInt(m_parkState)));
            break;

        case 604: // drive backwards until final parking position reached
            if( m_odometryData.distance_sum <= (m_rememberDist - m_perpendicularBackward)  )
            {
                sendSpeed( 0 );
                sendParkState ( PARKING_FINISHED );
                sendBlink( BLINK_STOP );
                m_parkState = 605;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP: 604: Next: " + cString::FromInt(m_parkState)));
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
            sendBlink( BLINK_LEFT );
            m_rememberDistTwo = m_odometryData.distance_sum;
            sendSpeed( 1 );
            m_headingAtStart = m_angleAbs;
            m_outOfLot = false;
            m_pulloutState = 802;
            break;


        case 802:
            if( m_IRFrontRightCur > TH_SHORT && m_IRFrontLeftCur > TH_SHORT && m_outOfLot == false)
            {
                m_rememberDist = m_odometryData.distance_sum;
                m_outOfLot = true;
            }

            if( (m_odometryData.distance_sum >= m_rememberDistTwo + m_pullLeftStraightBlind) || (m_odometryData.distance_sum > m_rememberDist + m_pullLeftStraight) )
            {
                sendSteeringAngle( STEER_LEFT_MAX );
                m_pulloutState = 803;
            }
            if(m_logging == true)
                LOG_ERROR(cString("PP:802 Next " + cString::FromInt(m_pulloutState) ));
            break;


        case 803:
            if( m_angleAbs >= (m_headingAtStart + m_pullLeftAngle) )
            {
                sendParkState( PULLOUT_FINISHED );
                sendBlink( BLINK_STOP );

                // set back all the stuff
                m_parkTrigger = 10;
                m_searchState = 0;
                m_parkState = 0;
                m_pulloutState = 0;
                m_distEntry = 0;
                m_headingAtStart = 0;
                m_rememberDist = 0.0;
                m_gotControl = false;

            }
            break;
            if(m_logging == true)
                LOG_ERROR(cString("PP:803 Next " + cString::FromInt(m_pulloutState) ));

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
        sendBlink( BLINK_RIGHT );
        m_rememberDistTwo = m_odometryData.distance_sum;
        sendSpeed( 1 );
        m_headingAtStart = m_angleAbs;
        m_outOfLot = false;
        m_pulloutState = 702;
        break;


    case 702:
        if( m_IRFrontRightCur > TH_SHORT && m_IRFrontLeftCur > TH_SHORT && m_outOfLot == false)
        {
            m_rememberDist = m_odometryData.distance_sum;
            m_outOfLot = true;
        }

        if( (m_odometryData.distance_sum >= m_rememberDistTwo + m_pullRightStraightBlind) || (m_odometryData.distance_sum > m_rememberDist + m_pullRightStraight) )
        {
            sendSteeringAngle( STEER_RIGHT_MAX );
            m_pulloutState = 703;
        }
        if(m_logging == true)
            LOG_ERROR(cString("PP:702 Next " + cString::FromInt(m_pulloutState) ));
        break;


    case 703:
        if( m_angleAbs >= (m_headingAtStart - m_pullRightAngle) )
        {
            sendParkState( PULLOUT_FINISHED );
            sendBlink( BLINK_STOP );

             //set back all the stuff
                m_parkTrigger = 10;
                m_searchState = 0;
                m_parkState = 0;
                m_pulloutState = 0;
                m_distEntry = 0;
                m_headingAtStart = 0;
                m_rememberDist = 0.0;
                m_gotControl = false;
                m_outOfLot = false;

        }
        break;
        if(m_logging == true)
            LOG_ERROR(cString("PP:703 Next " + cString::FromInt(m_pulloutState) ));

    default:
        break;

    }
    RETURN_NOERROR;
}


tResult cSWE_ParkPilot::sendSpeed(tFloat32 speed)
{

    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSpeedOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSpeedOut, pMediaSample, pCoder);

        pCoder->Set("f32Value", (tVoid*)&(speed));
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_outputSpeed.Transmit(pMediaSample);

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


tResult cSWE_ParkPilot::sendParkState(tInt8 value)
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


tResult cSWE_ParkPilot::sendBlink(tInt8 blink)
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescLightOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescLightOutput->WriteLock(pMediaSample, &pCoder));
    pCoder->Set("int8Value", (tVoid*)&(blink));
    m_pCoderDescLightOutput->Unlock(pCoder);

    //transmit media sample over output pin
    //RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_outputBlink.Transmit(pMediaSample));

    RETURN_NOERROR;
}



