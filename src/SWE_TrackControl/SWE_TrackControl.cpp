#include <cmath>
#include "stdafx.h"
#include "SWE_TrackControl.h"


#include <iostream>
#include <fstream>

#define WHEELBASE 359 //distance between both axles in mm
#define STATUS_NORMAL 0  //= normal status
#define STATUS_ENDOFTURN 1  //= ended turn maneuver
#define STATUS_ATSTOPLINE 2 //= stopped at stopline
#define DEBUG_OUTPUT false //DEBUG
#define CLOSE_STOPLINE 1200 //when approaching stopline this close, be careful to use camera steering



ADTF_FILTER_PLUGIN("SWE TrackControl", OID_ADTF_SWE_TRACKCONTROL, cSWE_TrackControl)

cSWE_TrackControl::cSWE_TrackControl(const tChar* __info) : cFilter(__info), m_PerpenticularPoint(1.0, 0.0), m_oManeuverObject(_clock)
{
    m_old_steeringAngle = 0.0;
    m_input_intersectionIndicator = 0;

    SetPropertyFloat("Wheelbase in mm", 359);
    SetPropertyBool("Use new angle calculation", true);
    SetPropertyBool("Stop at virtual stoplines", true); //DEBUG
    SetPropertyBool("InvertSteering", true); //Invert all steering angles? Normal is positive left. This somehow differs between our cars.... don't ask...

    SetPropertyBool("Use Testmode", false);
    SetPropertyInt("Testmode start command", 1);
    SetPropertyInt("Testmode start speed", 2);
    SetPropertyInt("Stopdist Stopline to Wheel in mm", 120); //how many mm to stop in fron of the stopline (with the front wheels)
    SetPropertyFloat("Steering Dead angle in degree", 3);

}

cSWE_TrackControl::~cSWE_TrackControl()
{
}

tResult cSWE_TrackControl::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oIntersectionPoints.Create("tracking_Point", new cMediaType(0, 0, 0, "tIntersectionsNew"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));

    RETURN_IF_FAILED(m_oCommands.Create("KI_Commands", new cMediaType(0, 0, 0, "tKITC"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oCommands));

    RETURN_IF_FAILED(m_oOdometry.Create("Odometry_Data", new cMediaType(0, 0, 0, "tOdometry"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOdometry));

    RETURN_IF_FAILED(m_oCrossingIndicator.Create("Crossing_Indicator", new cMediaType(0, 0, 0, "tCrossingIndicator"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oCrossingIndicator));

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Struct for steering angle Point Transmission
    tChar const * strDescSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSteeringAngle);
    cObjectPtr<IMediaType> pTypeSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strDescSteeringAngle,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSteeringAngle));

    RETURN_IF_FAILED(m_oSteeringAngle.Create("Steering_Angle", pTypeSteeringAngle, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngle));

    // Struct for Intersection Point Transmission
    tChar const * strDescMiddlePoint = pDescManager->GetMediaDescription("tPoint2d");
    RETURN_IF_POINTER_NULL(strDescMiddlePoint);
    cObjectPtr<IMediaType> pTypeMiddlePoint = new cMediaType(0, 0, 0, "tPoint2d", strDescMiddlePoint,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMiddlePoint->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMiddlePoint));

    RETURN_IF_FAILED(m_oMiddlePoint.Create("Middle_Point", pTypeMiddlePoint, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oMiddlePoint));

    // Struct for Gear Output to Speed Control
    // SPEED OUTPUT
    tChar const * strDescSpeed = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSpeed);
    cObjectPtr<IMediaType> pTypeSpeed = new cMediaType(0, 0, 0, "tSignalValue", strDescSpeed,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSpeed->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSpeedOut));

    RETURN_IF_FAILED(m_outputSpeed.Create("Speed_Signal", pTypeSpeed, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputSpeed));


    // Struct for status output to AI
    tChar const * strDescStatus = pDescManager->GetMediaDescription("tInt8SignalValue");
    RETURN_IF_POINTER_NULL(strDescStatus);
    cObjectPtr<IMediaType> pTypeStatus = new cMediaType(0, 0, 0, "tInt8SignalValue", strDescStatus,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeStatus->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescStatus));

    RETURN_IF_FAILED(m_oStatus.Create("TCData_to_AI", pTypeStatus, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oStatus));


    RETURN_NOERROR;
}

tResult cSWE_TrackControl::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {

        //m_referencePoint.x = GetPropertyFloat("Reference Point x-Coord");
        m_wheelbase = GetPropertyFloat("Wheelbase in mm", 359);


    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::Start(__exception)
{

    m_property_useNewCalc =             (tBool)GetPropertyBool("Use new angle calculation", true);
    m_property_stopAtVirtualSL =        (tBool)GetPropertyBool("Stop at virtual stoplines", true);
    m_property_useTestMode =            (tBool)GetPropertyBool("Use Testmode", false);
    m_property_TestModeStartCommand =   (tInt8)GetPropertyInt("Testmode start command", 1);
    m_property_TestModeStartSpeed =     (tInt8)GetPropertyInt("Testmode start speed", 2);
    m_property_InvSteering =            (tBool)GetPropertyBool("InvertSteering", true);
    m_property_StoplineWheelDist =      (tInt32)GetPropertyInt("Stopdist Stopline to Wheel in mm", 120);
    m_property_SteeringDeadAngle =      (tFloat32)GetPropertyFloat("Steering Dead angle in degree", 3);

    m_input_maxGear = 0;
    m_input_Command = -1;
    m_input_intersectionIndicator = 0;
    m_angleAbs = 0;
    m_old_steeringAngle = 0.0;

    m_status_noSteering = false;
    m_status_noGears = false;
    m_status_my_state = IDLE;

    m_oManeuverObject.Reset();

    m_stoplineData.crossingType = 0;
    m_stoplineData.isRealStopLine = false;
    m_stoplineData.StopLinePoint1.x = 0;
    m_stoplineData.StopLinePoint1.y = 0;
    m_stoplineData.StopLinePoint2.x = 0;
    m_stoplineData.StopLinePoint2.y = 0;

    m_odometryData.angle_heading = 0;
    m_odometryData.distance_sum = 0;
    m_odometryData.velocity = 0;
    m_odometryData.distance_x = 0;
    m_odometryData.distance_y = 0;

    m_firstRun = true;

    SendGear(0);
    SendSteering(0);

    return cFilter::Start(__exception_ptr);
}

tResult cSWE_TrackControl::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_TrackControl::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_TrackControl::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    m_mutex.Enter();

    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
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

        if (pSource == &m_oIntersectionPoints)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("intersecPoint.xCoord", (tVoid*)&(m_input_trackingPoint.x));
            pCoder->Get("intersecPoint.yCoord", (tVoid*)&(m_input_trackingPoint.y));
            pCoder->Get("Indicator", (tVoid*)&(m_input_intersectionIndicator));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------

            ReactToInput(-1); //no command

        }
        else if(pSource == &m_oOdometry)
        {

            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));
            pCoder->Get("distance_x", (tVoid*)&(m_odometryData.distance_x));
            pCoder->Get("distance_y", (tVoid*)&(m_odometryData.distance_y));
            pCoder->Get("angle_heading", (tVoid*)&(m_odometryData.angle_heading));
            pCoder->Get("velocity", (tVoid*)&(m_odometryData.velocity));
            pCoder->Get("distance_sum", (tVoid*)&(m_odometryData.distance_sum));
            m_pCoderDescInputMeasured->Unlock(pCoder);

            m_angleAbs = m_angleAbs + m_odometryData.angle_heading;


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------

            if((m_property_useTestMode) && (m_firstRun))
            {
                m_firstRun = false;
                m_input_Command = m_property_TestModeStartCommand;
                m_input_maxGear = m_property_TestModeStartSpeed;

                if (DEBUG_OUTPUT)
                    LOG_ERROR(cString("TC: testmode start: " + cString::FromInt32(m_input_Command)));

                ReactToInput(m_input_Command);
            }
            else
                ReactToInput(-1); // no command

        }
        else if(pSource == &m_oCommands)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("tInt8ValueSpeed", (tVoid*)&(m_input_maxGear));
            pCoder->Get("tInt8ValueCommand", (tVoid*)&(m_input_Command));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------
            ReactToInput(m_input_Command);

        }
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

            ReactToInput(99); //special command - new stopline data
        }

        // -------------------------------------------------------------------

    }

    m_mutex.Leave();

    RETURN_NOERROR;
}


//---------------------------------------------------------------------------------------------------
// --------------------------------  Calculate Steering Angle  --------------------------------------
// --------------------------------------------------------------------------------------------------

tFloat64 cSWE_TrackControl::CalcSteeringAngleTrajectory( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator )
{
    tFloat64 steeringAngle = 0;

    if( intersectionIndicator != 0 )
    {
        steeringAngle = acos(trackingPoint.dot(m_PerpenticularPoint)/cv::norm(trackingPoint));
        if(trackingPoint.y < 0)
        {
            steeringAngle = (-1.0)*steeringAngle;
        }
    }
    else
    {
        steeringAngle = m_old_steeringAngle;
    }

    return steeringAngle;
}

tFloat64 cSWE_TrackControl::CalcSteeringAngleCircle( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator )
{
    tFloat32 steeringAngle = 0;

    // transform tracking point in rear axis coo sys
    cv::Point2d trackingPoint_ra;
    trackingPoint_ra.x = trackingPoint.x + m_wheelbase;
    trackingPoint_ra.y = trackingPoint.y;

    // calculate steering angle
    if( intersectionIndicator != 0 )
    {
        tFloat32 alpha = (tFloat32)std::atan2( (tFloat32)trackingPoint_ra.y, (tFloat32)trackingPoint_ra.x );
        tFloat32 distance = (tFloat32)cv::norm( trackingPoint_ra );
        tFloat32 radius = (tFloat32)(distance / ( 2* std::sin( alpha + 1e-3 ) ));

        steeringAngle = (tFloat32)std::atan( m_wheelbase / radius );
    }
    else
    {
        steeringAngle = m_old_steeringAngle;
    }

    //keep somewhat sane values
    if(steeringAngle > 0.785) //~45 degrees
        steeringAngle = 0.785;
    else if (steeringAngle < -0.785)
        steeringAngle = -0.785;

    //DEBUG
    /*
    if (DEBUG_OUTPUT)
        LOG_ERROR(cString("TC: in Calc Steering Angle old angle: ") + cString::FromFloat64(m_old_steeringAngle) + cString("intersection Indicator: " + cString::FromInt32(intersectionIndicator)) );
    */

    return steeringAngle;
}



//----------------------------------------------------------------------------------------
// --------------------------------  State Machine  --------------------------------------
// ---------------------------------------------------------------------------------------

tResult cSWE_TrackControl::ReactToInput(tInt32 command)
{


    // -----------  init default behaviour ------------

    m_outputSteeringAngle = 0;
    m_outputGear = m_input_maxGear;
    m_outputStatus = STATUS_NORMAL;

    m_status_noGears = false;
    m_status_noSteering = false;


    // calculate steering angle for normal operation (the default)

    if(m_property_useNewCalc) //two alternative modes of calculation
    {
        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: calculating steering angle circle:"));

        m_outputSteeringAngle = 180.0/CV_PI * (tFloat32)( CalcSteeringAngleCircle( m_input_trackingPoint, m_input_intersectionIndicator ) );
    }
    else
    {
        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: calculating steering angle normal:"));

        m_outputSteeringAngle = 180.0/CV_PI* (tFloat32)( CalcSteeringAngleTrajectory( m_input_trackingPoint, m_input_intersectionIndicator ) );
    }


    // ---------- STATES ------------

    switch(m_status_my_state)
    {


    case IDLE:

        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: in IDLE  command:" + cString::FromInt32(command)));

        switch(command)
        {

        case 0: //Emergency stop

            GotoEMERGENCY_STOP();

            break;

        case 1: //follow road

            GotoNORMAL_OPERATION();

            break;

        case 2: //turn left

            GotoTURN_LEFT();

            break;

        case 3: //turn right

            GotoTURN_RIGHT();

            break;

        case 4: // overtake - here normal operation

            GotoNORMAL_OPERATION();

            break;

        case 5: //go straight (crossroads)

            GotoGO_STRAIGHT();

            break;

        case 7: // no speed, follow road but only steer

            GotoNO_SPEED();

            break;


        default: // no (legal) command - stay in state

            //m_outputStatus= STATUS_NORMAL;
            m_status_noSteering = true;
            m_status_noGears = true;

            break;
        }

        break;



    case NORMAL_OPERATION:

        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: in NORMAL_OPERATION  command:" + cString::FromInt32(command)));

        switch(command)
        {

        case 0: //Emergency stop

            GotoEMERGENCY_STOP();

            break;

        case 6: // go idle

            GotoIDLE();

            break;

        case 7: // no speed

            GotoNO_SPEED();

            break;

        case 99: // Stopline

            GotoSTOPLINE();

            break;


        default: // no (legal) command - stay in state

            m_outputStatus= STATUS_NORMAL;
            m_status_noSteering = false;
            m_status_noGears = false;
            //use steering angle already calculated
            m_outputGear = 3; //use max speed allowed

            break;
        }

        break;




    case STOP_AT_STOPLINE_INPROGRESS:

        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: in STOP_AT_STOPLINE_INPROGRESS  command:" + cString::FromInt32(command)));

        switch(command)
        {

        case 0: //Emergency stop

            GotoEMERGENCY_STOP();

            break;


        case 6: // go idle

            GotoIDLE();

            break;


        case 99: // new Stopline

            //try to update with new stopline (might be rejected)
            m_oManeuverObject.Start(TC_STOP_AT_STOPLINE, m_angleAbs, m_odometryData.distance_sum, (m_stoplineData.StopLinePoint1.x - m_property_StoplineWheelDist), (m_stoplineData.StopLinePoint2.x - m_property_StoplineWheelDist), m_stoplineData.isRealStopLine);


            if(m_property_stopAtVirtualSL) //always stop at stopline
                m_outputGear = (tInt8)m_oManeuverObject.GetGear();
            else
            {
                if(m_oManeuverObject.GetStoplineType()) //when real stopline
                    m_outputGear = (tInt8)m_oManeuverObject.GetGear();//... stop there
                else
                    m_outputGear = 3; //when virtual go as fast as allowed
            }


            // use normal steering angle if not too close
            if( (m_oManeuverObject.GetStoplineDistance() < CLOSE_STOPLINE) && (m_oManeuverObject.GetStoplineType() == true) && ((m_input_trackingPoint.y > 150) && ( m_input_intersectionIndicator != 0) ) ) //tracking point probably erronous?
            {
                m_oManeuverObject.GetSteeringAngle();
            }
            else if (m_input_intersectionIndicator == 0) //no tracking point?
                m_oManeuverObject.GetSteeringAngle();


            m_status_noGears = false;
            m_status_noSteering = false;
            m_outputStatus= STATUS_NORMAL;

            break;


        default: // no (legal) command - stay in state

            //------------------- update data ------------------

            m_oManeuverObject.CalcStep(m_angleAbs, m_odometryData.distance_sum);

            if(m_property_stopAtVirtualSL) //always stop at stopline
                m_outputGear = (tInt8)m_oManeuverObject.GetGear();
            else
            {
                if(m_oManeuverObject.GetStoplineType()) //when real stopline
                    m_outputGear = (tInt8)m_oManeuverObject.GetGear();//... stop there
                else
                    m_outputGear = 3; //when virtual go as fast as allowed
            }


            // use normal steering angle if not too close
            if( (m_oManeuverObject.GetStoplineDistance() < CLOSE_STOPLINE) && (m_oManeuverObject.GetStoplineType() == true) && ((m_input_trackingPoint.y > 150) && ( m_input_intersectionIndicator != 0) ) ) //tracking point probably erronous?
            {
                m_oManeuverObject.GetSteeringAngle();
            }
            else if (m_input_intersectionIndicator == 0) //no tracking point?
                m_oManeuverObject.GetSteeringAngle();



            // ------------- is maneuver finished? -------------

            if (m_oManeuverObject.GetStatus() == TC_STOP_AT_STOPLINE)
            {
                m_status_noGears = false;
                m_status_noSteering = false;
                m_outputStatus= STATUS_NORMAL;
            }
            else //maneuver finished go to idle (but send last speed value)
            {
                m_status_my_state = IDLE;
                m_status_noGears = false;
                m_status_noSteering = false;
                m_outputStatus= STATUS_ATSTOPLINE; //tell the AI we're stopped at a stopline
            }

            break;
        }

        break;



    case GO_STRAIGHT_INPROGRESS:

        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: in GO_STRAIGHT_INPROGRESS  command:" + cString::FromInt32(command)));

        switch(command)
        {

        case 0: //Emergency stop

            GotoEMERGENCY_STOP();

            break;

        case 6: // go idle

            GotoIDLE();

            break;


        default: // no (legal) command - stay in state

            //------------------- update data ------------------

            m_oManeuverObject.CalcStep(m_angleAbs, m_odometryData.distance_sum);

            m_outputGear = (tInt8)m_oManeuverObject.GetGear();

            //steering angle OK?
            if( m_input_intersectionIndicator != 0 )
            {
                // use m_outputSteeringAngle already calculated
            }
            else
                m_outputSteeringAngle = m_oManeuverObject.GetSteeringAngle();


            // ------------- is maneuver finished? -------------

            if(m_oManeuverObject.GetStatus() == NO_MANEUVER)
            {
                // return to normal operation
                GotoNORMAL_OPERATION();
                m_outputStatus = STATUS_ENDOFTURN;
            }
            else
            {
                m_outputStatus = STATUS_NORMAL;
                m_status_noSteering = false;
                m_status_noGears = false;
            }


            break;
        }

        break;



    case TURN_INPROGRESS:

        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: in TURN_INPROGRESS  command:" + cString::FromInt32(command)));

        switch(command)
        {

        case 0: //Emergency stop

            GotoEMERGENCY_STOP();

            break;

        case 6: // go idle

            GotoIDLE();

            break;


        default: // no (legal) command - stay in state

            //------------------- update data ------------------

            m_oManeuverObject.CalcStep(m_angleAbs, m_odometryData.distance_sum);

            m_outputGear = (tInt8)m_oManeuverObject.GetGear();
            m_outputSteeringAngle = m_oManeuverObject.GetSteeringAngle();


            // ------------- is maneuver finished? -------------

            if(m_oManeuverObject.GetStatus() == NO_MANEUVER)
            {
                // return to normal operation
                GotoNORMAL_OPERATION();
                m_outputStatus = STATUS_ENDOFTURN;
                m_status_noSteering = false;
                m_status_noGears = false;
            }
            else
            {
                m_outputStatus = STATUS_NORMAL;
                m_status_noSteering = false;
                m_status_noGears = false;
            }

            /*
            if (DEBUG_OUTPUT)
                LOG_ERROR(cString("TC: in TURN_INPROGRESS  steering:" + cString::FromFloat64(m_outputSteeringAngle)));

            if (DEBUG_OUTPUT)
                LOG_ERROR(cString("TC: in TURN_INPROGRESS  gear:" + cString::FromInt32(m_outputGear)));
                */

            break;
        }

        break;




    case NO_SPEED:

        if (DEBUG_OUTPUT)
            LOG_ERROR(cString("TC: in NO_SPEED  command:" + cString::FromInt32(command)));

        switch(command)
        {

        case 0: //Emergency stop

            GotoEMERGENCY_STOP();

            break;

        case 1: //follow road

            GotoNORMAL_OPERATION();

            break;

        case 6: // go idle

            GotoIDLE();

            break;

        default: // no (legal) command - stay in state

            m_outputStatus= STATUS_NORMAL;
            m_status_noSteering = false;
            m_status_noGears = true;
            //use steering angle already calculated

            break;
        }

        break;



    default:
        m_outputStatus= STATUS_NORMAL;
        m_outputGear = 0;
        m_status_my_state = IDLE;
        LOG_ERROR(cString("TC: ERROR! Illegal command from KI "));
        break;

    }



    // ---------- OUTPUT DATA -------------

    //reset command
    m_input_Command = -1;

    //compensate for dead angle in steering (under the assumption that the wheels always try to turn the least possible amount)
    if(m_outputSteeringAngle < 0)
        m_outputSteeringAngle -= m_property_SteeringDeadAngle;
    else if (m_outputSteeringAngle > 0)
        m_outputSteeringAngle += m_property_SteeringDeadAngle;

    if(m_property_InvSteering)
        m_outputSteeringAngle *= -1;


    //send steering angle
    if (!m_status_noSteering)
    {
        SendSteering(m_outputSteeringAngle);

        if(m_property_InvSteering)
            m_old_steeringAngle = -m_outputSteeringAngle;
        else
            m_old_steeringAngle = m_outputSteeringAngle;
    }

    // keep gear within boundries
    if(m_outputGear >= 0)
    {
        if(m_input_maxGear >= 0 )
        {
            if (m_input_maxGear < m_outputGear)
                m_outputGear = m_input_maxGear;
        }
        else // this combination is invalid
        {
            m_outputGear = 0;
            LOG_ERROR(cString("TC: ERROR! Calculated Output Gear was: " + cString::FromInt32(m_outputGear) + " but Limit from KI was:" + cString::FromInt32(m_input_maxGear)));
        }
    }
    if(m_outputGear < 0)
    {
        if(m_input_maxGear < 0 )
        {
            if(m_input_maxGear > m_outputGear)
                m_outputGear = m_input_maxGear;
        }
        else
        {
            m_outputGear = 0;
            LOG_ERROR(cString("TC: ERROR! Calculated Output Gear was: " + cString::FromInt32(m_outputGear) + " but Limit from KI was:" + cString::FromInt32(m_input_maxGear)));
        }
    }

    //send speed signal
    if (!m_status_noGears)
        SendGear(m_outputGear);

    //send status to KI/central control
    SendStatus(m_outputStatus);

    //send tracking point for visualization
    SendTrackingPoint();

    RETURN_NOERROR;
}



//----------------------------------------------------------------------------------------
// ------------------------------------  OUTPUT  -----------------------------------------
// ---------------------------------------------------------------------------------------


tResult cSWE_TrackControl::SendSteering(tFloat32 outputAngle)
{


    static tFloat32 lastOutput = 777;
    static tTimeStamp lastOutputTime = 0;

    // only send changes or every 500ms
    if((lastOutput != outputAngle) || (_clock->GetTime () - lastOutputTime  > 500000))
    {
        lastOutput = outputAngle;
        lastOutputTime = _clock->GetTime ();
        // generate Coder object
        cObjectPtr<IMediaCoder> pCoder;

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSampleOutput;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

        //allocate memory with the size given by the descriptor
        // ADAPT: m_pCoderDescPointLeft
        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescSteeringAngle->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSampleOutput->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        // ADAPT: m_pCoderDescPointLeft
        //cObjectPtr<IMediaCoder> pCoder;
        RETURN_IF_FAILED(m_pCoderDescSteeringAngle->WriteLock(pMediaSampleOutput, &pCoder));
        pCoder->Set("f32Value", (tVoid*)&(outputAngle));
        m_pCoderDescSteeringAngle->Unlock(pCoder);

        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oSteeringAngle.Transmit(pMediaSampleOutput));
    }



    RETURN_NOERROR;
}

tResult cSWE_TrackControl::SendTrackingPoint()
{

    // generate Coder object
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescMiddlePoint->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    //cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescMiddlePoint->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("xCoord", (tVoid*)&(m_input_trackingPoint.x));
    pCoder->Set("yCoord", (tVoid*)&(m_input_trackingPoint.y));
    m_pCoderDescMiddlePoint->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oMiddlePoint.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::SendGear( const tInt8 outputGear )
{

    //tUInt32 timeStamp = 0;
    tFloat32 my_outputGear;
    static tFloat32 lastOutput = 777;
    static tTimeStamp lastOutputTime = 0;

    my_outputGear = (tFloat32)outputGear;

    // only send changes or every 500ms but never more often than every 10ms
    if(((lastOutput != my_outputGear) || (_clock->GetTime () - lastOutputTime  > 500000)) && (_clock->GetTime () - lastOutputTime  > 10000))
    {
        lastOutput = outputGear;
        lastOutputTime = _clock->GetTime ();

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

            pCoder->Set("f32Value", (tVoid*)&(my_outputGear));
            pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        }

        //transmit media sample over output pin
        pMediaSample->SetTime(_clock->GetStreamTime());
        m_outputSpeed.Transmit(pMediaSample);
    }

    RETURN_NOERROR;
}


tResult cSWE_TrackControl::SendStatus( const tInt8 status )
{

    // generate Coder object
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescStatus->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    //cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescStatus->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("int8Value", (tVoid*)&(status));
    m_pCoderDescStatus->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oStatus.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}



//----------------------------------------------------------------------------------------
// ----------------------------  State Transitions ---------------------------------------
// ---------------------------------------------------------------------------------------


tResult cSWE_TrackControl::GotoIDLE()
{

    m_outputStatus= STATUS_NORMAL;
    m_status_my_state = IDLE;
    m_status_noSteering = true;
    m_status_noGears = true;

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::GotoNORMAL_OPERATION()
{

    m_status_my_state = NORMAL_OPERATION;
    m_outputStatus = STATUS_NORMAL;
    m_status_noSteering = false;
    m_status_noGears = false;

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::GotoTURN_LEFT()
{

    m_status_my_state = TURN_INPROGRESS;

    //init turn
    m_oManeuverObject.Reset();
    m_oManeuverObject.Start(TC_TURN_LEFT, m_angleAbs, m_odometryData.distance_sum, 0 , 0, false);

    m_outputStatus = STATUS_NORMAL;
    m_status_noSteering = true;
    m_status_noGears = true;

    RETURN_NOERROR;
}
tResult cSWE_TrackControl::GotoTURN_RIGHT()
{

    m_status_my_state = TURN_INPROGRESS;

    //init turn
    m_oManeuverObject.Reset();
    m_oManeuverObject.Start(TC_TURN_RIGHT, m_angleAbs, m_odometryData.distance_sum, 0 , 0, false);

    m_outputStatus = STATUS_NORMAL;
    m_status_noSteering = true;
    m_status_noGears = true;

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::GotoGO_STRAIGHT()
{

    m_status_my_state = TURN_INPROGRESS;

    //init turn
    m_oManeuverObject.Reset();
    m_oManeuverObject.Start(TC_GO_STRAIGHT, m_angleAbs, m_odometryData.distance_sum, 0 , 0, false);

    m_outputStatus = STATUS_NORMAL;
    m_status_noSteering = true;
    m_status_noGears = true;

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::GotoEMERGENCY_STOP()
{

    m_outputStatus= STATUS_NORMAL;
    m_outputGear = 0;
    m_status_my_state = IDLE;
    m_status_noSteering = true;
    m_status_noGears = false;

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::GotoNO_SPEED()
{

    m_outputStatus= STATUS_NORMAL;
    m_status_my_state = NO_SPEED;
    m_status_noSteering = false;
    m_status_noGears = true;

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::GotoSTOPLINE()
{
    m_outputStatus = STATUS_NORMAL;

    m_oManeuverObject.Reset();

    //if stopline accepted go to new state
    if (m_oManeuverObject.Start(TC_STOP_AT_STOPLINE, m_angleAbs, m_odometryData.distance_sum, m_stoplineData.StopLinePoint1.x , m_stoplineData.StopLinePoint2.x, m_stoplineData.isRealStopLine) != 1)
    {
        m_status_my_state = STOP_AT_STOPLINE_INPROGRESS;
        m_status_noGears = false;
        m_status_noSteering = false;
    }

    RETURN_NOERROR;
}
