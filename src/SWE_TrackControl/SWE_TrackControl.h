#ifndef _SWE_TRACKCONTROL_H_
#define _SWE_TRACKCONTROL_H_

#include "stdafx.h"
#include "SWE_Maneuver.h"



#define OID_ADTF_SWE_TRACKCONTROL "adtf.swe.trackcontrol"

/*!
* track controller SmartWheels Erlangen, Robert de Temple. Questions? Just ask ! robert punkt detemple kringel gmail punkt com
* This Module realizes the calculation of steering angles for road following, emergency stops, the approach of stoplines and crossroads maneuvers like left and right turns
* !!! Attention !!! This module uses a coordinate system with the FRONT axis as zero for various, mostly historic, reasons. Positive X is to the front, positive Y to the left, angles are counted clockwise from the x axis.

    TC talkback to AI/KI:
    0 = normal operation            (default - nothing to report)
    1 = finished turning maneuver   (this includes going straight over crossroads)
    2 = just stopped at stopline    (if the stopline was virtual the car might still be moving)


    commands from AI/KI
    0* = emergency stop     (ALWAYS works)
    1 = follow road         (default behavior, stops at stoplines)
    2 = turn left
    3 = turn right
    4 = pass obstacle       (not implemented)
    5 = go straight         (crossroads)
    6* = idle               (no outputs, like a reset -> all commands can be issued after this but stopline will be IGNORED)
    7 = only steering       (follow road, no speed output, stoplines will be IGNORED)

    Max Speed:
    (Gears) 3,2,1,0,-1,-2 (-1, -2 not used here)

    * these two commands will always be executed, no matter what state trackcontrol currently has, so be careful issuing these! All other commands (usually) finish first before accepting a new command.
*/

class cSWE_TrackControl : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_TRACKCONTROL, "SWE TrackControl", OBJCAT_DataFilter, "Track Control", 1, 0,0, "pre alpha version");

    // create pins
    cInputPin m_oIntersectionPoints;
    cInputPin m_oCommands;   //commands from the KI
    cInputPin m_oOdometry;
    cInputPin m_oCrossingIndicator;

    cOutputPin m_oSteeringAngle;
    cOutputPin m_oMiddlePoint;
    cOutputPin m_outputSpeed;
    cOutputPin m_oStatus;

public:
    cSWE_TrackControl(const tChar* __info);
    virtual ~cSWE_TrackControl();


protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:

    enum my_status
    {
        IDLE,
        NORMAL_OPERATION,
        NO_SPEED,
        STOP_AT_STOPLINE_INPROGRESS,
        GO_STRAIGHT_INPROGRESS,
        TURN_INPROGRESS
    };

    /*! creates all the input Pins*/
    tResult CreateInputPins(__exception = NULL);
    /*! creates all the output Pins*/
    tResult CreateOutputPins(__exception = NULL);

    /*! react to new inputs */
    tResult ReactToInput(tInt32 command);

    /*! calculate steering angle => sets wheel direction towards tracking point */
    tFloat64 CalcSteeringAngleTrajectory( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator );

    /*! calculate steering angle new version => let turning circle intersect with tracking point */
    tFloat64 CalcSteeringAngleCircle( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator );

    /*! set steering angle */
    tResult SendSteering(tFloat32 outputAngle);

    /*! set speed */
    tResult SendGear( const tInt8 outputGear );

    /*! set speed */
    tResult SendTrackingPoint();

    /*! send status signals back to KI/AI */
    tResult SendStatus( const tInt8 status );

    /*! state transitions */

    tResult GotoIDLE();
    tResult GotoNORMAL_OPERATION();
    tResult GotoTURN_LEFT();
    tResult GotoTURN_RIGHT();
    tResult GotoGO_STRAIGHT();
    tResult GotoEMERGENCY_STOP();
    tResult GotoNO_SPEED();
    tResult GotoSTOPLINE();


    /*! member variables */

    tBool m_property_useNewCalc;
    tBool m_property_stopAtVirtualSL;
    tBool m_property_InvSteering;

    tInt8 m_input_maxGear;
    tInt32 m_input_Command;
    tInt32 m_property_StoplineWheelDist;

    tInt8 m_input_intersectionIndicator;

    tFloat32 m_angleAbs;

    tFloat64 m_old_steeringAngle;

    tFloat64 m_wheelbase;

    cv::Point2d m_PerpenticularPoint;
    cv::Point2d m_input_trackingPoint;

    tFloat32 m_outputSteeringAngle;
    tFloat32 m_property_SteeringDeadAngle;
    tInt8 m_outputGear;
    tInt8 m_outputStatus;

    my_status m_status_my_state;

    tBool m_status_noSteering;
    tBool m_status_noGears;

    tBool m_property_useTestMode;
    tInt8 m_property_TestModeStartCommand;
    tInt8 m_property_TestModeStartSpeed;

    tBool m_firstRun;

    SWE_Maneuver m_oManeuverObject;


    /*! Lock */
    cCriticalSection m_mutex;


    /*! struct containing the odometry input data */
    typedef struct
    {
        tFloat32	 distance_x;			// x dist in mm
        tFloat32	 distance_y;            // y dist in mm
        tFloat32   	 angle_heading;         // heading in radians
        tFloat32     velocity;              // velocity
        tFloat32     distance_sum;          // sum of driven distance
    }odometryData;

    odometryData m_odometryData;

    typedef struct
    {
        tBool isRealStopLine;
        tInt crossingType;
        cv::Point2d StopLinePoint1;
        cv::Point2d StopLinePoint2;
    }stoplineData;

    stoplineData m_stoplineData;


    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSteeringAngle;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescMiddlePoint;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSpeedOut;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescStatus;

};

#endif // _SWE_TrackControl_H_
