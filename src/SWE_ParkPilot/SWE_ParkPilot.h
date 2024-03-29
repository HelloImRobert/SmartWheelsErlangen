#ifndef _SWE_PARKPILOT_H_
#define _SWE_PARKPILOT_H_

#include "stdafx.h"
#include "math.h"


#define OID_ADTF_SWE_PARKPILOT "adtf.swe.parkpilot"

/*!
* Intersection Point Calculator
*/
class cSWE_ParkPilot : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_PARKPILOT, "SWE ParkPilot", OBJCAT_DataFilter, "ParkPilot", 1, 0,0, "pre alpha version");

    // PINS
    cInputPin m_inputParkTrigger;
    cInputPin m_pin_input_ir_right;
    cInputPin m_pin_input_ir_left;
    cInputPin m_inputOdometry;

    cOutputPin m_outputSpeed;
    cOutputPin m_outputSteering;
    cOutputPin m_oOutputParking;
    cOutputPin m_outputBlink;
    cOutputPin  m_outputWarn;
    cOutputPin m_outputTCP;


    // FOR TEST START
    cOutputPin m_AccelerateOutputPin;
    // FOR TEST ENDE

public:
    cSWE_ParkPilot(const tChar* __info);
    virtual ~cSWE_ParkPilot();

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);



private:

    tFloat32    m_IRFrontRightCur;
    tFloat32    m_IRRearRightCur;
    tFloat32    m_IRFrontLeftCur;
    tFloat32    m_lastIRshort;
    tFloat32    m_checkPointOne;

    tFloat32    m_distStartPark;
    tFloat32    m_headingAtStart;
    tFloat32    m_distEntry;
    tFloat32 	m_distExit;
    tFloat32    m_angleAbs;
    tFloat32	m_rememberDist;
    tFloat32    m_rememberDistTwo;

    // Angles for maneuveringö
    tFloat32	m_centralAngle;
    tFloat32    m_counterAngle;
    tFloat32    m_manAngleOne;
    tFloat32    m_manAngleTwo;
    tFloat32    m_finishAngle;

    //Properties
    tFloat32     m_pullFirst;    
    tFloat32     m_pullSecond;
    tFloat32     m_pullThird;
    tFloat32     m_pullFourth;
    tFloat32    m_centralAngleSteering;
    tBool       m_logging;
    tFloat32    m_AlongsideSize;
    tFloat32    m_CrossSize;
    tFloat32    m_headingAngleForward;
    tFloat32    m_steeringAnlgeBackward;
    tFloat32    m_perpendicularBackward;
    tFloat32    m_pullLeftStraight;
    tFloat32    m_pullRightStraight;
    tFloat32    m_straightForward;
    tInt32      m_stopTime;
    tFloat32    m_driftComp;
    tFloat32    m_pullDriftComp;
    tFloat32    m_pullLeftAngle;
    tFloat32    m_pullRightAngle;
    tFloat32    m_pullCounterSide;
    tInt32      m_startTimer;
    tBool       m_setBack;
    tFloat32    m_setBackDist;
    tBool       m_skipStraight;

    std::vector<tFloat32> m_initTest_vect;

    tInt8       m_parkTrigger;

    tBool       m_searchActivated;
    tBool       m_minDistReached;
    tBool       m_carStopped;
    tBool       m_parkAlongside;
    tBool       m_parkCross;
    tBool       m_pullLeft;
    tBool       m_pullRight;
    tBool       m_gotControl;
    tBool       m_firstIR;
    tBool       m_activeManeuvering;

    tInt16      m_searchState;
    tInt16      m_parkState;
    tInt16      m_pulloutState;

    tBool       m_testing;

    cv::Point2d m_objects[10];

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



    /*! creates all the input Pins*/
    tResult CreateInputPins(__exception = NULL);
    /*! creates all the output Pins*/
    tResult CreateOutputPins(__exception = NULL);

    tTimeStamp GetTime();

    // Park functions
    tResult searchRoutineAlongside();
    tResult searchRoutineCross();

    tResult parkRoutineAlongside();
    tResult parkRoutineCross();
    tResult pullOutAlongsideRight();
    tResult pullOutAlongsideLeft();
    tResult pullOutCrossRight();
    tResult pullOutCrossLeft();

    tResult sendSpeed(tFloat32 speed);
    tResult sendVelocity(tFloat32 vel);
    tResult sendSteeringAngle(tFloat32 steeringAngle);
    tResult sendParkState(tInt8 value);
    tResult sendBlink(tInt8 blink);
    tResult sendWarn(tBool warn);

    tResult jumpIntoStates();


    // Media Descriptors
    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDesc;

    cObjectPtr<IMediaTypeDescription> m_pCoderParkTrigger;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescOdometry;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescObjectData;
    cObjectPtr<IMediaTypeDescription> m_pCoderIRR;
    cObjectPtr<IMediaTypeDescription> m_pCoderIRL;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescStop;

    cObjectPtr<IMediaTypeDescription> m_pCoderDescSpeedOut;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSteeringOut;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescParkStateOut;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescParkOutput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescLightOutput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescWarnOutput;
};

#endif // _SWE_PARKPILOT_H_
