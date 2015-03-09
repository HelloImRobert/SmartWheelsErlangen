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
    cInputPin m_inputObjectData;
    cInputPin m_pin_input_ir;
    cInputPin m_inputOdometry;
    cInputPin m_inputStopFlag;

    cOutputPin m_outputSpeed;
    cOutputPin m_outputSteering;
    cOutputPin m_outputParkState;

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

    tFloat32    m_distStartPark;
    tFloat32    m_headingAtStart;
    tFloat32    m_AngleAdjustment;
    tFloat32    m_DistanceAdjustment;
    tFloat32    m_distEntry;
    tFloat32 	m_distExit;
    tFloat32	m_distStart;
    tFloat32	m_centralAngle;
    tFloat32    m_counterAngle;
    tFloat32	m_rememberDist;
    tFloat32    m_securityBuffer;
    tFloat32    m_headingAngleForward;
    tFloat32    m_steeringAnlgeBackward;
    tFloat32    m_perpendicularBackward;
    tFloat32    m_pullLeftStraight;
    tFloat32    m_pullRightStraight;
    tFloat32    m_straightForward;

    std::vector<tFloat32> m_initTest_vect;

    tInt8       m_parkTrigger;

    tBool       m_searchActivated;
    tBool       m_minDistReached;
    tBool       m_carStopped;

    tInt16      m_searchState;
    tInt16      m_parkState;
    tInt16      m_pulloutState;

    tBool       m_debug_bool;


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

    // Park functions
    tResult searchRoutineAlongside();
    tResult searchRoutineCross();

    tResult parkRoutineAlongsideEasy();
    tResult parkRoutineAlongsideNormal();
    tResult parkRoutineCross();
    tResult pullOutAlongsideRight();
    tResult pullOutAlongsideLeft();
    tResult pullOutCrossRight();
    tResult pullOutCrossLeft();

    tResult sendSpeed(tFloat32 speed);
    tResult sendVelocity(tFloat32 vel);
    tResult sendSteeringAngle(tFloat32 steeringAngle);
    tResult sendParkState(tInt8 parkState);


    // Media Descriptors
    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDesc;

    cObjectPtr<IMediaTypeDescription> m_pCoderParkTrigger;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescOdometry;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescObjectData;
    cObjectPtr<IMediaTypeDescription> m_pCoderIR;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescStop;

    cObjectPtr<IMediaTypeDescription> m_pCoderDescSpeedOut;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSteeringOut;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescParkStateOut;
};

#endif // _SWE_PARKPILOT_H_
