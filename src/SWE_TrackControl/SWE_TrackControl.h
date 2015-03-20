#ifndef _SWE_TRACKCONTROL_H_
#define _SWE_TRACKCONTROL_H_

#include "stdafx.h"



#define OID_ADTF_SWE_TRACKCONTROL "adtf.swe.trackcontrol"

/*!
* track controller
*/
class cSWE_TrackControl : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_TRACKCONTROL, "SWE TrackControl", OBJCAT_DataFilter, "Track Control", 1, 0,0, "pre alpha version");

    // create pins
    cInputPin m_oIntersectionPoints;
    cInputPin m_oCommands;   //commands from the KI
    cInputPin m_oOdometry;  //TODO

    cOutputPin m_oSteeringAngle;
    cOutputPin m_oMiddlePoint;
    cOutputPin m_oGear; //TODO

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
    /*! creates all the input Pins*/
    tResult CreateInputPins(__exception = NULL);
    /*! creates all the output Pins*/
    tResult CreateOutputPins(__exception = NULL);

    /*! react to new inputs */
    tResult ReactToInput();

    /*! calculate steering angle => sets wheel direction towards tracking point */
    tFloat64 CalcSteeringAngleTrajectory( cv::Point2d trackingPoint, tInt8 intersectionIndicator );

    /*! calculate steering angle new version => let turning circle intersect with tracking point */
    tFloat64 CalcSteeringAngleCircle( cv::Point2d trackingPoint, tInt8 intersectionIndicator );

    /*! set steering angle */
    tResult SendSteering(tFloat32 outputAngle);

    /*! set speed */
    tResult SendGear(tFloat32 outputGear);

    /*! set speed */
    tResult SendTrackingPoint();


    /*! member variables */

    tBool m_property_useNewCalc;

    tFloat32 m_input_maxGear;
    tInt32   m_input_Command;

    tInt8 m_input_intersectionIndicator;

    tFloat32 m_angleAbs;

    tFloat64 m_old_steeringAngle;

    cv::Point2d m_PerpenticularPoint;
    cv::Point2d m_input_trackingPoint;



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



    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSteeringAngle;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescMiddlePoint;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescGear;

};

#endif // _SWE_TrackControl_H_
