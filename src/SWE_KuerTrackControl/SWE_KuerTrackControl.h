#ifndef _SWE_KUERTRACKCONTROL_H_
#define _SWE_KUERTRACKCONTROL_H_

#include "stdafx.h"

#define OID_ADTF_SWE_KUERTRACKCONTROL "adtf.swe.kuertrackcontrol"


class cSWE_KuerTrackControl : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_KUERTRACKCONTROL, "SWE KuerTrackControl", OBJCAT_DataFilter, "Kuer Track Control", 1, 0,0, "pre alpha version");

    // create pins
    cInputPin m_oIntersectionPoints;

    cOutputPin m_oSteeringAngle;

public:
    cSWE_KuerTrackControl(const tChar* __info);
    virtual ~cSWE_KuerTrackControl();


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

    /*! calculate steering angle => sets wheel direction towards tracking point */
    tFloat64 CalcSteeringAngleTrajectory( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator );

    /*! set steering angle */
    tResult SendSteering(tFloat32 outputAngle);

    tFloat32 m_old_steeringAngle;
    cv::Point2d m_PerpenticularPoint;

    cv::Point2d m_input_trackingPoint;

    tFloat32 m_outputSteeringAngle;
    tFloat32 m_property_SteeringDeadAngle;
    tInt8 m_input_intersectionIndicator;
    tBool m_property_InvSteering;
    tFloat32 m_filter;


    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSteeringAngle;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescMiddlePoint;

};

#endif // _SWE_KuerTrackControl_H_
