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

    cOutputPin m_oSteeringAngle;
    cOutputPin m_oMiddlePoint;

public:
    cSWE_TrackControl(const tChar* __info);
    virtual ~cSWE_TrackControl();

    tFloat64 CalcSteeringAngle(cv::Point2d leftIntersectionPoint, cv::Point2d rightIntersectionPoint, tUInt32 intersectionIndicator);

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

    cv::Point2d m_PerpenticularPoint;
    tFloat64 m_steeringAngle;

    cv::Point2d m_middlePoint;



    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSteeringAngle;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescMiddlePoint;

};

#endif // _SWE_TrackControl_H_
