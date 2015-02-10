#ifndef _SWE_TRACKCONTROL_H_
#define _SWE_TRACKCONTROL_H_

#include "stdafx.h"
#include "swe_globalObjects.h"


#define OID_ADTF_TEMPLATE_FILTER "adtf.swe.trackcontrol"


//*************************************************************************************************
class cSWE_TrackControl : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, "SWE_TrackControl", adtf::OBJCAT_DataFilter);

protected:
    cInputPin    m_oIntersectionPointLeft;
    cInputPin    m_oIntersectionPointRight;
    cOutputPin   m_oSteeringAngle;
    cInputPin    m_oIntersectionPoints;

public:
    cSWE_TrackControl(const tChar* __info);
    virtual ~cSWE_TrackControl();

protected:
    // init, shutdown etc.
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

    // implementation of functionality
    tResult CalcSteeringAngle();

private:
    // holds last steering Angle
    tFloat32 m_SteeringAngle;
    // holds last recieved intersection points
    cv::Point2d m_IntersectionPointLeft;
    cv::Point2d m_IntersectionPointRight;
    cv::Point2d m_MiddlePoint;
    cv::Point2d m_PerpenticularPoint;

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInput;
};

//*************************************************************************************************
#endif // _TEMPLATE_PROJECT_FILTER_H_
