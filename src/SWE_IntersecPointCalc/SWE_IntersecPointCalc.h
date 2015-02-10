#ifndef _SWE_INTERSECPOINTCALC_H_
#define _SWE_INTERSECPOINTCALC_H_

#include "stdafx.h"
#include "swe_cboundary.h"


#define OID_ADTF_SWE_INTERSECPOINTCALC "adtf.swe.intersecpointcalc"

/*!
* Intersection Point Calculator
*/
class cSWE_IntersecPointCalc : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_INTERSECPOINTCALC, "SWE Intersection Point calculation", OBJCAT_DataFilter, "Intersection Point Calculation", 1, 0,0, "pre alpha version");

    // create pins
    cInputPin m_oLines;

    cOutputPin m_oIntersectionPoints;

public:
    cSWE_IntersecPointCalc(const tChar* __info);
    virtual ~cSWE_IntersecPointCalc();

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

    // parameter for road width
        double m_roadWidth;
        // parameter for max allowed deviation of boundary from road width (for determination of valid boundary objects)
        double m_maxRoadWidthDeviation;

        // point to which distances to boundaries are calculated (usually front axis)
        cv::Point2d m_referencePoint;

        tUInt32 intersecPointCalc(std::pair<cv::Point2d, cv::Point2d>& intersectionPoints, std::vector<SWE_cBoundary> boundaries);


        double m_intersectionLineAngle;

        double m_intersectionLineDistance;

        double m_distMissingBoundary;

    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescPoints;

};

#endif // _SWE_INTERSECPOINTCALC_H_
