#ifndef _SWE_INTERSECPOINTCALC_H_
#define _SWE_INTERSECPOINTCALC_H_

#include "stdafx.h"


#define OID_ADTF_SWE_INTERSECPOINTCALC "adtf.swe.intersecpointcalc"


//*************************************************************************************************
class cSWE_IntersecPointCalc : public adtf::cFilter
{
    //ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, "SWE_IntersecPointCalc", adtf::OBJCAT_DataFilter);
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_INTERSECPOINTCALC, "SWE Intersection Point calculation", OBJCAT_DataFilter, "Intersection Point Calculation", 1, 0,0, "pre alpha version");

private:
    cOutputPin    m_oIntersectionPointLeft;
    cOutputPin    m_oIntersectionPointRight;
    cInputPin     m_oTrigger;

public:
    cSWE_IntersecPointCalc(const tChar* __info);
    virtual ~cSWE_IntersecPointCalc();

protected:
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    // implementation of functionality
    tResult CalcIntersecPoints();

private:
    // holds last calculated intersection points
    cv::Point2d m_IntersectionPointLeft;
    cv::Point2d m_IntersectionPointRight;

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;

    tFloat64 n;
};

//*************************************************************************************************
#endif // _SWE_INTERSECPOINTCALC_H_
