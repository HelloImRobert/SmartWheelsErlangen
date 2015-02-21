#ifndef _SWE_COOTRANSCAMTOFRONTAXIS_H_
#define _SWE_COOTRANSCAMTOFRONTAXIS_H_

#include "stdafx.h"


#define OID_ADTF_SWE_COOTRANSCAMTOFRONTAXIS "adtf.swe.cootranscamtofrontaxis"

/*!
* Intersection Point Calculator
*/
class cSWE_CooTransCamToFrontAxis : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_COOTRANSCAMTOFRONTAXIS, "SWE Intersection Point calculation", OBJCAT_DataFilter, "Intersection Point Calculation", 1, 0,0, "pre alpha version");

    // create pins
    cInputPin m_oLines;

    cOutputPin m_oIntersectionPoints;

public:
    cSWE_CooTransCamToFrontAxis(const tChar* __info);
    virtual ~cSWE_CooTransCamToFrontAxis();

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



    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescPoints;

};

#endif // _SWE_COOTRANSCAMTOFRONTAXIS_H_
