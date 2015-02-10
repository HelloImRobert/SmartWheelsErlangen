#ifndef _SWE_DUMMY_H_
#define _SWE_DUMMY_H_

#include "stdafx.h"


#define OID_ADTF_SWE_DUMMY "adtf.swe.dummy"

/*!
* Intersection Point Calculator
*/
class cSWE_Dummy : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_DUMMY, "SWE Dummy", OBJCAT_DataFilter, "Dummy", 1, 0,0, "pre alpha version");

    cInputPin m_oInputMeasured;				// the input pin for the measured value
    cInputPin m_oInputSetPoint;				// the input pin for the set point value
    cOutputPin m_oIntersectionPointLeft;
    //cOutputPin m_oIntersectionPointRight;

    //cOutputPin m_oOutputManipulated;

public:
    cSWE_Dummy(const tChar* __info);
    virtual ~cSWE_Dummy();

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
    cObjectPtr<IMediaTypeDescription> m_pCoderDescPointLeft;

};

#endif // _SWE_DUMMY_H_
