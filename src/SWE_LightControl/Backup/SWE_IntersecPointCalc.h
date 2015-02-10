#ifndef _SWE_INTERSECPOINTCALC_H_
#define _SWE_INTERSECPOINTCALC_H_

#include "stdafx.h"

#define OID_ADTF_SWE_INTERSECPOINTCALC "adtf.swe.intersecpointcalc"

/*!
* motor speed controller
*/
class cSWE_IntersecPointCalc : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_INTERSECPOINTCALC, "SWE Intersection Point calculation", OBJCAT_DataFilter, "Intersection Point Calculation", 1, 0,0, "pre alpha version");

		cInputPin m_oInputMeasured;				// the input pin for the measured value
		cInputPin m_oInputSetPoint;				// the input pin for the set point value
        cOutputPin m_oIntersectionPointLeft;
        //cOutputPin m_oIntersectionPointRight;

        //cOutputPin m_oOutputManipulated;

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


        tFloat32 m_measuredVariable;

        cv::Point2d m_IntersectionPointLeft;
        //cv::Point2d m_IntersectionPointRight;

        tResult CalcIntersecPoints();
		/*! returns the currentstreamtime*/
		tTimeStamp GetTime();

        /*! Coder Descriptors for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescPointLeft;
	
};

#endif // _SWE_INTERSECPOINTCALC_H_
