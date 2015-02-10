#ifndef _SWE_KICONTROL_H_
#define _SWE_KICONTROL_H_

#include "stdafx.h"


#define OID_ADTF_SWE_KICONTROL "adtf.swe.kicontrol"

/*!
* Intersection Point Calculator
*/

//Filter Beginn
class cSWE_KIControl : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_KICONTROL, "SWE KIControl", OBJCAT_DataFilter, "KIControl", 1, 0,0, "pre alpha version");

    cInputPin m_oInputMeasured;				// the input pin for the measured value
    cInputPin m_oInputSetPoint;			// the input pin for the set point value

    //MB Input
    cInputPin m_oInputRoadData;
    cInputPin m_oInputObjectData;

    //Was das?V
    cOutputPin m_oIntersectionPointLeft;  //cOutputPin m_oIntersectionPointRight;

    //MB Output
    cOutputPin m_oOutputDriverCourse;



    //cOutputPin m_oOutputManipulated;

public:
    cSWE_KIControl(const tChar* __info);
    virtual ~cSWE_KIControl();

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

    //MB Funktionen die benoetigt werden
    void ObjectAvoidance();
    void DriverCalc();
    //MB Objekte/Variablen die benoetigt werden
    cObjectPtr<IMediaTypeDescription> m_pCoderDescDriverDATA;
    //-------------------------------//



    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescPointLeft;

};

#endif // _SWE_KICONTROL_H_
