#ifndef _SWE_LIGHTCONTROL_H_
#define _SWE_LIGHTCONTROL_H_


#include "stdafx.h"

#define OID_ADTF_SWE_LIGHTCONTROL "adtf.swe.LightControl"

/*!
* Intersection Point Calculator
*/

//Filter Beginn
class cSWE_LightControl : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_LIGHTCONTROL, "SWE LightControl", OBJCAT_DataFilter, "LightControl", 1, 0,0, "pre alpha version");

    //MB Input
    cInputPin m_oInputLightData;

    //MB Output
    cOutputPin m_oOutputheadlight;
    cOutputPin m_oOutputturnleft;
    cOutputPin m_oOutputturnright;
    cOutputPin m_oOutputbrake;
    cOutputPin m_oOutputreverse;


    //cOutputPin m_oOutputManipulated;

public:
    cSWE_LightControl(const tChar* __info);
    virtual ~cSWE_LightControl();

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
    tResult LichtAn(tInt8 value);
    //MB Objekte/Variablen die benoetigt werden

    //-------------------------------//



    /*! Coder Descriptors for the pins*/

    // Coder Descriptors for input pins
    cObjectPtr<IMediaTypeDescription> m_pCoderDescLightData;
    // Coder Descriptors for output pins
    cObjectPtr<IMediaTypeDescription> m_pCoderDescLightOutput;
<<<<<<< HEAD
=======
    cObjectPtr<IMediaTypeDescription> m_pCoderDescLightOutput2;
>>>>>>> c0c2c65b60afe550fc26415f8ce3b68640ba4011

};

#endif // _SWE_LIGHTCONTROL_H_
