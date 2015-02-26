//#ifndef _SWE_PARKPILOT_H_
//#define _SWE_PARKPILOT_H_

//#include "stdafx.h"


//#define OID_ADTF_SWE_PARKPILOT "adtf.swe.parkpilot"

///*!
//* Intersection Point Calculator
//*/
//class cSWE_ParkPilot : public adtf::cFilter
//{
//    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_PARKPILOT, "SWE ParkPilot", OBJCAT_DataFilter, "ParkPilot", 1, 0,0, "pre alpha version");

//    cInputPin m_inputParkTrigger;
//    cInputPin m_ObjectData;

//    //cOutputPin m_oOutputManipulated;

//public:
//    cSWE_ParkPilot(const tChar* __info);
//    virtual ~cSWE_ParkPilot();

//protected: // overwrites cFilter
//    tResult Init(tInitStage eStage, __exception = NULL);
//    tResult Start(__exception = NULL);
//    tResult Stop(__exception = NULL);
//    tResult Shutdown(tInitStage eStage, __exception = NULL);
//    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);




//private:

//    tFloat32 m_IRFrontRight;
//    tFloat32 m_IRRearRight;

//    /*! creates all the input Pins*/
//    tResult CreateInputPins(__exception = NULL);
//    /*! creates all the output Pins*/
//    tResult CreateOutputPins(__exception = NULL);

//    // Park functions
//    tResult searchLot();


//    // Media Descriptors
//    /*! Coder Descriptors for the pins*/
//    cObjectPtr<IMediaTypeDescription> m_pCoderDesc;
//    cObjectPtr<IMediaTypeDescription> m_pCoderDescPointsIn;
//};

//#endif // _SWE_PARKPILOT_H_
