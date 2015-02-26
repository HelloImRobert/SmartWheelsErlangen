//#include <cmath>
//#include "stdafx.h"
//#include "SWE_ParkPilot.h"

//#include <iostream>
//#include <fstream>

//#define POS_IR_SIDE_RIGHT -150.0      //y-Value

//ADTF_FILTER_PLUGIN("SWE ParkPilot", OID_ADTF_SWE_PARKPILOT, cSWE_ParkPilot)

//cSWE_ParkPilot::cSWE_ParkPilot(const tChar* __info) : cFilter(__info)
//{


//}

//cSWE_ParkPilot::~cSWE_ParkPilot()
//{
//}

//tResult cSWE_ParkPilot::CreateInputPins(__exception)
//{
//    RETURN_IF_FAILED(m_inputParkTrigger.Create("Park Trigger", new cMediaType(0, 0, 0, "tSBool"), static_cast<IPinEventSink*> (this)));
//    RETURN_IF_FAILED(RegisterPin(&m_inputParkTrigger));
//    RETURN_NOERROR;
//}

//tResult cSWE_ParkPilot::CreateOutputPins(__exception)
//{
////    cObjectPtr<IMediaDescriptionManager> pDescManager;
////    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));



////    // Left Intersection Point
////    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
////    tChar const * strDescPointLeft = pDescManager->GetMediaDescription("tPoint2d");
////    RETURN_IF_POINTER_NULL(strDescPointLeft);
////    cObjectPtr<IMediaType> pTypePointLeft = new cMediaType(0, 0, 0, "tPoint2d", strDescPointLeft,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
////    RETURN_IF_FAILED(pTypePointLeft->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPointLeft));

////    RETURN_IF_FAILED(m_oIntersectionPointLeft.Create("left_Intersection_Point", pTypePointLeft, static_cast<IPinEventSink*> (this)));
////    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointLeft));


//      RETURN_NOERROR;
//}

//tResult cSWE_ParkPilot::Init(tInitStage eStage, __exception)
//{
//    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

//    if (eStage == StageFirst)
//    {
//        CreateInputPins(__exception_ptr);
//        CreateOutputPins(__exception_ptr);
//    }
//    else if (eStage == StageNormal)
//    {

//    }
//    else if(eStage == StageGraphReady)
//    {

//    }

//    RETURN_NOERROR;
//}

//tResult cSWE_ParkPilot::Start(__exception)
//{
//    return cFilter::Start(__exception_ptr);
//}

//tResult cSWE_ParkPilot::Stop(__exception)
//{
//    return cFilter::Stop(__exception_ptr);
//}

//tResult cSWE_ParkPilot::Shutdown(tInitStage eStage, __exception)
//{
//    return cFilter::Shutdown(eStage,__exception_ptr);
//}

//tResult cSWE_ParkPilot::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
//{
//    cObjectPtr<IMediaType> pType;
//    pSource->GetMediaType(&pType);


////    if (pType != NULL)
////    {
////        cObjectPtr<IMediaTypeDescription> pMediaTypeDescParkBool;
////        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescParkBool));
////        m_pCoderDescParkBool = pMediaTypeDescParkBool;

////    }

//    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDesc != NULL)
//    {

//        RETURN_IF_POINTER_NULL( pMediaSample);

//        if(pSource == &m_inputParkTrigger)
//        {
//           cObjectPtr<IMediaCoder> pCoder;
//           RETURN_IF_FAILED(m_pCoderDesc->Lock(pMediaSample, &pCoder));
//           tBool goPark = false;
//           pCoder->Get("tBool", (tVoid*)&goPark);
//           m_pCoderDesc->Unlock(pCoder);

//           if( goPark == true )
//           {
//               searchLot();
//           }

//        }
//        else if(pSource == &m_ObjectData)
//        {
//            cv::Point2d objectData[10];
//            cObjectPtr<IMediaCoder> pCoder;
//            RETURN_IF_FAILED(m_pCoderDesc->Lock(pMediaSample, &pCoder));
//            pCoder->Get("tBool", (tVoid*)&objectData);
//            m_pCoderDesc->Unlock(pCoder);

//            m_IRFrontRight = (-1) * (objectData[1].y - POS_IR_SIDE_RIGHT);
//            m_IRRearRight = (-1) * (objectData[4].y - POS_IR_SIDE_RIGHT);
//        }



//    }
//    RETURN_NOERROR;
//}

//tResult cSWE_ParkPilot::searchLot()
//{

//    // Go forward
//    // Evaluate IR Front Left trough thresholding


//    RETURN_NOERROR;
//}


///*
//computes intersetion points and indicator for steering angle calculation
//indicator values:
//1: two intersection points found
//0: less than two intersection points found
//else: computation error
//*/
