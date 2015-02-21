#include <cmath>
#include "stdafx.h"
#include "SWE_CooTransCamToFrontAxis.h"


#include <iostream>
#include <fstream>




ADTF_FILTER_PLUGIN("SWE Intersection Point calculation", OID_ADTF_SWE_COOTRANSCAMTOFRONTAXIS, cSWE_CooTransCamToFrontAxis)

cSWE_CooTransCamToFrontAxis::cSWE_CooTransCamToFrontAxis(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("Reference Point x-Coord",0.0);
    SetPropertyFloat("Reference Point y-Coord",0.0);
    SetPropertyFloat("Intersection Line Distance",300.0);
    SetPropertyFloat("Intersection Line Angle",0.0);
    SetPropertyFloat("Road Width",900.0);
    SetPropertyFloat("max Road Width Deviation",100);
    SetPropertyFloat("Distance Missing Boundary",490);

    //SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID");
}

cSWE_CooTransCamToFrontAxis::~cSWE_CooTransCamToFrontAxis()
{
}

tResult cSWE_CooTransCamToFrontAxis::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oLines.Create("Line_Boundaries", new cMediaType(0, 0, 0, "tLineBoundaries"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oLines));
    RETURN_NOERROR;
}

tResult cSWE_CooTransCamToFrontAxis::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Struct for Intersection Point Transmission
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescIntersecPoints = pDescManager->GetMediaDescription("tIntersections");
    RETURN_IF_POINTER_NULL(strDescIntersecPoints);
    cObjectPtr<IMediaType> pTypeIntersecPoints = new cMediaType(0, 0, 0, "tIntersections", strDescIntersecPoints,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeIntersecPoints->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPoints));

    RETURN_IF_FAILED(m_oIntersectionPoints.Create("Intersection_Points", pTypeIntersecPoints, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));

    RETURN_NOERROR;
}

tResult cSWE_CooTransCamToFrontAxis::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {

//        m_referencePoint.x = GetPropertyFloat("Reference Point x-Coord");
//        m_referencePoint.y = GetPropertyFloat("Reference Point y-Coord");
//        m_intersectionLineDistance = GetPropertyFloat("Intersection Line Distance");
//        m_intersectionLineAngle = GetPropertyFloat("Intersection Line Angle");
//        m_roadWidth = GetPropertyFloat("Road Width");
//        m_maxRoadWidthDeviation = GetPropertyFloat("max Road Width Deviation");
//        m_distMissingBoundary = GetPropertyFloat("Distance Missing Boundary");

    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_CooTransCamToFrontAxis::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_CooTransCamToFrontAxis::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_CooTransCamToFrontAxis::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_CooTransCamToFrontAxis::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
    // ADAPT: pMediaTypeDescInputMeasured, m_pCoderDescInputMeasured !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // NOCH SO BAUEN, DASS IN FKT CREATE_INPUT_PINS EINGEFUEGT WERDEN KANN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);
    if (pType != NULL)
    {
        cObjectPtr<IMediaTypeDescription> pMediaTypeDescInputMeasured;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescInputMeasured));
        m_pCoderDescInputMeasured = pMediaTypeDescInputMeasured;
    }

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescInputMeasured != NULL)
    {
        RETURN_IF_POINTER_NULL( pMediaSample);

        if (pSource == &m_oLines)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // init temporary objects
            cv::Point2d leftFront;
            cv::Point2d leftRear;
            cv::Point2d rightFront;
            cv::Point2d rightRear;

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("leftFrontX", (tVoid*)&(leftFront.x));
            pCoder->Get("leftFrontY", (tVoid*)&(leftFront.y));
            pCoder->Get("leftRearX", (tVoid*)&(leftRear.x));
            pCoder->Get("leftRearY", (tVoid*)&(leftRear.y));
            pCoder->Get("rightFrontX", (tVoid*)&(rightFront.x));
            pCoder->Get("rightFrontY", (tVoid*)&(rightFront.y));
            pCoder->Get("rightRearX", (tVoid*)&(rightRear.x));
            pCoder->Get("rightRearY", (tVoid*)&(rightRear.y));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // CALCUALTIONS -------------------------------------------------------------------

            // calculate intersection points with boundaries



            // TRANSMIT OUTPUT VALUES -------------------------------------------------------------------


            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleOutput;
            RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

            //allocate memory with the size given by the descriptor
            // ADAPT: m_pCoderDescPointLeft
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescPoints->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOutput->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            // ADAPT: m_pCoderDescPointLeft
            //cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescPoints->WriteLock(pMediaSampleOutput, &pCoder));
//            pCoder->Set("xCoordLeft", (tVoid*)&(intersectionPoints.first.x));
            m_pCoderDescPoints->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPoints.Transmit(pMediaSampleOutput));

            //std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe4.txt");
            //file << lineBoundaries[0] << endl << lineBoundaries[1] << endl << lineBoundaries[2] << endl << lineBoundaries[3] << endl;
            //file.close();

        }
        else
            RETURN_NOERROR;
        // -------------------------------------------------------------------

    }
    RETURN_NOERROR;
}
