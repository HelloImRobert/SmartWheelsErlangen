#include <cmath>
#include "stdafx.h"
#include "SWE_TrackControl.h"


#include <iostream>
#include <fstream>




ADTF_FILTER_PLUGIN("SWE TrackControl", OID_ADTF_SWE_TRACKCONTROL, cSWE_TrackControl)

cSWE_TrackControl::cSWE_TrackControl(const tChar* __info) : cFilter(__info), m_PerpenticularPoint(1.0, 0.0)
{
    m_steeringAngle = 0.0;

    SetPropertyFloat("Reference Point x-Coord",0.0);
    SetPropertyFloat("Reference Point y-Coord",0.0);
    SetPropertyFloat("Intersection Line Distance",200.0);
    SetPropertyFloat("Intersection Line Angle",0.0);
    SetPropertyFloat("Road Width",500.0);
    SetPropertyFloat("max Road Width Deviation",100);
    SetPropertyFloat("Distance Missing Boundary",500);

    //SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID");
}

cSWE_TrackControl::~cSWE_TrackControl()
{
}

tResult cSWE_TrackControl::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oIntersectionPoints.Create("Intersection_Points", new cMediaType(0, 0, 0, "tIntersections"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));
    RETURN_NOERROR;
}

tResult cSWE_TrackControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Struct for Intersection Point Transmission
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSteeringAngle);
    cObjectPtr<IMediaType> pTypeSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strDescSteeringAngle,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSteeringAngle));

    RETURN_IF_FAILED(m_oSteeringAngle.Create("Steering_Angle", pTypeSteeringAngle, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngle));

    // Struct for Intersection Point Transmission
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescMiddlePoint = pDescManager->GetMediaDescription("tPoint2d");
    RETURN_IF_POINTER_NULL(strDescMiddlePoint);
    cObjectPtr<IMediaType> pTypeMiddlePoint = new cMediaType(0, 0, 0, "tPoint2d", strDescMiddlePoint,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMiddlePoint->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMiddlePoint));

    RETURN_IF_FAILED(m_oMiddlePoint.Create("Middle_Point", pTypeMiddlePoint, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oMiddlePoint));

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {

        //m_referencePoint.x = GetPropertyFloat("Reference Point x-Coord");


    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_TrackControl::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_TrackControl::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_TrackControl::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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

        if (pSource == &m_oIntersectionPoints)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // init temporary objects
            cv::Point2d leftIntersectionPoint;
            cv::Point2d rightIntersectionPoint;
            tUInt32 intersectionIndicator;

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("xCoordLeft", (tVoid*)&(leftIntersectionPoint.x));
            pCoder->Get("yCoordLeft", (tVoid*)&(leftIntersectionPoint.y));
            pCoder->Get("xCoordRight", (tVoid*)&(rightIntersectionPoint.x));
            pCoder->Get("yCoordRight", (tVoid*)&(rightIntersectionPoint.y));
            pCoder->Get("intersecIndicator", (tVoid*)&(intersectionIndicator));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // CALCUALTIONS -------------------------------------------------------------------

            //leftIntersectionPoint.x = 200; leftIntersectionPoint.y = 100;
            //rightIntersectionPoint.x = 200; rightIntersectionPoint.y = -350;

            tFloat32 steeringAngle = -180.0/3.14*(CalcSteeringAngle( leftIntersectionPoint, rightIntersectionPoint, intersectionIndicator ));


            // TRANSMIT OUTPUT VALUES -------------------------------------------------------------------


            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleOutput;
            RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

            //allocate memory with the size given by the descriptor
            // ADAPT: m_pCoderDescPointLeft
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSteeringAngle->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOutput->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            // ADAPT: m_pCoderDescPointLeft
            //cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSteeringAngle->WriteLock(pMediaSampleOutput, &pCoder));
            pCoder->Set("f32Value", (tVoid*)&(steeringAngle));
            m_pCoderDescSteeringAngle->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oSteeringAngle.Transmit(pMediaSampleOutput));

            //std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe4.txt");
            //file << lineBoundaries[0] << endl << lineBoundaries[1] << endl << lineBoundaries[2] << endl << lineBoundaries[3] << endl;
            //file.close();



            //allocate memory with the size given by the descriptor
            // ADAPT: m_pCoderDescPointLeft
            //cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescMiddlePoint->GetMediaSampleSerializer(&pSerializer);
            nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOutput->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            // ADAPT: m_pCoderDescPointLeft, steering Angle
            //cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescMiddlePoint->WriteLock(pMediaSampleOutput, &pCoder));
            pCoder->Set("xCoord", (tVoid*)&(m_middlePoint.x));
            pCoder->Set("yCoord", (tVoid*)&(m_middlePoint.y));
            m_pCoderDescMiddlePoint->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oMiddlePoint.Transmit(pMediaSampleOutput));

        }
        else
            RETURN_NOERROR;
        // -------------------------------------------------------------------

    }
    RETURN_NOERROR;
}

tFloat64 cSWE_TrackControl::CalcSteeringAngle( cv::Point2d leftIntersectionPoint, cv::Point2d rightIntersectionPoint, tUInt32 intersectionIndicator )
{
    tFloat64 steeringAngle = 0;

    if( intersectionIndicator != 0 )
    {
        m_middlePoint = (leftIntersectionPoint + rightIntersectionPoint)*0.5;
        steeringAngle = acos(m_middlePoint.dot(m_PerpenticularPoint)/cv::norm(m_middlePoint));
        if(m_middlePoint.y < 0)
        //cv::Point2d middlePoint = (leftIntersectionPoint + rightIntersectionPoint)*0.5;
        //steeringAngle = acos(middlePoint.dot(m_PerpenticularPoint)/cv::norm(middlePoint));
        //if(middlePoint.y < 0)
        {
            steeringAngle = (-1.0)*steeringAngle;
        }
        m_steeringAngle = steeringAngle;
    }
    else
    {
        steeringAngle = m_steeringAngle;
    }

    return steeringAngle;
}
