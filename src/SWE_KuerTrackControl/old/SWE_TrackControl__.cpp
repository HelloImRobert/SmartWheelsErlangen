#include <cmath>
#include "stdafx.h"
#include "SWE_TrackControl.h"
#include <template_data.h>

#include <iostream>
#include <fstream>

extern tFloat64 environmentRep::testVal1;


/// Create filter shell
ADTF_FILTER_PLUGIN("SWE_TrackControl", OID_ADTF_TEMPLATE_FILTER, cSWE_TrackControl);


cSWE_TrackControl::cSWE_TrackControl(const tChar* __info):cFilter(__info), m_PerpenticularPoint(1.0, 0.0)
{
    m_IntersectionPointLeft.x = 500.0;
    m_IntersectionPointLeft.y = -1000.0;
    m_IntersectionPointRight.x = 500.0;
    m_IntersectionPointRight.y = 200.0;
}

cSWE_TrackControl::~cSWE_TrackControl()
{

}

tResult cSWE_TrackControl::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            // in StageFirst you can create and register your static pins.
            if (eStage == StageFirst)
    {
        //TODO:

        // get a media type for the input pin
        RETURN_IF_FAILED(m_oIntersectionPointLeft.Create("left_Intersection_Point", new cMediaType(0, 0, 0, "tPoint2d"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointLeft));
        
        // create and register the input pin
        RETURN_IF_FAILED(m_oIntersectionPointRight.Create("right_Intersection_Point", new cMediaType(0, 0, 0, "tPoint2d"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointRight));

        // create and register the input pin
        RETURN_IF_FAILED(m_oIntersectionPoints.Create("Intersection_Points", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));

        // create and register output pin with specified data type
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

        RETURN_IF_FAILED(m_oSteeringAngle.Create("Steering_Angle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngle));

        RETURN_NOERROR;

    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.



    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cSWE_TrackControl::OnPinEvent(IPin* pSource,
                                      tInt nEventCode,
                                      tInt nParam1,
                                      tInt nParam2,
                                      IMediaSample* pMediaSample)
{
    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
    // ADAPT: pMediaTypeDescInputMeasured, m_pCoderDescInputMeasured !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // NOCH SO BAUEN, DASS IN FKT CREATE_INPUT_PINS EINGEFUEGT WERDEN KANN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);
    if (pType != NULL)
    {
        cObjectPtr<IMediaTypeDescription> pMediaTypeDescInput;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescInput));
        m_pCoderDescInput = pMediaTypeDescInput;
    }
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &m_oIntersectionPointLeft)
        {
            //-------------- read in values from Pin -------------------------
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            /* use if deleting members for point coords/////////////////////////////////////////////////////////
            //write values with zero
            tFloat64 xCoord = 0;
            tFloat64 yCoord = 0;
            */
            tUInt32 Point1_timeStamp = 0;
            tFloat64 Point1X = 8.88;
            tFloat64 Point1Y = 8.88;


            //get values from media sample
            //pCoderInput->Get("x", (tVoid*)&(m_IntersectionPointLeft.x));
            //pCoderInput->Get("y", (tVoid*)&(m_IntersectionPointLeft.y));
            pCoderInput->Get("xCoord", (tVoid*)&Point1X);
            pCoderInput->Get("yCoord", (tVoid*)&Point1Y);
            pCoderInput->Get("ui32Point2dTimestamp", (tVoid*)&Point1_timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            //-------------- do caculations and assign output value -------------------------

            RETURN_IF_FAILED(CalcSteeringAngle());


            //-------------- write output to output pin -------------------------

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSample;
            AllocMediaSample((tVoid**)&pMediaSample);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSample->AllocBuffer(nSize);

            cObjectPtr<IMediaCoder> pCoderOutput;

            //write date to the media sample with the coder of the descriptor
            m_pCoderDescSignal->WriteLock(pMediaSample, &pCoderOutput);

            pCoderOutput->Set("f32Value", (tVoid*)&(m_SteeringAngle));
            pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&Point1_timeStamp);
            m_pCoderDescSignal->Unlock(pCoderOutput);

            //transmit media sample over output pin
            RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oSteeringAngle.Transmit(pMediaSample));
        }
        else if (pSource == &m_oIntersectionPointRight)
        {
            //-------------- read in values from Pin -------------------------
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            /* use if deleting members for point coords/////////////////////////////////////////////////////////
            //write values with zero
            tFloat64 xCoord = 0;
            tFloat64 yCoord = 0;
            */
            tUInt32 Point2_timeStamp = 0;


            //get values from media sample
            pCoderInput->Get("xCoord", (tVoid*)&m_IntersectionPointRight.x);
            pCoderInput->Get("yCoord", (tVoid*)&m_IntersectionPointRight.y);
            pCoderInput->Get("ui32Point2dTimestamp", (tVoid*)&Point2_timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);


            //-------------- do caculations and assign output -------------------------

            RETURN_IF_FAILED(CalcSteeringAngle());


            //-------------- write output to output pin -------------------------

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSample;
            AllocMediaSample((tVoid**)&pMediaSample);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSample->AllocBuffer(nSize);

            cObjectPtr<IMediaCoder> pCoderOutput;

            //write date to the media sample with the coder of the descriptor
            m_pCoderDescSignal->WriteLock(pMediaSample, &pCoderOutput);

            pCoderOutput->Set("f32Value", (tVoid*)&(m_SteeringAngle));
            pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&Point2_timeStamp);
            m_pCoderDescSignal->Unlock(pCoderOutput);

            //transmit media sample over output pin
            RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oSteeringAngle.Transmit(pMediaSample));
        }
        else if (pSource == &m_oIntersectionPoints)
        {
            //-------------- read in values from Pin -------------------------
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescInput->Lock(pMediaSample, &pCoderInput));

            /* use if deleting members for point coords/////////////////////////////////////////////////////////
            //write values with zero
            tFloat64 xCoord = 0;
            tFloat64 yCoord = 0;
            */
            tUInt32 Point2_timeStamp = 0;
            tFloat64 input = 0.00;


            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&input);

            m_pCoderDescInput->Unlock(pCoderInput);


            //-------------- do caculations and assign output -------------------------

            RETURN_IF_FAILED(CalcSteeringAngle());


            //-------------- write output to output pin -------------------------

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSample;
            AllocMediaSample((tVoid**)&pMediaSample);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescInput->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSample->AllocBuffer(nSize);

            cObjectPtr<IMediaCoder> pCoderOutput;

            //write date to the media sample with the coder of the descriptor
            m_pCoderDescInput->WriteLock(pMediaSample, &pCoderOutput);

            pCoderOutput->Set("f32Value", (tVoid*)&(input));
            //pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&Point2_timeStamp);
            m_pCoderDescInput->Unlock(pCoderOutput);

            //transmit media sample over output pin
            RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oSteeringAngle.Transmit(pMediaSample));
        }
        else
            RETURN_NOERROR;
        // -------------------------------------------------------------------

    }

    RETURN_NOERROR;
}


// implementing custom functionality ///////////////////////////

tResult cSWE_TrackControl::CalcSteeringAngle()
{
    m_MiddlePoint = (m_IntersectionPointLeft + m_IntersectionPointRight)*0.5;
    m_SteeringAngle = acos(m_MiddlePoint.dot(m_PerpenticularPoint)/cv::norm(m_MiddlePoint));

    //////////////////////////////////////////////////////////////////// TO BE DELETED

    std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe.txt");
    file << m_IntersectionPointLeft.x << endl
         << m_IntersectionPointLeft.y << endl
         << m_MiddlePoint.x << endl
         << m_MiddlePoint.y << endl
         << m_SteeringAngle;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////


    RETURN_NOERROR;
}
