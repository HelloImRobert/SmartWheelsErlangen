#include <cmath>
#include "stdafx.h"
#include "SWE_IntersecPointCalc.h"

#include <iostream>
#include <fstream>


/// Create filter shell
ADTF_FILTER_PLUGIN("SWE_IntersecPointCalc", OID_ADTF_SWE_INTERSECPOINTCALC, cSWE_IntersecPointCalc);


cSWE_IntersecPointCalc::cSWE_IntersecPointCalc(const tChar* __info):cFilter(__info)
{
    n = 0.0;
}

cSWE_IntersecPointCalc::~cSWE_IntersecPointCalc()
{

}

tResult cSWE_IntersecPointCalc::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_IntersecPointCalc::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_IntersecPointCalc::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            // in StageFirst you can create and register your static pins.
            if (eStage == StageFirst)
    {
        // get a media type for the input pin
        RETURN_IF_FAILED(m_oTrigger.Create("Trigger", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oTrigger));

        // create and register output pin with specified data type
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tPoint2d");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tPoint2d", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));
        RETURN_IF_FAILED(m_oIntersectionPointLeft.Create("left_Intersection_Point", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointLeft));

        RETURN_IF_FAILED(m_oIntersectionPointRight.Create("right_Intersection_Point", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointRight));

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

tResult cSWE_IntersecPointCalc::Shutdown(tInitStage eStage, __exception)
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


tResult cSWE_IntersecPointCalc::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &m_oTrigger)
        {
            //-------------- read in values from Pin -------------------------

            //create new media coder
            cObjectPtr<IMediaCoder> pCoder;

            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder)); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            //m_pCoderDescSignal->Lock(pMediaSample, &pCoder); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStampTTT = 0;

            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&(value));
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&(timeStampTTT));
            m_pCoderDescSignal->Unlock(pCoder);

            std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe4.txt");
            file << value << endl
                 << timeStampTTT << endl;

            //-------------- do caculations and assign output -------------------------

            CalcIntersecPoints();

            //-------------- write output to output pin -------------------------

            // create new media samples
            cObjectPtr<IMediaSample> pMediaSampleLeftPoint;
            cObjectPtr<IMediaSample> pMediaSampleRightPoint;
            AllocMediaSample((tVoid**)&pMediaSampleLeftPoint);
            AllocMediaSample((tVoid**)&pMediaSampleRightPoint);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleLeftPoint->AllocBuffer(nSize);
            pMediaSampleRightPoint->AllocBuffer(nSize);

            cObjectPtr<IMediaCoder> pCoderOutput;
            //write date to the media sample with the coder of the descriptor
            m_pCoderDescSignal->WriteLock(pMediaSampleLeftPoint, &pCoderOutput);
            pCoder->Set("ui32Point2dTimestamp", 0);//(tVoid*)&timeStamp);
            pCoder->Set("xCoord", (tVoid*)&(m_IntersectionPointLeft.x));
            pCoder->Set("yCoord", (tVoid*)&(m_IntersectionPointLeft.y));
            m_pCoderDescSignal->Unlock(pCoderOutput);
            //transmit media sample over output pin
            RETURN_IF_FAILED(pMediaSampleLeftPoint->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPointLeft.Transmit(pMediaSampleLeftPoint));

            //write date to the media sample with the coder of the descriptor
            m_pCoderDescSignal->WriteLock(pMediaSampleRightPoint, &pCoderOutput);
            pCoder->Set("ui32Point2dTimestamp", 0);//(tVoid*)&timeStamp);
            pCoder->Set("xCoord", (tVoid*)&(m_IntersectionPointRight.x));
            pCoder->Set("yCoord", (tVoid*)&(m_IntersectionPointRight.y));
            m_pCoderDescSignal->Unlock(pCoderOutput);
            //transmit media sample over output pin
            RETURN_IF_FAILED(pMediaSampleRightPoint->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPointRight.Transmit(pMediaSampleRightPoint));
        }
        /*
        else if (pSource == &m_oInputSetPoint)
        {
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;

            //get values from media sample
            pCoder->Get("f32Value", (tVoid*)&value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoder);


            // ----------------- We broke it here --------------------
            //un-pwm it to mm/s
            value = value - 90.0;

            if ( value < -50.0 ) // prevent stupid values
            {
                value = -50.0;
            }
            else if ( value > 50.0 )
            {
                value = 50.0;
            }

            value = value * 20;	//full throttle + 1000mm/s

            m_setPoint = value;
        } */
        else
            RETURN_NOERROR;
        // -------------------------------------------------------------------

    }

    RETURN_NOERROR;
}


// implementing custom functionality ///////////////////////////

tResult cSWE_IntersecPointCalc::CalcIntersecPoints()
{
    //////////////////////////////////////////////////////////////////// TO BE DELETED

    n++;
    m_IntersectionPointLeft.x = 500.66;
    m_IntersectionPointLeft.y = -200.66-n;
    m_IntersectionPointRight.x = 500.66;
    m_IntersectionPointRight.y = 200.66-n;

    std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe2.txt");
    file << m_IntersectionPointLeft.x << endl
         << m_IntersectionPointLeft.y << endl
         << m_IntersectionPointRight.x << endl
         << m_IntersectionPointRight.y << endl;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////


    RETURN_NOERROR;
}
