#include <cmath>
#include "stdafx.h"
#include "SWE_IntersecPointCalc.h"

#include <iostream>
#include <fstream>

ADTF_FILTER_PLUGIN("SWE Intersection Point calculation", OID_ADTF_SWE_INTERSECPOINTCALC, cSWE_IntersecPointCalc)

cSWE_IntersecPointCalc::cSWE_IntersecPointCalc(const tChar* __info) : cFilter(__info), m_measuredVariable(0), m_IntersectionPointLeft(8,9)
{
	SetPropertyFloat("Controller Kp value",1);
	SetPropertyFloat("Controller Ki value",1);
	SetPropertyFloat("Controller Kd value",1);
	SetPropertyFloat("Controller Precontrol Value", 1);

	SetPropertyInt("Sample Interval [msec]",1);
	SetPropertyBool("use automatically calculated sample interval",1);
	SetPropertyInt("Controller Typ", 1);
	SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID");
}

cSWE_IntersecPointCalc::~cSWE_IntersecPointCalc()
{
}

tResult cSWE_IntersecPointCalc::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(m_oInputMeasured.Create("measured variable", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInputMeasured));
	RETURN_IF_FAILED(m_oInputSetPoint.Create("set point", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInputSetPoint));
	RETURN_NOERROR;
}

tResult cSWE_IntersecPointCalc::CreateOutputPins(__exception)
{
	cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));



    // Left Intersection Point
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescPointLeft = pDescManager->GetMediaDescription("tPoint2d");
    RETURN_IF_POINTER_NULL(strDescPointLeft);
    cObjectPtr<IMediaType> pTypePointLeft = new cMediaType(0, 0, 0, "tPoint2d", strDescPointLeft,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypePointLeft->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPointLeft));

    RETURN_IF_FAILED(m_oIntersectionPointLeft.Create("left_Intersection_Point", pTypePointLeft, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointLeft));


/*
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tPoint2d");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tPoint2d", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescInputMeasured));
	
    //RETURN_IF_FAILED(m_oOutputManipulated.Create("manipulated variable", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    //RETURN_IF_FAILED(RegisterPin(&m_oOutputManipulated));

    RETURN_IF_FAILED(m_oIntersectionPointLeft.Create("left_Intersection_Point", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointLeft));

    //RETURN_IF_FAILED(m_oIntersectionPointRight.Create("right_Intersection_Point", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    //RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPointRight));
*/


    RETURN_NOERROR;
}

tResult cSWE_IntersecPointCalc::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
		{
				CreateInputPins(__exception_ptr);
				CreateOutputPins(__exception_ptr);
		}
    else if (eStage == StageNormal)
		{

		}
	else if(eStage == StageGraphReady)
		{	
			
		}
				
    RETURN_NOERROR;
}

tResult cSWE_IntersecPointCalc::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult cSWE_IntersecPointCalc::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_IntersecPointCalc::Shutdown(tInitStage eStage, __exception)
{
	return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_IntersecPointCalc::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
        std::ofstream file1("/home/odroid/Desktop/Ausgabe/ausgabe5.txt");
                    file1 << m_IntersectionPointLeft.x << endl
                         << m_IntersectionPointLeft.y << endl;
                    file1.close();

		RETURN_IF_POINTER_NULL( pMediaSample);
		
		if (pSource == &m_oInputMeasured)
		{
            tFloat32 outputValue = 0.0;
            std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe4.txt");
                        file << outputValue << endl
                             << m_IntersectionPointLeft.y << endl;
                        file.close();
                cObjectPtr<IMediaCoder> pCoder;
                RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));
                

                //get values from media sample        
                pCoder->Get("f32Value", (tVoid*)&(outputValue));
                //pCoder->Get("f32Value", (tVoid*)&(m_IntersectionPointLeft.y));
                //pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescInputMeasured->Unlock(pCoder);

                //-------------- do caculations and assign output -------------------------



                CalcIntersecPoints();

                //-------------- write output to output pin -------------------------
				
                //create new media sample
                cObjectPtr<IMediaSample> pMediaSample;
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

                //allocate memory with the size given by the descriptor
                // ADAPT: m_pCoderDescPointLeft
                cObjectPtr<IMediaSerializer> pSerializer;
                m_pCoderDescPointLeft->GetMediaSampleSerializer(&pSerializer);
                tInt nSize = pSerializer->GetDeserializedSize();
                pMediaSample->AllocBuffer(nSize);

                //write date to the media sample with the coder of the descriptor
                // ADAPT: m_pCoderDescPointLeft
                //cObjectPtr<IMediaCoder> pCoder;
                RETURN_IF_FAILED(m_pCoderDescPointLeft->WriteLock(pMediaSample, &pCoder));

                //pCoder->Set("f32Value", (tVoid*)&(outputValue));

                //pCoderOutput->Set("ui32Point2dTimestamp", 7);
                pCoder->Set("xCoord", (tVoid*)&(m_IntersectionPointLeft.x));
                pCoder->Set("yCoord", (tVoid*)&(m_IntersectionPointLeft.y));
                m_pCoderDescPointLeft->Unlock(pCoder);

                //transmit media sample over output pin
                // ADAPT: m_oIntersectionPointLeft
                RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
                RETURN_IF_FAILED(m_oIntersectionPointLeft.Transmit(pMediaSample));

                //RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
                //RETURN_IF_FAILED(m_oOutputManipulated.Transmit(pMediaSample));
				
		}
		else if (pSource == &m_oInputSetPoint)
        {
			cObjectPtr<IMediaCoder> pCoder;
                    RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            	//write values with zero
            	tFloat32 value = 0;
            	tUInt32 timeStamp = 0;
            
            	//get values from media sample        
            	pCoder->Get("f32Value", (tVoid*)&value);
           	pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescInputMeasured->Unlock(pCoder);

		}
		else
			RETURN_NOERROR;
// -------------------------------------------------------------------		

	}
	RETURN_NOERROR;
}


tResult cSWE_IntersecPointCalc::CalcIntersecPoints()
{


    RETURN_NOERROR;
}

tTimeStamp cSWE_IntersecPointCalc::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}
