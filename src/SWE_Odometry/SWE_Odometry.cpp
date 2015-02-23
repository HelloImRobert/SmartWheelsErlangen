#include "stdafx.h"
#include "SWE_Odometry.h"

#define WHEELBASE 400

ADTF_FILTER_PLUGIN("SWE_Odometry", OID_ADTF_SWE_ODOMETRY, SWE_Odometry)

SWE_Odometry::SWE_Odometry(const tChar* __info) : cFilter(__info)
{
	m_steeringAngle = 0;
	m_slippageAngle = 0;
	m_velocityLeft = 0;
	m_velocityRight = 0;
	
	m_currTimeStamp = 0;
	m_oldTimeStamp = GetTime();
	m_lastPinEvent = GetTime();
    m_lastTriggerTime = GetTime();

	m_distanceX_sum = 0;
	m_distanceY_sum = 0;
	m_heading_sum = 0;

	//odometryData m_odometryData;
	
    m_odometryData.distance_x = 0.0;
    m_odometryData.distance_y = 0.0;
    m_odometryData.angle_heading = 0.0;
    m_odometryData.velocity = 0.0;
    m_odometryData.distance_sum = 0.0;

    SetPropertyFloat("Velocity Filter Strength",0.5);



}

SWE_Odometry::~SWE_Odometry()
{
}

tResult SWE_Odometry::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
		{
            cObjectPtr<IMediaDescriptionManager> pDescManager;
            RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


            // ----------------- create input pins ------------------

            tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
            RETURN_IF_POINTER_NULL(strDescSignalValue);        
            cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
	


            RETURN_IF_FAILED(m_oInputWheelRight.Create("Sensor_left_Wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));
            RETURN_IF_FAILED(m_oInputWheelLeft.Create("Sensor_right_Wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));
			RETURN_IF_FAILED(m_oInputSteeringAngle.Create("SteeringAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oInputSteeringAngle));


            // ------------------ create output pins ---------------------
            //TODO: create applicable media description
            tChar const * strDescOdometry = pDescManager->GetMediaDescription("tOdometry");
            RETURN_IF_POINTER_NULL(strDescOdometry);
            cObjectPtr<IMediaType> pTypeOdometry = new cMediaType(0, 0, 0, "tOdometry", strDescOdometry,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
            RETURN_IF_FAILED(pTypeOdometry->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescOdometryOut));


            RETURN_IF_FAILED(m_oOutputOdometry.Create("Odometry_Output", pTypeOdometry, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oOutputOdometry));



            // ------------------ further init ---------------------

            m_filterStrength = (tFloat32)GetPropertyFloat("Velocity Filter Strength", 0.5);

            if(m_filterStrength >= 1.0)
            {
                m_filterStrength = 0.99;
            }
            else if (m_filterStrength < 0.0)
            {
                m_filterStrength = 0.0;
            }
		

    RETURN_NOERROR;
		}
    else if (eStage == StageNormal)
		{
			
		}
	else if(eStage == StageGraphReady)
		{	
			
		}
				
    RETURN_NOERROR;
}

tResult SWE_Odometry::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {		

		tTimeStamp timeIntervall = 0;

		if (pSource == &m_oInputSteeringAngle)
		{	
			m_lastPinEvent = GetTime();

			// read-out the incoming Media Sample
			cObjectPtr<IMediaCoder> pCoderInput;
                	RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

			//get values from media sample        
                	pCoderInput->Get("f32Value", (tVoid*)&m_buffer);
                	pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_currTimeStamp);
                	m_pCoderDescSignal->Unlock(pCoderInput);

			//processing the data
			timeIntervall = m_currTimeStamp - m_oldTimeStamp;
			calcSingleOdometry( timeIntervall );

			m_steeringAngle = m_buffer;

		}
        else if (pSource == &m_oInputWheelLeft)
		{	

			m_lastPinEvent = GetTime();				
			
			// read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));
		               
             //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_buffer);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_currTimeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);
		        
            //processing the data
            m_velocityLeft = m_buffer;
            timeIntervall = m_currTimeStamp - m_oldTimeStamp;
            calcSingleOdometry( timeIntervall );

            updateVelocity();

            //if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sensorfilter received: ID %x Value %f",ID,value));

        }
        else if (pSource == &m_oInputWheelRight)
		{
			m_lastPinEvent = GetTime();

			// read-out the incoming Media Sample
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));
		               
            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&m_buffer);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&m_currTimeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);
		        
            //processing the data
            m_velocityRight = m_buffer;
			timeIntervall = m_currTimeStamp - m_oldTimeStamp;
            calcSingleOdometry( timeIntervall );

            updateVelocity();
		
            //if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sensorfilter received: ID %x Value %f",ID,value));
        }
	}
	RETURN_NOERROR;
}

tTimeStamp SWE_Odometry::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tResult SWE_Odometry::updateVelocity()
{

    //calculate and filter velocity
    m_velocityFiltered = ((m_velocityLeft + m_velocityRight) / 2) * (1 - m_filterStrength) + m_velocityFiltered * m_filterStrength;

    //Ausgabe als Media Sample
    cObjectPtr<IMediaSample> pNewSample;
    RETURN_IF_FAILED( AllocMediaSample((tVoid**) &pNewSample) );
    RETURN_IF_POINTER_NULL( pNewSample );
    RETURN_IF_FAILED( pNewSample->Update( m_lastPinEvent, &m_velocityFiltered, sizeof(m_velocityFiltered), IMediaSample::MSF_None) );
    m_oOutputOdometry.Transmit( pNewSample );

    RETURN_NOERROR;
}


tResult SWE_Odometry::sendData()
{
    //create new media sample
    cObjectPtr<IMediaCoder> pCoder;
    cObjectPtr<IMediaSample> pMediaSampleOutput;

    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    // ADAPT: m_pCoderDescPointLeft
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescOdometryOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    // ADAPT: m_pCoderDescPointLeft
    //cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescOdometryOut->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("distance_x", (tVoid*)&(m_odometryData.distance_x));
    pCoder->Set("distance_y", (tVoid*)&(m_odometryData.distance_y));
    pCoder->Set("angle_heading", (tVoid*)&(m_odometryData.angle_heading));
    pCoder->Set("velocity", (tVoid*)&(m_odometryData.velocity));
    pCoder->Set("distance_sum", (tVoid*)&(m_odometryData.distance_sum));
    m_pCoderDescOdometryOut->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(m_lastTriggerTime));
    RETURN_IF_FAILED(m_oOutputOdometry.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}

void SWE_Odometry::calcSingleOdometry(tTimeStamp timeIntervall)
{
	tFloat32 velocityMean;
	tFloat32 radius;
	tFloat32 distance;
	tFloat32 angle;




	//calculate single movement step

	velocityMean = ( m_velocityLeft + m_velocityRight ) / 2;
	
	if ( (m_steeringAngle + m_slippageAngle) > 1.4) //prevent crazy steering angles
	{
		radius = WHEELBASE / ( tan( 1.4 ) );
	}
	else
	{
		radius = WHEELBASE / ( tan( m_steeringAngle + m_slippageAngle ) );
	}
	
	if ( timeIntervall < 0 ) timeIntervall = 0;

	angle = velocityMean * timeIntervall / radius;
	distance = 2 * radius * sin( angle * 0.5 );

	
	//divide vector into components + add to sum

    m_distanceAllSum = m_distanceAllSum + distance;
	m_distanceX_sum = m_distanceX_sum + cos( angle + m_heading_sum ) * distance;
	m_distanceY_sum = m_distanceY_sum + sin( angle + m_heading_sum ) * distance;
	
	m_heading_sum = m_heading_sum + angle;
	
	// stuff

	m_oldTimeStamp = m_currTimeStamp;
}

void SWE_Odometry::odometryOutput()
{

    m_odometryData.distance_x = m_distanceX_sum;

    m_odometryData.distance_y = m_distanceY_sum;
	
    m_odometryData.angle_heading = m_heading_sum;

    m_odometryData.velocity = m_velocityFiltered;

    m_odometryData.distance_sum = m_distanceAllSum;
	
	m_distanceX_sum = 0;
	m_distanceY_sum = 0;
	m_heading_sum = 0;

	//output data

    sendData();
}
