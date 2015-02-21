#include "stdafx.h"
#include "SWE_DistanceMeasurement.h"



ADTF_FILTER_PLUGIN("SWE_DistanceMeasurement", OID_ADTF_SWE_DISTANCEMEASUREMENT, SWE_DistanceMeasurement)

//SENSOR X AND Y VALUES IN FAHRZEUG KOS in cm (Calibration-Filter liefert Datem schon in cm !!!)
#define DIST_IR_FRONT_SIDE 45              //x-Value
#define DIST_IR_FRONT_CENTER 48            //x-Value
#define DIST_IR_REAR_CENTER -11            //x-Value
#define DIST_IR_FRONT_SIDE_LEFT 15        //y-Value
#define DIST_IR_FRONT_SIDE_RIGHT -15        //y-Value
#define DIST_IR_REAR_SIDE_LEFT 15         //y-Value
#define DIST_IR_REAR_SIDE_RIGHT -15         //y-Value

SWE_DistanceMeasurement::SWE_DistanceMeasurement(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("Filter Strength", 0.7);

}

SWE_DistanceMeasurement::~SWE_DistanceMeasurement()
{

}

tResult SWE_DistanceMeasurement::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        _filter_strength = (tFloat32)GetPropertyFloat("Filter Strength", 0.7);


        //pin descriptor
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

        //uss sensors
        RETURN_IF_FAILED(m_pin_input_uss_front_left.Create("uss_front_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_uss_front_left));
        RETURN_IF_FAILED(m_pin_input_uss_front_right.Create("uss_front_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_uss_front_right));
        RETURN_IF_FAILED(m_pin_input_uss_rear_left.Create("uss_rear_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_uss_rear_left));
        RETURN_IF_FAILED(m_pin_input_uss_rear_right.Create("uss_rear_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_uss_rear_right));

        //IR sensors
        //center front
        RETURN_IF_FAILED(m_pin_input_ir_front_center_long.Create("ir_front_center_long", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_center_long));
        RETURN_IF_FAILED(m_pin_input_ir_front_center_short.Create("ir_front_center_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_center_short));

        //front left
        RETURN_IF_FAILED(m_pin_input_ir_front_left_long.Create("ir_front_l_long", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_left_long));
        RETURN_IF_FAILED(m_pin_input_ir_front_left_short.Create("ir_front_l_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_left_short));
        //front right
        RETURN_IF_FAILED(m_pin_input_ir_front_right_long.Create("ir_front_r_long", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_right_long));
        RETURN_IF_FAILED(m_pin_input_ir_front_right_short.Create("ir_front_r_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_right_short));
        //rear
        RETURN_IF_FAILED(m_pin_input_ir_rear_center_short.Create("ir_rear_center_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_rear_center_short));
        RETURN_IF_FAILED(m_pin_input_ir_rear_left_short.Create("ir_rear_l_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_rear_left_short));
        RETURN_IF_FAILED(m_pin_input_ir_rear_right_short.Create("ir_rear_r_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_rear_right_short));





        // Output pins
        RETURN_IF_FAILED(m_pin_output_sensorData.Create("SensorData", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_sensorData));


        // just for testing issues
        /////////////////////////////////////////////////////////////////
        RETURN_IF_FAILED(m_pin_output_ir_front_left_long.Create("IR_front_left_long", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_ir_front_left_long));
        RETURN_IF_FAILED(m_pin_output_ir_front_left_short.Create("IR_front_left_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_ir_front_left_short));
        RETURN_IF_FAILED(m_pin_output_ir_rear_left_short.Create("IR_rear_left_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_ir_rear_left_short));
        /////////////////////////////////////////////////////////////////

    }
    else if (eStage == StageNormal)
    {
			
    }
    else if(eStage == StageGraphReady)
    {
			
    }
				
    RETURN_NOERROR;
}

tResult SWE_DistanceMeasurement::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

            //write values with zero
            tFloat32 signalValue = 0;
            if (pMediaSample != NULL && m_pCoderDescSignal != NULL)
            {
                // read-out the incoming Media Sample
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

                //get values from media sample
                pCoderInput->Get("f32Value", (tVoid*)&signalValue);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&_currTimeStamp);
                m_pCoderDescSignal->Unlock(pCoderInput);
            }
            else
            RETURN_ERROR(ERR_FAILED);


            //USS
            if (pSource == &m_pin_input_uss_front_left)
            {
                _mean.uss_front_left = weightedMean(signalValue,_mean.uss_front_left);
            }
            else if (pSource == &m_pin_input_uss_front_right)
            {
                _mean.uss_front_right = weightedMean(signalValue,_mean.uss_front_right);
            }
            else if (pSource == &m_pin_input_uss_rear_right)
            {
                _mean.uss_rear_right = weightedMean(signalValue,_mean.uss_rear_right);
            }
            else if (pSource == &m_pin_input_uss_rear_left)
            {
                _mean.uss_rear_left = weightedMean(signalValue,_mean.uss_rear_left);
            }

            //IR
            else if (pSource == &m_pin_input_ir_front_left_long)
            {

                _mean.ir_front_left_long = weightedMean(signalValue,_mean.ir_front_left_long);

                // just for testing issues
                /////////////////////////////////////////////////////////////////
                //create new media sample
                cObjectPtr<IMediaSample> pNewMediaSample;
                AllocMediaSample((tVoid**)&pNewMediaSample);

                //allocate memory with the size given by the descriptor
                cObjectPtr<IMediaSerializer> pSerializer;
                m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
                tInt nSize = pSerializer->GetDeserializedSize();
                pNewMediaSample->AllocBuffer(nSize);

                //write date to the media sample with the coder of the descriptor
                cObjectPtr<IMediaCoder> pCoderOutput;
                m_pCoderDescSignal->WriteLock(pNewMediaSample, &pCoderOutput);

                pCoderOutput->Set("f32Value", (tVoid*)&(_mean.ir_front_left_long));
                m_pCoderDescSignal->Unlock(pCoderOutput);

                //transmit media sample over output pin
                pNewMediaSample->SetTime(pMediaSample->GetTime());
                m_pin_output_ir_front_left_long.Transmit(pNewMediaSample);
                /////////////////////////////////////////////////////////////////

            }
            else if (pSource == &m_pin_input_ir_front_left_short)
            {

                _mean.ir_front_left_short = weightedMean(signalValue,_mean.ir_front_left_short);

                // just for testing issues
                /////////////////////////////////////////////////////////////////
                //create new media sample
                cObjectPtr<IMediaSample> pNewMediaSample;
                AllocMediaSample((tVoid**)&pNewMediaSample);

                //allocate memory with the size given by the descriptor
                cObjectPtr<IMediaSerializer> pSerializer;
                m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
                tInt nSize = pSerializer->GetDeserializedSize();
                pNewMediaSample->AllocBuffer(nSize);

                //write date to the media sample with the coder of the descriptor
                cObjectPtr<IMediaCoder> pCoderOutput;
                m_pCoderDescSignal->WriteLock(pNewMediaSample, &pCoderOutput);

                pCoderOutput->Set("f32Value", (tVoid*)&(_mean.ir_front_left_short));
                m_pCoderDescSignal->Unlock(pCoderOutput);

                //transmit media sample over output pin
                pNewMediaSample->SetTime(pMediaSample->GetTime());
                m_pin_output_ir_front_left_short.Transmit(pNewMediaSample);
                /////////////////////////////////////////////////////////////////
            }
            else if (pSource == &m_pin_input_ir_front_center_long)
            {
                _mean.ir_front_center_long = weightedMean(signalValue,_mean.ir_front_center_long);
            }
            else if (pSource == &m_pin_input_ir_front_center_short)
            {
                _mean.ir_front_center_short = weightedMean(signalValue,_mean.ir_front_center_short);
            }
            else if (pSource == &m_pin_input_ir_front_right_long)
            {
                _mean.ir_front_right_long = weightedMean(signalValue,_mean.ir_front_right_long);
            }
            else if (pSource == &m_pin_input_ir_front_right_short)
            {
                _mean.ir_front_right_short = weightedMean(signalValue,_mean.ir_front_right_short);
            }
            else if (pSource == &m_pin_input_ir_rear_center_short)
            {
                _mean.ir_rear_center_short = weightedMean(signalValue,_mean.ir_rear_center_short);
            }
            else if (pSource == &m_pin_input_ir_rear_left_short)
            {
                _mean.ir_rear_left_short = weightedMean(signalValue,_mean.ir_rear_left_short);

                // just for testing issues
                /////////////////////////////////////////////////////////////////
                //create new media sample
                cObjectPtr<IMediaSample> pNewMediaSample;
                AllocMediaSample((tVoid**)&pNewMediaSample);

                //allocate memory with the size given by the descriptor
                cObjectPtr<IMediaSerializer> pSerializer;
                m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
                tInt nSize = pSerializer->GetDeserializedSize();
                pNewMediaSample->AllocBuffer(nSize);

                //write date to the media sample with the coder of the descriptor
                cObjectPtr<IMediaCoder> pCoderOutput;
                m_pCoderDescSignal->WriteLock(pNewMediaSample, &pCoderOutput);

                pCoderOutput->Set("f32Value", (tVoid*)&(_mean.ir_rear_left_short));
                m_pCoderDescSignal->Unlock(pCoderOutput);

                //transmit media sample over output pin
                pNewMediaSample->SetTime(pMediaSample->GetTime());
                m_pin_output_ir_rear_left_short.Transmit(pNewMediaSample);
                /////////////////////////////////////////////////////////////////
            }
            else if (pSource == &m_pin_input_ir_rear_right_short)
            {
                _mean.ir_rear_right_short = weightedMean(signalValue,_mean.ir_rear_right_short);
            }

            // Transform and fusion Sensor Data into Vehicle COS and send Data
            transfrom();
            sendData();
    }
    RETURN_NOERROR;
}

tTimeStamp SWE_DistanceMeasurement::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}


tResult SWE_DistanceMeasurement::sendData()
{
        //Ausgabe als Media Sample
        cObjectPtr<IMediaSample> pNewSample;
        RETURN_IF_FAILED( AllocMediaSample((tVoid**) &pNewSample) );
        RETURN_IF_POINTER_NULL( pNewSample );
        RETURN_IF_FAILED( pNewSample->Update( _currTimeStamp, &_mean, sizeof(_mean), IMediaSample::MSF_None) );
        m_pin_output_sensorData.Transmit( pNewSample );

    RETURN_NOERROR;
}


tFloat32 SWE_DistanceMeasurement::weightedMean(tFloat32 latest, tFloat32 old)
{
       return _filter_strength*old + (1 - _filter_strength)* latest;
}

void SWE_DistanceMeasurement::transfrom()
{

    /*! This bullshit is to be tested */
    //float32 fusionBuffer;


    // Transform USS
    //
    //


    // Transform (and fusion) IR
    // Take SR for dist < 15 cm
    // Combine SR and LR for dist 15 - 40 cm
    // Take LR for dist > 40 cm

    // FRONT LEFT

    if(_mean.ir_front_left_short <= 5.0)
    {
        _transformed.ir_front_left.first = 9999;
        _transformed.ir_front_left.second = 9999;
    }
    else if(_mean.ir_front_left_short > 5.0 && _mean.ir_front_left_short <= 15.0 && _mean.ir_front_left_long < 15.0)
    {
        _transformed.ir_front_left.first = DIST_IR_FRONT_SIDE;
        _transformed.ir_front_left.second = _mean.ir_front_left_short + DIST_IR_FRONT_SIDE_LEFT;
    }
    else if(_mean.ir_front_left_short > 15.0 && _mean.ir_front_left_short < 40.0 && _mean.ir_front_left_long > 15.0 && _mean.ir_front_left_long < 40.0)
    {
        //fusion
        //fusionBuffer = ( _mean.ir_front_left_short + _mean.ir_front_left_long ) / 2;

        //transform
       // _transformed.ir_front_left.first = DIST_IR
      //  _transformed.ir_front_left.second = tan( DIST_IR_FRONT_SIDE / (fusionBuffer + DIST_IR_FRONT_SIDE_LEFT) );
    }
    else if(_mean.ir_front_left_short > 15.0 && _mean.ir_front_left_long >= 40.0 && _mean.ir_front_left_long < 60.0)
    {
        _transformed.ir_front_left.first = sqrt( DIST_IR_FRONT_SIDE * DIST_IR_FRONT_SIDE + (_mean.ir_front_left_long + DIST_IR_FRONT_SIDE_LEFT) * (_mean.ir_front_left_long + DIST_IR_FRONT_SIDE_LEFT) );
        _transformed.ir_front_left.second = tan( DIST_IR_FRONT_SIDE / (_mean.ir_front_left_long + DIST_IR_FRONT_SIDE_LEFT) );
    }
    else if(_mean.ir_front_left_short > 15.0 && _mean.ir_front_left_long >= 60.0)
    {
        _transformed.ir_front_left.first = 9999;
        _transformed.ir_front_left.second = 9999;
    }

    // FRONT RIGHT

    // FRONT CENTER


    // REAR
    _transformed.ir_rear_center.first = _mean.ir_rear_center_short + DIST_IR_REAR_CENTER;;
    _transformed.ir_rear_center.second = 0.0;

    _transformed.ir_rear_left.first = 0.0;
    _transformed.ir_rear_left.second = _mean.ir_rear_left_short + DIST_IR_REAR_SIDE_LEFT;

    _transformed.ir_rear_right.first = 0.0;
    _transformed.ir_rear_right.second = -_mean.ir_rear_left_short + DIST_IR_REAR_SIDE_RIGHT;


}


