/*! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * zu klaeren: wo findet Umrechnung in mm statt??
    Optionen: > in den calibration files (xml)
              > in den calibration filtern
              > in diesem filter
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/




#include "SWE_DistanceMeasurement.h"

ADTF_FILTER_PLUGIN("SWE_DistanceMeasurement", OID_ADTF_SWE_DISTANCEMEASUREMENT, SWE_DistanceMeasurement)

//SENSOR X AND Y VALUES IN FAHRZEUG KOS in mm (Calibration-Filter liefert Daten in cm (noch) !!!)
#define POS_IR_FRONT_SIDE 		450.0		//x-Value
#define POS_IR_FRONT_CENTER 	480.0       //x-Value
#define POS_IR_REAR_CENTER 		-110.0      //x-Value
#define POS_IR_REAR_SIDE 		68.0		//x-Value
#define POS_IR_FRONT_SIDE_LEFT 	150.0       //y-Value
#define POS_IR_FRONT_SIDE_RIGHT -150.0      //y-Value
#define POS_IR_REAR_SIDE_LEFT 	150.0       //y-Value
#define POS_IR_REAR_SIDE_RIGHT 	-150.0      //y-Value
#define POS_USS_FRONT 			485.0		//x-Value
#define POS_USS_FRONT_LEFT 		62			//y-Value
#define POS_USS_FRONT_RIGHT 	-62			//y-Value
#define POS_USS_REAR			-110    	//x-Value
#define POS_USS_REAR_LEFT 		56			//y-Value
#define POS_USS_REAR_RIGHT 		-56			//y-Value
#define INVALID_VALUE 			9999
#define USS_SINUS 				0.258819
#define USS_COSINUS 			0.965926

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

        //USS sensors
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


        // Output pin

        RETURN_IF_FAILED(m_pin_output_detectedPoints.Create( "DetectedPoints", new adtf::cMediaType( MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_FLOAT32 ) , static_cast<IPinEventSink*> (this) ) );
        RETURN_IF_FAILED( RegisterPin( &m_pin_output_detectedPoints ) );

        // try3 failed (original)
        //RETURN_IF_FAILED(m_pin_output_detectedPoints.Create("DetectedPoints", new cMediaType(0, 0, 0, "tFloat32"), static_cast<IPinEventSink*> (this)));
        //RETURN_IF_FAILED(RegisterPin(&m_pin_output_detectedPoints));

        //try2 failed
        //RETURN_IF_FAILED(m_pin_output_detectedPoints.Create("DetectedPoints", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        //RETURN_IF_FAILED(RegisterPin(&m_pin_output_detectedPoints));

        // try1 failed
        //RETURN_IF_FAILED(m_pin_output_detectedPoints.Create( "DetectedPoints",
        //                    cObjectPtr<IMediaType>( new adtf::cMediaType( MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_FLOAT32 ) ), NULL ) );
        //RETURN_IF_FAILED( RegisterPin( &m_pin_output_detectedPoints ) );

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
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&_timeOfLastSample);
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
            }
            else if (pSource == &m_pin_input_ir_front_left_short)
            {
                _mean.ir_front_left_short = weightedMean(signalValue,_mean.ir_front_left_short);
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
            }
            else if (pSource == &m_pin_input_ir_rear_right_short)
            {
                _mean.ir_rear_right_short = weightedMean(signalValue,_mean.ir_rear_right_short);
            }

            // Transform and fusion Sensor Data into Vehicle COS and send Data
            transfrom();
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
        RETURN_IF_FAILED( pNewSample->Update( _timeOfLastSample, &_detected_vect, sizeof(_detected_vect), IMediaSample::MSF_None) );
        m_pin_output_detectedPoints.Transmit( pNewSample );

    RETURN_NOERROR;
}


tFloat32 SWE_DistanceMeasurement::weightedMean(tFloat32 latest, tFloat32 old)
{
       return _filter_strength*old + (1 - _filter_strength)* latest;
}



/* brief:
    transforms sensor data into vehicle coordinate system and checks if values are valid. If so sensor data is written
    to _detected_vect which is send to KI
*/
tResult SWE_DistanceMeasurement::transfrom()
{

    tFloat32 fusionBuffer;


    // Transform USS
    // FRONT LEFT
    if(_mean.uss_front_left <= 200)
    {
        _transformed.uss_front_left.first = INVALID_VALUE;
        _transformed.uss_front_left.second = INVALID_VALUE;
    }
    else if(_mean.uss_front_left > 200 && _mean.uss_front_left < 1500)
    {
        _transformed.uss_front_left.first = POS_USS_FRONT + USS_COSINUS * _mean.uss_front_left;
        _transformed.uss_front_left.second = POS_USS_FRONT_LEFT + USS_SINUS * _mean.uss_front_left;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.uss_front_left.first;
        _detected_vect.back().second = _transformed.uss_front_left.second;
    }
    else if(_mean.uss_front_left >= 1500)
    {
        _transformed.uss_front_left.first = INVALID_VALUE;
        _transformed.uss_front_left.second = INVALID_VALUE;
    }

    // FRONT RIGHT
    if(_mean.uss_front_right <= 200)
    {
        _transformed.uss_front_right.first = INVALID_VALUE;
        _transformed.uss_front_right.second = INVALID_VALUE;
    }
    else if(_mean.uss_front_right > 200 && _mean.uss_front_right < 1500)
    {
        _transformed.uss_front_right.first = POS_USS_FRONT + USS_COSINUS * _mean.uss_front_right;
        _transformed.uss_front_right.second = POS_USS_FRONT_RIGHT - USS_SINUS * _mean.uss_front_right;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.uss_front_right.first;
        _detected_vect.back().second = _transformed.uss_front_right.second;
    }
    else if(_mean.uss_front_right >= 1500)
    {
        _transformed.uss_front_right.first = INVALID_VALUE;
        _transformed.uss_front_right.second = INVALID_VALUE;
    }

    // REAR LEFT
    if(_mean.uss_rear_left <= 200)
    {
        _transformed.uss_rear_left.first = INVALID_VALUE;
        _transformed.uss_rear_left.second = INVALID_VALUE;
    }
    else if(_mean.uss_rear_left > 200 && _mean.uss_rear_left < 1500)
    {
        _transformed.uss_rear_left.first = POS_USS_REAR - USS_COSINUS * _mean.uss_rear_left;
        _transformed.uss_rear_left.second = POS_USS_REAR_LEFT + USS_SINUS * _mean.uss_rear_left;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.uss_rear_left.first;
        _detected_vect.back().second = _transformed.uss_rear_left.second;
    }
    else if(_mean.uss_rear_left >= 1500)
    {
        _transformed.uss_rear_left.first = INVALID_VALUE;
        _transformed.uss_rear_left.second = INVALID_VALUE;
    }

    // REAR RIGHT
    if(_mean.uss_rear_right <= 200)
    {
        _transformed.uss_rear_right.first = INVALID_VALUE;
        _transformed.uss_rear_right.second = INVALID_VALUE;
    }
    else if(_mean.uss_rear_right > 200 && _mean.uss_rear_right < 1500)
    {
        _transformed.uss_rear_right.first = POS_USS_REAR - USS_COSINUS * _mean.uss_rear_right;
        _transformed.uss_rear_right.second = POS_USS_REAR_RIGHT - USS_SINUS * _mean.uss_rear_right;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.uss_rear_right.first;
        _detected_vect.back().second = _transformed.uss_rear_right.second;
    }
    else if(_mean.uss_rear_right >= 1500)
    {
        _transformed.uss_rear_right.first = INVALID_VALUE;
        _transformed.uss_rear_right.second = INVALID_VALUE;
    }



    // Transform IR
    // FRONT LEFT
    if(_mean.ir_front_left_short <= 50)
    {
        _transformed.ir_front_left.first = INVALID_VALUE;
        _transformed.ir_front_left.second = INVALID_VALUE;
    }
    else if(_mean.ir_front_left_short > 50 && _mean.ir_front_left_short <= 150)
    {
        _transformed.ir_front_left.first = POS_IR_FRONT_SIDE;
        _transformed.ir_front_left.second = _mean.ir_front_left_short + POS_IR_FRONT_SIDE_LEFT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_left.first;
        _detected_vect.back().second = _transformed.ir_front_left.second;
    }
    else if(_mean.ir_front_left_short > 150 && _mean.ir_front_left_short < 400 && _mean.ir_front_left_long > 150 && _mean.ir_front_left_long < 400)
    {
        //fusion
        fusionBuffer = ( _mean.ir_front_left_short + _mean.ir_front_left_long ) / 2;

        //transform
        _transformed.ir_front_left.first = POS_IR_FRONT_SIDE;
        _transformed.ir_front_left.second = fusionBuffer + POS_IR_FRONT_SIDE_LEFT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_left.first;
        _detected_vect.back().second = _transformed.ir_front_left.second;
    }
    else if(_mean.ir_front_left_short > 150 && _mean.ir_front_left_long >= 400 && _mean.ir_front_left_long < 600)
    {
        _transformed.ir_front_left.first = POS_IR_FRONT_SIDE;
        _transformed.ir_front_left.second = _mean.ir_front_left_long + POS_IR_FRONT_SIDE_LEFT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_left.first;
        _detected_vect.back().second = _transformed.ir_front_left.second;
    }
    else if(_mean.ir_front_left_short > 150 && _mean.ir_front_left_long >= 600)
    {
        _transformed.ir_front_left.first = INVALID_VALUE;
        _transformed.ir_front_left.second = INVALID_VALUE;
    }


    // FRONT RIGHT
    if(_mean.ir_front_right_short <= 50)
    {
        _transformed.ir_front_right.first = INVALID_VALUE;
        _transformed.ir_front_right.second = INVALID_VALUE;
    }
    else if(_mean.ir_front_right_short > 50 && _mean.ir_front_right_short <= 150)
    {
        _transformed.ir_front_right.first = POS_IR_FRONT_SIDE;
        _transformed.ir_front_right.second = -_mean.ir_front_right_short + POS_IR_FRONT_SIDE_RIGHT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_right.first;
        _detected_vect.back().second = _transformed.ir_front_right.second;
    }
    else if(_mean.ir_front_right_short > 150 && _mean.ir_front_right_short < 400 && _mean.ir_front_right_long > 150 && _mean.ir_front_right_long < 400)
    {
        //fusion
        fusionBuffer = ( _mean.ir_front_right_short + _mean.ir_front_right_long ) / 2;

        //transform
        _transformed.ir_front_right.first = POS_IR_FRONT_SIDE;
        _transformed.ir_front_right.second = -fusionBuffer + POS_IR_FRONT_SIDE_RIGHT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_right.first;
        _detected_vect.back().second = _transformed.ir_front_right.second;
    }
    else if(_mean.ir_front_right_short > 150 && _mean.ir_front_right_long >= 400 && _mean.ir_front_right_long < 600)
    {
        _transformed.ir_front_right.first = POS_IR_FRONT_SIDE;
        _transformed.ir_front_right.second = -_mean.ir_front_right_long + POS_IR_FRONT_SIDE_RIGHT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_right.first;
        _detected_vect.back().second = _transformed.ir_front_right.second;
    }
    else if(_mean.ir_front_right_short > 150 && _mean.ir_front_right_long >= 600)
    {
        _transformed.ir_front_right.first = INVALID_VALUE;
        _transformed.ir_front_right.second = INVALID_VALUE;
    }


    // FRONT CENTER
    if(_mean.ir_front_center_short <= 50)
    {
        _transformed.ir_front_center.first = INVALID_VALUE;
        _transformed.ir_front_center.second = INVALID_VALUE;
    }
    else if(_mean.ir_front_center_short > 50 && _mean.ir_front_center_short <= 150)
    {
        _transformed.ir_front_center.first = _mean.ir_front_center_short + POS_IR_FRONT_CENTER;
        _transformed.ir_front_right.second = 0.0;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_center.first;
        _detected_vect.back().second = _transformed.ir_front_center.second;
    }
    else if(_mean.ir_front_center_short > 150 && _mean.ir_front_center_short < 400 && _mean.ir_front_center_long > 150 && _mean.ir_front_center_long < 400)
    {
        //fusion
        fusionBuffer = ( _mean.ir_front_center_short + _mean.ir_front_center_long ) / 2;

        //transform
        _transformed.ir_front_center.first = -fusionBuffer + POS_IR_FRONT_CENTER;
        _transformed.ir_front_right.second = 0.0;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_center.first;
        _detected_vect.back().second = _transformed.ir_front_center.second;
    }
    else if(_mean.ir_front_center_short > 150 && _mean.ir_front_center_long >= 400 && _mean.ir_front_center_long < 600)
    {
        _transformed.ir_front_center.first = _mean.ir_front_center_long + POS_IR_FRONT_CENTER;
        _transformed.ir_front_center.second = 0.0;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_front_center.first;
        _detected_vect.back().second = _transformed.ir_front_center.second;
    }
    else if(_mean.ir_front_center_short > 150 && _mean.ir_front_center_long >= 600)
    {
        _transformed.ir_front_center.first = INVALID_VALUE;
        _transformed.ir_front_center.second = INVALID_VALUE;
    }


    // REAR CENTER
    if(_mean.ir_rear_center_short <= 50)
    {
        _transformed.ir_rear_center.first = INVALID_VALUE;
        _transformed.ir_rear_center.second = INVALID_VALUE;
    }
    else if(_mean.ir_rear_center_short > 50 && _mean.ir_rear_center_short < 150)
    {
        _transformed.ir_rear_center.first = -_mean.ir_rear_center_short + POS_IR_REAR_CENTER;
        _transformed.ir_rear_center.second = 0.0;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_rear_center.first;
        _detected_vect.back().second = _transformed.ir_rear_center.second;
    }
    else if(_mean.ir_rear_center_short >= 150)
    {
        _transformed.ir_rear_center.first = INVALID_VALUE;
        _transformed.ir_rear_center.second = INVALID_VALUE;
    }


    // REAR LEFT
    if(_mean.ir_rear_left_short <= 50)
    {
        _transformed.ir_rear_left.first = INVALID_VALUE;
        _transformed.ir_rear_left.second = INVALID_VALUE;
    }
    else if(_mean.ir_rear_left_short > 50 && _mean.ir_rear_left_short < 150)
    {
        _transformed.ir_rear_left.first = POS_IR_REAR_SIDE;
        _transformed.ir_rear_left.second = _mean.ir_rear_left_short + POS_IR_REAR_SIDE_LEFT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_rear_left.first;
        _detected_vect.back().second = _transformed.ir_rear_left.second;
    }
    else if(_mean.ir_rear_left_short >= 150)
    {
        _transformed.ir_rear_left.first = INVALID_VALUE;
        _transformed.ir_rear_left.second = INVALID_VALUE;
    }


    // REAR RIGHT
    if(_mean.ir_rear_right_short <= 50)
    {
        _transformed.ir_rear_right.first = INVALID_VALUE;
        _transformed.ir_rear_right.second = INVALID_VALUE;
    }
    else if(_mean.ir_rear_right_short > 50 && _mean.ir_rear_right_short < 150)
    {
        _transformed.ir_rear_right.first = POS_IR_REAR_SIDE;
        _transformed.ir_rear_right.second = -_mean.ir_rear_right_short + POS_IR_REAR_SIDE_RIGHT;

        _detected_vect.push_back(_new_vect_entry);
        _detected_vect.back().first = _transformed.ir_rear_right.first;
        _detected_vect.back().second = _transformed.ir_rear_right.second;
    }
    else if(_mean.ir_rear_right_short >= 150)
    {
        _transformed.ir_rear_left.first = INVALID_VALUE;
        _transformed.ir_rear_left.second = INVALID_VALUE;
    }


    //for test issues:
    LOG_INFO( cString::Format( "POINTS DETECTED: " + cString::FromInt( _detected_vect.size() )  ) );

    // send detected points
    if(_detected_vect.size() > 0)
    {
        sendData();
        _detected_vect.erase( _detected_vect.begin(), _detected_vect.end() );
    }

    RETURN_NOERROR;
}

