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
#define INVALIDE_LOW 			150.0
#define INVALIDE_HIGH           9999.0
#define USS_SINUS 				0.258819
#define USS_COSINUS 			0.965926
#define CM_MM                   10          //Umrechnung cm in mm
#define TIMER_RESOLUTION        1000000.0f

SWE_DistanceMeasurement::SWE_DistanceMeasurement(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("Filter Strength", 0.7);
    SetPropertyFloat("IRscaler", 1.3);
    SetPropertyInt("Stop Time in ms", 800);
//    _mean.uss_front_left = INVALIDE_HIGH;
//    _mean.uss_front_right = INVALIDE_HIGH;
//    _mean.uss_rear_right = INVALIDE_HIGH;
//    _mean.uss_rear_left = INVALIDE_HIGH;
//    _mean.ir_front_center_long = INVALIDE_HIGH;
//    _mean.ir_front_center_short = INVALIDE_HIGH;
//    _mean.ir_front_right_long = INVALIDE_HIGH;
//    _mean.ir_front_right_short = INVALIDE_HIGH;
//    _mean.ir_front_left_long = INVALIDE_HIGH;
//    _mean.ir_front_left_short = INVALIDE_HIGH;
//    _mean.ir_rear_center_short = INVALIDE_HIGH;
//    _mean.ir_rear_left_short = INVALIDE_HIGH;
//    _mean.ir_rear_right_short = INVALIDE_HIGH;



}

SWE_DistanceMeasurement::~SWE_DistanceMeasurement()
{

}

tResult SWE_DistanceMeasurement::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {

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

        tChar const * strDescPoints = pDescManager->GetMediaDescription("tPointArray");
        RETURN_IF_POINTER_NULL(strDescPoints);
        cObjectPtr<IMediaType> pTypePoints = new cMediaType(0, 0, 0, "tPointArray", strDescPoints,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypePoints->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPointsOut));

        RETURN_IF_FAILED(m_pin_output_detectedPoints.Create("DetectedPoints", pTypePoints, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_detectedPoints));


    }
    else if (eStage == StageNormal)
    {
        _filter_strength = (tFloat32)GetPropertyFloat("Filter Strength");
        _IRscaler = (tFloat32)GetPropertyFloat("IRscaler");
        _stopTime =  (tInt32)GetPropertyInt("Stop Time in ms", 800) * (TIMER_RESOLUTION / 1000.0);
    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult SWE_DistanceMeasurement::Start(__exception)
{
    _timerStart = GetTime();
    return cFilter::Start(__exception_ptr);
}

tTimeStamp SWE_DistanceMeasurement::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
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
                signalValue = CM_MM * signalValue;
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

                if( GetTime() - _timerStart >= _stopTime)
                {
                    sendData();
                }
            }

            // Transform and fusion Sensor Data into Vehicle COS and send Data
            transfrom();
    }
    RETURN_NOERROR;
}


tResult SWE_DistanceMeasurement::sendData()
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescPointsOut->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSample->AllocBuffer(nSize);


        RETURN_IF_FAILED(m_pCoderDescPointsOut->WriteLock(pMediaSample, &pCoder));


        for(int i=0; i < 10; i++)
        {
            stringstream elementSetter;

                elementSetter << "tPoint[" << i << "].xCoord";
                const string& tempRef1 = elementSetter.str();
                const tChar* tempPointer1 = tempRef1.c_str();
                pCoder->Set(tempPointer1, (tVoid*)&(_detected_array[i].x));
                elementSetter.str(std::string());

                elementSetter << "tPoint[" << i << "].yCoord";
                const string& tempRef2 = elementSetter.str();
                const tChar* tempPointer2 = tempRef2.c_str();
                pCoder->Set(tempPointer2, (tVoid*)&(_detected_array[i].y));
                elementSetter.str(std::string());

        }

       m_pCoderDescPointsOut->Unlock(pCoder);


        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_pin_output_detectedPoints.Transmit(pMediaSample));
        // OUTPUT SPLINES -----------------------------------------------------------
    RETURN_NOERROR;
}


tFloat32 SWE_DistanceMeasurement::weightedMean(tFloat32 latest, tFloat32 old)
{
    tFloat32 mean = (_filter_strength * old) + ( (1.0 - _filter_strength) * latest );
    return mean;
}



/* brief:
    transforms sensor data into vehicle coordinate system and checks if values are valid. If so sensor data is written
    to _detected_vect which is send to KI
*/
tResult SWE_DistanceMeasurement::transfrom()
{

    // Transform USS
    // FRONT LEFT 0
    if(_mean.uss_front_left <= 200)
    {
        _transformed.uss_front_left.x = 480;
        _transformed.uss_front_left.y = 60;
    }
    else if(_mean.uss_front_left > 200 && _mean.uss_front_left < 1500)
    {
        _transformed.uss_front_left.x = POS_USS_FRONT + USS_COSINUS * _mean.uss_front_left;
        _transformed.uss_front_left.y = POS_USS_FRONT_LEFT + USS_SINUS * _mean.uss_front_left;
    }
    else if(_mean.uss_front_left >= 1500)
    {
        _transformed.uss_front_left.x = INVALIDE_HIGH;
        _transformed.uss_front_left.y = INVALIDE_HIGH;
    }
    
    _detected_array[0] = _transformed.uss_front_left;

    // FRONT RIGHT 1
    if(_mean.uss_front_right <= 200)
    {
        _transformed.uss_front_right.x = 480;
        _transformed.uss_front_right.y = -60;
    }
    else if(_mean.uss_front_right > 200 && _mean.uss_front_right < 1500)
    {
        _transformed.uss_front_right.x = POS_USS_FRONT + USS_COSINUS * _mean.uss_front_right;
        _transformed.uss_front_right.y = POS_USS_FRONT_RIGHT - USS_SINUS * _mean.uss_front_right;

    }
    else if(_mean.uss_front_right >= 1500)
    {
        _transformed.uss_front_right.x = INVALIDE_HIGH;
        _transformed.uss_front_right.y = INVALIDE_HIGH;
    }
    
    _detected_array[1] = _transformed.uss_front_right;


    // REAR LEFT 2
    if(_mean.uss_rear_left <= 200)
    {
        _transformed.uss_rear_left.x = -110;
        _transformed.uss_rear_left.y = 60;
    }
    else if(_mean.uss_rear_left > 200 && _mean.uss_rear_left < 1500)
    {
        _transformed.uss_rear_left.x = POS_USS_REAR - USS_COSINUS * _mean.uss_rear_left;
        _transformed.uss_rear_left.y = POS_USS_REAR_LEFT + USS_SINUS * _mean.uss_rear_left;

    }
    else if(_mean.uss_rear_left >= 1500)
    {
        _transformed.uss_rear_left.x = -INVALIDE_HIGH;
        _transformed.uss_rear_left.y = INVALIDE_HIGH;
    }
    
    _detected_array[2] = _transformed.uss_rear_left;


    // REAR RIGHT 3
    if(_mean.uss_rear_right <= 200)
    {
        _transformed.uss_rear_right.x = -110;
        _transformed.uss_rear_right.y = -60;
    }
    else if(_mean.uss_rear_right > 200 && _mean.uss_rear_right < 1500)
    {
        _transformed.uss_rear_right.x = POS_USS_REAR - USS_COSINUS * _mean.uss_rear_right;
        _transformed.uss_rear_right.y = POS_USS_REAR_RIGHT - USS_SINUS * _mean.uss_rear_right;

    }
    else if(_mean.uss_rear_right >= 1500)
    {
        _transformed.uss_rear_right.x = -INVALIDE_HIGH;
        _transformed.uss_rear_right.y = INVALIDE_HIGH;
    }
    
    _detected_array[3] = _transformed.uss_rear_right;


    // FRONT LEFT 4
    if(_mean.ir_front_left_short <= 50)
    {
        _transformed.ir_front_left.x = 480;
        _transformed.ir_front_left.y = INVALIDE_LOW;
    }
    else if(_mean.ir_front_left_short > 50 && _mean.ir_front_left_short <= 250)
    {
        _transformed.ir_front_left.x = POS_IR_FRONT_SIDE;
        _transformed.ir_front_left.y = (_mean.ir_front_left_short * _IRscaler) + POS_IR_FRONT_SIDE_LEFT;

    }
    else if(_mean.ir_front_left_short > 250 && _mean.ir_front_left_long > 200 && _mean.ir_front_left_long < 600)
    {
        _transformed.ir_front_left.x = POS_IR_FRONT_SIDE;
        _transformed.ir_front_left.y = (_mean.ir_front_left_long * _IRscaler) + POS_IR_FRONT_SIDE_LEFT;

    }
    else if(_mean.ir_front_left_short > 300 && _mean.ir_front_left_long >= 600)
    {
        _transformed.ir_front_left.x = 480;
        _transformed.ir_front_left.y = INVALIDE_HIGH;
    }
    
    _detected_array[4] = _transformed.ir_front_left;



    // FRONT RIGHT 5
    if(_mean.ir_front_right_short <= 50)
    {
        _transformed.ir_front_right.x = 480;
        _transformed.ir_front_right.y = -INVALIDE_LOW;
        //LOG_ERROR(cString("DM 1: Short: " + cString::FromFloat64(_mean.ir_front_right_short) + "; Long: " + cString::FromFloat64(_mean.ir_front_right_long) + "; Comb: " + cString::FromFloat64(_transformed.ir_front_right.y) ));
    }
    else if(_mean.ir_front_right_short > 50 && _mean.ir_front_right_short <= 250)
    {
        _transformed.ir_front_right.x = POS_IR_FRONT_SIDE;
        _transformed.ir_front_right.y = -(_mean.ir_front_right_short * _IRscaler) + POS_IR_FRONT_SIDE_RIGHT;

        //LOG_ERROR(cString("DM 2: Short: " + cString::FromFloat64(_mean.ir_front_right_short) + "; Long: " + cString::FromFloat64(_mean.ir_front_right_long) + "; Comb: " + cString::FromFloat64(_transformed.ir_front_right.y) ));
    }
      else if(_mean.ir_front_right_short > 250 && _mean.ir_front_right_long > 200 && _mean.ir_front_right_long < 600)
    {
        _transformed.ir_front_right.x = POS_IR_FRONT_SIDE;
        _transformed.ir_front_right.y = -(_mean.ir_front_right_long * _IRscaler) + POS_IR_FRONT_SIDE_RIGHT;

        //LOG_ERROR(cString("DM 3: Short: " + cString::FromFloat64(_mean.ir_front_right_short) + "; Long: " + cString::FromFloat64(_mean.ir_front_right_long) + "; Comb: " + cString::FromFloat64(_transformed.ir_front_right.y) ));
    }
    else if(_mean.ir_front_right_short > 300 && _mean.ir_front_right_long >= 600)
    {
        _transformed.ir_front_right.x = 480;
        _transformed.ir_front_right.y = INVALIDE_HIGH;
        //LOG_ERROR(cString("DM 4: Short: " + cString::FromFloat64(_mean.ir_front_right_short) + "; Long: " + cString::FromFloat64(_mean.ir_front_right_long) + "; Comb: " + cString::FromFloat64(_transformed.ir_front_right.y) ));
    }
    
    _detected_array[5] = _transformed.ir_front_right;



    // FRONT CENTER 6
    if(_mean.ir_front_center_short <= 50)
    {
        _transformed.ir_front_center.x = 480;
        _transformed.ir_front_center.y = 0;
    }
    else if(_mean.ir_front_center_short > 50 && _mean.ir_front_center_short <= 250)
    {
        _transformed.ir_front_center.x = (_mean.ir_front_center_short * _IRscaler) + POS_IR_FRONT_CENTER;
        _transformed.ir_front_center.y = 0.0;

    }
    else if(_mean.ir_front_center_short > 250 && _mean.ir_front_center_long > 200 && _mean.ir_front_center_long < 600)
    {
        _transformed.ir_front_center.x = (_mean.ir_front_center_long * _IRscaler) + POS_IR_FRONT_CENTER;
        _transformed.ir_front_center.y = 0.0;

    }
    else if(_mean.ir_front_center_short > 150 && _mean.ir_front_center_long >= 600)
    {
        _transformed.ir_front_center.x = 0;
        _transformed.ir_front_center.y = INVALIDE_HIGH;
    }
    
    _detected_array[6] = _transformed.ir_front_center;


    // REAR CENTER 7
    if(_mean.ir_rear_center_short <= 50)
    {
        _transformed.ir_rear_center.x = -110;
        _transformed.ir_rear_center.y = 0;
    }
    else if(_mean.ir_rear_center_short > 50 && _mean.ir_rear_center_short <= 250)
    {
        _transformed.ir_rear_center.x = -(_mean.ir_rear_center_short * _IRscaler) + POS_IR_REAR_CENTER;
        _transformed.ir_rear_center.y = 0.0;
    }
    else if(_mean.ir_rear_center_short > 250)
    {
        _transformed.ir_rear_center.x = 0;
        _transformed.ir_rear_center.y = INVALIDE_HIGH;
    }
    
    _detected_array[7] = _transformed.ir_rear_center;


    // REAR LEFT 8
    if(_mean.ir_rear_left_short <= 50)
    {
        _transformed.ir_rear_left.x = 65;
        _transformed.ir_rear_left.y = INVALIDE_LOW;
    }
    else if(_mean.ir_rear_left_short > 50 && _mean.ir_rear_left_short <= 250)
    {
        _transformed.ir_rear_left.x = POS_IR_REAR_SIDE;
        _transformed.ir_rear_left.y = (_mean.ir_rear_left_short * _IRscaler) + POS_IR_REAR_SIDE_LEFT;

    }
    else if(_mean.ir_rear_left_short > 250)
    {
        _transformed.ir_rear_left.x = 65;
        _transformed.ir_rear_left.y = INVALIDE_HIGH;
    }
    
    _detected_array[8] = _transformed.ir_rear_left;


    // REAR RIGHT 9
    if(_mean.ir_rear_right_short <= 50)
    {
        _transformed.ir_rear_right.x = 65;
        _transformed.ir_rear_right.y = -INVALIDE_LOW;
    }
    else if(_mean.ir_rear_right_short > 50 && _mean.ir_rear_right_short <= 250)
    {
        _transformed.ir_rear_right.x = POS_IR_REAR_SIDE;
        _transformed.ir_rear_right.y = -(_mean.ir_rear_right_short * _IRscaler) + POS_IR_REAR_SIDE_RIGHT;
    }
    else if(_mean.ir_rear_right_short > 250)
    {
        _transformed.ir_rear_left.x = 65;
        _transformed.ir_rear_left.y = INVALIDE_HIGH;
    }
    
    _detected_array[9] = _transformed.ir_rear_right;

    RETURN_NOERROR;
}

