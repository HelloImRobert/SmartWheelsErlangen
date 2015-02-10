#ifndef _SWE_DISTANCEMEASUREMENT_H_
#define _SWE_DISTANCEMEASUREMENT_H_

#define OID_ADTF_SWE_DISTANCEMEASUREMENT "adtf.aadc.SWE_distancemeasurement"


#include "math.h"

class SWE_DistanceMeasurement : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_DISTANCEMEASUREMENT, "SWE_DistanceMeasurement", OBJCAT_DataFilter, "SWE_DistanceMeasurement filter", 1, 0, 0, "Beta Version");

    cInputPin               m_pin_input_uss_front_left;
    cInputPin               m_pin_input_uss_front_right;
    cInputPin               m_pin_input_uss_rear_left;
    cInputPin               m_pin_input_uss_rear_right;

    cInputPin               m_pin_input_ir_front_center_long;
    cInputPin               m_pin_input_ir_front_center_short;
    cInputPin               m_pin_input_ir_front_left_long;
    cInputPin               m_pin_input_ir_front_left_short;
    cInputPin               m_pin_input_ir_front_right_long;
    cInputPin               m_pin_input_ir_front_right_short;

    cInputPin               m_pin_input_ir_rear_center_short;
    cInputPin               m_pin_input_ir_rear_left_short;
    cInputPin               m_pin_input_ir_rear_right_short;

    cOutputPin              m_pin_output_sensorData;

    // for testing issues ++++++++++++++
    cOutputPin              m_pin_output_ir_front_left_long;
    cOutputPin              m_pin_output_ir_front_left_short;
    cOutputPin              m_pin_output_ir_rear_left_short;
    // +++++++++++++++++++++++++++++++++

    public:
        SWE_DistanceMeasurement(const tChar* __info);
        virtual ~SWE_DistanceMeasurement();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	
    private:
    /*!gets the actual time as a tTimeStamp */
    tTimeStamp GetTime();
		
    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignal;


    /*! raw sensor data */
    typedef struct
    {
        tFloat32	 ir_front_center_short;
        tFloat32	 ir_front_center_long;
        tFloat32	 ir_front_right_short;
        tFloat32	 ir_front_right_long;
        tFloat32	 ir_front_left_short;
        tFloat32     ir_front_left_long;
        tFloat32     ir_rear_center_short;
        tFloat32	 ir_rear_right_short;
        tFloat32	 ir_rear_left_short;
        tFloat32     uss_front_left;
        tFloat32     uss_front_right;
        tFloat32     uss_rear_left;
        tFloat32     uss_rear_right;
    }sensorData;

    /*! sensor data transformed into vehicle coordinate system */
    /*! first: Dist to VCOS Center
     *  second: Angle */

    typedef struct
    {
        std::pair <tFloat32,tFloat32>    ir_front_center;
        std::pair <tFloat32,tFloat32>	 ir_front_right;
        std::pair <tFloat32,tFloat32>	 ir_front_left;
        std::pair <tFloat32,tFloat32>    ir_rear_center;
        std::pair <tFloat32,tFloat32>	 ir_rear_right;
        std::pair <tFloat32,tFloat32>	 ir_rear_left;
        std::pair <tFloat32,tFloat32>    uss_front_left;
        std::pair <tFloat32,tFloat32>    uss_front_right;
        std::pair <tFloat32,tFloat32>    uss_rear_left;
        std::pair <tFloat32,tFloat32>    uss_rear_right;
    }XYSensorData;

    /*! Private member variables */
    sensorData          _mean;
    XYSensorData      _transformed;
    tFloat32            _filter_strength;
    tTimeStamp          _currTimeStamp;

    /* SWE METHODS */

    /*! Helper method to send Media Sample */
    tResult sendData();


    /*! Calculate weighted floating mean value; window size is floating to infinity */
    tFloat32 weightedMean(tFloat32 latest, tFloat32 old);

    /*! Transform Sensor Data into Car Coordinate System */
    void transfrom();



    /*! Odometry output, reset*/
    // void odometryOutput();
};



////*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
