#ifndef _SWE_DISTANCEMEASUREMENT_H_
#define _SWE_DISTANCEMEASUREMENT_H_

#define OID_ADTF_SWE_DISTANCEMEASUREMENT "adtf.aadc.SWE_distancemeasurement"

#include "stdafx.h"
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

    cOutputPin              m_pin_output_detectedPoints;

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
    /*! first: x coordinate
     *  second: y coordinate */

    typedef struct
    {
        cv::Point2d    	ir_front_center;
        cv::Point2d	 	ir_front_right;
        cv::Point2d	 	ir_front_left;
        cv::Point2d    	ir_rear_center;
        cv::Point2d	 	ir_rear_right;
        cv::Point2d	 	ir_rear_left;
        cv::Point2d    uss_front_left;
        cv::Point2d    uss_front_right;
        cv::Point2d    uss_rear_left;
        cv::Point2d    uss_rear_right;
    }XYSensorData;

    /*! Private member variables */
    sensorData          	_mean;
    XYSensorData      		_transformed;
    tFloat32            	_filter_strength;
    tFloat32                _IRadjustment;
    tTimeStamp          	_timeOfLastSample;
    Point2d _detected_array[10] ;
    //std::pair <tFloat32,tFloat32> _new_vect_entry;

    
    cObjectPtr<IMediaTypeDescription> m_pCoderDescPointsOut;


    /*! Helper method to send Media Sample */
    tResult sendData();


    /*! Calculate weighted floating mean value; window size is floating to infinity */
    tFloat32 weightedMean(tFloat32 latest, tFloat32 old);

    /*! Transform Sensor Data into Car Coordinate System */
    tResult transfrom();

};



////*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
