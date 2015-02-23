#ifndef _SWE_ODOMETRY_H_
#define _SWE_ODOMETRY_H_

#define OID_ADTF_SWE_ODOMETRY "adtf.aadc.SWE_odometry"


#include "math.h"
/*!
* Simple odometry based on bicycle model. The relative position and heading are updated whenever a new data sample arrives. Output of the accumulated position change in position and heading every time a trigger signal arrives (accumulated since last trigger).


->velocity continuously sends the current (estimated) car speed

----- Coordinate System -----

new car position = *
old car position (last trigger) = o

          ^ X
          |
     *    |
          |
          |
<---------o--------->
Y         |        -Y
          |
          |
          |
          V -X

new heading:
          ^ X
  +  alpha|
    '     |
      '   |
        ' |
<---------o--------->
Y         |        -Y
          |
          |
          |
          V -X

*/
class SWE_Odometry : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_ODOMETRY, "SWE_Odometry", OBJCAT_DataFilter, "SWE_Odometry filter", 1, 0, 0, "Beta Version");

        /*! input pin for the steering angle */
		cInputPin m_oInputSteeringAngle;
        /*! input pin for the left wheel optical sensor*/
        cInputPin m_oInputWheelLeft;
        /*! input pin for the right wheel optical sensor*/
        cInputPin m_oInputWheelRight;
        /*! input pin that tells the odometry which direction the wheels are turning */
 //       cInputPin m_oInputDirection;
        /*! input pin for the heading gyro signal (might be called "roll" at the gyro output filter)*/
        // cInputPin m_oGyro;
        /*! input pin for the trigger signal */
        cInputPin m_oInputTrigger;
        /*! output pin for the odometry data */
		cOutputPin m_oOutputOdometry;
        /*! output pin for the vehicle speed */
      //  cOutputPin m_oOutputVelocity;

    public:
        SWE_Odometry(const tChar* __info);
        virtual ~SWE_Odometry();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);        
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	
    private:
	/*!gets the actual time as a tTimeStamp */
	tTimeStamp GetTime();
		
	/*! Coder Descriptor for the pins*/
	    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescOdometryOut;

	/*! struct containing the odometry output data */
	typedef struct
	{
        tFloat32	 distance_x;			// distance x in mm
        tFloat32	 distance_y; 		// distance y in mm
        tFloat32   	 angle_heading;		// heading in radians
        tFloat32     velocity;          //in mm/s
        tFloat32     distance_sum;   // sum of driven distance
	}odometryData;

	/*! Private member variables */
	tFloat32 m_steeringAngle;
	tFloat32 m_slippageAngle;
	tFloat32 m_velocityLeft;
	tFloat32 m_velocityRight;
	tFloat32 m_buffer;

    tFloat32 m_velocityFiltered;
    tFloat32 m_filterStrength;
	
	tTimeStamp m_currTimeStamp;
	tTimeStamp m_oldTimeStamp;
	tTimeStamp m_lastPinEvent;
    tTimeStamp m_lastTriggerTime;

	tFloat32 m_distanceX_sum;
	tFloat32 m_distanceY_sum;
	tFloat32 m_heading_sum;
    tFloat32 m_distanceAllSum;

	odometryData m_odometryData;

	/* SWE METHODS */

	/*! Helper method to send Media Sample */
    tResult sendData();

    /*! calculate and send velocity */
    tResult updateVelocity();


	/*! Odometry single step */
	void calcSingleOdometry(tTimeStamp timeIntervall);


	/*! Odometry output, reset*/
	void odometryOutput();
};



//*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
