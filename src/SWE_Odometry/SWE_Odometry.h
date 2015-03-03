#ifndef _SWE_ODOMETRY_H_
#define _SWE_ODOMETRY_H_

#define OID_ADTF_SWE_ODOMETRY "adtf.aadc.swe.odometry"

#include "stdafx.h"
#include "math.h"
#include "SWE_cSmartSlidingWindow.h"
/*!
* SmartWheels-Erlangen (Robert de Temple)
* Odometry based on bicycle model. Uses mainly gyro and wheel sensors.
* The relative position and heading are internally updated whenever a new data sample arrives.
* Output of the accumulated position change in position and heading every time a trigger signal arrives (accumulated since last trigger).
* All data sent out has the timestamp of the trigger signal.

-> odometry data contains the odometry data.
-> velocity continuously sends the current (estimated) car speed, its value is smoothed for the speed controller.

odometry data structure:

        tFloat32	 distance_x;		// relative x coordinates in mm (since last sample)
        tFloat32	 distance_y; 		// relative y coordinates in mm
        tFloat32   	 angle_heading;		// realtive heading in radians
        tFloat32     velocity;          // current velocity in mm/s
        tFloat32     distance_sum;      // sum of distance driven, this data counts forwards and backwards depending on direction and is the accumulation of travel as measured  by wheelsensor ticks. All other odometry data is based on estimated speed, therefore there is some difference between both values.

        + ui32ArduinoTimestamp          // timestamp of the trigger signal triggering the output

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
  +  angle|
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
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_ODOMETRY, "SWE_Odometry", OBJCAT_DataFilter, "Odometry", 1, 0, 0, "pre alpha version");

        /*! input pin for the steering angle -- in RAD */
        cInputPin m_oInputSteeringAngle; //TODO: not clear what type of angle we get

        /*! input pin for the left wheel optical sensor -- ticks*/
        cInputPin m_oInputWheelLeft;

        /*! input pin for the right wheel optical sensor -- ticks*/
        cInputPin m_oInputWheelRight;

        /*! input pin that tells the odometry which direction the wheels are turning -- true = forwards*/
        cInputPin m_oInputDirection;

        /*! input pin for the heading/yaw gyro signal -- in RAD */
        cInputPin m_oInputYaw;//TODO

        /*! input pin for the trigger signal */
        cInputPin m_oInputTrigger;

        /*! output pin for the odometry data */
		cOutputPin m_oOutputOdometry;

        /*! output pin for the vehicle speed */
        cOutputPin m_oOutputVelocity;

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
        cObjectPtr<IMediaTypeDescription>  m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription>  m_pCoderDescOdometryOut;
        cObjectPtr<IMediaTypeDescription>  m_pCoderBool;
        cObjectPtr<IMediaTypeDescription>  m_pCoderGyro;

	/*! Private member variables */
    /*! input variables*/
	tFloat32 m_steeringAngle;
    tFloat32 m_Yaw;
	tFloat32 m_buffer;
    tFloat32 m_wheelCounter_left;
    tFloat32 m_wheelCounter_right;

    tInt32 m_currentDirection;
    tBool m_boolBuffer;

    tTimeStamp m_currTimeStamp;
    tTimeStamp m_oldTimeStamp;
    tTimeStamp m_lastPinEvent;
    tTimeStamp m_lastTriggerTime;
    tTimeStamp m_lastLeftWheelTime;
    tTimeStamp m_lastRightWheelTime;

    tFloat32 m_lastwheelCounter_left;
    tFloat32 m_lastwheelCounter_right;

	
    /*! other variables */
    tFloat32 m_slippageAngle;
    tFloat32 m_velocityLeft;
    tFloat32 m_velocityLeft_last;
    tFloat32 m_velocityRight;
    tFloat32 m_velocityRight_last;
    tFloat32 m_velocityUnfiltered;
    tFloat32 m_velocityFiltered;
	tFloat32 m_distanceX_sum;
	tFloat32 m_distanceY_sum;
	tFloat32 m_heading_sum;
    tFloat32 m_distanceAllSum;
    tInt32   m_velocityResolution; //resolution of the velocity calculation 10 ~ max 10% error, 5 ~ max 20% error etc.

    tFloat32 m_filterStrength;
    tFloat32 m_wheelCircumfence;

    /*! sliding window filter for the left wheel*/
    SWE_cSmartSlidingWindow m_SlidingWindowCntLeftWheel;
    /*! sliding window filter for the right wheel*/
    SWE_cSmartSlidingWindow m_SlidingWindowCntRightWheel;

    /*! Lock */
    cCriticalSection m_mutex;


    /*! METHODS */

    /*! calc and send odometry */
    tResult CalcSingleOdometry(tTimeStamp timeIntervall); // a single integration step //TODO
    tResult SendOdometry(tTimeStamp timestamp); //TODO

    /*! helpers */
    tFloat32 GetAngleDiff(tFloat32 angle_old, tFloat32 angle_new); //TODO
    tFloat32 CalcDistance(tInt32 direction, tFloat32 ticks); //TODO

    /*! calculate and send velocity */
    tResult CalcVelocity(); //TODO
    tResult  FilterVelocity(tFloat32 filter_strength, tFloat32 old_velocity, tFloat32 new_velocity);
    tResult  SendVelocity(); //TODO

    tFloat32 FilterTicks(tTimeStamp last_tick, tTimeStamp curr_tick, tFloat32 last_Count, tFloat32 curr_count);//TODO  prevent double hits due to jittery sensors
};



//*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
