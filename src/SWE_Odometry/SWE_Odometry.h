#ifndef _SWE_ODOMETRY_H_
#define _SWE_ODOMETRY_H_

#define OID_ADTF_SWE_ODOMETRY "adtf.aadc.swe.odometry"

#include "stdafx.h"
#include "math.h"
#include "SWE_cSlidingWindow.h"
/*!
* Simple odometry based on bicycle model. Uses mainly gyro and wheel sensors. The relative position and heading are internally updated whenever a new data sample arrives.
* Output of the accumulated position change in position and heading every time a trigger signal arrives (accumulated since last trigger).
* All data sent out has the timestamp of the trigger signal.

-> odometry data contains the odometry data. Further explanation at the struct definition further down in this file.
-> velocity continuously sends the current (estimated) car speed, this pin is especially smoothed for the speed controller

odometry data structure:

        tFloat32	 distance_x;		// relative x coordinates in mm (since last sample)
        tFloat32	 distance_y; 		// relative y coordinates in mm
        tFloat32   	 angle_heading;		// realtive heading in radians
        tFloat32     velocity;          // current velocity in mm/s
        tFloat32     distance_sum;      // sum of distance driven

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
        cInputPin m_oInputWheelLeft; //TODO

        /*! input pin for the right wheel optical sensor -- ticks*/
        cInputPin m_oInputWheelRight; //TODO

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
    SWE_cSlidingWindow m_SlidingWindowCntLeftWheel;
    /*! sliding window filter for the right wheel*/
    SWE_cSlidingWindow m_SlidingWindowCntRightWheel;





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


    /*!calculates the speed in mm/s of the wheel with the given parameters
            @param counterValue        the actual value of the counter
            @param lastCounterValue    the last value of the counter which must be a certain interval ago (defined with the size of the sliding window)
            @param currentTimestamp    the actual timestamp
            @param lastTimestamp       the timestamp of the lastCounterValue
            @param direction           the direction of travel
            */
    tFloat32 CalcMms(tFloat32 counterValue, tFloat32 lastCounterValue, tTimeStamp currentTimestamp, tTimeStamp lastTimestamp, tFloat32 direction);
};



//*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
