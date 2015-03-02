#ifndef _SWE_ODOMETRY_H_
#define _SWE_ODOMETRY_H_

#define OID_ADTF_SWE_ODOMETRY "adtf.aadc.swe.odometry"

#include "stdafx.h"
#include "math.h"
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
        cInputPin m_oInputDirection; //TODO -

        /*! input pin for the heading/yaw gyro signal -- in RAD */
        cInputPin m_oInputYaw;//TODO

        /*! input pin for the trigger signal */
        cInputPin m_oInputTrigger;//TODO

        /*! output pin for the odometry data */
		cOutputPin m_oOutputOdometry;

        /*! output pin for the vehicle speed */
        cOutputPin m_oOutputVelocity;//TODO

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
    /*! input variables -- always make working copies within funcitons to ensure consistency when working with those*/
	tFloat32 m_steeringAngle;
    tFloat32 m_Yaw;
	tFloat32 m_buffer;

    tBool m_currentDirection;
    tBool m_boolBuffer;

    tTimeStamp m_currTimeStamp;
    tTimeStamp m_oldTimeStamp;
    tTimeStamp m_lastPinEvent;
    tTimeStamp m_lastTriggerTime;

	
    /*! other variables */
    tFloat32 m_slippageAngle;
    tFloat32 m_velocityLeft;
    tFloat32 m_velocityRight;
    tFloat32 m_velocityFiltered;
    tFloat32 m_filterStrength;
	tFloat32 m_distanceX_sum;
	tFloat32 m_distanceY_sum;
	tFloat32 m_heading_sum;
    tFloat32 m_distanceAllSum;



    /*! METHODS */

    /*! calc and send odometry */
    tResult CalcSingleOdometry(tTimeStamp timeIntervall); // a single integration step //TODO
    tResult SendOdometry(tTimeStamp timestamp); //TODO

    /*! helpers */
    tFloat32 GetAngleDiff(tFloat32 angle_old, tFloat32 angle_new); //TODO
    tInt32   GetAbsoluteDistance();

    /*! calculate and send velocity */
    tResult CalcVelocity(); //TODO
    tResult SendVelocity(); //TODO
};



//*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
