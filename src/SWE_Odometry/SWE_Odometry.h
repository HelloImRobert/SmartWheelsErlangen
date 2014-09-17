#ifndef _SWE_ODOMETRY_H_
#define _SWE_ODOMETRY_H_

#define OID_ADTF_SWE_ODOMETRY "adtf.aadc.swe.odometry"

#include "stdafx.h"
#include "math.h"
#include "SWE_cSmartSlidingWindow.h"
#include "cSlidingWindow.h"
#include "cmath"
/*!
* Advanced Odometry implementing sensor fusion of gyro, accelerometer and wheel sensors.
*
* ----------------------------------------------------------------------------------------------------
* !!!!! ATTENTION !!!!! When using the accelerometer with this odometry:
* Drift is very strong at the startup of both gyro and accelerometer sensors.
* Therefore let the sensors settle for AT LEAST 10 seconds after each reset before using the odometry.
* ------------------------------------------------------------------------------------------------------
*
* SmartWheels-Erlangen (Robert de Temple) If you have any questions => robert punkt detemple kringel gmail punkt com (the code is a mess, we know it, but it works very well)
* The relative position and heading are internally updated whenever a new data sample arrives.
* Output of the accumulated position change in position and heading every time a trigger signal arrives (accumulated since last trigger).
* All data sent has the timestamp of the trigger signal causing the output. This allows for synchonisation with image data (SWE_birdeyetrafo generates the trigger signals in our case)

-> odometry data contains the odometry data.
-> velocity continuously sends the current (estimated) car speed, its value is smoothed for the speed controller.

odometry data structure:

        tFloat32	 distance_x;		// relative x coordinates in mm (since last sample)
        tFloat32	 distance_y; 		// relative y coordinates in mm
        tFloat32   	 angle_heading;		// realtive heading in radians
        tFloat32     velocity;          // current velocity in mm/s
        tFloat32     distance_sum;      // sum of distance driven. Absolute measured value, no interpolation, accurate to about 2cm. This data counts forwards and backwards depending on direction and is the accumulation of travel as measured  by wheelsensor pulses. All other odometry data is based on estimated speed, therefore there is some difference between both values.

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


        /*! input pin for the left wheel optical sensor -- pulses*/
        cInputPin m_oInputWheelLeft;

        /*! input pin for the right wheel optical sensor -- pulses*/
        cInputPin m_oInputWheelRight;

        /*! input pin that tells the odometry which direction the wheels are turning -- true = forwards*/
        cInputPin m_oInputDirection;

        /*! input pin for the heading/yaw gyro signal -- in RAD */
        cInputPin m_oInputYaw;

        /*! input pin for the pitch gyro signal -- in RAD */
        cInputPin m_oInputPitch;

        /*! input pin for the accelerometer signal, Y-Axis -- in RAD */
        cInputPin m_oInputAccel;

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
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);     
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	
    private:
		
    /*! Coder Descriptor for the pins, you usually need one for every outpit pin and one for every type of input pin*/
        cObjectPtr<IMediaTypeDescription>  m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription>  m_pCoderDescSignal_direction;
        cObjectPtr<IMediaTypeDescription>  m_pCoderDescSignal_yaw;
        cObjectPtr<IMediaTypeDescription>  m_pCoderDescOdometryOut;
        cObjectPtr<IMediaTypeDescription>  m_pCoderBool;
        cObjectPtr<IMediaTypeDescription>  m_pCoderGyro;
        cObjectPtr<IMediaTypeDescription>  m_pCoderVelocityOut;

	/*! Private member variables */
    /*! input variables*/
    tFloat32 m_yaw;
    tFloat32 m_wheelCounter_left;
    tFloat32 m_wheelCounter_right;
    tFloat32 m_wheelDelta_left;
    tFloat32 m_wheelDelta_right;

    tFloat32 m_currentDirection;
    tBool m_boolBuffer;

    tTimeStamp m_lastPinEvent;
    tTimeStamp m_lastTriggerTime;
    tTimeStamp m_yawSampleTime_now;
    tTimeStamp m_accelerometerTimestamp_last;
    tTimeStamp m_accelerometerTimestamp_now;
    tTimeStamp m_lastTimeStamp_wheels;

    tFloat32 m_heading_lastStep;        //heading after last odometry step
    tFloat32 m_heading_input_last;
    tFloat32 m_heading_now;             //current reported heading (by sensor)
    tFloat32 m_pitch_now;               //current pitch angle
    tFloat32 m_pitch_input_last;
    tFloat32 m_accelerometerValue_now; //current accelerometer reading in mm/s²
    tFloat32 m_accelerometerValue_old;

    //DEBUG
    tFloat32 debugvar;                  //debugging variable
	
    /*! other variables */
    //tFloat32 m_slippageAngle;
    tFloat32 m_velocityLeft;          // current velocities measured on each wheel
    tFloat32 m_velocityRight;
    tFloat32 m_velocityWheelSensors;  //current velocity of car (wheel measurements)
    tFloat32 m_velocityCombined;      //current velocity of car (combined measurements)
    tFloat32 m_velocityFiltered;      //velocity low-pass filtered for speed control
    tFloat32 m_velocityAccelerometer; //current (last step) velocity gain estimated by accelerometer values in mm/s
    tFloat32 m_distanceX_sum;         //translation change since last trigger
    tFloat32 m_distanceY_sum;         // """"
    tFloat32 m_heading_sum;           //rotation change since last trigger
    tFloat32 m_heading_sumsum;          //rotation change since start
    tFloat32 m_distanceAllSum;        //accumulated distance (counts backwards when driving backwards) //Improvement: Float32 breakes down after a few kilometers
    tFloat32 m_distanceAllSum_acc;    // "            "       derived from the combined velocity data
    tFloat32 m_distanceAllSum_wheel;  // "            "       derived from the wheel speed sensors
    tInt32   m_velocityResolution;    //resolution of the velocity calculation 10 ~ max 10% error, 5 ~ max 20% error etc.
    tFloat32 m_current_turning_radius;//the current turning radius of the car
    tBool    m_wheelsync;             //has the other wheel data arrived yet?

    // definable parameters
    tFloat32 m_filterStrength;              //how strong should the velocity value on the velocity output pin be smoothed?
    tFloat32 m_wheelCircumfence;
    tFloat32 m_acceleromter_weight;         //how much does the accelerometer data influence the output?
    tFloat32 m_accelerometer_compensation;  //constant magnitude drift compensation value
    tFloat32 m_accelOffsetValue;            //constant offset to the accelerometer data
    tFloat32 m_pitchCompensation;           //strength of pitch compensation
    tBool    m_useHighresDist;              //use distance estimation based on velocity? (instead based on wheel sensor data) -> much higher resolution but prone to some small drift
    tBool    m_useAccelerometer;            //calculate velocity based on accelerometer data? (you want to turn this off when testing on a bench)
    tBool    m_showAccel;
    tBool    m_setdirectionone;
    tFloat32 m_accelScaling;                //factor to scale the accelerometer values - scaling is applied after pitch compensation
    tFloat32 m_distanceDriftCompensation;
    tFloat32 m_tickPerTurn;
    tBool m_property_useHeuristicFilter;

    /*! sliding window filter for the left wheel*/
    SWE_cSmartSlidingWindow m_SlidingWindowCntLeftWheel;
    /*! sliding window filter for the right wheel*/
    SWE_cSmartSlidingWindow m_SlidingWindowCntRightWheel;

    /*! sliding window for yaw rate estimation*/
    cSlidingWindow m_yawSlidingWindow;

    /*! Lock */
    cCriticalSection m_mutex;


    /*! METHODS */

    /*! calc and send odometry */
    tResult CalcOdometryStep(tTimeStamp time_now, tTimeStamp time_last); // a single integration step //TODO
    tResult SendOdometry(tTimeStamp timestamp);
    tFloat32  GetExtrapolatedHeadingDiff(tTimeStamp time_now);      //extrapolate heading from last known heading sample to this moment

    /*! helpers */
    tFloat32 GetAngleDiff(tFloat32 angle_new, tFloat32 angle_old);  //get the smaller difference in heading angle. Angles: +PI.....-PI
    tFloat32 CalcDistance(tInt32 direction, tFloat32 pulses);       //get the distance between the wheelpulses
    tFloat32 CompensatePitchInfluence(tFloat32 accel_value);        //comensate the influence of gravity due to non-zero pitching angles on the accelerometer values
    tTimeStamp GetTime();                                           //returns the current time in microseconds

    /*! calculate and send velocity */
    tResult  ProcessPulses(tTimeStamp timeStamp);                   //does everything that needs to be done when a new sample of wheelsensor data arrived
    tResult  CalcVelocity();                                        //calculate velocity from accumulated wheelsensor data
    tResult  CalcCombinedVelocity();                                //do sensor fusion on velocities to gain a stable, more dynamic velocity value
    tFloat32 FilterVelocity(tFloat32 filter_strength, tFloat32 old_velocity, tFloat32 new_velocity); //simple low pass filtering
    tResult  SendVelocity();
    tResult  FilterPulses();                                        // heuristic filter to filter out false multiple hits at slow speeds due to jittery sensors
};



//*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
