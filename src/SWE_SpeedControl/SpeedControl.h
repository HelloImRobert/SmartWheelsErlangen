#ifndef _SPEEDCONTROL_H_
#define _SPEEDCONTROL_H_

#include "stdafx.h"
#include "math.h"

#define OID_ADTF_SWE_SPEEDCONTROL "adtf.aadc.swe.SpeedControl"

/*!
* motor speed controller SmartWheels-Erlangen (Robert de Temple) If you have any questions mail me at: robert *punkt* detemple *kringel* gmail *punkt* com
* - The controller has a set of "gears" 3,....0,...-2 that define the speed and the direction the car has to go. 3 full speed, 2, 1 crawl, 0 stop, -1, -2 fast reverse.
* - It uses active braking. The strength of this braking depends on the kind of transition (e.g. braking to a full stop or just a "normal" change of speed).
* - It also signals the use of brake lights and reverse gear lights according to the situation.
* This controller only uses the cars velocity for speed transiton (to find out if the target gear/speed has been reached (more or less, very inpercise)). Use a secondary, cascaded controller e.g. SWE_PIDController for precise speeds.
* Any transition to or through 0 (full stop) e.g. when changing directions results in short minimum stop time defined in the parameters.
* - This is to make sure that the car has really stopped before any change in direction might happen.
* - The controller then signals the stop to any filters that need this information (as the odometry is usually not that good at very slow speeds).
* - The controller also signals the current direction of travel to the odometry as the installed sensors on the car cannot measure this by themselves (hence also the enforced full stop).
*/
class SpeedControl : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_SPEEDCONTROL, "SWE motor speed control", OBJCAT_DataFilter, "Speed Control", 1, 0,0, "beta version");

        cInputPin m_oInputVelocity;				// the input pin for the measured velocity in mm/s
        cInputPin m_oInputSetPoint;				// the input pin for the set point value (gear) -2(fast reverse), -1, 0(stop), 1, 2, 3(full speed ahead)
        cOutputPin m_oOutputPWM;                // output pin: (PWM) signal to the motor 100 -- -100
        cOutputPin m_oOutputCarStopped;         // output pin: car has stopped
        cOutputPin m_oOutputbrakelight;         // output pins for brakelights and reverse lights
        cOutputPin m_oOutputreverse;            //   "     "    "       "       "     "      "
        cOutputPin m_oOutputDirection;          // the direction in wich the car currently travelles -> needed by the odometry 1, 0,-1
        cOutputPin m_oOutputSetPoint;           // the current speed this controller expects or wishes the car to be -> used as input setPoint for a cascaded secondary controller
        cOutputPin m_oOutputControllerStrength; // for secondary cascaded controller (see SWE_PIDController)
    public:
        SpeedControl(const tChar* __info);
        virtual ~SpeedControl();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);        
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);	

	private:

 // member functions

		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);	

		/*! calculates the manipulated value for the given values, it uses the setpoint in m_setPoint
        @param measuredSpeed	the Speed measured by the odometry
		*/
        tFloat32 GetControllerValue();

        /*! update state of car according to current speed */
        tResult UpdateState();

        /*! switch on/off brake lights
        @param state lights on or off
        */
        tResult SetBrakeLights (tBool state);

        /*! switch on/off reverse lights
        @param state lights on or off
        */
        tResult SetReverseLights (tBool state);

        /*! tell the odometry what direction we're going
        @param state going forwards yes or no
        */
        tResult SetDirection (tFloat32 state);

        /*! tell the parkpilot that we've stopped
        @param state unimportant payload
        */
        tResult SetCarStopped (tBool state);


        /*! send out the pwm value
        @param pwm_value the pwm value to be sent -100 - +100
        */
        tResult SetPWM (tFloat32 pwm_value);

        /*! wait a defined amount of time
        @param idletime time to wait in microseconds
        */
        tResult WaitIdle(tUInt32 idletime);

		/*! returns the currentstreamtime*/
		tTimeStamp GetTime();

        /*! sends the current setPoint to the controller behind it */

        tResult SetSetPoint(tFloat32 value);

        /*! sends the current Controller Strength to the controller behind it */

        tResult SetControllerStrength(tFloat32 value);



 // member variables
		
		/*! holds the last measuredValue */
        tFloat32 m_velocity;

        /*! holds the current setpoint/gear */
        tInt32 m_gear;

        /*! holds the last active gear/setpoint*/
        tFloat32 m_currentState;

        tFloat32 m_lastState;

        /*! current direction of travel as the wheel sensors can't sense that*/

        tFloat32 m_goingForwards; //1 = forward, 0 = stopped, -1 = backwards

		/*! holds the last sample time */
		tTimeStamp m_lastSampleTime;

        /*! start time for the timer */
        tUInt32 m_timerStart;
        tUInt32 m_timerStart_init;

        /*! tells the controller if it has to wait bc the car just stopped */
        tBool m_no_wait;

        /*! test run procedure to try out different gears */
        tResult TestRun(tInt32 code);


        /*! all other thresholds and definable variables */
        tFloat32 m_threshold_p3;
        tFloat32 m_threshold_p2;
        tFloat32 m_threshold_p1;
        tFloat32 m_threshold_p0;
        tFloat32 m_threshold_n0;
        tFloat32 m_threshold_n1;
        tFloat32 m_threshold_n2;

        tFloat32 m_pwm_p3;
        tFloat32 m_pwm_p2;
        tFloat32 m_pwm_p1;
        tFloat32 m_pwm_0;
        tFloat32 m_pwm_n1;
        tFloat32 m_pwm_n2;

        tFloat32 m_pwm_boost_p3;
        tFloat32 m_pwm_boost_p2;
        tFloat32 m_pwm_boost_p1;
        tFloat32 m_pwm_boost_n1;
        tFloat32 m_pwm_boost_n2;

        tFloat32 m_setPoint_p3;
        tFloat32 m_setPoint_p2;
        tFloat32 m_setPoint_p1;
        tFloat32 m_setPoint_0;
        tFloat32 m_setPoint_n1;
        tFloat32 m_setPoint_n2;

        tFloat32 m_strongBrake;
        tFloat32 m_lightBrake;

        tFloat32 m_inv_strongBrake;
        tFloat32 m_inv_lightBrake;

        tFloat32 m_pwmScaler;

        tUInt32 m_stopTime;

        /*! values of last messages send */

        tFloat32 m_last_pwm;
        tBool m_last_brakeLights;
        tBool m_last_reverseLights;
        tFloat32 m_last_DirectionSent;

        tInt32 m_initRun;
        tInt32 m_testRun;

        tTimeStamp m_testTimer_start;
        tInt32     m_testState;

        /*! Lock */
        cCriticalSection m_mutex;


	    /*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignaltSignalValue;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalint8;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal_pwm;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal_direction;
        cObjectPtr<IMediaTypeDescription> m_pCodeOutputbrakelight;

	
};

#endif // _SPEEDCONTROL_H_

