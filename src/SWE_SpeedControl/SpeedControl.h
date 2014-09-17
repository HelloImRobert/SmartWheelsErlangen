
#ifndef _SPEEDCONTROL_H_
#define _SPEEDCONTROL_H_

#define OID_ADTF_SWE_SPEEDCONTROL "adtf.aadc.SWE_SpeedControl"

/*!
* motor speed controller
*/
class SpeedControl : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_SPEEDCONTROL, "SWE motor speed control", OBJCAT_DataFilter, "Speed Control", 1, 0,0, "beta version");

        cInputPin m_oInputVelocity;				// the input pin for the measured value
        cInputPin m_oInputSetPoint;				// the input pin for the set point value (gear) -2(fast reverse), -1, 0(stop), 1, 2, 3(full speed ahead)
        cOutputPin m_oOutputPWM;                // the output pin for the manipulated value

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
		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);	

		/*! calculates the manipulated value for the given values, it uses the setpoint in m_setPoint
		@param measuredValue	the measuredValue
		*/
        tFloat32 getControllerValue(tFloat32 measuredSpeed);

        /*! update state of car according to current speed */
        tResult updateState();

        /*! switch on/off brake and reverse lights */
        tResult setBrakeLights (tBool state);
        tResult setReverseLights (tBool state);

		/*! returns the currentstreamtime*/
		tTimeStamp GetTime();
		
		/*! holds the last measuredValue */
        tFloat32 m_velocity;

        /*! holds the current setpoint/gear */
        tInt16 m_setPoint;

        /*! holds the last active gear/setpoint*/
        tFloat32 m_currentState;

		/*! holds the last sample time */
		tTimeStamp m_lastSampleTime;

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

        tFloat32 m_strongBrake;
        tFloat32 m_lightBrake;

        tFloat32 m_inv_strongBrake;
        tFloat32 m_inv_lightBrake;

        tFloat32 m_pwmScaler;

	    /*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignaltSignalValue;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalint8;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
	
};

#endif // _SPEEDCONTROL_H_

