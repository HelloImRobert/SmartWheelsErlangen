/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "stdafx.h"

#ifndef _SWE_ControllerFilter_H_
#define _SWE_ControllerFilter_H_

#define OID_ADTF_SWE_PIDCONTROLLER "adtf.swe.pid_controller"

/*!
*-------------------------------------------------------------------------------------------------------------------
* Greatly enhanced PID controller with feed-forward control to allow for cascading of controllers etc.. Specially designed to work with the car's motor.
* Smart-Wheels Erlangen (Robert de Temple). 
* Based on the PID controller provided by Audi.
*
* => All inputs are expected to be scaled in X/second.
* - max/min output: is the maximum value the controller combined with the feed forward signal can put out (usually the limits of the motor inputs (+/- 100)
* - use feed forward: enables the feed forward input. the value generated by the controller is added to the feed forward signal.
* - max controller influence upper/lower: the maximum ouput the controller can internally generate, to limit it's influence. This also limits the internal accumulation variable of the integrator as otherwise there would be a strong affinity for stable oscillations due to the strong non-liearity introduced by the limits (read up on the theory of non-linear controller structures if you wonder why). => these limits can be dynamically scaled with the controller_strength input pin. e.g to turn off the controller influence on braking maneuvers.
* - Feed-Forward = 0 => output = 0: if true then the controller generates no output if the feed-forward signla is 0. This basically turns of the controller when the pre-control sets the motor to stop. This is a safety measure to enable safe stops. i.e. the car could start to accelerate if you pick it up without this measure.
* ------------------------------------------------------------------------------------------------------------------
*/

class SWE_ControllerFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_PIDCONTROLLER, "SWE_PID_Controller", OBJCAT_DataFilter, "SWE_PID_Controller", 1, 0,0, "Beta Version");

    cInputPin m_oInputMeasured;                // the input pin for the measured value
    cInputPin m_oInputSetPoint;                // the input pin for the set point value
    cInputPin m_oInputFeedForward;	           // Input for the feed-forward control value
    cInputPin m_oInputControllerStrength;      // Input to set the current controller strenght => dynamic scaling of the upper and lower bounds of the controller influence, this value is not always needed for normal operation. It is used to reduce the influence of the controller on braking maneuvers etc. by our speed control.
    cOutputPin m_oOutputManipulated;           // the output pin for the manipulated value

public:
    SWE_ControllerFilter(const tChar* __info);
    virtual ~SWE_ControllerFilter();
    
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
    @param measuredValue    the measuredValue
    */
    tFloat32 getControllerValue(tFloat32 measuredValue);

    /*! returns the currentstreamtime*/
    tTimeStamp GetTime();

    /*! limits value to the given bounds */
    tFloat32 LimitValue(tFloat32 inputVar, tFloat32 upperBound, tFloat32 lowerBound);

    /*! holds the last measuredValue */
    tFloat32 m_measuredVariable;
    /*! holds the last measured error */
    tFloat32 m_lastMeasuredError;
    /*! holds the last setpoint */
    tFloat32 m_setPoint ;
    /*! holds the last feed-forward value */
    tFloat32 m_feedForward ;
    /*! holds the last sample time */
    tTimeStamp m_lastSampleTime;
    /*! holds the accumulatedVariable for the controller*/
    tFloat32 m_accumulatedVariable;
    /*! dynamic scaling of the upper and lower bounds of the controller influence */
    tFloat32 m_controllerStrength;

    /*! these variables hold the property values */
    tFloat32 m_Kp;
    tFloat32 m_Ki;
    tFloat32 m_Kd;
    tFloat32 m_sampleIntervall;

    tFloat32 m_maxOutput;
    tFloat32 m_minOutput;

    tFloat32 m_maxInfluence_upper;
    tFloat32 m_maxInfluence_lower;

    tInt32 m_type;

    tBool m_useFF;
    tBool m_offMeansOff;
    tBool m_useAutoSampleTime;

    /*! last time the output was used */
    tTimeStamp m_lastOutputTime;

    /*! Lock */
    cCriticalSection m_mutex;

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    
};

#endif // _SWE_ControllerFilter_H_

