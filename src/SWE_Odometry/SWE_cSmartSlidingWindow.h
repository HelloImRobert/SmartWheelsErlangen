/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/

#include "stdafx.h"

#ifndef _SLIDINGWINDOW_H_
#define _SLIDINGWINDOW_H_

/*! SmartWheels-Erlangen (Robert de Temple): This class defines a smart sliding window filter.
 * The datatypes are tInt32 and tTimeStamp (hoping that Audi will stop that tFloat stupidity for counting variables soon ;) )
 * It returns the amount of ticks and the timeframe in which they occured.
 * The length of the timeframe depends on the data and is dynamically adjusted to always ensure a minimum accuracy without wasting any dynamics on unneccessary averaging.
 * 1/in_resolution is the maximum allowable error in this measurement (at least theoretically).
 * Higher values result in higher accuracy but also slower dynamics.
 * The maximum averaged window size + delay is in_resolution samples (~ in_resolution / 18 seconds) and only occurs at very low speeds.
 * At higher speeds the delay goes down to 1 samples.
 * Its interface is based on code provided by Audi (cSlidingWindow)*/

class SWE_cSmartSlidingWindow
{
public:

    /*! constructor of the class
    @param in_max_length sets the max length of the queue
    */
    SWE_cSmartSlidingWindow(tInt32 in_max_length, tFloat32 in_maxDelay);

    /*! destructor */
    ~SWE_cSmartSlidingWindow();

    /*! adds a new tFloat32 value to the internal queue
    @param inputValues    the value of the new sample
    @param inputTime    the timestamp of the new sample
    */
    tVoid AddNewValue(tInt32 inputValues, tTimeStamp inputTime);

    tInt32 GetTicks();

    tTimeStamp GetTime();

    /*! reset everything */
    tResult Reset();

    /*! set the resolution */
    tResult SetResolution(tInt32 in_resolution);


private:

    /*! gets the first sample of the defined sliding window*/
    tInt32 GetBeginValue();
    /*! gets the last sample of the defined sliding window*/
    tInt32 GetEndValue();
    /*! gets the timestamp of the first sample of the defined sliding window*/
    tTimeStamp GetBeginTime();
    /*! gets the timestamp of the last sample of the defined sliding window*/
    tTimeStamp GetEndTime();
    /*! get the resulting resolution if the last sample were to be cut off */
    tFloat32 GetNextRes();


    /*! queue which holds all the samples with their timestamps as a pair*/
    queue< pair<tInt32,tTimeStamp> > m_inputValues;

    /*! length of the queue, i.e. the length of the sliding window*/
    tInt32 m_maxLength;
    tInt32 m_resolution;
    tFloat32 m_queueValueSum;
    tTimeStamp m_queueTimeSum;
    tTimeStamp m_lastTickTime;
    tFloat32 m_maxDelay;
    tInt32 m_lastTickValue;
};

#endif
