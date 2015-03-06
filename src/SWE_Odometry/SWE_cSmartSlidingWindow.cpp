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
//#include "stdafx.h"
#include "SWE_cSmartSlidingWindow.h"

#define TIMESTAMP_RESOLUTION 1000 //TODO : micro- or milli-seconds?
#define AVG_UPDATES_PER_SECOND 18.0f


SWE_cSmartSlidingWindow::SWE_cSmartSlidingWindow( tInt32 in_max_length, tFloat32 in_maxDelay):
m_maxLength(in_max_length), m_resolution(10), m_queueValueSum(0), m_queueTimeSum(0), m_maxDelay(in_maxDelay), m_lastPulseTime(0) , m_lastPulseValue(0)
{
}

SWE_cSmartSlidingWindow::~SWE_cSmartSlidingWindow(){}

tVoid SWE_cSmartSlidingWindow::AddNewValue(tInt32 inputValues, tTimeStamp inputTime)
{

    // ------- discard old values -------

    // are those/this last sample(s) too old to be kept?
    while ( ( m_inputValues.size() > 0 )  && ( ( inputTime - ( m_lastPulseTime - m_queueTimeSum ) ) > 2*m_maxDelay ) ) //last sample too old?
    {
            m_queueTimeSum -= GetEndTime();
            m_queueValueSum -= GetEndValue();

            m_inputValues.pop();
    }

    // ------------- fill the queue ------------

    if (m_lastPulseValue < inputValues) //was a pulse registered?
    {
        tInt32 delta_pulses;
        tFloat32 delta_time;

        delta_pulses = inputValues - m_lastPulseValue;
        delta_time = inputTime - m_lastPulseTime;

        if (delta_time < m_maxDelay)// not too slow/old to be registered?
        {
        //save relative values
        m_inputValues.push(make_pair<tInt32,tTimeStamp>(delta_pulses, delta_time));

        //save absolute values
        m_queueValueSum += delta_pulses;
        m_queueTimeSum += delta_time;
        }

        m_lastPulseTime = inputTime;
        m_lastPulseValue = inputValues;
    }


    // ---------- shorten the queue ----------------
    //shorten until minimum resolution reached, to keep a short delay and use minimal averaging

    while ( (m_inputValues.size() > 1) &&  (  (m_inputValues.size() > m_maxLength)  ||  (GetNextRes() >= m_resolution ) )) //is cutting off the last sample OK? 1. long enough to cut sth. off AND (2. cutting off results in high enough resolution OR 3. queue is too long)
    {
            m_queueTimeSum -= GetEndTime();
            m_queueValueSum -= GetEndValue();

            m_inputValues.pop();
    }
}

tTimeStamp SWE_cSmartSlidingWindow::GetTime()
{
    return m_queueTimeSum;
}

tInt32 SWE_cSmartSlidingWindow::GetPulses()
{
    return m_queueValueSum;
}


tInt32 SWE_cSmartSlidingWindow::GetEndValue()
{
    if (m_inputValues.size() > 0)
        return m_inputValues.back().first;
    else
        return 0;
}

tTimeStamp SWE_cSmartSlidingWindow::GetEndTime()
{
    if (m_inputValues.size() > 0)
        return m_inputValues.back().second;
    else
        return 0;
}

tResult SWE_cSmartSlidingWindow::SetResolution(tInt32 in_resolution)
{

    m_resolution = in_resolution;

    RETURN_NOERROR;
}

tFloat32 SWE_cSmartSlidingWindow::GetNextRes()
{
    tFloat32 next_res;

    if (m_inputValues.size() > 0)
        next_res = (m_queueValueSum - GetEndValue()) * ((m_queueTimeSum - GetEndTime()) / ((1.0 / AVG_UPDATES_PER_SECOND) * TIMESTAMP_RESOLUTION ) );
    else
        next_res = 0;

    return next_res;
}

tResult SWE_cSmartSlidingWindow::Reset()
{

    while (m_inputValues.size() > 0)
    {
        m_inputValues.pop();
    }

    m_queueValueSum = 0;
    m_queueTimeSum = 0.0;

    RETURN_NOERROR;
}

