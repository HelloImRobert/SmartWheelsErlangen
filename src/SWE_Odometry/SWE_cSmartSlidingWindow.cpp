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

#define TIMESTAMP_RESOLUTION 1000.0f //TODO : micro- or milli-seconds?
#define AVG_UPDATES_PER_SECOND 18.0f


SWE_cSmartSlidingWindow::SWE_cSmartSlidingWindow( tUInt32 in_max_length, tFloat32 in_maxDelay):
    m_maxLength(in_max_length), m_resolution(10), m_queueValueSum(0), m_queueTimeSum(0), m_lastPulseTime(0) , m_maxDelay(in_maxDelay), m_firstSample (true)
{
}

SWE_cSmartSlidingWindow::~SWE_cSmartSlidingWindow(){}

tVoid SWE_cSmartSlidingWindow::AddNewValue(tInt32 inputDelta, tTimeStamp inputTime)
{
    if (m_firstSample)
    {
        m_lastPulseTime = inputTime;
        m_firstSample = false;
    }

    // ------- discard old values -------

    // are those/this last sample(s) too old to be kept?
    while ( ( m_myQueue.size() > 0 )  && ( ( inputTime - ( m_lastPulseTime - m_queueTimeSum ) ) > 2*m_maxDelay ) ) //last sample too old?
    {
            m_queueTimeSum -= GetEndTime();
            m_queueValueSum -= GetEndValue();

            m_myQueue.pop();
    }


    // ------------- fill the queue ------------

    if (inputDelta > 0) //was a new pulse registered?
    {
        tInt32 delta_time;

        delta_time = inputTime - m_lastPulseTime;

        if ((delta_time < m_maxDelay) && ( delta_time > 0))// not too slow/old to be registered, or first pulse after a long time?
        {
            //save relative values
            m_myQueue.push(make_pair<tInt32,tTimeStamp>(inputDelta, (tTimeStamp)delta_time));

            //save absolute values
            m_queueValueSum += inputDelta;
            m_queueTimeSum += delta_time;
        }

        m_lastPulseTime = inputTime;

        //DEBUG
        debugvar = m_queueTimeSum;
    }


    // ---------- shorten the queue ----------------
    //shorten until minimum resolution reached, to keep a short delay and use minimal averaging

    while ( (m_myQueue.size() > 2) &&  (  (m_myQueue.size() > m_maxLength)  ||  (GetNextRes() >= m_resolution ) )) //is cutting off the last sample OK? 1. long enough to cut sth. off AND (2. cutting off results in high enough resolution OR 3. queue is too long)
    {
            m_queueTimeSum -= GetEndTime();
            m_queueValueSum -= GetEndValue();

            m_myQueue.pop();
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
    if (m_myQueue.size() > 0U)
        return m_myQueue.front().first;
    else
        return 0;
}

tTimeStamp SWE_cSmartSlidingWindow::GetEndTime()
{
    if (m_myQueue.size() > 0U)
        return m_myQueue.front().second;
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

    if (m_myQueue.size() > 0U)
        next_res = (m_queueValueSum - GetEndValue())   *   (((tFloat32)(m_queueTimeSum - GetEndTime()) * AVG_UPDATES_PER_SECOND) / TIMESTAMP_RESOLUTION) ;
    else
        next_res = 0;

    return next_res;
}

tResult SWE_cSmartSlidingWindow::Reset()
{

    while (m_myQueue.size() > 0U)
    {
        m_myQueue.pop();
    }

    m_queueValueSum = 0;
    m_queueTimeSum = 0.0;
    m_firstSample = true;

    RETURN_NOERROR;
}

