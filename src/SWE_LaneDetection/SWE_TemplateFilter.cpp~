/**
 *
 * ADTF Template Project
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: belkera $
 * $Date: 2011-06-30 16:51:21 +0200 (Thu, 30 Jun 2011) $
 * $Revision: 26514 $
 *
 * @remarks
 *
 */
#include "stdafx.h"
#include "SWE_TemplateFilter.h"
#include <template_data.h>

/// Create filter shell
ADTF_FILTER_PLUGIN("SWE_TemplateFilter", OID_ADTF_TEMPLATE_FILTER, cTemplateProjectFilter);


cTemplateProjectFilter::cTemplateProjectFilter(const tChar* __info):cFilter(__info)
{

}

cTemplateProjectFilter::~cTemplateProjectFilter()
{

}

tResult cTemplateProjectFilter::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        // get a media type for the input pin
        cObjectPtr<IMediaType> pInputType;
        RETURN_IF_FAILED(AllocMediaType(&pInputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));
        
        // create and register the input pin
        RETURN_IF_FAILED(m_oTemplateInput.Create("input_template", pInputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oTemplateInput));

        // get a media type for the output pin
        cObjectPtr<IMediaType> pOutputType;
        RETURN_IF_FAILED(AllocMediaType(&pOutputType, MEDIA_TYPE_TEMPLATE, MEDIA_SUBTYPE_TEMPLATE, __exception_ptr));
        
        // create and register the output pin
        RETURN_IF_FAILED(m_oTemplateOutput.Create("output_template", pOutputType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oTemplateOutput));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cTemplateProjectFilter::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: 
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cTemplateProjectFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &m_oTemplateInput)
        {
            // this will store the value for our new sample
            tTemplateData fNewValue;
            
            // now lets access the data in the sample,
            // the Lock method gives you access to the buffer of the sample.
            // we use a scoped sample lock to ensure that the lock is released in all circumstances.

            {
                // this will aquire the read lock on the sample and declare and initialize a pointer to the data
                __sample_read_lock(pMediaSample, tTemplateData, pData);
                // now we can access the sample data through the pointer
                fNewValue = *pData + 1.0;
                // the read lock on the sample will be released when leaving this scope
            }

            // now we need a new media sample to forward the data.
            cObjectPtr<IMediaSample> pNewSample;
            if (IS_OK(AllocMediaSample(&pNewSample)))
            {
                // now set its data
                // we reuse the timestamp from the incoming media sample. Please see the api documentation
                // (ADTF Extreme Programmers -> The ADTF Streamtime) for further reference on how sample times are handled in ADTF
                pNewSample->Update(pMediaSample->GetTime(), &fNewValue, sizeof(tTemplateData), 0);

                // and now we can transmit it
                m_oTemplateOutput.Transmit(pNewSample);
            }
        }
    }

    RETURN_NOERROR;
}
