/**
 *
 * ADTF Template Project Filter.
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
#ifndef _SWE_TemplateFilter_H_
#define _SWE_TemplateFilter_H_

#define OID_ADTF_TEMPLATE_FILTER "adtf.swe.TemplateFilter"


//*************************************************************************************************
class cSWE_TemplateFilter : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, "SWE_TemplateFilter", adtf::OBJCAT_DataFilter);

protected:
   cVideoPin		m_oVideoInputPin;		/**< the input pin for the video*/
//TODO:
   //cOutputPin     	m_oGLCOutput;			/**< the output pin*/

public:
    cSWE_TemplateFilter(const tChar* __info);
    virtual ~cSWE_TemplateFilter();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

private:
	cv::Mat m_grey;					/**< matrix for the gray image*/
	std::string m_pathExternalCameraParams; 
	tInt m_KernelWidth;
	tInt m_CountStdDevs;
	tInt m_KernelWidthPeaks;
	tInt m_RoadWidth;
	tInt m_RoadWidthTolerance;
	tInt m_FitWidth;
	tInt m_MaxCountLanes;
};

//*************************************************************************************************
#endif // _TEMPLATE_PROJECT_FILTER_H_
