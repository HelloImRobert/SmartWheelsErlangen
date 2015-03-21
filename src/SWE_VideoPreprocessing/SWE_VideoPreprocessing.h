#ifndef _SWE_VIDEOPREPROCESSING_H_
#define _SWE_VIDEOPREPROCESSING_H_

#define OID_ADTF_SWE_VIDEOPREPROCESSING "adtf.swe.videopreprocessing"

#include <iostream>
#include <fstream>

class SWE_VideoPreprocessing : public adtf::cFilter
{
        ADTF_DECLARE_FILTER_VERSION( OID_ADTF_SWE_VIDEOPREPROCESSING , "SWE_VideoPreprocessing", OBJCAT_DataFilter, "SWE_VideoPreprocessing filter", 1, 0, 0, "Beta Version");

        public:
            SWE_VideoPreprocessing(const tChar* __info);
            virtual ~SWE_VideoPreprocessing();

            cVideoPin		_oVideoInputPin;               /**< the input pin for the video*/
            cVideoPin       _oColorVideoOutputPin;              /**< outputpin for the lane detection */
            cVideoPin       _oColorVideoOutputPinCropped;              /**< outputpin for the bitcode detection */
            cOutputPin      _oOutputTrigger;                    /*! trigger event to synchronize the calculation of odometry etc. */

        protected:

            tResult Init(tInitStage eStage, __exception);
            tResult Shutdown(tInitStage eStage, __exception);

            // implements IPinEventSink
            tResult OnPinEvent( IPin* pSource , tInt nEventCode , tInt nParam1 ,  tInt nParam2 , IMediaSample* pMediaSample );

        private:

            tResult ProcessInput(IMediaSample* pSample);
            tResult InitPinFormats();


            cv::VideoWriter resultVideo;
            bool _dumpVideo;

            tBitmapFormat _sColorBitMapOutputFormat;            /**< the inputformat of the video*/
            tBitmapFormat _sColorBitMapOutputFormatCropped;            /**< the inputformat of the cropped video*/
            const tBitmapFormat*   _sBitMapInputFormat;    /**< the outputformat for the debug video*/

            // Parameters for the algorithm
            std::string _pathExternalCameraParams;         /**< Path to the xml with the points of the inverse perspective transformation*/

            tTimeStamp m_frameCounter;

            // descriptor
            cObjectPtr<IMediaTypeDescription>  _pCodeTrigger;
        };

        //*************************************************************************************************
        #endif
