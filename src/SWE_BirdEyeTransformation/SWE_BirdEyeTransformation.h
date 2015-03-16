#ifndef _SWE_BIRDEYETRANSFORMATION_H_
#define _SWE_BIRDEYETRANSFORMATION_H_

#define OID_ADTF_BIRDEYETRANSFORMATION_FILTER "adtf.swe.birdeyetrafo"

#include <iostream>
#include <fstream>

class cSWE_BirdEyeTransformation : public adtf::cFilter
{
        ADTF_DECLARE_FILTER_VERSION( OID_ADTF_BIRDEYETRANSFORMATION_FILTER , "SWE_BirdEyeTrafo", OBJCAT_DataFilter, "SWE_BirdEyeTrafo filter", 1, 0, 0, "Beta Version");

        public:
            cSWE_BirdEyeTransformation(const tChar* __info);
            virtual ~cSWE_BirdEyeTransformation();

            cVideoPin		_oVideoInputPin;               /**< the input pin for the video*/
            cVideoPin       _oColorVideoOutputPin;              /**< outputpin for the debug video */
            cOutputPin      _oOutputTrigger;                    /*! trigger event to synchronize the calculation of odometry etc. */

        protected:

            tResult Init(tInitStage eStage, __exception);
            tResult Shutdown(tInitStage eStage, __exception);

            // implements IPinEventSink
            tResult OnPinEvent( IPin* pSource , tInt nEventCode , tInt nParam1 ,  tInt nParam2 , IMediaSample* pMediaSample );

        private:

            tResult ProcessInput(IMediaSample* pSample);
            tResult InitTransformationMatrices( std::string pathExternalCameraParams );
            tResult InitPinFormats();

            cv::Mat _backProjectionMatrix;
            cv::Mat _projectionMatrix;                     /**< the projection Matrix for the inverse Perspective Mapping*/

            cv::VideoWriter resultVideo;
            bool _dumpVideo;

            tBitmapFormat _sColorBitMapOutputFormat;            /**< the inputformat of the video*/
            const tBitmapFormat*   _sBitMapInputFormat;    /**< the outputformat for the debug video*/

            // Parameters for the algorithm
            std::string _pathExternalCameraParams;         /**< Path to the xml with the points of the inverse perspective transformation*/
            FileStorage _inversePerspectiveFileStorage;    /**< Stream to deserialize the XML with the points*/

            tInt32 m_frameCounter;

            // descriptor
            cObjectPtr<IMediaTypeDescription>  _pCodeTrigger;
        };

        //*************************************************************************************************
        #endif
