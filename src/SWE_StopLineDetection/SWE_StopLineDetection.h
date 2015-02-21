#ifndef _SWE_STOPLINEDETECTION_H_
#define _SWE_STOPLINEDETECTION_H_

#define OID_ADTF_STOPLINEDETECTION_FILTER "adtf.swe.stoplinedetection"

#include <iostream>
#include <fstream>

/**
 * @brief
 */
class cSWE_StopLineDetection : public adtf::cFilter
{
        ADTF_DECLARE_FILTER_VERSION( OID_ADTF_STOPLINEDETECTION_FILTER , "SWE_StopLineDetection", OBJCAT_DataFilter, "SWE_StopLineDetection filter", 1, 0, 0, "Beta Version");

        public:
            cSWE_StopLineDetection(const tChar* __info);
            virtual ~cSWE_StopLineDetection();

            cVideoPin		_oVideoInputPin;               /**< the input pin for the video*/
            cInputPin       m_oActivationPin;
            cVideoPin       _oColorVideoOutputPin;              /**< outputpin for the debug video */
            cOutputPin      m_oLines;

        protected:

            tResult Init(tInitStage eStage, __exception);
            tResult Shutdown(tInitStage eStage, __exception);

            // implements IPinEventSink
            tResult OnPinEvent( IPin* pSource , tInt nEventCode , tInt nParam1 ,  tInt nParam2 , IMediaSample* pMediaSample );

        private:

            tResult ProcessInput(IMediaSample* pSample);

            tResult InitTransformationMatrices( std::string pathExternalCameraParams );
            tResult InitPinFormats();

            std::pair< size_t, float > SearchMaximum(cv::Mat scalarMat);
            float computeEnergy(const cv::Mat& src, const cv::Point2i& start, const cv::Point2i& end);

            cv::Mat _backProjectionMatrix;
            cv::Mat _projectionMatrix;                     /**< the projection Matrix for the inverse Perspective Mapping*/

            tBitmapFormat _sColorBitMapOutputFormat;            /**< the inputformat of the video*/
            const tBitmapFormat*   _sBitMapInputFormat;    /**< the outputformat for the debug video*/

            // Parameters for cutting the image
            tInt m_startHeight;

            // Parameters for the algorithm
            std::string _pathExternalCameraParams;         /**< Path to the xml with the points of the inverse perspective transformation*/
            cv::FileStorage _inversePerspectiveFileStorage;    /**< Stream to deserialize the XML with the points*/

            tBool m_isActive;
            float m_threshold;
            size_t m_fitWidth;
            tInt _CountStdDevs;                            /**< the count of stdDevs for the Tresholding*/

            /*! Coder Descriptors for the pins*/
            cObjectPtr<IMediaTypeDescription> m_pCoderDescLines;
        };

        //*************************************************************************************************
        #endif
