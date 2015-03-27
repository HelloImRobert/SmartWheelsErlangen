#ifndef _SWE_BACKLIGHTSFINDER_H_
#define _SWE_BACKLIGHTSFINDER_H_

#define OID_ADTF_SWE_BACKLIGHTSFINDER "adtf.swe.backlightsfinder"

class SWE_BackLightsFinder : public adtf::cFilter
{
        ADTF_DECLARE_FILTER_VERSION( OID_ADTF_SWE_BACKLIGHTSFINDER , "SWE_BackLightsFinder", OBJCAT_DataFilter, "SWE_BackLightsFinder filter", 1, 0, 0, "Beta Version");

        public:
            SWE_BackLightsFinder(const tChar* __info);
            virtual ~SWE_BackLightsFinder();

            cVideoPin		_oVideoInputPin;               /**< the input pin for the video*/
            cVideoPin       _oColorVideoOutputPin;              /**< outputpin for debugging */
            cInputPin       m_oInputDistances;
            cOutputPin      m_oTrackingPoint;

            /*! output pin for the vehicle speed */
            cOutputPin m_oOutputDistance;

        protected:

            tResult Init(tInitStage eStage, __exception);
            tResult Shutdown(tInitStage eStage, __exception);

            // implements IPinEventSink
            tResult OnPinEvent( IPin* pSource , tInt nEventCode , tInt nParam1 ,  tInt nParam2 , IMediaSample* pMediaSample );

        private:

            std::vector< cv::Point > _referenceContour;

            // Parameters for the algorithm
            std::string _path;
            cv::FileStorage _fs;

            tResult ProcessInput(IMediaSample* pSample);
            tResult InitPinFormats();

            tBitmapFormat _sColorBitMapOutputFormat;            /**< the inputformat of the video*/

            const tBitmapFormat*   _sBitMapInputFormat;    /**< the outputformat for the debug video*/

            cv::Point getOrientation(const std::vector< cv::Point >& contour, std::vector<cv::Point2d>& eigen_vecs, std::vector<double>& eigen_vals);

            tResult transmitTrackingPoint(cv::Point2d trackingPoint);
            tResult transmitResultVideo(cv::Size size , std::vector< vector< cv::Point > >& contours , cv::Point &trackingPoint);
            tResult sendDistance(float velocity);

            void writeShape( std::vector< cv::Point >& points );
            std::vector< cv::Point > readShape();

            int _lowHueTresh;
            int _highHueTresh;

            int _lowSaturationTresh;
            int _highSaturationThresh;

            int _lowValueTresh;
            int _highValueTresh;

            size_t _startHeight;

            double _contourLengthThreshold;

            cv::Point2d _objectDistances[10];

            cCriticalSection m_mutex;

            cObjectPtr<IMediaTypeDescription> m_pCoderDescPoints;
            cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;

            cObjectPtr<IMediaTypeDescription>  m_pCoderVelocityOut;
        };

        //*************************************************************************************************
        #endif
