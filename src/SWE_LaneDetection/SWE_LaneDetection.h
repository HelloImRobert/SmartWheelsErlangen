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
#ifndef _SWE_LANEDETECTION_H_
#define _SWE_LANEDETECTION_H_

#define OID_ADTF_LANEDETECTION_FILTER "adtf.swe.lanedetection"

#include <iostream>
#include <fstream>
#include "SWE_CrossingAnalyzer.h"

/**
 * @brief The cSWE_LaneDetection class
 * For this filter the X-Direction is defined as the direction the car is currently heading.
 *      -----> y Direction
 *      |
 *      |
 *      v
 *      x Direction
 */
class cSWE_LaneDetection : public adtf::cFilter
{
        ADTF_DECLARE_FILTER_VERSION( OID_ADTF_LANEDETECTION_FILTER , "SWE_LaneDetection", OBJCAT_DataFilter, "SWE_LaneDetection filter", 1, 0, 0, "Beta Version");

        public:
            cSWE_LaneDetection(const tChar* __info);
            virtual ~cSWE_LaneDetection();

            cVideoPin		_oVideoInputPin;               /**< the input pin for the video*/
            cVideoPin       _oInternRepresentationVideoOutputPin;              /**< outputpin for the debug video */
            cVideoPin       _oColorVideoOutputPin;              /**< outputpin for the debug video */
            cInputPin       m_oIntersectionPoints;
            cInputPin       m_oInputTrigger;
            cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
            cOutputPin     m_oSplinesPin;
            cOutputPin     m_CrossingIndicatorPin;

        protected:

            tResult Init(tInitStage eStage, __exception);
            tResult Shutdown(tInitStage eStage, __exception);
            tResult Start(__exception = NULL);

            // implements IPinEventSink
            tResult OnPinEvent( IPin* pSource , tInt nEventCode , tInt nParam1 ,  tInt nParam2 , IMediaSample* pMediaSample );

        private:

            /*! Coder Descriptors for the pins*/
            cObjectPtr<IMediaTypeDescription> m_pCoderDescCrossingIndicator;
            cObjectPtr<IMediaTypeDescription> m_pCoderDescSplines;

            tResult                     ProcessInput(IMediaSample* pSample);
            tResult                     InitTransformationMatrices( std::string pathExternalCameraParams );
            tResult                     InitPinFormats();

            tTimeStamp _timerStart;
            tBool _hassent;

            // internal Functions
            void                        getBlobDescriptions             (const std::vector< std::vector< cv::Point > >& contours , std::vector< BlobDescriptor >& blobs , std::vector<BlobDescriptor> &allBlobs);
            void                        getOrientation                  (BlobDescriptor& blob );
            int                         getOuterLaneBoundaries          (std::vector< BlobDescriptor >& blobs);
            std::pair< size_t, size_t > contourToSpline                 (const std::vector< cv::Point >& contour , const int splineSearchWidth , bool side = false );
            void                        drawSpline                      (cv::Mat& image, const std::vector< cv::Point2d >& splinePoints , const cv::Scalar& color);
            void                        drawStopLine                    (cv::Mat image, const CrossingDescriptor crossing , cv::Scalar color );
            void                        drawResultImage                 (cv::Mat& image, const std::vector<BlobDescriptor>& blobs, const int outerLaneBoundariesIndicator,
                                                                         const std::vector< BlobDescriptor* > middleLaneBoundary);
            void                        drawCrossing                    (const cv::Mat& image, const CrossingDescriptor& crossing);
            void                        project                         (const BlobDescriptor& inputBlob, BlobDescriptor& outputBlob ,
                                                                        const cv::Mat& projectionMatrix, int offset = 0 );
            void                        project                         (const std::vector< cv::Point2d >& inputContour, std::vector< cv::Point2d >& outputContour ,
                                                                         const cv::Mat& projectionMatrix, int offset = 0);
            bool                        checkBoundaryPresentAndComplex  (BlobDescriptor* boundary);
            bool                        calculateDirectionHistogram     (BlobDescriptor& blob);
            void                        transformToCarCoords            (std::vector< cv::Point2d >& spline );
            void                        transformFromCarCoords          (std::vector< cv::Point2d >& spline );

            void                        serializeLane                   (cObjectPtr<IMediaCoder>& pCoder , std::string lane , const std::vector< Point2d >& spline );
            tResult                     transmitLanes                   (const std::vector< Point2d >& leftSpline , const std::vector< Point2d >& middleSpline ,
                                                                         const std::vector< Point2d >& rightSpline );
            tResult                     transmitCrossingIndicator       (const tBool isRealStopLine , const tInt8 crossingType , const cv::Point2d& StopLinePoint1 ,
                                                                         const cv::Point2d& StopLinePoint2 );

            // Parameters for the algorithm
            bool                        _determineCrossing;
            double                      _ambigousAngle;
            bool                        _draw;
            double                      _resultImageScaleFactor;
            cv::Point                   _resultImageOffsetVector;
            double                      _widthFactor;
            double                      _middleDistanceHighThreshold;
            double                      _middleDistanceLowThreshold;
            double                      _minLengthMiddleBoundary;
            double                      _maxLengthMiddleBoundary;
            double                      _minOuterBoundaryLength;
            int                         _heightThresh;
            double                      _lowerAreaThreshold;
            size_t                      _startHeight;
            double                      _principalAxisLengthRatioThreshold;
            int                         _splineSearchWidth;
            int                         _CountStdDevs;                              /**< the count of stdDevs for the Tresholding*/

            double                      _distFrontToWarpedImageBottom;
            double                      _distFrontToFrontAxis;
            double                      _distSideImageToMid;

            CrossingAnalyzer            _analyzer;
            std::string                 _pathExternalCameraParams;                  /**< Path to the xml with the points of the inverse perspective transformation*/
            FileStorage                 _inversePerspectiveFileStorage;             /**< Stream to deserialize the XML with the points*/

            cv::Mat                     _backProjectionMatrix;
            cv::Mat                     _projectionMatrix;                          /**< the projection Matrix for the inverse Perspective Mapping*/
            cv::Mat                     _result;

            tBitmapFormat               _sInternRepresentationBitMapOutputFormat;              /**< the inputformat of the video*/
            tBitmapFormat               _sColorBitMapOutputFormat;                  /**< the inputformat of the video*/
            const tBitmapFormat*        _sBitMapInputFormat;                        /**< the outputformat for the debug video*/
        };

        //*************************************************************************************************
        #endif
