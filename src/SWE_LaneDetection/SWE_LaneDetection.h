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
            cVideoPin       _oGreyScaleVideoOutputPin;              /**< outputpin for the debug video */
            cVideoPin       _oColorVideoOutputPin;              /**< outputpin for the debug video */
            cOutputPin     m_oLines;
            cOutputPin     m_oSplines;

        protected:

            tResult Init(tInitStage eStage, __exception);
            tResult Shutdown(tInitStage eStage, __exception);

            // implements IPinEventSink
            tResult OnPinEvent( IPin* pSource , tInt nEventCode , tInt nParam1 ,  tInt nParam2 , IMediaSample* pMediaSample );

        private:

            tResult ProcessInput(IMediaSample* pSample);
            tResult InitTransformationMatrices( std::string pathExternalCameraParams );
            tResult InitPinFormats();

            std::pair< cv::Point, double > computeEnergy(const cv::Mat& src, const cv::Point2i& start, const cv::Point2i& directionStart, const cv::Point2i& directionEnd, size_t steps);
            cv::Point findNextPoint(const cv::Mat& src, const cv::Point2i& start, const cv::Point2i& directionStart, size_t steps);

            cv::Mat _backProjectionMatrix;
            cv::Mat _projectionMatrix;                     /**< the projection Matrix for the inverse Perspective Mapping*/

            tBitmapFormat _sGreyScaleBitMapOutputFormat;            /**< the inputformat of the video*/
            tBitmapFormat _sColorBitMapOutputFormat;            /**< the inputformat of the video*/
            const tBitmapFormat*   _sBitMapInputFormat;    /**< the outputformat for the debug video*/

            // Parameters for cutting the image
            tInt _leftBorder;
            tInt _rightBorder;
            tInt _upperBorder;
            tInt _lowerBorder;
            tBool _applyCut;                                    /**< Boolean indicating wether the image should be cut*/

            // Parameters for the algorithm
            std::string _pathExternalCameraParams;         /**< Path to the xml with the points of the inverse perspective transformation*/
            FileStorage _inversePerspectiveFileStorage;    /**< Stream to deserialize the XML with the points*/

            tInt _CountStdDevs;                            /**< the count of stdDevs for the Tresholding*/

            tFloat64 _MinAngle;
            tFloat64 _MaxAngle;

            tFloat32 _thetaMax;
            tFloat32 _rhoMax;

            /*! Coder Descriptors for the pins*/
            cObjectPtr<IMediaTypeDescription> m_pCoderDescLines;
            cObjectPtr<IMediaTypeDescription> m_pCoderDescSplines;
        };

        //*************************************************************************************************
        #endif
