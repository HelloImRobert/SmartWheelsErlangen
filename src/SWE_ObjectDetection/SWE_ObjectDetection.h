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
#ifndef _SWE_OBJECTDETECTION_H_
#define _SWE_OBJECTDETECTION_H_

#define OID_ADTF_OBJECTDETECTION_FILTER "adtf.swe.ObjectDetection"

#include <iostream>
#include <fstream>
#include <deque>

class cSWE_ObjectDetection : public adtf::cFilter
{
        ADTF_DECLARE_FILTER_VERSION( OID_ADTF_OBJECTDETECTION_FILTER , "SWE_ObjectDetection", OBJCAT_DataFilter, "SWE_ObjectDetection filter", 1, 0, 0, "Beta Version");

        public:
            cSWE_ObjectDetection(const tChar* __info);
            virtual ~cSWE_ObjectDetection();

            cVideoPin		m_oVideoInputPin;               /**< the input pin for the video*/
            cVideoPin       m_oGreyScaleVideoOutputPin;              /**< outputpin for the debug video */

        protected:

            tResult Init(tInitStage eStage, __exception);
            tResult Shutdown(tInitStage eStage, __exception);

            // implements IPinEventSink
            tResult OnPinEvent( IPin* pSource , tInt nEventCode , tInt nParam1 ,  tInt nParam2 , IMediaSample* pMediaSample );

        private:

            tResult ProcessInput(IMediaSample* pSample);

            tResult InitTransformationMatrices( std::string pathExternalCameraParams );
            tResult InitPinFormats();
            tResult FilterImageInput( cv::Mat& image );

            cv::Mat m_backProjectionMatrix;
            cv::Mat m_projectionMatrix;                     /**< the projection Matrix for the inverse Perspective Mapping*/

            tBitmapFormat m_sGreyScaleBitMapOutputFormat;            /**< the inputformat of the video*/
            const tBitmapFormat*   m_sBitMapInputFormat;    /**< the outputformat for the debug video*/

            // Parameters for cutting the image
            tInt m_leftBorder;
            tInt m_rightBorder;
            tInt m_upperBorder;
            tInt m_lowerBorder;
            tBool m_cut;                                    /**< Boolean indicating wether the image should be cut*/

            // Parameters for the algorithm
            std::string m_pathExternalCameraParams;         /**< Path to the xml with the points of the inverse perspective transformation*/
            FileStorage m_inversePerspectiveFileStorage;    /**< Stream to deserialize the XML with the points*/

            tInt m_KernelWidth;                             /**< the width of the Kernel for the Detection of OBJECT Markings*/
            tInt m_CountStdDevs;                            /**< the count of stdDevs for the Tresholding*/
            tInt m_KernelWidthPeaks;                        /**< the width of the Kernel for the Smoothing 1D OBJECT position indicator*/
            tInt m_RoadWidth;                               /**< the expected width of the road in pixels*/
            tInt m_RoadWidthTolerance;                      /**< the tolerance on the width of the road*/
            tInt m_FitWidth;                                /**< the width around the candidate positions for the OBJECTs to fit the (sp)lines*/
            tInt m_MaxCountOBJECTs;                           /**< the maximum number of OBJECTs which should be found*/

            tFloat64 m_MinAngle;
            tFloat64 m_MaxAngle;

            // Kernels for the Algorithm
            cv::Mat m_YDirKernel;                           /**< the Second Derivate of Gaussian Kernel in Y-Direction for the Detection of OBJECT Markings*/
            cv::Mat m_XDirKernel;                           /**< the Gaussian Kernel in X-Direction for the Detection of OBJECT Markings*/


            //new members
            size_t _MagicCounter;
            std::deque< Mat > _Images;
        };

        //*************************************************************************************************
        #endif
