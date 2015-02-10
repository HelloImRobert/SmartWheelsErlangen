#include "stdafx.h"
#include "SWE_ObjectDetection.h"
#include <template_data.h>

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_ObjectDetection", OID_ADTF_OBJECTDETECTION_FILTER , cSWE_ObjectDetection);

// Macros used to decouple text from code
#define LEFT_SEARCH_BORDER "Cut Left Border for OBJECT Detection"
#define RIGHT_SEARCH_BORDER "Cut Right Border for OBJECT Detection"
#define UPPER_SEARCH_BORDER "Cut Upper Border for OBJECT Detection"
#define LOWER_SEARCH_BORDER "Cut Lower Border for OBJECT Detection"
#define CORRESPING_POINTS_XML "Path to external Camera Params xml"
#define KERNEL_WIDTH "Width of the Kernel"
#define SMOOTHING_KERNEL_WIDTH "Width of the Smoothing Kernel"
#define COUNT_OF_STDDEVS "Stddevs for the Thresholding"
#define ROAD_WIDTH "Width of the Road in pixels"
#define ROAD_WIDTH_TOLERANCE "Tolerance of the Distance between OBJECTs"
#define FIT_WIDTH "Width of the area for line fitting"
#define MAX_COUNT_OBJECTS "Maximum possible count of OBJECTs"
#define MAX_ANGULAR_DEVIATION "Allowed deviation from 90°"
#define FRAME_MEMORY "number of frames used for hole filling"

/**
 * @brief cSWE_ObjectDetection::cSWE_ObjectDetection
 * Constructs a cSWE_ObjectDetection object, defining it's properties, to be set by ADTF;
 * @param __info The info object for the Filter
 */
cSWE_ObjectDetection::cSWE_ObjectDetection(const tChar* __info):cFilter(__info)
{
    // set the cut parameter default to not cut the image
    m_cut = tFalse;

    SetPropertyInt( MAX_ANGULAR_DEVIATION , 20 );
    SetPropertyBool( MAX_ANGULAR_DEVIATION NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( MAX_ANGULAR_DEVIATION NSSUBPROP_MINIMUM  , 1 );
    SetPropertyInt( MAX_ANGULAR_DEVIATION NSSUBPROP_MAXIMUM  , 90 );

    SetPropertyInt( LEFT_SEARCH_BORDER , 0);
    SetPropertyBool( LEFT_SEARCH_BORDER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( LEFT_SEARCH_BORDER NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( LEFT_SEARCH_BORDER NSSUBPROP_MAXIMUM  , 640);

    SetPropertyInt( RIGHT_SEARCH_BORDER , 640);
    SetPropertyBool( RIGHT_SEARCH_BORDER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( RIGHT_SEARCH_BORDER NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( RIGHT_SEARCH_BORDER NSSUBPROP_MAXIMUM  , 640);

    SetPropertyInt( UPPER_SEARCH_BORDER , 0);
    SetPropertyBool( UPPER_SEARCH_BORDER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( UPPER_SEARCH_BORDER NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( UPPER_SEARCH_BORDER NSSUBPROP_MAXIMUM  , 480);

    SetPropertyInt( LOWER_SEARCH_BORDER, 480);
    SetPropertyBool( LOWER_SEARCH_BORDER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( LOWER_SEARCH_BORDER NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( LOWER_SEARCH_BORDER NSSUBPROP_MAXIMUM  , 480);

    SetPropertyInt( KERNEL_WIDTH , 3 );
    SetPropertyBool( KERNEL_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( KERNEL_WIDTH NSSUBPROP_MINIMUM  , 3);
    SetPropertyInt( KERNEL_WIDTH NSSUBPROP_MAXIMUM  , 25);

    SetPropertyInt( COUNT_OF_STDDEVS , 3);
    SetPropertyBool( COUNT_OF_STDDEVS NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( COUNT_OF_STDDEVS NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( COUNT_OF_STDDEVS NSSUBPROP_MAXIMUM  , 5);

    SetPropertyInt(SMOOTHING_KERNEL_WIDTH, 3);
    SetPropertyBool(SMOOTHING_KERNEL_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt(SMOOTHING_KERNEL_WIDTH NSSUBPROP_MINIMUM  , 3);
    SetPropertyInt(SMOOTHING_KERNEL_WIDTH NSSUBPROP_MAXIMUM  , 25);

    SetPropertyInt( ROAD_WIDTH_TOLERANCE , 10);
    SetPropertyBool( ROAD_WIDTH_TOLERANCE  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( ROAD_WIDTH_TOLERANCE  NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( ROAD_WIDTH_TOLERANCE  NSSUBPROP_MAXIMUM  , 9999);

    SetPropertyInt( FIT_WIDTH , 15);
    SetPropertyBool( FIT_WIDTH  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( FIT_WIDTH  NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( FIT_WIDTH  NSSUBPROP_MAXIMUM  , 9999);

    SetPropertyInt( MAX_COUNT_OBJECTS , 4);
    SetPropertyBool( MAX_COUNT_OBJECTS  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( MAX_COUNT_OBJECTS  NSSUBPROP_MINIMUM  , 1);
    SetPropertyInt( MAX_COUNT_OBJECTS  NSSUBPROP_MAXIMUM  , 20);

    SetPropertyStr( CORRESPING_POINTS_XML , "/home/odroid/Desktop/points.xml");
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);



    SetPropertyInt( FRAME_MEMORY , 3);
    SetPropertyBool( FRAME_MEMORY  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( FRAME_MEMORY  NSSUBPROP_MINIMUM  , 1);
    SetPropertyInt( FRAME_MEMORY NSSUBPROP_MAXIMUM  , 60);


}

/**
 * @brief cSWE_ObjectDetection::~cSWE_ObjectDetection
 * Destructs a cSWE_ObjectDetection object
 */
cSWE_ObjectDetection::~cSWE_ObjectDetection()
{

}

/**
 * @brief cSWE_ObjectDetection::Init
 * @param eStage the current stage of initalization
 * @return a value indicating the succes of the stage
 */
tResult cSWE_ObjectDetection::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    if (eStage == StageFirst)
    {
        // register a Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // register a Video Output
        RETURN_IF_FAILED(m_oGreyScaleVideoOutputPin.Create("GreyScale_Video_Output", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGreyScaleVideoOutputPin));
    }
    else if (eStage == StageNormal)
    {
        // fetch the parameters of the image cutting
        m_leftBorder = GetPropertyInt( LEFT_SEARCH_BORDER );
        m_rightBorder = GetPropertyInt( RIGHT_SEARCH_BORDER );
        m_upperBorder = GetPropertyInt( UPPER_SEARCH_BORDER );
        m_lowerBorder = GetPropertyInt( LOWER_SEARCH_BORDER );

        // fetch the algorithmic parameters
        m_KernelWidth = GetPropertyInt( KERNEL_WIDTH );
        m_CountStdDevs = GetPropertyInt(COUNT_OF_STDDEVS);
        m_KernelWidthPeaks = GetPropertyInt( SMOOTHING_KERNEL_WIDTH );
        m_RoadWidth = GetPropertyInt( ROAD_WIDTH );
        m_RoadWidthTolerance = GetPropertyInt( ROAD_WIDTH_TOLERANCE );
        m_FitWidth = GetPropertyInt( FIT_WIDTH );
        m_MaxCountOBJECTs = GetPropertyInt( MAX_COUNT_OBJECTS );
        m_MinAngle = 90.0 - GetPropertyInt( MAX_ANGULAR_DEVIATION );
        m_MaxAngle = 90.0 + GetPropertyInt( MAX_ANGULAR_DEVIATION );

        // read the parameters from a file and setup a transformation matrix
        InitTransformationMatrices( GetPropertyStr( CORRESPING_POINTS_XML ) ); 

        _MagicCounter = GetPropertyInt(FRAME_MEMORY);

        for( size_t i = 0 ; i < _MagicCounter ; ++i )
        {
            _Images.push_back( Mat::zeros( 240 , 320 , CV_16UC1 ) );
        }


        // initialize the formats of the pins;
        InitPinFormats();
    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

/**
 * @brief cSWE_ObjectDetection::InitPinFormats Helper initalization of the Formats of the Pins
 * @return a value indicating the succes of the stage
 */
tResult cSWE_ObjectDetection::InitPinFormats()
{
    // fetch the format of the inputpin
    m_sBitMapInputFormat = m_oVideoInputPin.GetFormat();

    // setup the constant parameters of the Color outputformat
    m_sGreyScaleBitMapOutputFormat.nBitsPerPixel = 16;
    m_sGreyScaleBitMapOutputFormat.nPixelFormat = cImage::PF_GREYSCALE_16;
    m_sGreyScaleBitMapOutputFormat.nPaletteSize = 0;
    m_sGreyScaleBitMapOutputFormat.nWidth = 320;
    m_sGreyScaleBitMapOutputFormat.nHeight = 240;
    m_sGreyScaleBitMapOutputFormat.nBytesPerLine = 320 * 2;
    m_sGreyScaleBitMapOutputFormat.nSize = m_sGreyScaleBitMapOutputFormat.nBytesPerLine * 240;

    // set the format to the outputpin
    m_oGreyScaleVideoOutputPin.SetFormat( &m_sGreyScaleBitMapOutputFormat , NULL );

    RETURN_NOERROR;
}

/**
 * @brief cSWE_ObjectDetection::InitTransformationMatrix Helper initalization of the Transformation Matrix
 * @param pathExternalCameraParams string containing the path the the XML storing the point correspondences
 * @return a value indicating the succes of the stage
 */
tResult cSWE_ObjectDetection::InitTransformationMatrices( std::string pathExternalCameraParams )
{
    m_inversePerspectiveFileStorage.open( pathExternalCameraParams , FileStorage::READ );

    int length = 4;
    cv::Point2f source_points [ length ];
    cv::Point2f dest_points [ length ];

    // read array of corresponding Points from XML
    for( int i = 0 ; i < length ; i++ )
    {
        string currentDestinationName , currentSourceName;
        {
            // convoluted way to concatenate the arrayName with an integer, but standard C++
            std::stringstream stringStream;
            stringStream << "DestinationPoints" << i;
            currentDestinationName = stringStream.str();
        }
        {
            std::stringstream stringStream;
            stringStream << "SourcePoints" << i;
            currentSourceName = stringStream.str();
        }

        // read from XML
        m_inversePerspectiveFileStorage[ currentDestinationName ] >> dest_points[ i ];
        m_inversePerspectiveFileStorage[ currentSourceName ] >> source_points[ i ];
    }

    m_inversePerspectiveFileStorage.release();

    m_projectionMatrix = Mat::zeros( 640 , 480 , CV_8UC1 );
    m_projectionMatrix = cv::getPerspectiveTransform( source_points , dest_points );

    m_backProjectionMatrix = Mat::zeros( 640 , 480 , CV_8UC1 );
    m_backProjectionMatrix = cv::getPerspectiveTransform( dest_points , source_points );

    RETURN_NOERROR;
}

/**
 * @brief cSWE_ObjectDetection::Shutdown The deinitialization method of this filter
 * @param eStage the stage of deinitialization
 * @return a value indicating the succes of the stage
 */
tResult cSWE_ObjectDetection::Shutdown(tInitStage eStage, __exception)
{   
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

/**
 * @brief cSWE_ObjectDetection::OnPinEvent The method reacting on incoming samples
 * @param pSource the Pin transmitting the input
 * @param nEventCode
 * @param nParam1
 * @param nParam2
 * @param pMediaSample the actual input
 * @return a value indicating the succes of the processing
 */
tResult cSWE_ObjectDetection::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
         RETURN_IF_POINTER_NULL(pMediaSample);
         if(pSource == &m_oVideoInputPin)
         {
             ProcessInput(pMediaSample);
         }
    }
    RETURN_NOERROR;
}

/**
 * @brief cSWE_ObjectDetection::FilterImageInput Does Temporal Filtering of the image with the last two images to reduce noise
 * @param cv::Mat the input Image
 * @return a value indicating the succes of the processing
 */
tResult cSWE_ObjectDetection::FilterImageInput(cv::Mat& image)
{
    //shift old images to the past (OPT)
    _Images.push_back( image.clone() );
    _Images.pop_front();

    //process the images
    for( int i = 0; i < image.rows; ++i)
    {
        //iterate through row
        for ( int j = 0; j < image.cols; ++j)
        {
            if( image.at<  unsigned short >( i , j ) < 1024 || image.at< unsigned short >( i , j ) > 60000 )
            {
                for( deque< Mat >::iterator iter = _Images.begin() ; iter != _Images.end() ; iter++ )
                {
                    unsigned short value = iter->at<  unsigned short >( i , j );
                    if( !( value < 1024 || value > 60000 ) )
                    {
                        image.at<  unsigned short >( i , j ) = value;
                        break;
                    }
                }
            }
        }

            //if ((p_image1[j] < 512)  || (p_image1[j] > 16000)) //check if dubious values present
            //{





                /*
               if ((p_image2[j] > 512) && (p_image2[j] < 16000))
               {
                    p_image[j] = p_image2[j];
               }
               else if ((p_image3[j] > 512) && (p_image3[j] < 16000))
               {
                    p_image[j] = p_image3[j];
               }
               */

            //}


    }

    /**
    Mat& ScanImageAndReduceC(Mat& I, const uchar* const table)
    {
        // accept only char type matrices
        CV_Assert(I.depth() != sizeof(uchar));

        int channels = I.channels();

        int nRows = I.rows;
        int nCols = I.cols * channels;

        if (I.isContinuous())
        {
            nCols *= nRows;
            nRows = 1;
        }

        int i,j;
        uchar* p;
        for( i = 0; i < nRows; ++i)
        {
            p = I.ptr<uchar>(i);
            for ( j = 0; j < nCols; ++j)
            {
                p[j] = table[p[j]];
            }
        }
        return I;
    }
    */


    //return

    RETURN_NOERROR;
}

/**
 * @brief cSWE_ObjectDetection::ProcessInput The actual image processing of the ObjectDetection
 * ... Description of the algorithm
 * @param pMediaSample the input Image
 * @return a value indicating the succes of the processing
 */
tResult cSWE_ObjectDetection::ProcessInput(IMediaSample* pMediaSample)
{
    tUInt16* pData;

    /** I broke it from here... (Robert) **/

    // transform the input to a opencv Mat
    pMediaSample->Lock((const tVoid**) &pData);
    Mat image( 240 , 320 , CV_16UC1 , pData );



    //filter image to reduce noise
    FilterImageInput(image);


    // apply an inverse Perspective mapping
    Mat warpedImage;
    cv::warpPerspective( image , warpedImage , m_projectionMatrix , image.size() );






    /** ...till here. **/


    // transmit a video of the current result to the video outputpin
    if (m_oGreyScaleVideoOutputPin.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, image.data, m_sGreyScaleBitMapOutputFormat.nSize , 0);
            m_oGreyScaleVideoOutputPin.Transmit(pNewRGBSample);
        }
    }

    RETURN_NOERROR;
}
