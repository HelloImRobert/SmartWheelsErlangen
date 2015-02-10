#include "stdafx.h"
#include "SWE_LaneDetection.h"
#include <template_data.h>

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_LaneDetection", OID_ADTF_LANEDETECTION_FILTER , cSWE_LaneDetection);

// Macros used to decouple text from code
#define LEFT_SEARCH_BORDER "Cut Left Border for Lane Detection"
#define RIGHT_SEARCH_BORDER "Cut Right Border for Lane Detection"
#define UPPER_SEARCH_BORDER "Cut Upper Border for Lane Detection"
#define LOWER_SEARCH_BORDER "Cut Lower Border for Lane Detection"
#define CORRESPING_POINTS_XML "Path to external Camera Params xml"
#define KERNEL_WIDTH "Width of the Kernel"
#define SMOOTHING_KERNEL_WIDTH "Width of the Smoothing Kernel"
#define COUNT_OF_STDDEVS "Stddevs for the Thresholding"
#define ROAD_WIDTH "Width of the Road in pixels"
#define ROAD_WIDTH_TOLERANCE "Tolerance of the Distance between Lanes"
#define FIT_WIDTH "Width of the area for line fitting"
#define MAX_COUNT_LANES "Maximum possible count of lanes"
#define MAX_ANGULAR_DEVIATION "Allowed deviation from 90°"

bool sort_smaller(cv::Vec2f* lane1, cv::Vec2f* lane2){
        return (*lane1)[1] > (*lane2)[1];
    }

/**
 * @brief cSWE_LaneDetection::cSWE_LaneDetection
 * Constructs a cSWE_LaneDetection object, defining it's properties, to be set by ADTF;
 * @param __info The info object for the Filter
 */
cSWE_LaneDetection::cSWE_LaneDetection(const tChar* __info):cFilter(__info)
{
    // set the cut parameter default to not cut the image
    _applyCut = tFalse;

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

    SetPropertyInt( ROAD_WIDTH , 30);
    SetPropertyBool( ROAD_WIDTH  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( ROAD_WIDTH  NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( ROAD_WIDTH  NSSUBPROP_MAXIMUM  , 9999);

    SetPropertyInt( ROAD_WIDTH_TOLERANCE , 10);
    SetPropertyBool( ROAD_WIDTH_TOLERANCE  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( ROAD_WIDTH_TOLERANCE  NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( ROAD_WIDTH_TOLERANCE  NSSUBPROP_MAXIMUM  , 9999);

    SetPropertyInt( FIT_WIDTH , 15);
    SetPropertyBool( FIT_WIDTH  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( FIT_WIDTH  NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( FIT_WIDTH  NSSUBPROP_MAXIMUM  , 9999);

    SetPropertyInt( MAX_COUNT_LANES , 4);
    SetPropertyBool( MAX_COUNT_LANES  NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( MAX_COUNT_LANES  NSSUBPROP_MINIMUM  , 1);
    SetPropertyInt( MAX_COUNT_LANES  NSSUBPROP_MAXIMUM  , 20);

    SetPropertyStr( CORRESPING_POINTS_XML , "/home/odroid/Desktop/points.xml");
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);
}

/**
 * @brief cSWE_LaneDetection::~cSWE_LaneDetection
 * Destructs a cSWE_LaneDetection object
 */
cSWE_LaneDetection::~cSWE_LaneDetection()
{

}

/**
 * @brief cSWE_LaneDetection::Init
 * @param eStage the current stage of initalization
 * @return a value indicating the succes of the stage
 */
tResult cSWE_LaneDetection::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    if (eStage == StageFirst)
    {
        // register a Video Input
        RETURN_IF_FAILED(_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&_oVideoInputPin));

        // register a Video Output
        RETURN_IF_FAILED(_oGreyScaleVideoOutputPin.Create("GreyScale_Video_Output", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oGreyScaleVideoOutputPin));

        // register a Video Output
        RETURN_IF_FAILED(_oColorVideoOutputPin.Create("Color_Video_Output", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oColorVideoOutputPin));

        // Output Pin for Lines
        // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescLines = pDescManager->GetMediaDescription("tLineBoundaries");
        RETURN_IF_POINTER_NULL(strDescLines);
        cObjectPtr<IMediaType> pTypeLines = new cMediaType(0, 0, 0, "tLineBoundaries", strDescLines,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeLines->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLines));

        RETURN_IF_FAILED(m_oLines.Create("Line_Boundaries", pTypeLines, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oLines));
    }
    else if (eStage == StageNormal)
    {
        // fetch the parameters of the image cutting
        _leftBorder = GetPropertyInt( LEFT_SEARCH_BORDER );
        _rightBorder = GetPropertyInt( RIGHT_SEARCH_BORDER );
        _upperBorder = GetPropertyInt( UPPER_SEARCH_BORDER );
        _lowerBorder = GetPropertyInt( LOWER_SEARCH_BORDER );

        // fetch the algorithmic parameters
        _KernelWidth = GetPropertyInt( KERNEL_WIDTH );
        _CountStdDevs = GetPropertyInt(COUNT_OF_STDDEVS);
        _SmoothingKernelWidth = GetPropertyInt( SMOOTHING_KERNEL_WIDTH );
        _RoadWidth = GetPropertyInt( ROAD_WIDTH );
        _RoadWidthTolerance = GetPropertyInt( ROAD_WIDTH_TOLERANCE );
        _FitWidth = GetPropertyInt( FIT_WIDTH );
        _MaxCountLanes = GetPropertyInt( MAX_COUNT_LANES );
        _MinAngle = 90.0 - GetPropertyInt( MAX_ANGULAR_DEVIATION );
        _MaxAngle = 90.0 + GetPropertyInt( MAX_ANGULAR_DEVIATION );

        // setup the convolution kernels
        InitKernels();

        // read the parameters from a file and setup a transformation matrix
        InitTransformationMatrices( GetPropertyStr( CORRESPING_POINTS_XML ) );

        // initialize the formats of the pins;
        InitPinFormats();

    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

/**
 * @brief cSWE_LaneDetection::InitPinFormats Helper initalization of the Formats of the Pins
 * @return a value indicating the succes of the stage
 */
tResult cSWE_LaneDetection::InitPinFormats()
{
    // fetch the format of the inputpin
    _sBitMapInputFormat = _oVideoInputPin.GetFormat();

    // setup the constant parameters of the Color outputformat
    _sColorBitMapOutputFormat.nBitsPerPixel = 24;
    _sColorBitMapOutputFormat.nPixelFormat = cImage::PF_RGB_888;
    _sColorBitMapOutputFormat.nPaletteSize = 0;
    _sColorBitMapOutputFormat.nWidth = 640;
    _sColorBitMapOutputFormat.nHeight = 480;
    _sColorBitMapOutputFormat.nBytesPerLine = 640 * 3;
    _sColorBitMapOutputFormat.nSize = _sColorBitMapOutputFormat.nBytesPerLine * 480;

    // setup the constant parameters of the GreyScale outputformat
    _sGreyScaleBitMapOutputFormat.nBitsPerPixel = 8;
    _sGreyScaleBitMapOutputFormat.nPixelFormat = cImage::PF_GREYSCALE_8;
    _sGreyScaleBitMapOutputFormat.nPaletteSize = 0;

    // calculate the image size if the image is cut
    tInt32 height = _lowerBorder - _upperBorder;
    tInt32 width = _rightBorder - _leftBorder;

    // if the cut is valid
    if ( height > 0 && width > 0 )
    {
        // setup the algorithm to cut
        _applyCut = tTrue;

        // calculate the format
        _sGreyScaleBitMapOutputFormat.nWidth = width;
        _sGreyScaleBitMapOutputFormat.nHeight = height;
        _sGreyScaleBitMapOutputFormat.nBytesPerLine = width;
        _sGreyScaleBitMapOutputFormat.nSize = _sGreyScaleBitMapOutputFormat.nBytesPerLine * height;
     }
     else
     {
        // just use the default (input) format
        _sGreyScaleBitMapOutputFormat.nWidth = 640;
        _sGreyScaleBitMapOutputFormat.nHeight = 480;
        _sGreyScaleBitMapOutputFormat.nBytesPerLine = 640;
        _sGreyScaleBitMapOutputFormat.nSize = _sGreyScaleBitMapOutputFormat.nBytesPerLine * 480;
     }

    // set the format to the outputpin
    _oGreyScaleVideoOutputPin.SetFormat( &_sGreyScaleBitMapOutputFormat , NULL );
    _oColorVideoOutputPin.SetFormat( &_sColorBitMapOutputFormat , NULL );

    RETURN_NOERROR;
}

/**
 * @brief cSWE_LaneDetection::InitTransformationMatrix Helper initalization of the Transformation Matrix
 * @param pathExternalCameraParams string containing the path the the XML storing the point correspondences
 * @return a value indicating the succes of the stage
 */
tResult cSWE_LaneDetection::InitTransformationMatrices( std::string pathExternalCameraParams )
{
    _inversePerspectiveFileStorage.open( pathExternalCameraParams , FileStorage::READ );

    int length = 9;
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
        _inversePerspectiveFileStorage[ currentDestinationName ] >> dest_points[ i ];
        _inversePerspectiveFileStorage[ currentSourceName ] >> source_points[ i ];
    }

    _inversePerspectiveFileStorage.release();

    _projectionMatrix = cv::getPerspectiveTransform( source_points , dest_points );
    _backProjectionMatrix = cv::getPerspectiveTransform( dest_points , source_points );

    RETURN_NOERROR;
}

/**
 * @brief cSWE_LaneDetection::InitKernels Helper initalization of the Image Processing Kernels
 * @return a value indicating the succes of the stage
 */
tResult cSWE_LaneDetection::InitKernels()
{
    // setup the image processing separable Kernel
/*
    int halfKernelWidth = (int)floor( _KernelWidth / 2.0);

    double sigmaSquared = pow( 1.8 * ( CV_PI ) , 2 );

    double sumKernelEntries = 0;
    _YDirKernel = Mat1d(_KernelWidth, 1, 0.0);
    for (int i = -halfKernelWidth; i <= halfKernelWidth; i++)
    {
        double iSquared = pow(i, 2);
        double entry = (exp(-0.5 * iSquared) / sigmaSquared) - (iSquared * exp(-(0.5 / sigmaSquared)* iSquared) / pow(sigmaSquared, 2));

        _YDirKernel.at< double >(i + halfKernelWidth) = entry;
        sumKernelEntries += entry;
    }

    _YDirKernel /= sumKernelEntries;*/

    Mat dummy;
    cv::getDerivKernels( _YDirKernel , dummy , 2 , 2 , _KernelWidth , false , CV_32F );

    _XDirKernel = cv::getGaussianKernel(_KernelWidth, -1, CV_32F);

    RETURN_NOERROR;
}

/**
 * @brief cSWE_LaneDetection::Shutdown The deinitialization method of this filter
 * @param eStage the stage of deinitialization
 * @return a value indicating the succes of the stage
 */
tResult cSWE_LaneDetection::Shutdown(tInitStage eStage, __exception)
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
 * @brief cSWE_LaneDetection::OnPinEvent The method reacting on incoming samples
 * @param pSource the Pin transmitting the input
 * @param nEventCode
 * @param nParam1
 * @param nParam2
 * @param pMediaSample the actual input
 * @return a value indicating the succes of the processing
 */
tResult cSWE_LaneDetection::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
         RETURN_IF_POINTER_NULL(pMediaSample);
         if(pSource == &_oVideoInputPin)
         {
             ProcessInput(pMediaSample);
         }
    }
    RETURN_NOERROR;
}

/**
 * @brief cSWE_LaneDetection::SearchLocalMaxima
 * Searches for local maxima in a matrix by simply looking up the predecessor and the successor of each element
 * and comparing them to the current pixel.
 * @param scalarMat a Matrix containing only scalar entries
 * @param peakLocations < OUT > locations of local maxima
 * @return a value indicating the succes of the processing
 */
tResult cSWE_LaneDetection::SearchLocalMaxima( Mat scalarMat , std::vector< int >& peakLocations )
{
    // setup values to access the successor and the predecessor
    ptrdiff_t successor = 1;
    ptrdiff_t predecessor = -1;

    // loop readonly over the matrix
    for( cv::MatConstIterator_< float > iter = scalarMat.begin< float >() ; iter != scalarMat.end< float >() ; iter++ )
    {
        // don't do anything at start or end
        if( iter != scalarMat.begin< float >() && iter != scalarMat.end< float >() )
        {
            // compare the values of this pixel with successor and predecessor
            if( *iter > iter[ predecessor ] && *iter > iter[ successor ] )
            {
                // add the local maximum to the output
                peakLocations.push_back( iter.pos().x );
            }
        }
    }
    RETURN_NOERROR;
}

/**
 * @brief cSWE_LaneDetection::SearchRoadMarkingLocations
 * Searches for candidate Locations of RoadMarkings by assuming a constant width of the road.
 * The search builds a chain of positions which are separated by a predefined number of Pixels.
 * It decides for the longest chain it can find.
 * @param peakLocations locations of local maxima
 * @param markingLocations < OUT > locations found to correspond best du lane boundaries
 * @return a value indicating the succes of the processing
 */
tResult cSWE_LaneDetection::SearchRoadMarkingLocations( const std::vector< int >& peakLocations , std::vector< int >& markingLocations )
{
    // loop over each location
    for (size_t i = 0; i < peakLocations.size(); i++)
    {
        // abort early if we can't find a longer chain anymore
        if (peakLocations.size() - i <= markingLocations.size())
        {
            RETURN_NOERROR;
        }

        // current point to build a chain on
        int match = peakLocations[i];

        // current chain
        std::vector< int > chain;

        // loop is aborted internally if the chain can't be continued
        while ( true )
        {
            // add the match to the chain at the start of the loop, to deal with initialization
            chain.push_back(match);

            // predict the location of the next peak
            int prediction = match + _RoadWidth;
            int upperBound = prediction + _RoadWidthTolerance;
            int lowerBound = prediction - _RoadWidthTolerance;

            // calculate potential matches in the predicted neighborhood
            std::vector< int > matches;
            for (size_t j = 0; j < peakLocations.size(); j++)
            {
                // if in predicted neighborhood
                if (peakLocations[j] < upperBound && peakLocations[j] > lowerBound)
                {
                    // add match to list
                    matches.push_back( peakLocations[j] );
                }
            }

            // calculate weights of trust for each match depending on the distance to it's prediction
            // weights are calculated as : 1 / abs( deviation )
            std::vector< double > weights;
            for (size_t j = 0; j < matches.size(); j++)
            {
                // calculate absolute value of deviation from prediction
                double weight = std::abs((double) matches[j] - (double)prediction);

                // handle the special case of the point being exactly where it was predicted to be
                if (weight <= 0.0)
                {
                    weight = 1.0;
                }
                else
                {
                    weight = 1 / weight;
                }
                weights.push_back(weight);
            }

            // calculated the match as a weighted mean
            double sumWeights = 0;
            double floatingMatch = 0;
            for (size_t j = 0; j < matches.size(); j++)
            {
                // sum up the weighted locations
                floatingMatch += matches[j] * weights[j];
                // sum up the weights for normalization
                sumWeights += weights[j];
            }

            // check if we had entries
            if (floatingMatch >= 0 && sumWeights >= 0)
            {
                // divide by the sum of the weights to get back locations
                floatingMatch /= sumWeights;
            }

            // round to nearest location
            match = ( int ) floor(floatingMatch + 0.5);

            // if we found any matches this iteration add the match to our chain and continue
            if (matches.size() > 0)
            {
                match = matches[0];
                continue;
            }

            // abort if we didn't find a match
            break;
        }

        // decide for the longest chain we can find
        // TODO: use prior knowledge of number of road markings
        if (chain.size() > markingLocations.size())
        {
            markingLocations = chain;
        }
    }

    RETURN_NOERROR;
}

/**
 * @brief cSWE_LaneDetection::ProcessInput The actual image processing of the LaneDetection
 * ... Description of the algorithm
 * @param pMediaSample the input Image
 * @return a value indicating the succes of the processing
 */
tResult cSWE_LaneDetection::ProcessInput(IMediaSample* pMediaSample)
{
    // transform the input to a opencv Mat
    tUInt8* pData = NULL;
    pMediaSample->Lock((const tVoid**) &pData);
    Mat image( _oVideoInputPin.GetFormat()->nHeight , _oVideoInputPin.GetFormat()->nWidth , CV_8UC3 , pData );
    pMediaSample->Unlock(pData);

    /*
    // transform the image to a grayscale image
    Mat greyScaleImage;
    cvtColor(image, greyScaleImage, CV_RGB2GRAY, 1);

    // apply an inverse Perspective mapping
    Mat warpedImage;
    cv::warpPerspective(greyScaleImage, warpedImage, _projectionMatrix, greyScaleImage.size());

    // cut the image to the desired size
    Mat cutImage;
    if ( _applyCut )
    {
        cutImage = warpedImage(cv::Range(_upperBorder, _lowerBorder), cv::Range(_leftBorder, _rightBorder)).clone();
    }
    else
    {
        cutImage = warpedImage;
    }

    // apply the filter for Detection of Lane Markers
    Mat blurredImage;
    cv::sepFilter2D(cutImage, blurredImage, CV_8UC1, _YDirKernel.t(), _XDirKernel);

    // aquire mean and stdDev of the image
    cv::Scalar mean, stdDev;
    cv::meanStdDev(blurredImage, mean, stdDev);

    // threshold the image to remove objects which don't necessary represent lane markings
    cv::threshold(blurredImage, blurredImage, mean[0] + _CountStdDevs * stdDev[0], 255, cv::THRESH_TOZERO);

    // sum the image up over the x - axis, the x-axis defined as the axis where the car is heading currently
    Mat columnSums;
    cv::reduce(blurredImage, columnSums, 0, CV_REDUCE_SUM, CV_32F);

    // smooth the column sums
    cv::GaussianBlur(columnSums, columnSums, cv::Size(_SmoothingKernelWidth, 1), 0);

    // find candidates for road marking locations
    std::vector< int > peakLocations;
    SearchLocalMaxima(columnSums, peakLocations);

    // refine candidates for road marking locations using prior knowledge about the width of the road
    std::vector< int > markingLocations;
    SearchRoadMarkingLocations(peakLocations, markingLocations);

    // fit lines to the neighborhoods of road marking locations
    for (size_t i = 0; i < markingLocations.size(); i++)
    {
        // calculate Neighborhood boundaries
        int leftBorder = markingLocations[i] - _FitWidth;
        int rightBorder = markingLocations[i] + _FitWidth;

        // if the calculated Boundaries are outside of the image set them to the image boundaries
        if (leftBorder < 0)
        {
            leftBorder = 0;
        }
        if (rightBorder > blurredImage.cols)
        {
            rightBorder = blurredImage.cols;
        }

        // get a ROI around the Neighborhood
        Mat roiedImage = blurredImage(cv::Range(0, blurredImage.rows), cv::Range(leftBorder, rightBorder));

        // find contours in the image
        vector< Point > pointsForFitting;

        for ( int j = 0 ; j < roiedImage.rows ; j++ )
        {
            for ( int k = 0 ; k < roiedImage.cols ; k++)
            {
                if (roiedImage.at< uchar >(j, k) > 0)
                {
                    pointsForFitting.push_back(Point(k, j));
                }
            }
        }

        // if a enough data was found
        if ( pointsForFitting.size() > 4 )
        {
            // fit a line
            Vec4f lines;
            cv::fitLine( Mat( pointsForFitting ), lines, CV_DIST_L2 , 0, 0.01, 0.01);

            // calculate the angle of the line to the y direction of the car
            double angle = atan2(lines[1], lines[0]) * 180.0 / CV_PI;

            // filter every line deviating to much from 90°
            if ( ( angle > _MinAngle && angle < _MaxAngle ) || ( angle < -_MinAngle && angle > -_MaxAngle ) )
            {
                // transform the line back to the image which was not roied
                int lines2BackTransformed = lines[2] + leftBorder;

                // calculate the left and right heightvalue for the line
                int lowerXValue = lines2BackTransformed + ( ( blurredImage.rows - lines[3] ) / lines[1]) * lines[0];
                int upperXValue = lines2BackTransformed + (-lines[3] / lines[1]) * lines[0];

                // paint the lines into the debug images
                cv::line(blurredImage, Point( upperXValue , 0), Point( lowerXValue , blurredImage.rows), Scalar(255, 0, 0), 2);
            }
        }
    }*/

    float oneDegree = CV_PI / 180;
    float thetaMax = 90 * oneDegree;
    float rhoMax = 0 * oneDegree;

    // transform the image to a grayscale image
    Mat greyScaleImage;
    cvtColor(image, greyScaleImage, CV_RGB2GRAY, 1);

    // apply the filter for Detection of Lane Markers
    Mat blurredImage;

    //cv::blur(greyScaleImage, blurredImage, cv::Size(7, 7));
    //cv::medianBlur(blurredImage, blurredImage, 3);
    cv::Sobel(greyScaleImage, blurredImage, CV_8UC1, 1, 1, 3);
    //cv::Canny(blurredImage, blurredImage, 170, 180);

    // aquire mean and stdDev of the image
    cv::Scalar mean, stdDev;
    cv::meanStdDev(blurredImage, mean, stdDev);

    //threshold the image to remove objects which don't necessary represent lane markings
    cv::threshold(blurredImage, blurredImage, mean[0] + _CountStdDevs * stdDev[0], 255, cv::THRESH_TOZERO);

    vector<Vec2f> lines;
    HoughLines(blurredImage, lines, 1, CV_PI / 180, 60, 0, 0);

    std::vector<Vec2f*> leftLines, rightLines;

    for (size_t i = 0; i < lines.size(); i++)
    {
        lines[i][1] = lines[i][0] > 0 ? lines[i][1] : lines[i][1] - (float)CV_PI;
        lines[i][0] = std::abs(lines[i][0]);
    }

    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec2f* line = &lines[i];
        double theta = (*line)[1];
        double rho = (*line)[0];

        if (rho > rhoMax)
        {
            if (theta >= 0 && theta < thetaMax)
            {
                leftLines.push_back( line ); // line in region from 0 to thetamax degree
            }
            else if (theta > -thetaMax && theta < 0)
            {
                rightLines.push_back( line );// line in region from -90 to thetamax degree
            }
        }
    }

    vector< Vec2f > lanes;

    int lanesNum;
    float rho, theta;
    if (!leftLines.empty()) {
        sort(leftLines.begin(), leftLines.end(), sort_smaller);
        lanesNum = (int)leftLines.size();
        rho = (*leftLines.at(lanesNum / 2))[0];
        theta = (*leftLines.at(lanesNum / 2))[1];
        lanes.push_back(cv::Vec2f(rho, theta));
    }
    if (!rightLines.empty()) {
        sort(rightLines.begin(), rightLines.end(), sort_smaller);
        lanesNum = (int)rightLines.size();
        rho = (*rightLines.at(lanesNum / 2))[0];
        theta = (*rightLines.at(lanesNum / 2))[1];
        lanes.push_back(cv::Vec2f(rho, theta));
    }

    double leftFrontX = -1;
    double leftFrontY = -1;
    double leftRearX = -1;
    double leftRearY = -1;
    double rightFrontX = -1;
    double rightFrontY = -1;
    double rightRearX = -1;
    double rightRearY = -1;


    for (size_t i = 0; i < lanes.size(); i++)
    {
        float rho = lanes[i][0], theta = lanes[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));

        if (theta >= 0 && theta < thetaMax)
        {
            line(image, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
        }
        else
        {
            line(image, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
        }

        vector< Point2d > vec;
        vec.push_back( pt1 );
        vec.push_back( pt2 );

        perspectiveTransform( vec , vec , _projectionMatrix );

        pt1 = vec[ 0 ];
        pt2 = vec[ 1 ];

        // scaling factor for calculating pixel in mm
        tFloat64 scaleFac = 0.95;
        // distance from front axis to nearest edge of camera picture
        tFloat64 distFrontToCam = 179+83-70;
        // value to correct camera not cenered in y-direction
        tFloat64 distMidToCam = 22;
        // picture heigth and width
        tFloat64 picHeight = 480;
        tFloat64 picWidth = 640;

        // transform points to vehicle coo sys in front axis center
        if (theta >= 0 && theta < thetaMax)
        {
            leftFrontY = scaleFac*((-1.0)*(pt1.x - picWidth/2.0)) + distMidToCam;
            leftFrontX = scaleFac*((-1.0)*(pt1.y - picHeight)) + distFrontToCam;
            leftRearY = scaleFac*((-1.0)*(pt2.x - picWidth/2.0)) + distMidToCam;
            leftRearX = scaleFac*((-1.0)*(pt2.y - picHeight)) + distFrontToCam;
        }
        else
        {
            rightFrontY = scaleFac*((-1.0)*(pt1.x - picWidth/2.0)) + distMidToCam;
            rightFrontX = scaleFac*((-1.0)*(pt1.y - picHeight)) + distFrontToCam;
            rightRearY = scaleFac*((-1.0)*(pt2.x - picWidth/2.0)) + distMidToCam;
            rightRearX = scaleFac*((-1.0)*(pt2.y - picHeight)) + distFrontToCam;
        }
    }

    // apply an inverse Perspective mapping
    Mat warpedImage = image;
    cv::warpPerspective(image, warpedImage, _projectionMatrix, greyScaleImage.size());

    {
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    // ADAPT: m_pCoderDescPointLeft
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescLines->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    // ADAPT: m_pCoderDescPointLeft
    //cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescLines->WriteLock(pMediaSample, &pCoder));


    pCoder->Set("leftFrontX", (tVoid*)&(leftFrontX));
    pCoder->Set("leftFrontY", (tVoid*)&(leftFrontY));
    pCoder->Set("leftRearX", (tVoid*)&(leftRearX));
    pCoder->Set("leftRearY", (tVoid*)&(leftRearY));
    pCoder->Set("rightFrontX", (tVoid*)&(rightFrontX));
    pCoder->Set("rightFrontY", (tVoid*)&(rightFrontY));
    pCoder->Set("rightRearX", (tVoid*)&(rightRearX));
    pCoder->Set("rightRearY", (tVoid*)&(rightRearY));
    m_pCoderDescLines->Unlock(pCoder);


    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oLines.Transmit(pMediaSample));
}

    // transmit a video of the current result to the video outputpin
    if (_oColorVideoOutputPin.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, warpedImage.data, _sColorBitMapOutputFormat.nSize , 0);
            _oColorVideoOutputPin.Transmit(pNewRGBSample);
        }
    }

    // transmit a video of the current result to the video outputpin
    if (_oGreyScaleVideoOutputPin.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, blurredImage.data, _sGreyScaleBitMapOutputFormat.nSize , 0);
            _oGreyScaleVideoOutputPin.Transmit(pNewRGBSample);
        }
    }
    RETURN_NOERROR;
}
