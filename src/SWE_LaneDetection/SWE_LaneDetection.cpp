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
#define COUNT_OF_STDDEVS "Stddevs for the Thresholding"
#define MAX_ANGULAR_DEVIATION "Allowed deviation from 90°"

bool sort_smaller(cv::Vec2f* lane1, cv::Vec2f* lane2)
{
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

    SetPropertyInt( COUNT_OF_STDDEVS , 3);
    SetPropertyBool( COUNT_OF_STDDEVS NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( COUNT_OF_STDDEVS NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( COUNT_OF_STDDEVS NSSUBPROP_MAXIMUM  , 5);

    SetPropertyStr( CORRESPING_POINTS_XML , "/home/odroid/AADC/calibration_files/points.xml");
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


        const float oneDegree = CV_PI / 180;

        // fetch the algorithmic parameters
        _CountStdDevs = GetPropertyInt(COUNT_OF_STDDEVS);
        _MinAngle = 90.0 - GetPropertyInt( MAX_ANGULAR_DEVIATION );
        _MaxAngle = 90.0 + GetPropertyInt( MAX_ANGULAR_DEVIATION );

        _thetaMax = 90 * oneDegree;
        _rhoMax = 0 * oneDegree;

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

    /*
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
     else*/
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

std::pair< cv::Point, double > cSWE_LaneDetection::computeEnergy(const cv::Mat& src, const cv::Point2i& start, const cv::Point2i& directionStart, const cv::Point2i& directionEnd , size_t steps )
{
    double sum = 0;

    cv::Point2i end;

    cv::Point2d direction = directionEnd - directionStart;
    double length = cv::norm(directionEnd - directionStart);
    if( length > 0.5 )
    {
        direction.x = direction.x / length;
        direction.y = direction.y / length;
    }

    if ( !std::signbit(direction.y) )
    {
        direction.y = -direction.y;
        direction.x = -direction.x;
    }

    end.x = std::floor( start.x + 500 * direction.x + 0.5 );
    end.y = std::floor( start.y + 500 * direction.y + 0.5 );

    LineIterator iter = LineIterator(src, start, end);

    cv::Point cur_point;
    for (size_t i = 0; i < steps ; i++, iter++)
    {
        cur_point = iter.pos();
        sum += src.at< uchar >(cur_point.y, cur_point.x);
    }
    return std::pair< cv::Point , double >( cur_point , sum );
}

cv::Point cSWE_LaneDetection::findNextPoint(const cv::Mat& src, const cv::Point2i& start, const cv::Point2i& end, size_t steps)
{
    std::vector< std::pair< cv::Point, double > > results;
    results.push_back(computeEnergy(src, end, start, end, steps));

    int triesHalf = 5;
    for (int i = -triesHalf; i <= triesHalf; i++)
    {
        cv::Point point2Try = end;
        point2Try.x = end.x + i;
        point2Try.y = end.y + std::abs( i );
        results.push_back(computeEnergy(src, end, start, point2Try, steps));
    }

    size_t index = 0;
    for (size_t i = 1; i < results.size(); i++ )
    {
        if ( results[ i ].second > results[ index ].second)
        {
            index = i;
        }
    }

    return results[ index ].first;
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

    // transform the image to a grayscale image
    Mat greyScaleImage;
    cvtColor(image, greyScaleImage, CV_RGB2GRAY, 1);

    double useless;
    imwrite("/home/odroid/Desktop/kreuzung.jpg" , image );

    // apply the filter for Detection of Lane Markers
    Mat blurredImage;

    //cv::blur(greyScaleImage, blurredImage, cv::Size(7, 7));
    //cv::medianBlur(blurredImage, blurredImage, 3);
    cv::Sobel(greyScaleImage, blurredImage, CV_8UC1, 1, 1, 3);

    // aquire mean and stdDev of the image
    cv::Scalar mean, stdDev;
    cv::meanStdDev(blurredImage, mean, stdDev);

    Mat thresholdedImage;
    //threshold the image to remove objects which don't necessary represent lane markings
    cv::threshold(blurredImage, thresholdedImage, mean[0] + _CountStdDevs * stdDev[0], 255, cv::THRESH_TOZERO);

    vector<Vec2f> lines;
    HoughLines(thresholdedImage, lines, 1, CV_PI / 180, 60, 0, 0);

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

        if (rho > _rhoMax)
        {
            if (theta >= 0 && theta < _thetaMax)
            {
                leftLines.push_back( line ); // line in region from 0 to thetamax degree
            }
            else if (theta > -_thetaMax && theta < 0)
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

//    for (size_t i = 0; i < lanes.size(); i++)
//    {
//        float rho = lanes[i][0], theta = lanes[i][1];

//        cv::Point start;
//        cv::Point pFirst;
//        cv::Point pSecond;

//        if ((theta < CV_PI / 4. || theta > 3. * CV_PI / 4.))
//        {
//            pFirst = cv::Point(rho / std::cos(theta), 0);
//            pSecond = cv::Point((rho - image.rows * std::sin(theta)) / std::cos(theta), image.rows);
//        }
//        else
//        {
//            pFirst = cv::Point(0, rho / std::sin(theta));
//            pSecond = cv::Point(image.cols, (rho - image.cols * std::cos(theta)) / std::sin(theta));
//        }

//        if (pFirst.y > pSecond.y)
//        {
//            start = pFirst;
//        }
//        else
//        {
//            start = pSecond;
//        }

//        int stepSize = 60;
//        size_t maxSteps = 18;

//        circle(image, start, 20, Scalar(255, 0, 0), 2, 8);

//        std::pair< cv::Point , double > result = computeEnergy(blurredImage, start, pFirst, pSecond, stepSize);

//        circle(image, result.first, 20, Scalar(255, 0, 0), 2, 8);

//        cv::Point next = result.first;
//        cv::Point next2;
//        cv::Point current = start;
//        for (size_t j = 0; j < maxSteps; j++)
//        {
//            next2 = findNextPoint(blurredImage, current, next, stepSize);
//            if( next2.x < 0 || next2.y < 0 || next2.x >= blurredImage.cols || next2.y >= blurredImage.rows )
//            {
//                break;
//            }
//            current = next;
//            next = next2;
//           circle(image, next, 20, Scalar(255, 0, 0), 2, 8);
//        }
//    }

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

        if (theta >= 0 && theta < _thetaMax)
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
        if (theta >= 0 && theta < _thetaMax)
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
    Mat warpedImage;
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
            pNewRGBSample->Update(tmStreamTime, image.data, _sColorBitMapOutputFormat.nSize , 0);
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
            pNewRGBSample->Update(tmStreamTime, thresholdedImage.data, _sGreyScaleBitMapOutputFormat.nSize , 0);
            _oGreyScaleVideoOutputPin.Transmit(pNewRGBSample);
        }
    }
    RETURN_NOERROR;
}
