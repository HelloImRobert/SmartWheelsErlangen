#include "stdafx.h"
#include "SWE_LaneDetection.h"
#include "SWE_CatmullRomSpline.h"
#include <template_data.h>

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_LaneDetection", OID_ADTF_LANEDETECTION_FILTER , cSWE_LaneDetection);

// Macros used to decouple text from code
#define CORRESPONDING_POINTS_XML "Path to external Camera Params xml"
#define COUNT_OF_STDDEVS "Stddevs for the Thresholding"
#define WIDTH_FACTOR "The factor by which the area exceeds the length of valid boundaries"
#define MIDDLE_DISTANCE_HIGH_THRESHOLD "The maximum distance for a middle boundary"
#define MIDDLE_DISTANCE_LOW_THRESHOLD "The minimum distance for a middle boundary"
#define MIN_OUTER_BOUNDARY_LENGTH "The minimum length an outer boundary must have"
#define DRAW_IMAGES "Boolean indicating wether visualizations should be drawn"
#define HEIGHT_THRESHOLD "The Threshold at which Blobs are removed"
#define LOWER_AREA_THRESHOLD "Minimum area of a blob to be considered"
#define START_HEIGHT "The heightvalue to which the image is cropped"
#define PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD "The ratio of eigenvalues at which a blob is rejected as boundary"
#define SPLINE_SEARCH_WIDTH "The width of pixels which is considered as a Y-Levelset for the spline reduction"
#define RESULTVID_OFFSET_X "X-Value of the offset for the resultvideo"
#define RESULTVID_OFFSET_Y "Y-Value of the offset for the resultvideo"
#define RESULTVID_SCALE "The scaling of the resultvideo"
#define MIN_LENGTH_MIDDLE_BOUNDARY "Minimum contour length of a middle laneboundary"
#define MAX_LENGTH_MIDDLE_BOUNDARY "Maximum contour length of a middle laneboundary"
#define AMBIGUOSANGLE "Minimum Angle, under which a line is classified as Ambiguos"
#define DIST_FRONT_TO_WARPED_IMAGE_BOTTOM "distance of the lowest point visible in the inverse perspective to the car coordinate system"
#define DIST_FRONT_TO_FRONT_AXIS "distance of the front point of the car to the front axis"
#define DIST_SIDE_IMAGE_TO_MID "distance of the middle of the image to the middle of the car in inverse perspective"

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

/**
 * @brief cSWE_LaneDetection::cSWE_LaneDetection
 * Constructs a cSWE_LaneDetection object, defining it's properties, to be set by ADTF;
 * @param __info The info object for the Filter
 */
cSWE_LaneDetection::cSWE_LaneDetection(const tChar* __info):cFilter(__info)
{
    SetPropertyFloat( AMBIGUOSANGLE , 0.06);
    SetPropertyBool( AMBIGUOSANGLE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( AMBIGUOSANGLE NSSUBPROP_MINIMUM  , 0.0001);
    SetPropertyFloat( AMBIGUOSANGLE NSSUBPROP_MAXIMUM  , 1 );

    SetPropertyFloat( RESULTVID_SCALE , 0.3);
    SetPropertyBool( RESULTVID_SCALE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( RESULTVID_SCALE NSSUBPROP_MINIMUM  , 0.1);
    SetPropertyFloat( RESULTVID_SCALE NSSUBPROP_MAXIMUM  , 1 );

    SetPropertyFloat( RESULTVID_OFFSET_X , 600);
    SetPropertyBool( RESULTVID_OFFSET_X NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( RESULTVID_OFFSET_X NSSUBPROP_MINIMUM  , 0);
    SetPropertyFloat( RESULTVID_OFFSET_X NSSUBPROP_MAXIMUM  , 9999 );

    SetPropertyFloat( RESULTVID_OFFSET_Y , 700);
    SetPropertyBool( RESULTVID_OFFSET_Y NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( RESULTVID_OFFSET_Y NSSUBPROP_MINIMUM  , 0 );
    SetPropertyFloat( RESULTVID_OFFSET_Y NSSUBPROP_MAXIMUM  , 9999 );

    SetPropertyFloat( COUNT_OF_STDDEVS , 2.0);
    SetPropertyBool( COUNT_OF_STDDEVS NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( COUNT_OF_STDDEVS NSSUBPROP_MINIMUM  , 0.5 );
    SetPropertyFloat( COUNT_OF_STDDEVS NSSUBPROP_MAXIMUM  , 10.0 );

    SetPropertyFloat( WIDTH_FACTOR , 25.0);
    SetPropertyBool( WIDTH_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( WIDTH_FACTOR NSSUBPROP_MINIMUM  , 0.01);
    SetPropertyFloat( WIDTH_FACTOR NSSUBPROP_MAXIMUM  , 99.0);

    SetPropertyFloat( MIDDLE_DISTANCE_HIGH_THRESHOLD , 660.0);
    SetPropertyBool( MIDDLE_DISTANCE_HIGH_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( MIDDLE_DISTANCE_HIGH_THRESHOLD NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( MIDDLE_DISTANCE_HIGH_THRESHOLD NSSUBPROP_MAXIMUM  , IMAGE_WIDTH);

    SetPropertyFloat( MIDDLE_DISTANCE_LOW_THRESHOLD , 360.0);
    SetPropertyBool( MIDDLE_DISTANCE_LOW_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( MIDDLE_DISTANCE_LOW_THRESHOLD NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( MIDDLE_DISTANCE_LOW_THRESHOLD NSSUBPROP_MAXIMUM  , IMAGE_WIDTH);

    SetPropertyFloat( MIN_LENGTH_MIDDLE_BOUNDARY , 180.0);
    SetPropertyBool(  MIN_LENGTH_MIDDLE_BOUNDARY NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( MIN_LENGTH_MIDDLE_BOUNDARY NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( MIN_LENGTH_MIDDLE_BOUNDARY NSSUBPROP_MAXIMUM  , 9999.0);

    SetPropertyFloat( MAX_LENGTH_MIDDLE_BOUNDARY , 1100.0);
    SetPropertyBool(  MAX_LENGTH_MIDDLE_BOUNDARY NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( MAX_LENGTH_MIDDLE_BOUNDARY NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( MAX_LENGTH_MIDDLE_BOUNDARY NSSUBPROP_MAXIMUM  , 9999.0);

    SetPropertyFloat( MIN_OUTER_BOUNDARY_LENGTH , 1200.0);
    SetPropertyBool( MIN_OUTER_BOUNDARY_LENGTH NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( MIN_OUTER_BOUNDARY_LENGTH NSSUBPROP_MINIMUM  , 0);
    SetPropertyFloat( MIN_OUTER_BOUNDARY_LENGTH NSSUBPROP_MAXIMUM  , 99999);

    SetPropertyBool( DRAW_IMAGES , true);
    SetPropertyBool( DRAW_IMAGES NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyStr( CORRESPONDING_POINTS_XML , "/home/odroid/AADC/calibration_files/SWE_cameraCalibrationPoints.XML");
    SetPropertyBool( CORRESPONDING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt( HEIGHT_THRESHOLD , 35);
    SetPropertyBool( HEIGHT_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( HEIGHT_THRESHOLD NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( HEIGHT_THRESHOLD NSSUBPROP_MAXIMUM  , 479);

    SetPropertyInt( START_HEIGHT , 265 );
    SetPropertyBool( START_HEIGHT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( START_HEIGHT NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( START_HEIGHT NSSUBPROP_MAXIMUM  , 479);

    SetPropertyFloat( LOWER_AREA_THRESHOLD , 180);
    SetPropertyBool( LOWER_AREA_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( LOWER_AREA_THRESHOLD NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( LOWER_AREA_THRESHOLD NSSUBPROP_MAXIMUM  , 99999.0);

    SetPropertyFloat( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD , 5.5 );
    SetPropertyBool( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD NSSUBPROP_MINIMUM  , 0.01);
    SetPropertyFloat( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD NSSUBPROP_MAXIMUM  , 99999.0);

    SetPropertyInt( SPLINE_SEARCH_WIDTH , 20 );
    SetPropertyBool( SPLINE_SEARCH_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( SPLINE_SEARCH_WIDTH NSSUBPROP_MINIMUM  , 2);
    SetPropertyInt( SPLINE_SEARCH_WIDTH NSSUBPROP_MAXIMUM  , 200);

    SetPropertyFloat( DIST_FRONT_TO_WARPED_IMAGE_BOTTOM , 310.0 );
    SetPropertyBool( DIST_FRONT_TO_WARPED_IMAGE_BOTTOM NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( DIST_FRONT_TO_WARPED_IMAGE_BOTTOM NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( DIST_FRONT_TO_WARPED_IMAGE_BOTTOM NSSUBPROP_MAXIMUM  , 99999.0);

    SetPropertyFloat( DIST_FRONT_TO_FRONT_AXIS , 120.0 );
    SetPropertyBool( DIST_FRONT_TO_FRONT_AXIS NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( DIST_FRONT_TO_FRONT_AXIS NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( DIST_FRONT_TO_FRONT_AXIS NSSUBPROP_MAXIMUM  , 99999.0);

    SetPropertyFloat( DIST_SIDE_IMAGE_TO_MID , 10.0 );
    SetPropertyBool( DIST_SIDE_IMAGE_TO_MID NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( DIST_SIDE_IMAGE_TO_MID NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( DIST_SIDE_IMAGE_TO_MID NSSUBPROP_MAXIMUM  , 320.0 );
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
        RETURN_IF_FAILED(_oInternRepresentationVideoOutputPin.Create("Intern_Representation_Video_Output", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oInternRepresentationVideoOutputPin));

        // register a Video Output
        RETURN_IF_FAILED(_oColorVideoOutputPin.Create("Color_Video_Output", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oColorVideoOutputPin));

        RETURN_IF_FAILED(m_oIntersectionPoints.Create("tracking_Point", new cMediaType(0, 0, 0, "tIntersectionsNew"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));

        // Output Pin for Crossings
        // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescCrossingIndicator = pDescManager->GetMediaDescription("tCrossingIndicator");
        RETURN_IF_POINTER_NULL(strDescCrossingIndicator);
        cObjectPtr<IMediaType> pTypeCrossingIndicator = new cMediaType(0, 0, 0, "tCrossingIndicator", strDescCrossingIndicator,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeCrossingIndicator->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescCrossingIndicator));

        RETURN_IF_FAILED(m_CrossingIndicatorPin.Create("Crossing_Indicator", pTypeCrossingIndicator, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_CrossingIndicatorPin));

        // Output pin for Splines

        tChar const * strDescSplines = pDescManager->GetMediaDescription("tSplineBoundaryNew");
        RETURN_IF_POINTER_NULL(strDescSplines);
        cObjectPtr<IMediaType> pTypeSplines = new cMediaType(0, 0, 0, "tSplineBoundaryNew", strDescSplines,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSplines->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSplines));

        RETURN_IF_FAILED(m_oSplinesPin.Create("Spline_Boundaries", pTypeSplines, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSplinesPin));

    }
    else if (eStage == StageNormal)
    {
        // fetch the algorithmic parameters
        _resultImageScaleFactor = GetPropertyFloat(RESULTVID_SCALE);
        _resultImageOffsetVector = cv::Point(GetPropertyInt(RESULTVID_OFFSET_X),GetPropertyInt(RESULTVID_OFFSET_Y));
        _widthFactor = GetPropertyFloat(WIDTH_FACTOR);
        _middleDistanceHighThreshold = GetPropertyFloat(MIDDLE_DISTANCE_HIGH_THRESHOLD);
        _middleDistanceLowThreshold = GetPropertyFloat(MIDDLE_DISTANCE_LOW_THRESHOLD);
        _minOuterBoundaryLength = GetPropertyFloat(MIN_OUTER_BOUNDARY_LENGTH);
        _heightThresh = GetPropertyInt(HEIGHT_THRESHOLD);
        _draw = GetPropertyBool(DRAW_IMAGES),
        _CountStdDevs = GetPropertyInt(COUNT_OF_STDDEVS);
        _lowerAreaThreshold = GetPropertyFloat(LOWER_AREA_THRESHOLD);
        _startHeight = GetPropertyInt(START_HEIGHT);
        _principalAxisLengthRatioThreshold = GetPropertyFloat(PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD);
        _splineSearchWidth = GetPropertyInt(SPLINE_SEARCH_WIDTH);
        _minLengthMiddleBoundary = GetPropertyFloat(MIN_LENGTH_MIDDLE_BOUNDARY);
        _maxLengthMiddleBoundary = GetPropertyFloat(MAX_LENGTH_MIDDLE_BOUNDARY);
        _ambigousAngle = GetPropertyFloat(AMBIGUOSANGLE);
        _distFrontToWarpedImageBottom = GetPropertyFloat(DIST_FRONT_TO_WARPED_IMAGE_BOTTOM);
        _distFrontToFrontAxis = GetPropertyFloat(DIST_FRONT_TO_FRONT_AXIS);
        _distSideImageToMid = GetPropertyFloat(DIST_SIDE_IMAGE_TO_MID);

        // read the parameters from a file and setup a transformation matrix
        InitTransformationMatrices( GetPropertyStr( CORRESPONDING_POINTS_XML ) );

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
    _sColorBitMapOutputFormat.nWidth = IMAGE_WIDTH;
    _sColorBitMapOutputFormat.nHeight = IMAGE_HEIGHT;
    _sColorBitMapOutputFormat.nBytesPerLine = IMAGE_WIDTH * 3;
    _sColorBitMapOutputFormat.nSize = _sColorBitMapOutputFormat.nBytesPerLine * IMAGE_HEIGHT;

    // setup the constant parameters of the GreyScale outputformat
    _sInternRepresentationBitMapOutputFormat.nBitsPerPixel = 24;
    _sInternRepresentationBitMapOutputFormat.nPixelFormat = cImage::PF_RGB_888;
    _sInternRepresentationBitMapOutputFormat.nPaletteSize = 0;
    _sInternRepresentationBitMapOutputFormat.nWidth = 1280;
    _sInternRepresentationBitMapOutputFormat.nHeight = 960;
    _sInternRepresentationBitMapOutputFormat.nBytesPerLine = 1280 * 3;
    _sInternRepresentationBitMapOutputFormat.nSize = _sInternRepresentationBitMapOutputFormat.nBytesPerLine * 960;

    // set the format to the outputpin
    _oInternRepresentationVideoOutputPin.SetFormat( &_sInternRepresentationBitMapOutputFormat , NULL );
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

void cSWE_LaneDetection::transformToCarCoords( std::vector< cv::Point2d >& spline )
{
    for( size_t i = 0; i < spline.size(); i++ )
    {
        spline[i].x = (-1.0)*( spline[i].x - static_cast< double >(IMAGE_WIDTH)/2.0 - _distSideImageToMid );
        spline[i].y = (-1.0)*( spline[i].y - static_cast< double >(IMAGE_HEIGHT) - _distFrontToWarpedImageBottom - _distFrontToFrontAxis );

        double temp = spline[i].x;

        spline[i].x = spline[i].y;
        spline[i].y = temp;

    }
}

void cSWE_LaneDetection::transformFromCarCoords( std::vector< cv::Point2d >& spline )
{
    for( size_t i = 0; i < spline.size(); i++ )
    {
        spline[i].x = (-1.0 * spline[i].x) + static_cast< double >(IMAGE_HEIGHT) + _distFrontToWarpedImageBottom + _distFrontToFrontAxis ;
        spline[i].y = (-1.0 * spline[i].y) + static_cast< double >(IMAGE_WIDTH)/2.0 + _distSideImageToMid;

        double temp = spline[i].x;

        spline[i].x = spline[i].y;
        spline[i].y = temp;
    }
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
        else if(pSource == &m_oIntersectionPoints)
        {
            cObjectPtr<IMediaType> pType;
            pSource->GetMediaType(&pType);
            cObjectPtr<IMediaTypeDescription> pMediaTypeDescInputMeasured;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescInputMeasured));
            m_pCoderDescInputMeasured = pMediaTypeDescInputMeasured;

            // init temporary objects
            cv::Point2d trackingPoint;

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("intersecPoint.xCoord", (tVoid*)&(trackingPoint.x));
            pCoder->Get("intersecPoint.yCoord", (tVoid*)&(trackingPoint.y));
            m_pCoderDescInputMeasured->Unlock(pCoder);

            std::vector< cv::Point2d > pointVec;
            pointVec.push_back(trackingPoint);
            transformFromCarCoords(pointVec);
            cv::Point point = pointVec[0];
            point.x *= _resultImageScaleFactor;
            point.y *= _resultImageScaleFactor;
            point += _resultImageOffsetVector;

            cv::circle(_result, point , 8 , CV_RGB(255, 0, 0), 2);

            // transmit a video of the current result to the video outputpin
            if (_oInternRepresentationVideoOutputPin.IsConnected())
            {
                cObjectPtr<IMediaSample> pNewRGBSample;
                if (IS_OK(AllocMediaSample(&pNewRGBSample)))
                {
                    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                    pNewRGBSample->Update(tmStreamTime, _result.data, _sInternRepresentationBitMapOutputFormat.nSize , 0);
                    _oInternRepresentationVideoOutputPin.Transmit(pNewRGBSample);
                }
            }
        }
        RETURN_NOERROR;
    }
    RETURN_ERROR(-1);
}

bool sort_arcLength( const cSWE_LaneDetection::BlobDescriptor& blob1, const cSWE_LaneDetection::BlobDescriptor blob2)
{
    return blob1.lengthContour > blob2.lengthContour;
}

bool sort_yVal( const cv::Point& point1, const cv::Point& point2 )
{
    return point1.y > point2.y;
}

bool sort_yVal_double(const cv::Point2d& point1, const cv::Point2d& point2)
{
    return point1.y > point2.y;
}

bool sort_xVal( const cv::Point& point1, const cv::Point& point2)
{
    return point1.x < point2.x;
}

/**
     * \brief A convenience overload to project a BlobDescriptors contour and update it's orientation parameters by passing the BlobDescriptor.
     * @param inputBlob the BlobDescriptor which should be transformed
     * @param outputBlob the BlobDescriptor which will receive the transformed coordinates
     * @param projectionMatrix the transformation Matrix to be used
     * @param offset an optional parameter to reverse the effect of cropped images
     */
void cSWE_LaneDetection::project(const BlobDescriptor& inputBlob, BlobDescriptor& outputBlob , const cv::Mat& projectionMatrix, int offset)
{
    //transform the points into a double vector
    std::vector< cv::Point2d > points;
    for(size_t i = 0 ; i < inputBlob.contour.size() ; i++ )
    {
        points.push_back( static_cast<cv::Point2d>( inputBlob.contour[ i ] ) );
    }
    project(points, points, projectionMatrix, offset);

    outputBlob.contour.clear();
    for(size_t i = 0 ; i < points.size() ; i++ )
    {
        outputBlob.contour.push_back( static_cast<cv::Point>( points[ i ] ) );
    }

    // update the orientation parameters
    getOrientation(outputBlob);
}

/**
    * \brief A function to apply a projection to a contour.
    * @param inputContour the contour which should be transformed
    * @param outputContour the contour which should be used for output
    * @param projectionMatrix the transformation Matrix to be used
    * @param offset an optional parameter to reverse the effect of cropped images
    */
void cSWE_LaneDetection::project(const std::vector< cv::Point2d >& inputContour, std::vector< cv::Point2d >& outputContour , const cv::Mat& projectionMatrix, int offset)
{
    std::vector< cv::Point2d > points;

    // add the offset
    for (size_t j = 0; j < inputContour.size(); j++)
    {
        points.push_back(inputContour[j]);
        points[j].y += offset;
    }

    // project
    perspectiveTransform(points, outputContour, projectionMatrix);
}

/**
    * \brief A function calculating the orientation of a BlobDescriptor.
    * This function calculates the eigenvectors and values of the contour of a BlobDescriptor,
    * using PCA. An orientation angle, the center of gravity and the ratio of the eigenvalues are subsequently calculated and
    * stored in the descriptor.
    * @param blob a BlobDescriptor with a valid contour, which will receive the new information
    */
void cSWE_LaneDetection::getOrientation(BlobDescriptor& blob)
{
    // ensure empty data containers
    blob.eigen_vals.clear();
    blob.eigen_vecs.clear();

    const std::vector< cv::Point >& contour = blob.contour;

    // bring the contour into the format used by the PCA
    cv::Mat contourAsMat = cv::Mat(contour.size(), 2, CV_64FC1);
    for (int i = 0; i < contourAsMat.rows; ++i)
    {
        contourAsMat.at<double>(i, 0) = contour[i].x;
        contourAsMat.at<double>(i, 1) = contour[i].y;
    }

    // do the PCA
    cv::PCA pca_analysis(contourAsMat, cv::Mat(), CV_PCA_DATA_AS_ROW);

    // calculate the center of gravity
    cv::Point centerOfGravity = cv::Point(pca_analysis.mean.at<double>(0, 0),pca_analysis.mean.at<double>(0, 1));
    blob.centerOfGravity = centerOfGravity;

    // store the results
    blob.eigen_vals.push_back( pca_analysis.eigenvalues.at<double>(0, 0) );
    blob.eigen_vecs.push_back( cv::Point2d(pca_analysis.eigenvectors.at<double>(0, 0), pca_analysis.eigenvectors.at<double>(0, 1)) );

    // store the second eigenvector and eigenvalue
    // they might not be calculated in case of line boundaries
    if (pca_analysis.eigenvectors.cols > 1 && pca_analysis.eigenvectors.rows > 1 && pca_analysis.eigenvalues.rows > 1)
    {
        blob.eigen_vals.push_back(pca_analysis.eigenvalues.at<double>(1, 0) );
        blob.eigen_vecs.push_back( cv::Point2d(pca_analysis.eigenvectors.at<double>(1, 0), pca_analysis.eigenvectors.at<double>(1, 1)) );
    }
    // store dummy values
    else
    {
        blob.eigen_vals.push_back(0.01);
        blob.eigen_vecs.push_back(Point2d(0, 0));
    }

    // calculate the ratio of the eigenvalues as a measure of elongation
    // avoid dividing by zero
    if (blob.eigen_vals[1] > 0.0001 )
    {
        blob.principalAxisLengthRatio = blob.eigen_vals[0] / blob.eigen_vals[1];
    }
    // store the maximum double value in case of a very small second eigenvalue
    else
    {
        blob.principalAxisLengthRatio = std::numeric_limits< double >().max();
    }

    // calculate the orientation angle
    blob.angleOfMainDirection = atan2(blob.eigen_vecs[0].y, blob.eigen_vecs[0].x);
}

/**
     * \brief A function verifying contours as candidates for lane boundaries and converting them into inverse perspective mapped descriptors with extra information.
     * The contours are first verified, by checking if they extend to the lowest parts of the image, specified by _startHeight.
     * Than the remaining contours get transformed and verified against a threshold _principalAxisLengthRatioThreshold, if they are elongated.
     * Next we check if they are linelike by comparing their area to their length. Finally we throw away those, to small to represent a boundary.
     * @param contours a vector of contours as found by opencv
     * @param blobs an outputvector of blobdescriptors which represent plausible laneboundaries
     */
void cSWE_LaneDetection::getBlobDescriptions( const std::vector< std::vector< cv::Point > >& contours, std::vector< BlobDescriptor >& blobs )
{
    // inspect every contour
    for (size_t i = 0; i < contours.size(); i++)
    {
        // count how many elements are present in a lower part of the image
        int contourElements = 0;
        for (size_t j = 0; j < contours[i].size(); j++)
        {
            int yIncrement = floor(( static_cast<double>(contours[i][j].x) - 320.0) * 0.1 );
            if (contours[i][j].y > _heightThresh + yIncrement )
            {
                contourElements++;
            }
        }
        // if at least some elements are present consider the contour
        if (contourElements > 0)
        {
            // create a descriptor and reduce the points of the contour
            BlobDescriptor descriptor;
            std::vector< cv::Point >& contour = descriptor.contour;

            double perspArcLength = arcLength(Mat(contours[i]), true);
            if( perspArcLength > 400 )
            {
                approxPolyDP(Mat(contours[i]), contour , perspArcLength*0.004, true);
            }
            else
            {
                approxPolyDP(Mat(contours[i]), contour , perspArcLength*0.04, true);
            }

            // calculate the orientation parameters of the contour
            getOrientation(descriptor);

            bool elongated = descriptor.principalAxisLengthRatio > _principalAxisLengthRatioThreshold;

            // decide on which side the object lies
            if (descriptor.angleOfMainDirection < -0.06)
            {
                descriptor.side = LEFT;
            }
            else if (descriptor.angleOfMainDirection > 0.06)
            {
                descriptor.side = RIGHT;
            }
            else
            {
                descriptor.side = AMBIGOUS;
            }

            // take the contour to the inverse perspective and recalculate it's orientation parameters
            project( descriptor , descriptor , _projectionMatrix , _startHeight );

            descriptor.complexBoundaryIndicator = calculateDirectionHistogram( descriptor );

            // calculate the length of the contour
            descriptor.lengthContour = arcLength(contour, true);

            // calculate the area of the contour, while handling the special case of a line boundary
            if (contour.size() > 2)
            {
                descriptor.areaContour = contourArea(contour);
            }
            else
            {
                descriptor.areaContour = descriptor.lengthContour;
            }

            // features to verify if we found a road boundary
            bool lineLike = descriptor.areaContour < _widthFactor * descriptor.lengthContour;
            bool large = descriptor.areaContour > _lowerAreaThreshold;

            // if it could be a roadboundary write it to our set
            if  ( elongated && lineLike && large && ( !(descriptor.side == AMBIGOUS) || descriptor.lengthContour < _minOuterBoundaryLength) )
            {
                blobs.push_back(descriptor);
            }
        }
    }

    // sort blobs depending on their arc_length in decreasing order
    sort(blobs.begin(), blobs.end(), sort_arcLength);
}

/**
     * \brief A function calculating the perpendicular distance of a point to a line.
     * @param directionVector the vector of direction along the line
     * @param originOfLine a point on the line
     * @param point the point for which the distance should be calculated
     * @return the perpendicular distance of the point to the line
     */
double getPerpendicularDistance(cv::Point2d directionVector, cv::Point originOfLine, cv::Point point)
{
    // calculated the normal vector of the line
    double lengthDirectionVector = cv::norm(directionVector);
    cv::Point2d normal = cv::Point2d(-directionVector.y / lengthDirectionVector, directionVector.x / lengthDirectionVector);

    // calculate the distance using Hesse-form
    return normal.ddot(point - originOfLine);
}

/**
     * \brief A function choosing the outerLaneBoundaries from a vector of candidates.
     * The first BlobDescriptor is assumed as the likeliest candidate for an outer lane boundary because it's the longest.
     * Therefore merely a minimum size requirement is checked on it. The second element is considered to be another outer lane boundary.
     * It is checked if it lies correctly in a corridor of the doubled distance to the middle Lane.
     * @param blobs a vector ordered by contour-length to choose the candidates from
     * @return an integer indicating if the first or second element in the vector are to be considered as lane boundaries
     */
int cSWE_LaneDetection::getOuterLaneBoundaries( std::vector< BlobDescriptor >& blobs )
{
    int outerLaneBoundariesIndicator = 0;

    if (blobs.size() > 0)
    {
        BlobDescriptor& firstBlob = blobs[0];

        // if the contour is larger than our threshold accept it as a outer lane boundary
        if (firstBlob.lengthContour > _minOuterBoundaryLength)
        {
            if(blobs.size() > 1)
            {
                std::vector<cv::Point2d> onePoint;
                onePoint.push_back(cv::Point2d(450,0));
                transformFromCarCoords(onePoint);
                cv::Point approxCarPosition = onePoint[0];

                bool sameSide = firstBlob.side == blobs[1].side;
                bool isNearer = cv::norm(firstBlob.centerOfGravity - approxCarPosition ) > cv::norm(blobs[1].centerOfGravity - approxCarPosition );
                bool isOuterBoundary = blobs[1].lengthContour > 1.0 * _minOuterBoundaryLength;

                if( sameSide && isNearer && isOuterBoundary )
                {
                    blobs.erase(blobs.begin());
                }
            }
            outerLaneBoundariesIndicator++;
        }

        // if we found one outer boundary, look for another
        bool found = outerLaneBoundariesIndicator > 0;
        bool moreCandidates = blobs.size() > 1;
        if (found && moreCandidates)
        {
            BlobDescriptor& secondBlob = blobs[1];

            // calculate the perpendicular distance of the first principal axis of the first blob to the second blob
            double distance = getPerpendicularDistance(firstBlob.eigen_vecs[0], firstBlob.centerOfGravity, secondBlob.centerOfGravity);

            if(secondBlob.side == RIGHT)
            {
                distance *= -1;
            }

            // calculated indicators if the candidate lies correctly
            bool aboveLowThreshold = distance > 2 * _middleDistanceLowThreshold;
            bool belowHighThreshold = distance < 2 * _middleDistanceHighThreshold;
            bool longEnough = secondBlob.lengthContour > _minOuterBoundaryLength;

            // if it's in the correct place, assume it's another outer boundary
            if (aboveLowThreshold && belowHighThreshold && longEnough )
            {
                outerLaneBoundariesIndicator++;
            }
        }
    }
    return outerLaneBoundariesIndicator;
}

/**
     * \brief A function reducing a contour to a pointset for a spline.
     * @param contour the contour to be reduced to a spline
     * @param splineSearchWidth the width of elements considered as a levelset for the y direction
     * @return a pair of indices representing the first and last index of the splinepoints in the contour
     */
std::pair< size_t, size_t > cSWE_LaneDetection::contourToSpline(const std::vector< cv::Point >& contour, const int splineSearchWidth, bool side)
{
    // buffer the contour and sort it according to it's y values
    std::vector< cv::Point > bufferedContour = contour;
    std::sort(bufferedContour.begin(), bufferedContour.end(), sort_yVal);

    // calculate the Thresholds for the levelset
    int upperThresholdY = bufferedContour[0].y - splineSearchWidth;
    int lowerThresholdY = bufferedContour[bufferedContour.size() - 1].y + splineSearchWidth;

    // if the thresholds overlap, make the levelset smaller
    if (upperThresholdY < lowerThresholdY)
    {
        upperThresholdY += ( splineSearchWidth - 1 );
        lowerThresholdY -= ( splineSearchWidth - 1 );
    }

    // extract the levelset
    std::vector< cv::Point > contourUpperYSet;
    std::vector< cv::Point > contourLowerYSet;
    for (size_t i = 0; i < bufferedContour.size(); i++)
    {
        if (bufferedContour[i].y > upperThresholdY )
        {
            contourUpperYSet.push_back(bufferedContour[i]);
        }
        if (bufferedContour[i].y < lowerThresholdY )
        {
            contourLowerYSet.push_back(bufferedContour[i]);
        }
    }

    // sort the levelset according to it's x value
    std::sort(contourLowerYSet.begin(), contourLowerYSet.end(), sort_xVal);
    std::sort(contourUpperYSet.begin(), contourUpperYSet.end(), sort_xVal);

    // choose the most left or most right point depending on the position in the levelset
    cv::Point startPoint = contourUpperYSet[0];
    cv::Point endPoint = contourLowerYSet[0];
    if (side)
    {
        startPoint = contourUpperYSet[ contourUpperYSet.size() - 1 ];
        endPoint = contourLowerYSet[ contourLowerYSet.size() - 1 ];
    }

    // find the indices in the contour for the calculated spline end-points
    size_t startIndex = -1;
    size_t endIndex = -1;
    for (size_t i = 0; i < contour.size(); ++i)
    {
        if (contour[i].x == startPoint.x && contour[i].y == startPoint.y)
        {
            startIndex = i;
        }
        if (contour[i].x == endPoint.x && contour[i].y == endPoint.y)
        {
            endIndex = i;
        }
    }

    // handle  the case where start and end are reversed
    if (startIndex > endIndex)
    {
        size_t buffer = startIndex;
        startIndex = endIndex;
        endIndex = buffer;
    }

    return std::pair< size_t, size_t>(startIndex, endIndex);
}

/**
     * \brief A function creating the image for visualization of the internal representation which was calculated.
     * @param image the image to draw on
     * @param blobs the vector of BlodDescriptors containing plausible lane candidates
     * @param outerLaneBoundariesIndicator the indicator for the outer lane boundaries
     * @param middleLaneBoundary the blobs considered as forming the middle lane boundary
     */
void cSWE_LaneDetection::drawResultImage(cv::Mat& image, const std::vector<BlobDescriptor>& blobs, const int outerLaneBoundariesIndicator,
                                         const std::vector< BlobDescriptor* > middleLaneBoundary)
{
    std::vector<Vec4i> hierarchy;

		// draw the orientation properties for every plausible blob
		for (size_t i = 0; i < blobs.size(); ++i)
		{
			const BlobDescriptor& blob = blobs[i];
            cv::Point currentCenterOfGravity = ( _resultImageScaleFactor * blob.centerOfGravity ) + _resultImageOffsetVector;

            if(blob.complexBoundaryIndicator)
            {
                circle(image, currentCenterOfGravity, 8, CV_RGB(255, 0, 255), 2);
            }

			circle(image, currentCenterOfGravity, 3, CV_RGB(255, 0, 255), 2);
            line(image, currentCenterOfGravity, currentCenterOfGravity + 0.02 * Point(blob.eigen_vecs[0].x * _resultImageScaleFactor * blob.eigen_vals[0], blob.eigen_vecs[0].y * _resultImageScaleFactor * blob.eigen_vals[0]), CV_RGB(255, 255, 0));
            line(image, currentCenterOfGravity, currentCenterOfGravity + 0.02 * Point(blob.eigen_vecs[1].x * _resultImageScaleFactor * blob.eigen_vals[1], blob.eigen_vecs[1].y * _resultImageScaleFactor * blob.eigen_vals[1]), CV_RGB(255, 255, 0));
		}

		// draw the outer lane boundaries
        for (int i = 0; i < outerLaneBoundariesIndicator; i++)
		{
            std::vector< std::vector< cv::Point > > contourToPaint;

			const BlobDescriptor& blob = blobs[i];
			
			// choose the color depending on the recognition
			Scalar color;
			if (blob.side == LEFT)
			{
				color = cv::Scalar(255, 0, 0);
			}
			else if (blob.side == RIGHT)
			{
				color = cv::Scalar(0, 255, 0);
			}
			else
			{
				color = cv::Scalar(0, 0, 255);
			}

			// offset the contours to fit nicely into the resultImage
            contourToPaint.push_back(blob.contour);
			for (size_t j = 0; j < contourToPaint[0].size(); ++j)
			{
                contourToPaint[0][j] *= _resultImageScaleFactor;
				contourToPaint[0][j] += _resultImageOffsetVector;
			}
			drawContours(image, contourToPaint, 0, color, 2, 8, hierarchy, 0, Point());
		}

		// paint the middle lane boundary
		cv::Scalar color = cv::Scalar(255, 255, 255);
		std::vector < std::vector < cv::Point > > contourToDraw;
		for (size_t i = 0; i < middleLaneBoundary.size(); ++i)
		{
			contourToDraw.push_back(middleLaneBoundary[i]->contour);
			
			// offset the contours to fit nicely into the resultImage
			for (size_t j = 0; j < contourToDraw[i].size(); ++j)
			{
                contourToDraw[i][j] *= _resultImageScaleFactor;
				contourToDraw[i][j] += _resultImageOffsetVector;
			}
		}

		for (size_t i = 0; i < middleLaneBoundary.size(); ++i)
		{
			drawContours(image, contourToDraw, i, color, 2, 8, hierarchy, 0, Point());
		}
}

/**
     * \brief A function drawing splines on images.
     * @param image the image to draw on
     * @param splinePoints the points for the spline
     * @param color the color for the spline
     */
void cSWE_LaneDetection::drawSpline( cv::Mat& image , const std::vector< cv::Point2d >& splinePoints , const cv::Scalar& color )
{
    std::vector< cv::Point2d > points;
    project(splinePoints , points , _backProjectionMatrix );

    CatMullRomSpline CRspline(points);

    size_t keyPoints = 500;
    double step = CRspline.getNumberOfPoints() / (double)(keyPoints - 1);
    for (size_t j = 0; j < keyPoints; j++)
    {
        Point dest = CRspline.getInterpolatedSplinePoint(j * step);
        circle(image, dest, 2, color , 2, 8);
    }
}

/**
     * \brief A function calculating properties of boundaries indicating complex boundaries.
     * @param image the image to draw on
     * @param blob a BlobDescriptor which should be examined
     */
bool cSWE_LaneDetection::calculateDirectionHistogram( const BlobDescriptor& blob)
{
    const std::vector< cv::Point >& contour = blob.contour;
    std::vector< cv::Point2d > directionVectors;
    std::vector< double > lengths;
    std::vector< double > angles;
    std::vector< double > curvature;
    size_t ninetyDegree = 0;

    if (contour.size() > 1)
    {
        for (size_t i = 1; i < contour.size(); ++i)
        {
            cv::Point2d directionVector = contour[i - 1] - contour[i];
            if (static_cast<int> (directionVector.y) != 0 && static_cast<int> (directionVector.x) != 0)
            {
                directionVectors.push_back(directionVector);
            }
        }
        {
            cv::Point2d directionVector = contour[0] - contour[contour.size() - 1];
            if(static_cast<int> (directionVector.y) != 0 && static_cast<int> (directionVector.x) != 0)
            {
                directionVectors.push_back(directionVector);
            }
        }

        for (size_t i = 0; i < directionVectors.size(); ++i)
        {
            double length = cv::norm(directionVectors[i]);
            lengths.push_back(length);
            double angle = std::acos(directionVectors[i].x / length);
            angles.push_back(angle);
        }

        for (size_t i = 1; i < angles.size(); i++)
        {
            double localCurvature = fabs(angles[i - 1] - angles[i]);
            curvature.push_back( localCurvature );

            double lengthTresh = 40;
            double curvatureThresh = 0.15;

            double higherCurvatureThresh = 0.5 * CV_PI + curvatureThresh;
            double lowerCurvatureThresh = 0.5 * CV_PI - curvatureThresh;

            bool bigLengths = lengths[i - 1] > lengthTresh && lengths[i] > lengthTresh;
            bool bigCurvature = localCurvature > lowerCurvatureThresh && localCurvature < higherCurvatureThresh;

            if ( bigCurvature && bigLengths )
            {
                ninetyDegree++;
            }
        }
    }

    if (ninetyDegree > 1 && _draw)
    {
        std::vector< cv::Point2d > vec;
        vec.push_back(blob.centerOfGravity);

        perspectiveTransform(vec, vec, _backProjectionMatrix);

        return true;
    }
    return false;
}

void cSWE_LaneDetection::serializeLane( cObjectPtr<IMediaCoder>& pCoder , std::string lane , const std::vector< Point2d >& spline )
{
    stringstream elementSetter;
    size_t end = std::min( static_cast< size_t >( 25 ), static_cast< size_t >( spline.size() ) );
    for(size_t j=0; j < end; j++)
    {
        elementSetter << lane + ".Points[" << j << "].xCoord";
        pCoder->Set(elementSetter.str().c_str(), (tVoid*)&(spline.at(j).x));
        elementSetter.str(std::string());

        elementSetter << lane + ".Points[" << j << "].yCoord";
        pCoder->Set(elementSetter.str().c_str(), (tVoid*)&(spline.at(j).y));
        elementSetter.str(std::string());
    }

    tInt8 BoundaryArrayCountTemp = static_cast<tInt8>(end);
    std::string tempString( lane + ".Count" );
    pCoder->Set( tempString.c_str() , (tVoid*)&(BoundaryArrayCountTemp));
}

tResult cSWE_LaneDetection::transmitLanes( const std::vector< Point2d >& leftSpline , const std::vector< Point2d >& middleSpline , const std::vector< Point2d >& rightSpline )
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSplines->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescSplines->WriteLock(pMediaSampleOutput, &pCoder));

    serializeLane( pCoder , "rightBoundary" , rightSpline );
    serializeLane( pCoder , "middleBoundary" , middleSpline );
    serializeLane( pCoder , "leftBoundary" , leftSpline );

    m_pCoderDescSplines->Unlock(pCoder);

    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oSplinesPin.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}

/**
 *
 */
tResult cSWE_LaneDetection::transmitCrossingIndicator( const tBool isRealStopLine , const tInt8 crossingType , const cv::Point2d& StopLinePoint1 , const cv::Point2d& StopLinePoint2 )
{
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleCrossIndicator;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleCrossIndicator));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescCrossingIndicator->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleCrossIndicator->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(m_pCoderDescCrossingIndicator->WriteLock(pMediaSampleCrossIndicator, &pCoder));

    pCoder->Set("isRealStopLine", (tVoid*)&(isRealStopLine));
    pCoder->Set("crossingType", (tVoid*)&(crossingType));
    pCoder->Set("StopLinePoint1.xCoord", (tVoid*)&(StopLinePoint1.x));
    pCoder->Set("StopLinePoint1.yCoord", (tVoid*)&(StopLinePoint1.y));
    pCoder->Set("StopLinePoint2.xCoord", (tVoid*)&(StopLinePoint2.x));
    pCoder->Set("StopLinePoint2.yCoord", (tVoid*)&(StopLinePoint2.y));

    m_pCoderDescCrossingIndicator->Unlock(pCoder);

    //transmit media sample over output pin

    RETURN_IF_FAILED(pMediaSampleCrossIndicator->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_CrossingIndicatorPin.Transmit(pMediaSampleCrossIndicator));

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

    // crop the image
    cv::Rect myROI( 0 , _startHeight , image.cols , image.rows - _startHeight );
    cv::Mat croppedImage( image , myROI );

    // transform the image to a grayscale image
    Mat greyScaleImage;
    cvtColor(croppedImage, greyScaleImage, CV_RGB2GRAY, 1);

    // aquire mean and stdDev of the image
    cv::Scalar mean, stdDev;
    cv::meanStdDev(greyScaleImage, mean, stdDev);

    // threshold the image to remove objects which don't necessary represent lane markings
    Mat thresholdedImage;
    double thresh = mean[0] + _CountStdDevs * stdDev[0];
    cv::threshold(greyScaleImage, thresholdedImage, thresh , 255, cv::THRESH_TOZERO);

    // extract contours from the image
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours(thresholdedImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    cv::Mat result = cv::Mat( 2 * image.rows, 2 * image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    // get enhanced Descriptions of the blobs with basic removal of candidates
    std::vector< BlobDescriptor > blobs;
    getBlobDescriptions( contours, blobs);

    // get the outer Lane Boundaries
    int outerLaneBoundariesIndicator = getOuterLaneBoundaries( blobs );

    std::vector< cv::Point2d > rightSpline;
    std::vector< cv::Point2d > leftSpline;

    // commit to decisions what we have found
    for (int i = 0; i < outerLaneBoundariesIndicator; ++i)
    {
        BlobDescriptor& blob = blobs[i];
        // only accept the candidate if we didn't already find a better
        if (blob.side == RIGHT && rightSpline.size() == 0 )
        {
            std::pair< size_t, size_t > indices = contourToSpline(blob.contour, _splineSearchWidth);

            if (indices.first != indices.second)
            {
                for (size_t i = indices.first; i <= indices.second; ++i)
                {
                    rightSpline.push_back( static_cast< cv::Point2d >( blob.contour[i] ));
                }
            }
        }
        // only accept the candidate if we didn't already find a better
        else if (blob.side == LEFT && leftSpline.size() == 0)
        {
            std::pair< size_t, size_t > indices = contourToSpline(blob.contour, _splineSearchWidth, true);
            if (indices.first != indices.second)
            {
                for (size_t i = indices.first; i <= indices.second; ++i)
                {
                    leftSpline.push_back( static_cast< cv::Point2d >( blob.contour[i] ));
                }
            }
        }
    }

    // decide on the middle lane boundaries by finding BlobDescriptors with an appropropriate distance to the found outer lane boundary
    std::vector< BlobDescriptor* > middleLaneBoundary;
    std::vector< double > distances;
    for (size_t i = 1; i < blobs.size(); ++i)
    {
        // calculate the perpendicular distance to the found outer lane boundary
        double distance = getPerpendicularDistance(blobs[0].eigen_vecs[0],blobs[ 0 ].centerOfGravity , blobs[ i ].centerOfGravity);
        if(blobs[0].side == LEFT)
        {
            distance *= -1;
        }
        distances.push_back(distance);
    }

    // pick all the blobs with an appropriate distance
    for (size_t i = 1; i < blobs.size(); ++i)
    {
        bool aboveLowThreshold = distances[i - 1] > _middleDistanceLowThreshold;
        bool belowHighThreshold = distances[i - 1] < _middleDistanceHighThreshold;
        bool sensibleLength = blobs[i].lengthContour > _minLengthMiddleBoundary && blobs[i].lengthContour < _maxLengthMiddleBoundary;
        if ( belowHighThreshold && aboveLowThreshold && sensibleLength)
        {
            middleLaneBoundary.push_back(&blobs[i]);
        }
    }

    if (_draw)
    {
        drawResultImage(result, blobs, outerLaneBoundariesIndicator, middleLaneBoundary);
    }

    // concatentate the blobs for the middle lane boundary to a single lane boundary and form a spline from it
    std::vector< cv::Point2d > middleSpline;
    if (middleLaneBoundary.size() > 0)
    {
        std::vector< std::pair< size_t, size_t > > indicesVector;
        for (size_t i = 0; i < middleLaneBoundary.size(); ++i)
        {
            std::pair< size_t, size_t > indices = contourToSpline(middleLaneBoundary[ i ]->contour , 4 );

            indicesVector.push_back(indices);
        }

        for (size_t i = 0; i < indicesVector.size(); ++i)
        {
            std::pair< size_t, size_t >& indices = indicesVector[i];
            int first = indices.first;
            int second = indices.second;

            if (indices.first != indices.second)
            {
                for (int j = second; j >= first; --j)
                {
                    middleSpline.push_back( static_cast< cv::Point2d >( middleLaneBoundary[ i ]->contour[j] ));
                }
            }
        }
        sort(middleSpline.begin(), middleSpline.end(), sort_yVal_double);
    }

    // draw the results on the original image;
    if (_draw)
    {
        if (leftSpline.size() > 0)
        {
            drawSpline(image, leftSpline, cv::Scalar(255, 0, 0));
        }
        if (rightSpline.size() > 0)
        {
            drawSpline(image, rightSpline, cv::Scalar(0, 255, 0));
        }
        if (middleSpline.size() > 0)
        {
            drawSpline(image, middleSpline, cv::Scalar(255, 0, 255));
        }
    }

    transformToCarCoords(rightSpline);
    transformToCarCoords(leftSpline);
    transformToCarCoords(middleSpline);

    transmitLanes(leftSpline,middleSpline,rightSpline);

    //TODO: STILL crashes;
    tBool isRealStopLine = true;
    tInt8 crossingType = 1;
    cv::Point2d p1(1,2);
    cv::Point2d p2(3,4);
    transmitCrossingIndicator(isRealStopLine, crossingType, p1, p2);

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

    _result = result;

    RETURN_NOERROR;
}
