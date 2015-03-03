#include "stdafx.h"
#include "SWE_LaneDetection.h"
#include "SWE_CatmullRomSpline.h"
#include <template_data.h>

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_LaneDetection", OID_ADTF_LANEDETECTION_FILTER , cSWE_LaneDetection);

// Macros used to decouple text from code
#define CORRESPING_POINTS_XML "Path to external Camera Params xml"
#define COUNT_OF_STDDEVS "Stddevs for the Thresholding"
#define DISTANCE_TO_RIGHT_REFERENCEPOINT "The distance of the ReferencePoint to the Right Lane"
#define DISTANCE_TO_LEFT_REFERENCEPOINT "The distance of the ReferencePoint to the Left Lane"
#define MINDISTANCE_TO_REFERENCEPOINT "The threshold for a blob to be considered als lying on the reference"
#define DRAW_IMAGES "Boolean indicating wether visualizations should be drawn"
#define HEIGHT_THRESHOLD "The Threshold at which Blobs are removed"
#define LOWER_AREA_THRESHOLD "Minimum area of a blob to be considered"
#define START_HEIGHT "The heightvalue to which the image is cropped"
#define PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD "The ratio of eigenvalues at which a blob is rejected as boundary"
#define SPLINE_SEARCH_WIDTH "The width of pixels which is considered as a Y-Levelset for the spline reduction"

/**
 * @brief cSWE_LaneDetection::cSWE_LaneDetection
 * Constructs a cSWE_LaneDetection object, defining it's properties, to be set by ADTF;
 * @param __info The info object for the Filter
 */
cSWE_LaneDetection::cSWE_LaneDetection(const tChar* __info):cFilter(__info)
{
    SetPropertyFloat( COUNT_OF_STDDEVS , 3.0);
    SetPropertyBool( COUNT_OF_STDDEVS NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( COUNT_OF_STDDEVS NSSUBPROP_MINIMUM  , 0.5 );
    SetPropertyFloat( COUNT_OF_STDDEVS NSSUBPROP_MAXIMUM  , 10.0 );

    SetPropertyInt( DISTANCE_TO_RIGHT_REFERENCEPOINT , 450);
    SetPropertyBool( DISTANCE_TO_RIGHT_REFERENCEPOINT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( DISTANCE_TO_RIGHT_REFERENCEPOINT NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( DISTANCE_TO_RIGHT_REFERENCEPOINT NSSUBPROP_MAXIMUM  , 999);

    SetPropertyInt( DISTANCE_TO_LEFT_REFERENCEPOINT , 150);
    SetPropertyBool( DISTANCE_TO_LEFT_REFERENCEPOINT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( DISTANCE_TO_LEFT_REFERENCEPOINT NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( DISTANCE_TO_LEFT_REFERENCEPOINT NSSUBPROP_MAXIMUM  , 999);

    SetPropertyFloat( MINDISTANCE_TO_REFERENCEPOINT , 120);
    SetPropertyBool( MINDISTANCE_TO_REFERENCEPOINT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( MINDISTANCE_TO_REFERENCEPOINT NSSUBPROP_MINIMUM  , 0);
    SetPropertyFloat( MINDISTANCE_TO_REFERENCEPOINT NSSUBPROP_MAXIMUM  , 999);

    SetPropertyBool( DRAW_IMAGES , true);
    SetPropertyBool( DRAW_IMAGES NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyStr( CORRESPING_POINTS_XML , "/home/odroid/AADC/calibration_files/points.xml");
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt( HEIGHT_THRESHOLD , 50);
    SetPropertyBool( HEIGHT_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( HEIGHT_THRESHOLD NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( HEIGHT_THRESHOLD NSSUBPROP_MAXIMUM  , 479);

    SetPropertyInt( START_HEIGHT , 255 );
    SetPropertyBool( START_HEIGHT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( START_HEIGHT NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( START_HEIGHT NSSUBPROP_MAXIMUM  , 479);

    SetPropertyFloat( LOWER_AREA_THRESHOLD , 20);
    SetPropertyBool( LOWER_AREA_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( LOWER_AREA_THRESHOLD NSSUBPROP_MINIMUM  , 0.0);
    SetPropertyFloat( LOWER_AREA_THRESHOLD NSSUBPROP_MAXIMUM  , 99999.0);

    SetPropertyFloat( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD , 5.0 );
    SetPropertyBool( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD NSSUBPROP_MINIMUM  , 0.01);
    SetPropertyFloat( PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD NSSUBPROP_MAXIMUM  , 99999.0);

    SetPropertyInt( SPLINE_SEARCH_WIDTH , 20 );
    SetPropertyBool( SPLINE_SEARCH_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( SPLINE_SEARCH_WIDTH NSSUBPROP_MINIMUM  , 2);
    SetPropertyInt( SPLINE_SEARCH_WIDTH NSSUBPROP_MAXIMUM  , 200);
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

        /*
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

        // Output pin for Splines
        tChar const * strDescSplines = pDescManager->GetMediaDescription("tSplineBoundaries");
        RETURN_IF_POINTER_NULL(strDescSplines);
        cObjectPtr<IMediaType> pTypeSplines = new cMediaType(0, 0, 0, "tSplineBoundaries", strDescSplines,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSplines->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSplines));

        RETURN_IF_FAILED(m_oSplines.Create("Spline_Boundaries", pTypeSplines, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSplines));
        */
    }
    else if (eStage == StageNormal)
    {
        // fetch the algorithmic parameters
        _rightDistanceReferencePoint = GetPropertyInt(DISTANCE_TO_RIGHT_REFERENCEPOINT);
        _leftDistanceReferencePoint = GetPropertyInt(DISTANCE_TO_LEFT_REFERENCEPOINT);
        _minDistanceToReferencePoint = GetPropertyFloat(MINDISTANCE_TO_REFERENCEPOINT);
        _heightThresh = GetPropertyInt(HEIGHT_THRESHOLD);
        _draw = GetPropertyBool(DRAW_IMAGES),
        _CountStdDevs = GetPropertyInt(COUNT_OF_STDDEVS);
        _lowerAreaThreshold = GetPropertyFloat(LOWER_AREA_THRESHOLD);
        _startHeight = GetPropertyInt(START_HEIGHT);
        _principalAxisLengthRatioThreshold = GetPropertyFloat(PRINCIPAL_AXIS_LENGTH_RATIO_THRESHOLD);
        _splineSearchWidth = GetPropertyInt(SPLINE_SEARCH_WIDTH);

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
    _sInternRepresentationBitMapOutputFormat.nBitsPerPixel = 24;
    _sInternRepresentationBitMapOutputFormat.nPixelFormat = cImage::PF_RGB_888;
    _sInternRepresentationBitMapOutputFormat.nPaletteSize = 0;
    _sInternRepresentationBitMapOutputFormat.nWidth = 640;
    _sInternRepresentationBitMapOutputFormat.nHeight = 480 - _startHeight;
    _sInternRepresentationBitMapOutputFormat.nBytesPerLine = 640 * 3;
    _sInternRepresentationBitMapOutputFormat.nSize = _sInternRepresentationBitMapOutputFormat.nBytesPerLine * ( 480 - _startHeight );

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

bool sort_arcLength( const cSWE_LaneDetection::BlobDescriptor& blob1, const cSWE_LaneDetection::BlobDescriptor blob2)
{
    return blob1.lengthContour > blob2.lengthContour;
}

bool sort_yVal( const cv::Point& point1, const cv::Point& point2 )
{
    return point1.y > point2.y;
}

bool sort_xVal( const cv::Point& point1, const cv::Point& point2)
{
    return point1.x < point2.x;
}

void cSWE_LaneDetection::getOrientation(BlobDescriptor& blob )
{
    const std::vector< cv::Point >& pts = blob.contour;

    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));

    blob.eigen_vals.push_back( pca_analysis.eigenvalues.at<double>(0, 0) );
    blob.eigen_vecs.push_back( Point2d(pca_analysis.eigenvectors.at<double>(0, 0), pca_analysis.eigenvectors.at<double>(0, 1)) );

    if (pca_analysis.eigenvectors.cols > 1 && pca_analysis.eigenvectors.rows > 1 && pca_analysis.eigenvalues.rows > 1)
    {
        blob.eigen_vals.push_back(pca_analysis.eigenvalues.at<double>(1, 0) );
        blob.eigen_vecs.push_back( Point2d(pca_analysis.eigenvectors.at<double>(1, 0), pca_analysis.eigenvectors.at<double>(1, 1)) );
    }
    else
    {
        blob.eigen_vals.push_back(0.01);
        blob.eigen_vecs.push_back(Point2d(0, 0));
    }

    blob.principalAxisLengthRatio = blob.eigen_vals[0] / blob.eigen_vals[1];
    blob.angleOfMainDirection = atan2(blob.eigen_vecs[0].y, blob.eigen_vecs[0].x);
    blob.centerOfGravity = pos;
}

void cSWE_LaneDetection::getBlobDescriptions(const std::vector< std::vector< cv::Point > >& contours, std::vector< BlobDescriptor >& blobs )
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        vector< Point > someContour;
        for (size_t j = 0; j < contours[i].size(); j++)
        {
            if (contours[i][j].y > _heightThresh)
            {
                someContour.push_back(contours[i][j]);
            }
        }
        if (someContour.size() > 0)
        {
            vector<Point> approx;
            approxPolyDP(Mat(contours[i]), approx,arcLength(Mat(contours[i]), true)*0.005, true);

            BlobDescriptor descriptor;
            descriptor.contour = approx;
            descriptor.lengthContour = arcLength(approx, true);

            if (approx.size() > 2)
            {
                descriptor.areaContour = contourArea(approx);
            }
            else
            {
                descriptor.areaContour = descriptor.lengthContour;
            }

            getOrientation(descriptor);

            if (descriptor.angleOfMainDirection < -0.1)
            {
                descriptor.side = LEFT;
            }
            else if (descriptor.angleOfMainDirection > 0.1)
            {
                descriptor.side = RIGHT;
            }
            else
            {
                descriptor.side = AMBIGOUS;
            }

            if  (
                 descriptor.principalAxisLengthRatio > _principalAxisLengthRatioThreshold &&
                 descriptor.areaContour > _lowerAreaThreshold &&
                 descriptor.areaContour < 6 * descriptor.lengthContour
                 )
            {
                blobs.push_back(descriptor);
            }
        }
    }

    sort(blobs.begin(), blobs.end(), sort_arcLength);
}

void cSWE_LaneDetection::drawResultImage(cv::Mat& image, const std::vector<BlobDescriptor>& blobs, const std::vector< bool > outerLaneBoundariesIndicator,
                                         const cv::Point& referencePoint, const std::vector< BlobDescriptor* > middleLaneBoundary )
{
    vector<Vec4i> hierarchy;

    for (size_t i = 0; i < blobs.size(); ++i)
    {
        const BlobDescriptor& blob = blobs[i];

        // Draw the principal components
        circle(image, blob.centerOfGravity, 3, CV_RGB(255, 0, 255), 2);
        line(image, blob.centerOfGravity, blob.centerOfGravity + 0.02 * Point(blob.eigen_vecs[0].x * blob.eigen_vals[0], blob.eigen_vecs[0].y * blob.eigen_vals[0]), CV_RGB(255, 255, 0));
        line(image, blob.centerOfGravity, blob.centerOfGravity + 0.02 * Point(blob.eigen_vecs[1].x * blob.eigen_vals[1], blob.eigen_vecs[1].y * blob.eigen_vals[1]), CV_RGB(255, 255, 0));
    }

    circle( image, referencePoint, 10, Scalar(255, 0, 0));

    for (size_t i = 0; i < outerLaneBoundariesIndicator.size(); i++)
    {
        std::vector< std::vector< cv::Point > > contourToPaint;

        const BlobDescriptor& blob = blobs[i];
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

        contourToPaint.push_back(blob.contour);
        drawContours(image, contourToPaint, 0, color, 2, 8, hierarchy, 0, Point());
    }

    cv::Scalar color = cv::Scalar(255, 255, 255);

    std::vector < std::vector < cv::Point > > contourToDraw;
    for (size_t j = 0; j < middleLaneBoundary.size(); ++j)
    {
        contourToDraw.push_back(middleLaneBoundary[j]->contour);
    }

    for (size_t i = 0; i < middleLaneBoundary.size();++i)
    {
        drawContours(image, contourToDraw, i, color, 2, 8, hierarchy, 0, Point());
    }
}

std::vector< bool > cSWE_LaneDetection::getOuterLaneBoundaries( std::vector< BlobDescriptor >& blobs )
{
    vector<Vec4i> hierarchy;
    std::vector< bool > outerLaneBoundariesIndicator;

    int endIndex = min(2, (int)blobs.size());
    double lengthFormerBlob = 0;
    for (int i = 0; i < endIndex; i++)
    {
        BlobDescriptor& blob = blobs[i];

        if (i != 1 || blob.lengthContour > 0.5 * lengthFormerBlob)
        {
            if (i == 0)
            {
                lengthFormerBlob = blob.lengthContour;
                outerLaneBoundariesIndicator.push_back( true );
            }
            else
            {
                outerLaneBoundariesIndicator.push_back( true );
            }
        }
    }
    return outerLaneBoundariesIndicator;
}

cv::Point cSWE_LaneDetection::getReferencePoint( const std::vector< bool >& outerLaneBoundariesIndicator , const std::vector< BlobDescriptor >& blobs , cv::Mat& image )
{
    cv::Point referencePoint(0, 0);
    if (outerLaneBoundariesIndicator.size() > 0)
    {
        const BlobDescriptor& blob = blobs[0];
        const std::vector< cv::Point >& contour = blob.contour;
        Side side = blob.side;

        int yMax = 0;
        size_t contourSize = contour.size();

        std::vector< const cv::Point* > yMinLocs;
        for (size_t i = 0; i < contourSize; i++)
        {
            int yCoord = contour[i].y;
            if (yCoord > yMax)
            {
                yMax = yCoord;
            }
        }
        for (size_t i = 0; i < contourSize; i++)
        {
            int yCoord = contour[i].y;
            if (yCoord == yMax)
            {
                yMinLocs.push_back(&(contour[i]));
            }
        }

        const cv::Point* targetPoint = NULL;

        if (side == RIGHT)
        {
            int xMax = image.cols;
            for (size_t i = 0; i < yMinLocs.size(); i++)
            {
                int xCoord = yMinLocs[i]->x;
                if (xCoord < xMax)
                {
                    xMax = xCoord;
                    targetPoint = yMinLocs[i];
                }
            }
            referencePoint.y = image.rows - 5;
            referencePoint.x = targetPoint->x - _rightDistanceReferencePoint;
        }
        else if (side == LEFT)
        {
            int xMax = 0;
            for (size_t i = 0; i < yMinLocs.size(); i++)
            {
                int xCoord = yMinLocs[i]->x;
                if (xCoord > xMax)
                {
                    xMax = xCoord;
                    targetPoint = yMinLocs[i];
                }
            }
            referencePoint.y = image.rows - 5;
            referencePoint.x = targetPoint->x + _leftDistanceReferencePoint;
        }
        else
        {
            //TODO: find a better solution, maybe employ inverse perspective
            referencePoint.y = blob.centerOfGravity.y;
            referencePoint.x = blob.centerOfGravity.x;
        }
    }

    return referencePoint;
}

bool cSWE_LaneDetection::findNextMiddleLaneBlob(std::vector< BlobDescriptor* >& middleLaneBoundary , const std::vector< bool >& outerLaneBoundariesIndicator,
                                                std::vector< BlobDescriptor >& blobs, const cv::Point& referencePoint , const double distanceThreshold)
{
    int indexOfMinDistance = -1;
    double minDistance = 1000;
    if (referencePoint.x != 0)
    {
        for (size_t i = outerLaneBoundariesIndicator.size(); i < blobs.size(); i++)
        {
            BlobDescriptor& blob = blobs[i];
            double lengthContour = blob.lengthContour;
            if (lengthContour > 40 && lengthContour < 440)
            {
                blob.distanceToReference = cv::norm(referencePoint - blob.centerOfGravity);
                double distance = blob.distanceToReference;

                if (distance < minDistance)
                {
                    indexOfMinDistance = i;
                    minDistance = distance;
                }
            }
        }
    }

    // rember to bound minDistance by < 1000;
    if (indexOfMinDistance != -1 && minDistance < distanceThreshold )
    {
        BlobDescriptor& blob = blobs[indexOfMinDistance];

        if (middleLaneBoundary.size() > 0)
        {
            if (blob.areaContour >= middleLaneBoundary[ middleLaneBoundary.size() - 1 ]->areaContour)
            {
                return false;
            }
        }

        // resolve ambiguities
        if (blobs[0].side == AMBIGOUS)
        {
            // calculate directionvector of a middleLaneBoundaries Center of Gravity
            // to the ambiguos boundaries Center of Gravity
            cv::Point direction = blobs[0].centerOfGravity - blob.centerOfGravity;

            // if it points upwards decide it's a right boundary
            if (direction.y < 0)
            {
                blobs[0].side = RIGHT;
            } // else it must be a left boundary ( we're outside the road ! )
            else
            {
                blobs[0].side = LEFT;
            }
        }

        middleLaneBoundary.push_back(&blob);
        return true;
    }
    return false;
}

std::pair< size_t, size_t > cSWE_LaneDetection::contourToSpline(std::vector< cv::Point >& contour, const int splineSearchWidth, bool side)
{
    std::vector< cv::Point > bufferedContour = contour;
    std::sort(bufferedContour.begin(), bufferedContour.end(), sort_yVal);

    int upperThresholdY = bufferedContour[0].y - splineSearchWidth;
    int lowerThresholdY = bufferedContour[bufferedContour.size() - 1].y + splineSearchWidth;

    if (upperThresholdY < lowerThresholdY)
    {
        upperThresholdY += ( splineSearchWidth - 1 );
        lowerThresholdY -= ( splineSearchWidth - 1 );
    }

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

    std::sort(contourLowerYSet.begin(), contourLowerYSet.end(), sort_xVal);
    std::sort(contourUpperYSet.begin(), contourUpperYSet.end(), sort_xVal);

    cv::Point startPoint = contourUpperYSet[0];
    cv::Point endPoint = contourLowerYSet[0];
    if (side)
    {
        startPoint = contourUpperYSet[ contourUpperYSet.size() - 1 ];
        endPoint = contourLowerYSet[ contourLowerYSet.size() - 1 ];
    }

    // just initialize them to prevent a warning - they are ALWAYS initialized afterwards
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
    if (startIndex > endIndex)
    {
        size_t buffer = startIndex;
        startIndex = endIndex;
        endIndex = buffer;
    }

    return std::pair< size_t, size_t>(startIndex, endIndex);
}

void cSWE_LaneDetection::drawSpline( cv::Mat& image , const std::vector< cv::Point2d >& splinePoints , const cv::Scalar& color )
{
    CatMullRomSpline CRspline(splinePoints);

    size_t keyPoints = 500;
    double step = CRspline.getNumberOfPoints() / (double)(keyPoints - 1);
    for (size_t j = 0; j < keyPoints; j++)
    {
        Point dest = CRspline.getInterpolatedSplinePoint(j * step);
        circle(image, dest, 2, color , 2, 8);
    }
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
    double thresh = mean[0] + 2 * stdDev[0];
    cv::threshold(greyScaleImage, thresholdedImage, thresh , 255, cv::THRESH_TOZERO);

    // extract contours from the image
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(thresholdedImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //TODO:	submodule ?

    cv::Mat result = cv::Mat(croppedImage.rows, croppedImage.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    // get enhanced Descriptions of the blobs with basic removal of candidates
    std::vector< BlobDescriptor > blobs;
    getBlobDescriptions(contours, blobs);

    // get the outer Lane Boundaries
    std::vector< bool > outerLaneBoundariesIndicator = getOuterLaneBoundaries( blobs );

    BlobDescriptor* rightLaneBoundary = NULL;
    BlobDescriptor* leftLaneBoundary = NULL;
    std::vector< BlobDescriptor* > middleLaneBoundary;

    // predict the location of the first middleLaneBlob
    cv::Point referencePoint = getReferencePoint(outerLaneBoundariesIndicator, blobs, result);

    // find a middleLaneBlob which is close enough to the prediction
    bool found = findNextMiddleLaneBlob(middleLaneBoundary, outerLaneBoundariesIndicator, blobs, referencePoint, _minDistanceToReferencePoint);

    // commit to decisions what we have found
    for (size_t i = 0; i < outerLaneBoundariesIndicator.size(); ++i)
    {
        BlobDescriptor& blob = blobs[i];
        if (blob.side == RIGHT)
        {
            if (rightLaneBoundary == NULL)
            {
                rightLaneBoundary = &blob;
            }
            else if (rightLaneBoundary->side == AMBIGOUS)
            {
                rightLaneBoundary = &blob;
            }
        }
        else if (blob.side == LEFT)
        {
            if (leftLaneBoundary == NULL)
            {
                leftLaneBoundary = &blob;
            }
            else if (leftLaneBoundary->side == AMBIGOUS)
            {
                leftLaneBoundary = &blob;
            }
        }
        //TODO: idea is to resolve ambiguity by guessing right, but that's dangerous -> speak to Matthias
        else if (blob.side == AMBIGOUS && rightLaneBoundary == NULL && leftLaneBoundary == NULL )
        {
            rightLaneBoundary = &blob;
        }
    }

    // starting from the first middleLaneBlob try to find some more
    size_t index = 0;
    while (found)
    {
        const BlobDescriptor& blob = *(middleLaneBoundary[ index ]);

        cv::Point updatedReferencePoint = blob.centerOfGravity;
        updatedReferencePoint.y -= 80;
        if (rightLaneBoundary != NULL)
        {
            updatedReferencePoint.x += 50.0 * rightLaneBoundary->eigen_vecs[0].x;
        }
        else if ( leftLaneBoundary != NULL )
        {
            updatedReferencePoint.x += 80.0 * leftLaneBoundary->eigen_vecs[0].x;
        }

        circle(result, updatedReferencePoint, 10, cv::Scalar(255, 0, 255));

        found = findNextMiddleLaneBlob(middleLaneBoundary, outerLaneBoundariesIndicator, blobs, updatedReferencePoint, 0.8 * _minDistanceToReferencePoint);
        index++;
    }

    // create the debugimage of the internal representation
    if (_draw)
    {
        drawResultImage(result, blobs , outerLaneBoundariesIndicator , referencePoint , middleLaneBoundary);
    }

    // transform back to original image from the cropped
    for (size_t i = 0; i < blobs.size(); i++)
    {
        std::vector< cv::Point >& contour = blobs[i].contour;
        for (size_t j = 0; j < contour.size(); j++)
        {
            contour[j].y += _startHeight;
        }
    }

    std::vector< cv::Point2d > rightSpline;
    std::vector< cv::Point2d > leftSpline;
    std::vector< cv::Point2d > middleSpline;

    if (rightLaneBoundary != NULL)
    {
        std::pair< size_t, size_t > indices = contourToSpline(rightLaneBoundary->contour , _splineSearchWidth );

        if (indices.first != indices.second)
        {
            for (size_t i = indices.first; i <= indices.second; ++i)
            {
                rightSpline.push_back(rightLaneBoundary->contour[i]);
            }
        }
    }

    if (leftLaneBoundary != NULL)
    {
        std::pair< size_t, size_t > indices = contourToSpline(leftLaneBoundary->contour , _splineSearchWidth , true );

        if (indices.first != indices.second)
        {
            for (size_t i = indices.first; i <= indices.second; ++i)
            {
                leftSpline.push_back(leftLaneBoundary->contour[i]);
            }
        }
    }

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
                    middleSpline.push_back(middleLaneBoundary[ i ]->contour[j]);
                }
            }
        }
        sort(middleSpline.begin(), middleSpline.end(), sort_yVal);
    }

    // draw on original image;
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

    if (leftSpline.size() > 0)
    {
        perspectiveTransform( leftSpline , leftSpline , _projectionMatrix );
    }
    if (rightSpline.size() > 0)
    {
        perspectiveTransform( rightSpline , rightSpline , _projectionMatrix );
    }
    if (middleSpline.size() > 0)
    {
        perspectiveTransform( middleSpline , middleSpline , _projectionMatrix );
    }

 /*******************************************************/
 /*          Here the transmit Section will begin       */
 /*******************************************************/

 /*
    double leftFrontX = -1;
    double leftFrontY = -1;
    double leftRearX = -1;
    double leftRearY = -1;
    double rightFrontX = -1;
    double rightFrontY = -1;
    double rightRearX = -1;
    double rightRearY = -1;


    vector< vector<cv::Point2d> > splines;
    splines.resize(lanes.size());
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
//        splines[i] = vec;

        // scaling factor for calculating pixel in mm
        tFloat64 scaleFac = 0.95;
        // distance from front axis to nearest edge of camera picture
        tFloat64 distFrontAxisToInvImage = 220-20-65;
        // value to correct camera not cenered in y-direction
        tFloat64 distMidToCam = -22;
        // picture heigth and width
        tFloat64 picHeight = 480;
        tFloat64 picWidth = 640;

        // transform points to vehicle coo sys in front axis center
        if (theta >= 0 && theta < _thetaMax)
        {
            leftFrontY = scaleFac*((-1.0)*(pt1.x - picWidth/2.0)) + distMidToCam;
            leftFrontX = scaleFac*((-1.0)*(pt1.y - picHeight)) + distFrontAxisToInvImage;
            leftRearY = scaleFac*((-1.0)*(pt2.x - picWidth/2.0)) + distMidToCam;
            leftRearX = scaleFac*((-1.0)*(pt2.y - picHeight)) + distFrontAxisToInvImage;

            splines[i].push_back(cv::Point2d(leftFrontX, leftFrontY));
            splines[i].push_back(cv::Point2d(leftRearX, leftRearY));
        }
        else
        {
            rightFrontY = scaleFac*((-1.0)*(pt1.x - picWidth/2.0)) + distMidToCam;
            rightFrontX = scaleFac*((-1.0)*(pt1.y - picHeight)) + distFrontAxisToInvImage;
            rightRearY = scaleFac*((-1.0)*(pt2.x - picWidth/2.0)) + distMidToCam;
            rightRearX = scaleFac*((-1.0)*(pt2.y - picHeight)) + distFrontAxisToInvImage;

            splines[i].push_back(cv::Point2d(rightFrontX, rightFrontY));
            splines[i].push_back(cv::Point2d(rightRearX, rightRearY));
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

        // OUTPUT LINES -----------------------------------------------------------
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
        // OUTPUT LINES -----------------------------------------------------------

        // OUTPUT SPLINES -----------------------------------------------------------
        //allocate memory with the size given by the descriptor
        // ADAPT: m_pCoderDescPointLeft
        //cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescSplines->GetMediaSampleSerializer(&pSerializer);
        nSize = pSerializer->GetDeserializedSize();
        pMediaSample->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        // ADAPT: m_pCoderDescPointLeft
        //cObjectPtr<IMediaCoder> pCoder;
        RETURN_IF_FAILED(m_pCoderDescSplines->WriteLock(pMediaSample, &pCoder));


        for(int i=0; i < splines.size(); i++)
        {
            stringstream elementSetter;

            for(int j=0; j < splines[i].size(); j++)
            {
                elementSetter << "BoundaryArray[" << i << "].Points[" << j << "].xCoord";
                const string& tempRef1 = elementSetter.str();
                const tChar* tempPointer1 = tempRef1.c_str();
                pCoder->Set(tempPointer1, (tVoid*)&(splines[i][j].x));
                elementSetter.str(std::string());

                elementSetter << "BoundaryArray[" << i << "].Points[" << j << "].yCoord";
                const string& tempRef2 = elementSetter.str();
                const tChar* tempPointer2 = tempRef2.c_str();
                pCoder->Set(tempPointer2, (tVoid*)&(splines[i][j].y));
                elementSetter.str(std::string());
            }

            elementSetter << "BoundaryArray[" << i << "].Count";
            const string& tempRef3 = elementSetter.str();
            const tChar* tempPointer3 = tempRef3.c_str();
            int BoundaryArrayCountTemp = splines[i].size();
            pCoder->Set(tempPointer3, (tVoid*)&(BoundaryArrayCountTemp));
            elementSetter.str(std::string());
        }
        int BoundaryCountTemp = splines.size();;
        pCoder->Set("BoundaryCount", (tVoid*)&(BoundaryCountTemp));
        m_pCoderDescSplines->Unlock(pCoder);


        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oSplines.Transmit(pMediaSample));
        // OUTPUT SPLINES -----------------------------------------------------------
    }
    */

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
    if (_oInternRepresentationVideoOutputPin.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, result.data, _sInternRepresentationBitMapOutputFormat.nSize , 0);
            _oInternRepresentationVideoOutputPin.Transmit(pNewRGBSample);
        }
    }

    RETURN_NOERROR;
}
