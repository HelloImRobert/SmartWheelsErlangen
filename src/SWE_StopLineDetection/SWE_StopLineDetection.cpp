#include "stdafx.h"
#include "SWE_StopLineDetection.h"
#include <template_data.h>

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_StopLineDetection", OID_ADTF_STOPLINEDETECTION_FILTER , cSWE_StopLineDetection);

// Macros used to decouple text from code
#define UPPER_SEARCH_BORDER "Cut Upper Border for StopLine Detection"
#define CORRESPING_POINTS_XML "Path to external Camera Params xml"
#define COUNT_OF_STDDEVS "Stddevs for the Thresholding"
#define THRESHOLD "Threshold for Detection"
#define FITWIDTH "Width for the are where the line is fitted"

/**
 * @brief cSWE_StopLineDetection::cSWE_StopLineDetection
 * Constructs a cSWE_StopLineDetection object, defining it's properties, to be set by ADTF;
 * @param __info The info object for the Filter
 */
cSWE_StopLineDetection::cSWE_StopLineDetection(const tChar* __info):cFilter(__info)
{
    m_isActive = tTrue;

    SetPropertyInt( UPPER_SEARCH_BORDER , 0);
    SetPropertyBool( UPPER_SEARCH_BORDER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( UPPER_SEARCH_BORDER NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( UPPER_SEARCH_BORDER NSSUBPROP_MAXIMUM  , 480);

    SetPropertyInt( FITWIDTH , 1);
    SetPropertyBool( FITWIDTH NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt( FITWIDTH NSSUBPROP_MINIMUM  , 0);
    SetPropertyInt( FITWIDTH NSSUBPROP_MAXIMUM  , 50);

    SetPropertyFloat( THRESHOLD , 60000.0);
    SetPropertyBool( THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( THRESHOLD NSSUBPROP_MINIMUM  , 1.0);

    SetPropertyFloat( COUNT_OF_STDDEVS , 2.0);
    SetPropertyBool( COUNT_OF_STDDEVS NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyFloat( COUNT_OF_STDDEVS NSSUBPROP_MINIMUM  , 1.0);
    SetPropertyInt( COUNT_OF_STDDEVS NSSUBPROP_MAXIMUM  , 5.0);

    SetPropertyStr( CORRESPING_POINTS_XML , "/home/odroid/AADC/calibration_files/points.xml");
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);
}

/**
 * @brief cSWE_StopLineDetection::~cSWE_StopLineDetection
 * Destructs a cSWE_StopLineDetection object
 */
cSWE_StopLineDetection::~cSWE_StopLineDetection()
{

}

/**
 * @brief cSWE_StopLineDetection::Init
 * @param eStage the current stage of initalization
 * @return a value indicating the succes of the stage
 */
tResult cSWE_StopLineDetection::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        // register a Video Input
        RETURN_IF_FAILED(m_oActivationPin.Create("ActivationFlag", new cMediaType(0, 0, 0, "tBoolSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oActivationPin));

        // register a Video Input
        RETURN_IF_FAILED(_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&_oVideoInputPin));

        // register a Video Output
        RETURN_IF_FAILED(_oColorVideoOutputPin.Create("Color_Video_Output", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oColorVideoOutputPin));

        // Output Pin for Lines
        // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescLines = pDescManager->GetMediaDescription("tStopLine");
        RETURN_IF_POINTER_NULL(strDescLines);
        cObjectPtr<IMediaType> pTypeLines = new cMediaType(0, 0, 0, "tStopLine", strDescLines,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeLines->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLines));

        RETURN_IF_FAILED(m_oLines.Create("Stop_Line", pTypeLines, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oLines));
    }
    else if (eStage == StageNormal)
    {
        // fetch the parameters of the image cutting
        m_startHeight = GetPropertyInt( UPPER_SEARCH_BORDER );

        // fetch the algorithmic parameters
        _CountStdDevs = GetPropertyInt(COUNT_OF_STDDEVS);

        m_fitWidth = GetPropertyInt(FITWIDTH);

        m_threshold = GetPropertyFloat(THRESHOLD);

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
 * @brief cSWE_StopLineDetection::InitPinFormats Helper initalization of the Formats of the Pins
 * @return a value indicating the succes of the stage
 */
tResult cSWE_StopLineDetection::InitPinFormats()
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

    // set the format to the outputpin
    _oColorVideoOutputPin.SetFormat( &_sColorBitMapOutputFormat , NULL );

    RETURN_NOERROR;
}

/**
 * @brief cSWE_StopLineDetection::InitTransformationMatrix Helper initalization of the Transformation Matrix
 * @param pathExternalCameraParams string containing the path the the XML storing the point correspondences
 * @return a value indicating the succes of the stage
 */
tResult cSWE_StopLineDetection::InitTransformationMatrices( std::string pathExternalCameraParams )
{
    _inversePerspectiveFileStorage.open( pathExternalCameraParams , cv::FileStorage::READ );

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
 * @brief cSWE_StopLineDetection::Shutdown The deinitialization method of this filter
 * @param eStage the stage of deinitialization
 * @return a value indicating the succes of the stage
 */
tResult cSWE_StopLineDetection::Shutdown(tInitStage eStage, __exception)
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
 * @brief cSWE_StopLineDetection::OnPinEvent The method reacting on incoming samples
 * @param pSource the Pin transmitting the input
 * @param nEventCode
 * @param nParam1
 * @param nParam2
 * @param pMediaSample the actual input
 * @return a value indicating the succes of the processing
 */
tResult cSWE_StopLineDetection::OnPinEvent(IPin* pSource,
                                       tInt nEventCode,
                                       tInt nParam1,
                                       tInt nParam2,
                                       IMediaSample* pMediaSample)
{
    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
    // ADAPT: pMediaTypeDescInputMeasured, m_pCoderDescInputMeasured !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // NOCH SO BAUEN, DASS IN FKT CREATE_INPUT_PINS EINGEFUEGT WERDEN KANN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);

    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if(pSource == &_oVideoInputPin)
        {
            if( m_isActive )
            {
                ProcessInput(pMediaSample);
            }
        }
        else if(pSource == &m_oActivationPin)
        {
            if( m_isActive )
            {
                m_isActive = false;
            }
            else
            {
                m_isActive = true;
            }
        }
    }
    RETURN_NOERROR;
}

std::pair< size_t , float > cSWE_StopLineDetection::SearchMaximum(cv::Mat scalarMat)
{
    float maxVal = 0;
    size_t index;

    // loop readonly over the matrix
    for (cv::MatConstIterator_< float > iter = scalarMat.begin< float >(); iter != scalarMat.end< float >(); iter++)
    {
        float val = *iter;
        if ( val > maxVal )
        {
            maxVal = val;
            index = iter.pos().y;
        }
    }
    return std::pair< size_t , float >(index, maxVal);
}

float cSWE_StopLineDetection::computeEnergy(const cv::Mat& src, const cv::Point2i& start, const cv::Point2i& end)
{
    double sum = 0;

    cv::LineIterator iter(src, start, end);

    cv::Point cur_point;
    for ( int i = 0; i < iter.count; i++, iter++)
    {
        cur_point = iter.pos();
        sum += src.at< uchar >(cur_point.y, cur_point.x);
    }
    return sum;
}

/**
 * @brief cSWE_StopLineDetection::ProcessInput The actual image processing of the StopLineDetection
 * ... Description of the algorithm
 * @param pMediaSample the input Image
 * @return a value indicating the succes of the processing
 */
tResult cSWE_StopLineDetection::ProcessInput(IMediaSample* pMediaSample)
{
    // transform the input to a opencv Mat
    tUInt8* pData = NULL;
    pMediaSample->Lock((const tVoid**) &pData);
    cv::Mat image( _oVideoInputPin.GetFormat()->nHeight , _oVideoInputPin.GetFormat()->nWidth , CV_8UC3 , pData );
    pMediaSample->Unlock(pData);

    // crop the image
    cv::Rect myROI( 100 , m_startHeight , image.cols - 200 , image.rows - m_startHeight );
    cv::Mat croppedImage( image , myROI );

    // transform the image to a grayscale image
    cv::Mat greyScaleImage;
    cv::cvtColor(croppedImage, greyScaleImage, CV_RGB2GRAY, 1);

    // aquire mean and stdDev of the image
    cv::Scalar mean, stdDev;
    cv::meanStdDev(greyScaleImage, mean, stdDev);

    double thresh = mean[0] + _CountStdDevs * stdDev[0];

    cv::Mat thresholdedImage;
    //threshold the image to remove objects which don't necessary represent lane markings
    cv::threshold(greyScaleImage, thresholdedImage, thresh, 255, cv::THRESH_TOZERO);

    // sum the image up over the x - axis, the x-axis defined as the axis where the car is heading currently
    cv::Mat columnSums;
    cv::reduce( thresholdedImage, columnSums, 1, CV_REDUCE_SUM, CV_32F);

    // find candidate for the stopline
    std::pair< size_t , float > maximum = SearchMaximum(columnSums);
    if (maximum.second > m_threshold && maximum.first > m_fitWidth && maximum.first < ( thresholdedImage.rows - m_fitWidth ) )
    {
        cv::Rect fitRoi(0, maximum.first - m_fitWidth, croppedImage.cols, 2 * m_fitWidth);
        cv::Mat roiedImage(thresholdedImage, fitRoi);
        cv::Mat fitArea= roiedImage.clone();

        cv::Mat skel(fitArea.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat temp(fitArea.size(), CV_8UC1);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        bool done;
        do
        {
            cv::morphologyEx(fitArea, temp, cv::MORPH_OPEN, element);
            cv::bitwise_not(temp, temp);
            cv::bitwise_and(fitArea, temp, temp);
            cv::bitwise_or(skel, temp, skel);
            cv::erode(fitArea, fitArea, element);

            double max;
            cv::minMaxLoc(fitArea, 0, &max);
            done = (max == 0);
        } while (!done);

        float maxEnergy = 0;
        cv::Point2i maxStart;
        cv::Point2i maxEnd;
        cv::Point2i start (0,0);
        cv::Point2i end (skel.cols, skel.rows);
        for (size_t i = 0; i < 2 * m_fitWidth; i++)
        {
            float energy = computeEnergy(skel, start, end);

            if (energy > maxEnergy)
            {
                maxEnergy = energy;
                maxStart = start;
                maxEnd = end;
            }
            start.y++;
            end.y--;
        }

        while ( maxStart.y < fitArea.rows && maxEnd.y < fitArea.rows )
        {
            std::vector< float > energies;

            maxStart.y++;
            energies.push_back( computeEnergy(roiedImage, maxStart , maxEnd) );
            maxEnd.y++;
            energies.push_back(computeEnergy(roiedImage, maxStart, maxEnd));
            maxStart.y--;
            energies.push_back(computeEnergy(roiedImage, maxStart, maxEnd));
            maxEnd.y--;

            float iter = 0;
            size_t pos = 0;

            for( size_t j = 0 ; j < energies.size() ; j++ )
            {
                float energy = energies[ j ];
                if( energy > iter )
                {
                    pos = j;
                    iter = energy;
                }
            }

            if (iter >= maxEnergy)
            {
                maxEnergy = iter;

                switch (pos)
                {
                case 0:
                    maxStart.y++;
                    break;
                case 1:
                    maxStart.y++;
                    maxEnd.y++;
                    break;
                case 2:
                    maxEnd.y++;
                    break;
                }
            }
            else
            {
                break;
            }
        }

        maxStart.x += 100;
        maxEnd.x += 100;
        maxStart.y += m_startHeight + maximum.first - m_fitWidth;
        maxEnd.y += m_startHeight + maximum.first - m_fitWidth;

        cv::line(image, maxStart, maxEnd, cv::Scalar( 0 , 255, 0), 2);

        // map inverse
        vector< Point2d > vec;
        vec.push_back( maxStart );
        vec.push_back( maxEnd );

        perspectiveTransform( vec , vec , _projectionMatrix );

        maxStart = vec[ 0 ];
        maxEnd = vec[ 1 ];

        // Transmit the found line
        {
            cObjectPtr<IMediaCoder> pCoder;

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSample;
            RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

            // OUTPUT LINE -----------------------------------------------------------
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


            pCoder->Set("startX", (tVoid*)&(maxStart.x));
            pCoder->Set("startY", (tVoid*)&(maxStart.y));
            pCoder->Set("endX", (tVoid*)&(maxEnd.x));
            pCoder->Set("endY", (tVoid*)&(maxEnd.y));
            m_pCoderDescLines->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oLines.Transmit(pMediaSample));
            // OUTPUT LINES -----------------------------------------------------------
        }
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


    RETURN_NOERROR;
}
