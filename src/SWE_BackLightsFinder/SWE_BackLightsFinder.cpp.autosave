#include "stdafx.h"
#include "SWE_BackLightsFinder.h"

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_BackLightsFinder",OID_ADTF_SWE_BACKLIGHTSFINDER , SWE_BackLightsFinder);

// Macros used to decouple text from code


SWE_BackLightsFinder::SWE_BackLightsFinder(const tChar* __info):cFilter(__info)
{
    _lowHueTresh = 140;
    _highHueTresh = 199;

    _lowSaturationTresh = 50;
    _highSaturationThresh = 255;

    _lowValueTresh = 40;
    _highValueTresh = 255;

    _contourLengthThreshold = 50;
}

SWE_BackLightsFinder::~SWE_BackLightsFinder()
{

}

tResult SWE_BackLightsFinder::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        // register a Video Input
        RETURN_IF_FAILED(_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&_oVideoInputPin));

        // register a Video Output
        RETURN_IF_FAILED(_oColorVideoOutputPin.Create("Color_Video_Output", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oColorVideoOutputPin));

        // register the distances inputPin
        RETURN_IF_FAILED(m_oInputDistances.Create("ObjectData", new cMediaType(0, 0, 0, "tPointArray"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputDistances));

        // create the tracking point outputPin
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        // Struct for Intersection Point Transmission
        // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
        tChar const * strDescIntersecPoints = pDescManager->GetMediaDescription("tIntersectionsNew");
        RETURN_IF_POINTER_NULL(strDescIntersecPoints);
        cObjectPtr<IMediaType> pTypeIntersecPoints = new cMediaType(0, 0, 0, "tIntersectionsNew", strDescIntersecPoints,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeIntersecPoints->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPoints));

        RETURN_IF_FAILED(m_oTrackingPoint.Create("tracking_point", pTypeIntersecPoints, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oTrackingPoint));
    }
    else if (eStage == StageNormal)
    {
        // initialize the formats of the pins;
        InitPinFormats();
    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

tResult SWE_BackLightsFinder::InitPinFormats()
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
 * @brief SWE_VideoPreprocessing::Shutdown The deinitialization method of this filter
 * @param eStage the stage of deinitialization
 * @return a value indicating the succes of the stage
 */
tResult SWE_BackLightsFinder::Shutdown(tInitStage eStage, __exception)
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
 * @brief SWE_VideoPreprocessing::OnPinEvent The method reacting on incoming samples
 * @param pSource the Pin transmitting the input
 * @param nEventCode
 * @param nParam1
 * @param nParam2
 * @param pMediaSample the actual input
 * @return a value indicating the succes of the processing
 */
tResult SWE_BackLightsFinder::OnPinEvent(IPin* pSource,
                                       tInt nEventCode,
                                       tInt nParam1,
                                       tInt nParam2,
                                       IMediaSample* pMediaSample)
{
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

        m_mutex.Enter();

        if(pSource == &_oVideoInputPin)
        {
            ProcessInput(pMediaSample);
        }
        else if(pSource == &m_oInputDistances)
        {
            cObjectPtr<IMediaType> pType;
            pSource->GetMediaType(&pType);
            if (pType != NULL)
            {
                cObjectPtr<IMediaTypeDescription> pMediaTypeDescInputMeasured;
                pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescInputMeasured);
                m_pCoderDescInputMeasured = pMediaTypeDescInputMeasured;
            }

            cObjectPtr<IMediaCoder> pCoder;
            m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder);

            stringstream elementGetter;
            for( size_t j = 0; j < 10; j++)
            {
                elementGetter << "tPoint[" << j << "].xCoord";
                pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(distances[j].x));
                elementGetter.str(std::string());

                elementGetter << "tPoint[" << j << "].yCoord";
                pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(distances[j].y));
                elementGetter.str(std::string());
            }
            m_pCoderDescInputMeasured->Unlock(pCoder);
        }

        m_mutex.Leave();
    }
    RETURN_NOERROR;
}

cv::Point SWE_BackLightsFinder::getOrientation(const std::vector< cv::Point >& contour, std::vector<cv::Point2d>& eigen_vecs , std::vector<double>& eigen_vals )
    {
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
        cv::Point centerOfGravity = cv::Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));

        // store the results
        eigen_vals.push_back(pca_analysis.eigenvalues.at<double>(0, 0));
        eigen_vecs.push_back(cv::Point2d(pca_analysis.eigenvectors.at<double>(0, 0), pca_analysis.eigenvectors.at<double>(0, 1)));

        // store the second eigenvector and eigenvalue
        // they might not be calculated in case of line boundaries
        if (pca_analysis.eigenvectors.cols > 1 && pca_analysis.eigenvectors.rows > 1 && pca_analysis.eigenvalues.rows > 1)
        {
            eigen_vals.push_back(pca_analysis.eigenvalues.at<double>(1, 0));
            eigen_vecs.push_back(cv::Point2d(pca_analysis.eigenvectors.at<double>(1, 0), pca_analysis.eigenvectors.at<double>(1, 1)));
        }
        // store dummy values
        else
        {
            eigen_vals.push_back(0.01);
            eigen_vecs.push_back(Point2d(0, 0));
        }

        return centerOfGravity;
    }


tResult SWE_BackLightsFinder::transmitTrackingPoint(cv::Point2d trackingPoint)
{
    // generate Coder object
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescPoints->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    tInt8 intersectionIndicator = 1;

    RETURN_IF_FAILED(m_pCoderDescPoints->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("intersecPoint.xCoord", (tVoid*)&(trackingPoint.x));
    pCoder->Set("intersecPoint.yCoord", (tVoid*)&(trackingPoint.y));
    pCoder->Set("Indicator", (tVoid*)&(intersectionIndicator));
    m_pCoderDescPoints->Unlock(pCoder);

    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oTrackingPoint.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}

tResult SWE_BackLightsFinder::transmitResultVideo(cv::Size size , std::vector< vector< cv::Point > >& contours )
{
    std::vector<Vec4i> hierarchy;
    RNG rng(12345);

    /// Draw contours
    Mat drawing = Mat::zeros( size , CV_8UC3 );
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
    }

    if (_oColorVideoOutputPin.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, drawing.data, _sColorBitMapOutputFormat.nSize , 0);
            _oColorVideoOutputPin.Transmit(pNewRGBSample);
        }
    }

    RETURN_NOERROR;
}

tResult SWE_BackLightsFinder::ProcessInput(IMediaSample* pMediaSample)
{
    tUInt8* pData = NULL;
    pMediaSample->Lock((const tVoid**) &pData);
    cv::Mat image( _oVideoInputPin.GetFormat()->nHeight , _oVideoInputPin.GetFormat()->nWidth , CV_8UC3 , pData );
    pMediaSample->Unlock(pData);

    Mat imageHSV;
    cvtColor(image, imageHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat thresholdedImage;

    inRange(imageHSV, Scalar(_lowHueTresh, _lowSaturationTresh, _lowValueTresh), Scalar(_highHueTresh, _highSaturationThresh, _highValueTresh), thresholdedImage); //Threshold the image

    // closing
    dilate(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
    erode(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));

    // extract contours from the image
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours(thresholdedImage.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    for (vector<vector<Point> >::iterator iter = contours.begin(); iter != contours.end();)
    {
        if (iter->size() < _contourLengthThreshold)
        {
            iter = contours.erase(iter);
        }
        else
        {
            ++iter;
        }
    }

    for (size_t i = 0 ; i < contours.size() ; ++i)
    {
        std::vector<cv::Point2d> eigen_vecs;
        std::vector<double> eigen_vals;
        cv::Point centerOfGravity = getOrientation(contours[0], eigen_vecs, eigen_vals);
    }

    transmitTrackingPoint(cv::Point2d(1400,0));
    transmitResultVideo( thresholdedImage.size() , contours );

    RETURN_NOERROR;
}
