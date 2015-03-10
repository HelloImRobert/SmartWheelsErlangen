#include "stdafx.h"
#include "SWE_BirdEyeTransformation.h"
#include <template_data.h>

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_BirdEyeTrafo", OID_ADTF_BIRDEYETRANSFORMATION_FILTER , cSWE_BirdEyeTransformation);

// Macros used to decouple text from code
#define CORRESPING_POINTS_XML "Path to external Camera Params xml"

cSWE_BirdEyeTransformation::cSWE_BirdEyeTransformation(const tChar* __info):cFilter(__info)
{
    SetPropertyStr( CORRESPING_POINTS_XML , "/home/odroid/AADC/calibration_files/SWE_cameraCalibrationPoints.XML");
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);

    m_frameCounter = 0;
}

cSWE_BirdEyeTransformation::~cSWE_BirdEyeTransformation()
{

}

tResult cSWE_BirdEyeTransformation::Init(tInitStage eStage, __exception)
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

        // create trigger pin

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescTrigger = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescTrigger);
        cObjectPtr<IMediaType> pTypeTrigger = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescTrigger,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeTrigger->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&_pCodeTrigger));

        RETURN_IF_FAILED(_oOutputTrigger.Create("Trigger", pTypeTrigger, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oOutputTrigger));

    }
    else if (eStage == StageNormal)
    {
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
 * @brief cSWE_BirdEyeTransformation::InitPinFormats Helper initalization of the Formats of the Pins
 * @return a value indicating the succes of the stage
 */
tResult cSWE_BirdEyeTransformation::InitPinFormats()
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
 * @brief cSWE_BirdEyeTransformation::InitTransformationMatrix Helper initalization of the Transformation Matrix
 * @param pathExternalCameraParams string containing the path the the XML storing the point correspondences
 * @return a value indicating the succes of the stage
 */
tResult cSWE_BirdEyeTransformation::InitTransformationMatrices( std::string pathExternalCameraParams )
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
 * @brief cSWE_BirdEyeTransformation::Shutdown The deinitialization method of this filter
 * @param eStage the stage of deinitialization
 * @return a value indicating the succes of the stage
 */
tResult cSWE_BirdEyeTransformation::Shutdown(tInitStage eStage, __exception)
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
 * @brief cSWE_BirdEyeTransformation::OnPinEvent The method reacting on incoming samples
 * @param pSource the Pin transmitting the input
 * @param nEventCode
 * @param nParam1
 * @param nParam2
 * @param pMediaSample the actual input
 * @return a value indicating the succes of the processing
 */
tResult cSWE_BirdEyeTransformation::OnPinEvent(IPin* pSource,
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

tResult cSWE_BirdEyeTransformation::ProcessInput(IMediaSample* pMediaSample)
{
    m_frameCounter++;

    // ----------- transform the input to a opencv Mat ----------
    tUInt8* pData = NULL;
    pMediaSample->Lock((const tVoid**) &pData);
    Mat image( _oVideoInputPin.GetFormat()->nHeight , _oVideoInputPin.GetFormat()->nWidth , CV_8UC3 , pData );
    pMediaSample->Unlock(pData);

    // ---------- send trigger signal -----------
    cObjectPtr<IMediaCoder> pCoder;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    tBool state = 1;

    _pCodeTrigger->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    RETURN_IF_FAILED(_pCodeTrigger->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("bValue", (tVoid*)&(state));
    _pCodeTrigger->Unlock(pCoder);

    //transmit media sample over output pin --> the timestamp is used for both output pins
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();

    //DEBUG:
    //tmStreamTime = m_frameCounter;

    //RETURN_IF_FAILED(pMediaSampleOutput->SetTime(tmStreamTime));
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(_oOutputTrigger.Transmit(pMediaSampleOutput));


    // ---------------- apply an inverse Perspective mapping ----------------
    Mat warpedImage;
    cv::warpPerspective(image, warpedImage, _projectionMatrix, image.size());

    // ----------- transmit a video of the current result to the video outputpin ------------
    if (_oColorVideoOutputPin.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {          
            pNewRGBSample->Update(tmStreamTime, warpedImage.data, _sColorBitMapOutputFormat.nSize , 0);
            _oColorVideoOutputPin.Transmit(pNewRGBSample);
        }
    }

    RETURN_NOERROR;
}
