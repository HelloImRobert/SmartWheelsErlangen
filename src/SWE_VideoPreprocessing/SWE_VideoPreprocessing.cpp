#include "stdafx.h"
#include "SWE_VideoPreprocessing.h"

// Create filter shell
ADTF_FILTER_PLUGIN("SWE_VideoPreprocessing",OID_ADTF_SWE_VIDEOPREPROCESSING , SWE_VideoPreprocessing);

// Macros used to decouple text from code
#define CORRESPING_POINTS_XML "Path to external Camera Params xml"
#define OUTPUT_FILE "The outputfile for the dumped video"
#define CONVERT_VIDEO "Bool indicating if the video should be dumped"
#define CROP_HEIGHT 250
#define CROP_WIDTH 320

SWE_VideoPreprocessing::SWE_VideoPreprocessing(const tChar* __info):cFilter(__info)
{
    SetPropertyStr( CORRESPING_POINTS_XML , "/home/odroid/AADC/calibration_files/SWE_cameraCalibration.XML");
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyStr( OUTPUT_FILE , "/home/odroid/Desktop/convertedVid.avi" );
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyBool( CONVERT_VIDEO , false );
    SetPropertyBool( CORRESPING_POINTS_XML NSSUBPROP_ISCHANGEABLE, tTrue);

    m_frameCounter = 0;
}

SWE_VideoPreprocessing::~SWE_VideoPreprocessing()
{

}

tResult SWE_VideoPreprocessing::Init(tInitStage eStage, __exception)
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

        // register a Video Output
        RETURN_IF_FAILED(_oColorVideoOutputPinCropped.Create("Color_Video_Output_Cropped", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&_oColorVideoOutputPinCropped));

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
        // initialize the formats of the pins;
        InitPinFormats();

        _dumpVideo = GetPropertyBool( CONVERT_VIDEO );

        if( _dumpVideo )
        {
            resultVideo.open(GetPropertyStr( OUTPUT_FILE ), resultVideo.fourcc('M', 'J', 'P', 'G'), 10, cv::Size(640, 480), true);
            if(!resultVideo.isOpened())
            {
                _dumpVideo = false;
            }
        }
    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

/**
 * @brief SWE_VideoPreprocessing::InitPinFormats Helper initalization of the Formats of the Pins
 * @return a value indicating the succes of the stage
 */
tResult SWE_VideoPreprocessing::InitPinFormats()
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


    // setup the constant parameters of the Color outputformat
    _sColorBitMapOutputFormatCropped.nBitsPerPixel = 24;
    _sColorBitMapOutputFormatCropped.nPixelFormat = cImage::PF_RGB_888;
    _sColorBitMapOutputFormatCropped.nPaletteSize = 0;
    _sColorBitMapOutputFormatCropped.nWidth = CROP_WIDTH;
    _sColorBitMapOutputFormatCropped.nHeight = CROP_HEIGHT;
    _sColorBitMapOutputFormatCropped.nBytesPerLine = _sColorBitMapOutputFormatCropped.nWidth * 3;
    _sColorBitMapOutputFormatCropped.nSize = _sColorBitMapOutputFormatCropped.nBytesPerLine * _sColorBitMapOutputFormatCropped.nHeight;

    // set the format to the outputpin
    _oColorVideoOutputPinCropped.SetFormat( &_sColorBitMapOutputFormatCropped , NULL );

    RETURN_NOERROR;
}


/**
 * @brief SWE_VideoPreprocessing::Shutdown The deinitialization method of this filter
 * @param eStage the stage of deinitialization
 * @return a value indicating the succes of the stage
 */
tResult SWE_VideoPreprocessing::Shutdown(tInitStage eStage, __exception)
{   
    if( _dumpVideo )
    {
        resultVideo.release();
    }

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
tResult SWE_VideoPreprocessing::OnPinEvent(IPin* pSource,
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

tResult SWE_VideoPreprocessing::ProcessInput(IMediaSample* pMediaSample)
{
    m_frameCounter++;

    // ----------- transform the input to a opencv Mat ----------
    tUInt8* pData = NULL;
    pMediaSample->Lock((const tVoid**) &pData);
    Mat image( _oVideoInputPin.GetFormat()->nHeight , _oVideoInputPin.GetFormat()->nWidth , CV_8UC3 , pData );
    pMediaSample->Unlock(pData);

    if( _dumpVideo )
    {
        resultVideo << image;
    }

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
    //tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    tTimeStamp tmStreamTime = m_frameCounter;

    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(tmStreamTime));
    //RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(_oOutputTrigger.Transmit(pMediaSampleOutput));




    // ----------- output unchanged video  ------------
    if (_oColorVideoOutputPin.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {          
            pNewRGBSample->Update(tmStreamTime, image.data, _sColorBitMapOutputFormat.nSize , 0);
            _oColorVideoOutputPin.Transmit(pNewRGBSample);
        }
    }





    // ---------------- output cropped video ----------------

    // crop the image
    cv::Rect myROI( 0 , 150, CROP_WIDTH , CROP_HEIGHT );
    cv::Mat croppedImage;
    image(myROI).copyTo(croppedImage);

    // output
    if (_oColorVideoOutputPinCropped.IsConnected())
    {
        cObjectPtr<IMediaSample> pNewRGBSample2;
        if (IS_OK(AllocMediaSample(&pNewRGBSample2)))
        {
            pNewRGBSample2->Update(tmStreamTime, croppedImage.data, _sColorBitMapOutputFormatCropped.nSize , 0);
            _oColorVideoOutputPinCropped.Transmit(pNewRGBSample2);
        }
    }



    RETURN_NOERROR;
}
