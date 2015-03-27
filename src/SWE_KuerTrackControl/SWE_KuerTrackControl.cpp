#include <cmath>
#include "stdafx.h"
#include "SWE_KuerTrackControl.h"



ADTF_FILTER_PLUGIN("SWE KuerTrackControl", OID_ADTF_SWE_KUERTRACKCONTROL, cSWE_KuerTrackControl)

cSWE_KuerTrackControl::cSWE_KuerTrackControl(const tChar* __info) : cFilter(__info), m_PerpenticularPoint(1.0, 0.0)
{
    m_old_steeringAngle = 0.0;
    m_input_intersectionIndicator = 0;

    SetPropertyBool("InvertSteering", true); //Invert all steering angles? Normal is positive left. This somehow differs between our cars.... don't ask...
    SetPropertyFloat("Steering Dead angle in degree", 3);
    SetPropertyFloat("FilterStrength", 0.5);

}

cSWE_KuerTrackControl::~cSWE_KuerTrackControl()
{
}

tResult cSWE_KuerTrackControl::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oIntersectionPoints.Create("tracking_Point", new cMediaType(0, 0, 0, "tIntersectionsNew"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));


    RETURN_NOERROR;
}

tResult cSWE_KuerTrackControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Struct for steering angle Point Transmission
    tChar const * strDescSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSteeringAngle);
    cObjectPtr<IMediaType> pTypeSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strDescSteeringAngle,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSteeringAngle));

    RETURN_IF_FAILED(m_oSteeringAngle.Create("Steering_Angle", pTypeSteeringAngle, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngle));



    RETURN_NOERROR;
}

tResult cSWE_KuerTrackControl::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {

        //m_referencePoint.x = GetPropertyFloat("Reference Point x-Coord");



    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_KuerTrackControl::Start(__exception)
{


    m_property_InvSteering =            (tBool)GetPropertyBool("InvertSteering", true);
    m_property_SteeringDeadAngle =      (tFloat32)GetPropertyFloat("Steering Dead angle in degree", 3);
    m_old_steeringAngle = 0.0;
    m_filter =                          (tFloat32)GetPropertyFloat("FilterStrength", 0.5);


    SendSteering(0);

    return cFilter::Start(__exception_ptr);
}

tResult cSWE_KuerTrackControl::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_KuerTrackControl::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_KuerTrackControl::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
    cObjectPtr<IMediaType> pType;
    pSource->GetMediaType(&pType);
    if (pType != NULL)
    {
        cObjectPtr<IMediaTypeDescription> pMediaTypeDescInputMeasured;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDescInputMeasured));
        m_pCoderDescInputMeasured = pMediaTypeDescInputMeasured;
    }

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescInputMeasured != NULL)
    {
        RETURN_IF_POINTER_NULL( pMediaSample);

        if (pSource == &m_oIntersectionPoints)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("intersecPoint.xCoord", (tVoid*)&(m_input_trackingPoint.x));
            pCoder->Get("intersecPoint.yCoord", (tVoid*)&(m_input_trackingPoint.y));
            pCoder->Get("Indicator", (tVoid*)&(m_input_intersectionIndicator));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------
            tInt8 intersectionIndicator = 1;
            m_outputSteeringAngle = 180/CV_PI*CalcSteeringAngleTrajectory( m_input_trackingPoint, intersectionIndicator );

            //compensate for dead angle in steering (under the assumption that the wheels always try to turn the least possible amount)
            if(m_outputSteeringAngle < 0)
                m_outputSteeringAngle -= m_property_SteeringDeadAngle;
            else if (m_outputSteeringAngle > 0)
                m_outputSteeringAngle += m_property_SteeringDeadAngle;

            if(m_property_InvSteering)
                m_outputSteeringAngle *= -1;

            // ------------------------------------------------------------------------------------------


            //send steering angle

            SendSteering(m_outputSteeringAngle);

            if(m_property_InvSteering)
                m_old_steeringAngle = -m_outputSteeringAngle;
            else
                m_old_steeringAngle = m_outputSteeringAngle;


        }
        // -------------------------------------------------------------------

    }

    RETURN_NOERROR;
}


//---------------------------------------------------------------------------------------------------
// --------------------------------  Calculate Steering Angle  --------------------------------------
// --------------------------------------------------------------------------------------------------

tFloat64 cSWE_KuerTrackControl::CalcSteeringAngleTrajectory( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator )
{
    tFloat64 steeringAngle = -1;

    if( intersectionIndicator != 0 )
    {
        steeringAngle = acos(trackingPoint.dot(m_PerpenticularPoint)/cv::norm(trackingPoint));
        if(trackingPoint.y < 0)
        {
            steeringAngle = (-1.0)*steeringAngle;
        }
    }
    else
    {
        steeringAngle = m_old_steeringAngle;
    }

        m_old_steeringAngle = steeringAngle;
        steeringAngle = m_filter * steeringAngle + (1.0 - m_filer) * m_old_steeringAngle;

    return steeringAngle;
}


//----------------------------------------------------------------------------------------
// ------------------------------------  OUTPUT  -----------------------------------------
// ---------------------------------------------------------------------------------------


tResult cSWE_KuerTrackControl::SendSteering(tFloat32 outputAngle)
{


    static tFloat32 lastOutput = 777;
    static tTimeStamp lastOutputTime = 0;

    // only send changes or every 500ms
    if((lastOutput != outputAngle) || (_clock->GetTime () - lastOutputTime  > 500000))
    {
        lastOutput = outputAngle;
        lastOutputTime = _clock->GetTime ();
        // generate Coder object
        cObjectPtr<IMediaCoder> pCoder;

        //create new media sample
        cObjectPtr<IMediaSample> pMediaSampleOutput;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

        //allocate memory with the size given by the descriptor
        // ADAPT: m_pCoderDescPointLeft
        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescSteeringAngle->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSampleOutput->AllocBuffer(nSize);

        //write date to the media sample with the coder of the descriptor
        // ADAPT: m_pCoderDescPointLeft
        //cObjectPtr<IMediaCoder> pCoder;
        RETURN_IF_FAILED(m_pCoderDescSteeringAngle->WriteLock(pMediaSampleOutput, &pCoder));
        pCoder->Set("f32Value", (tVoid*)&(outputAngle));
        m_pCoderDescSteeringAngle->Unlock(pCoder);

        //transmit media sample over output pin
        // ADAPT: m_oIntersectionPointLeft
        RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oSteeringAngle.Transmit(pMediaSampleOutput));
    }



    RETURN_NOERROR;
}
