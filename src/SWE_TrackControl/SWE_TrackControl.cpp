#include <cmath>
#include "stdafx.h"
#include "SWE_TrackControl.h"


#include <iostream>
#include <fstream>

#define WHEELBASE 359 //distance between both axles in mm



ADTF_FILTER_PLUGIN("SWE TrackControl", OID_ADTF_SWE_TRACKCONTROL, cSWE_TrackControl)

cSWE_TrackControl::cSWE_TrackControl(const tChar* __info) : cFilter(__info), m_PerpenticularPoint(1.0, 0.0)
{
    m_old_steeringAngle = 0.0;
    m_property_useNewCalc = false;

    SetPropertyFloat("Wheelbase", 360);
    SetPropertyBool("Use new angle calculation", false); //DEBUG

}

cSWE_TrackControl::~cSWE_TrackControl()
{
}

tResult cSWE_TrackControl::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oIntersectionPoints.Create("tracking_Point", new cMediaType(0, 0, 0, "tIntersectionsNew"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));

    RETURN_IF_FAILED(m_oCommands.Create("KI_Commands", new cMediaType(0, 0, 0, "tKITC"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oCommands));

    RETURN_IF_FAILED(m_oOdometry.Create("Odometry_Data", new cMediaType(0, 0, 0, "tOdometry"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOdometry));

    RETURN_IF_FAILED(m_oCrossingIndicator.Create("Crossing_Indicator", new cMediaType(0, 0, 0, "tCrossingIndicator"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oCrossingIndicator));

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Struct for Intersection Point Transmission
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSteeringAngle);
    cObjectPtr<IMediaType> pTypeSteeringAngle = new cMediaType(0, 0, 0, "tSignalValue", strDescSteeringAngle,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSteeringAngle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSteeringAngle));

    RETURN_IF_FAILED(m_oSteeringAngle.Create("Steering_Angle", pTypeSteeringAngle, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngle));

    // Struct for Intersection Point Transmission
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescMiddlePoint = pDescManager->GetMediaDescription("tPoint2d");
    RETURN_IF_POINTER_NULL(strDescMiddlePoint);
    cObjectPtr<IMediaType> pTypeMiddlePoint = new cMediaType(0, 0, 0, "tPoint2d", strDescMiddlePoint,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMiddlePoint->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMiddlePoint));

    RETURN_IF_FAILED(m_oMiddlePoint.Create("Middle_Point", pTypeMiddlePoint, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oMiddlePoint));

    // Struct for Gear Output to Speed Control
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescGear = pDescManager->GetMediaDescription("tGear");
    RETURN_IF_POINTER_NULL(strDescGear);
    cObjectPtr<IMediaType> pTypeGear = new cMediaType(0, 0, 0, "tGear", strDescGear,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeGear->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescGear));

    RETURN_IF_FAILED(m_oGear.Create("Gear", pTypeGear, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oGear));

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::Init(tInitStage eStage, __exception)
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
        m_wheelbase = GetPropertyFloat("Wheelbase");


    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::Start(__exception)
{

    m_property_useNewCalc = (tBool)SetPropertyBool("Use new angle calculation", false); //DEBUG

    m_input_maxGear = 0;
    m_input_Command = 1;
    m_input_intersectionIndicator = 0;
    m_angleAbs = 0;
    m_old_steeringAngle = 0;

    return cFilter::Start(__exception_ptr);
}

tResult cSWE_TrackControl::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_TrackControl::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_TrackControl::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    // Necessary to get Datatype from INPUT pins (datatypes of output pins are defined in INIT)
    // ADAPT: pMediaTypeDescInputMeasured, m_pCoderDescInputMeasured !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // NOCH SO BAUEN, DASS IN FKT CREATE_INPUT_PINS EINGEFUEGT WERDEN KANN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

            ReactToInput();

        }
        else if(pSource == &m_oOdometry)
        {

            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));
            pCoder->Get("distance_x", (tVoid*)&(m_odometryData.distance_x));
            pCoder->Get("distance_y", (tVoid*)&(m_odometryData.distance_y));
            pCoder->Get("angle_heading", (tVoid*)&(m_odometryData.angle_heading));
            pCoder->Get("velocity", (tVoid*)&(m_odometryData.velocity));
            pCoder->Get("distance_sum", (tVoid*)&(m_odometryData.distance_sum));
            m_pCoderDescInputMeasured->Unlock(pCoder);

            m_angleAbs = m_angleAbs + m_odometryData.angle_heading;


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------

            ReactToInput();

        }
        else if(pSource == &m_oCommands)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("tInt8ValueSpeed", (tVoid*)&(m_input_maxGear));
            pCoder->Get("tInt8ValueCommand", (tVoid*)&(m_input_Command));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------

            ReactToInput();
        }
        else if(pSource == &m_oCrossingIndicator)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            tBool isRealStopLine;
            tInt crossingType;
            cv::Point2d StopLinePoint1;
            cv::Point2d StopLinePoint2;

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("isRealStopLine", (tVoid*)&(isRealStopLine));
            pCoder->Get("crossingType", (tVoid*)&(crossingType));
            pCoder->Get("StopLinePoint1.xCoord", (tVoid*)&(StopLinePoint1.x));
            pCoder->Get("StopLinePoint1.yCoord", (tVoid*)&(StopLinePoint1.y));
            pCoder->Get("StopLinePoint2.xCoord", (tVoid*)&(StopLinePoint2.x));
            pCoder->Get("StopLinePoint2.yCoord", (tVoid*)&(StopLinePoint2.y));
            m_pCoderDescInputMeasured->Unlock(pCoder);


            // DO WHAT HAS TO BE DONE -------------------------------------------------------------------

            ReactToInput();
        }
        else
            RETURN_NOERROR;
        // -------------------------------------------------------------------

    }
    RETURN_NOERROR;
}


tFloat64 cSWE_TrackControl::CalcSteeringAngleTrajectory( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator )
{
    tFloat64 steeringAngle = 0;

    if( intersectionIndicator != 0 )
    {
        steeringAngle = acos(trackingPoint.dot(m_PerpenticularPoint)/cv::norm(trackingPoint));
        if(trackingPoint.y < 0)
        {
            steeringAngle = (-1.0)*steeringAngle;
        }
        m_old_steeringAngle = steeringAngle;
    }
    else
    {
        steeringAngle = m_old_steeringAngle;
    }

    return steeringAngle;
}

tFloat64 cSWE_TrackControl::CalcSteeringAngleCircle( const cv::Point2d& trackingPoint, const tInt8 intersectionIndicator )
{
    tFloat64 steeringAngle = 0;

    // transform tracking point in rear axis coo sys
    cv::Point2d trackingPoint_ra;
    trackingPoint_ra.x = trackingPoint.x + m_wheelbase;
    trackingPoint_ra.y = trackingPoint.y;

    // calculate steering angle
    if( intersectionIndicator != 0 )
    {
        double alpha = std::atan2( trackingPoint_ra.y, trackingPoint_ra.x );
        double distance = cv::norm( trackingPoint_ra );
        double radius = distance / ( 2* std::sin( alpha + 1e-3 ) );

        steeringAngle = std::atan( m_wheelbase / radius );

        m_old_steeringAngle = steeringAngle;
    }
    else
    {
        steeringAngle = m_old_steeringAngle;
    }

    return steeringAngle;
}

tResult cSWE_TrackControl::ReactToInput()
{

    tFloat32 steeringAngle = 0;
    tInt8 outputGear;

    outputGear = m_input_maxGear;


    if(m_property_useNewCalc) //two alternative modes of calculation
    {
        steeringAngle = -180.0/CV_PI * ( CalcSteeringAngleCircle( m_input_trackingPoint, m_input_intersectionIndicator ) );
    }
    else
    {
        steeringAngle = -180.0/CV_PI*( CalcSteeringAngleTrajectory( m_input_trackingPoint, m_input_intersectionIndicator ) );
    }


    SendSteering(steeringAngle);
    SendGear(outputGear); //DEBUG

    SendTrackingPoint();

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::SendSteering(tFloat32 outputAngle)
{

    // generate Coder object
    cObjectPtr<IMediaCoder> pCoder;
    //RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

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


    /*

      */

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::SendTrackingPoint()
{

    // generate Coder object
    cObjectPtr<IMediaCoder> pCoder;
    //RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOutput;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescMiddlePoint->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOutput->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    //cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescMiddlePoint->WriteLock(pMediaSampleOutput, &pCoder));
    pCoder->Set("xCoord", (tVoid*)&(m_input_trackingPoint.x));
    pCoder->Set("yCoord", (tVoid*)&(m_input_trackingPoint.y));
    m_pCoderDescMiddlePoint->Unlock(pCoder);

    //transmit media sample over output pin
    // ADAPT: m_oIntersectionPointLeft
    RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
    RETURN_IF_FAILED(m_oMiddlePoint.Transmit(pMediaSampleOutput));

    RETURN_NOERROR;
}

tResult cSWE_TrackControl::SendGear( const tInt8 outputGear )
{

    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescGear->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescGear, pMediaSample, pCoder);

        pCoder->Set("tGearValue", (tVoid*)&(outputGear));
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oGear.Transmit(pMediaSample);

    RETURN_NOERROR;
}
