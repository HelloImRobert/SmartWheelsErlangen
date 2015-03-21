#ifndef _SWE_TRAJECTORYPLANNING_H_
#define _SWE_TRAJECTORYPLANNING_H_

#include "stdafx.h"
#include "splineboundarycalcs.h"


#define OID_ADTF_SWE_TRAJECTORYPLANNING "adtf.swe.trajectoryplanning"

/*!
* Trajectory Planner
*/
class cSWE_TrajectoryPlanning : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_TRAJECTORYPLANNING, "SWE Trajectory Planning", OBJCAT_DataFilter, "Trajectory Planning", 1, 0,0, "pre alpha version");

    // create pins
    cInputPin m_oLines;
    cInputPin m_oSplines;

    cOutputPin m_oIntersectionPoints;
    cOutputPin m_oTrajectory;

public:
    cSWE_TrajectoryPlanning(const tChar* __info);
    virtual ~cSWE_TrajectoryPlanning();

    int doSth();

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:
    /*! creates all the input Pins*/
    tResult CreateInputPins(__exception = NULL);
    /*! creates all the output Pins*/
    tResult CreateOutputPins(__exception = NULL);



    // point to which distances to boundaries are calculated (usually front axis)
    cv::Point2d m_referencePoint;

    int processing(cv::Point2d& returnedPoint,
                   std::vector<cv::Point2d> &trajectory,
                   const std::vector< cv::Point2d >& rightBoundary,
                   const std::vector< cv::Point2d >& leftBoundary,
                   const std::vector< cv::Point2d >& middleBoundary);


    // maximum allowed starting distance to vehicle for plausible segment to be valid
    double _maxDistance;
    // maximum allowed starting angle to vehicle for plausible segment to be valid
    double _maxDirectionAngle;
    // maximum allowed angle between two spline segments to be in the same plausible segment
    double _breakAngle;
    // distance for insertion addition points in boundary spline
    double _insertionDistance;
    // road width
    double _roadWidth;
    // maximum allowed deviation from road width
    double _maxRoadWidthDeviation;
    // distance from vehicle to calculate tracked point on trajectory
    double _trackingPointDistance;
    // angle for intersection line from vehicle to trajectory
    double _intersectionAngle;

    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInputMeasured;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescPoints;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescTrajectory;

};

#endif // _SWE_TRAJECTORYPLANNING_H_
