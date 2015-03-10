#include <cmath>
#include "stdafx.h"
#include "SWE_TrajectoryPlanning.h"


#include <iostream>
#include <fstream>




ADTF_FILTER_PLUGIN("SWE Trajectory Planning", OID_ADTF_SWE_TRAJECTORYPLANNING, cSWE_TrajectoryPlanning)

cSWE_TrajectoryPlanning::cSWE_TrajectoryPlanning(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("maximum distance",1300.0);
    SetPropertyFloat("maximum direction angle",40*CV_PI/180);
    SetPropertyFloat("break Angle",40*CV_PI/180);
    SetPropertyFloat("insertion distance",100.0);
    SetPropertyFloat("road width",900.0);
    SetPropertyFloat("max road width deviation",100.0);
    SetPropertyFloat("tracking point distance",400.0);
    SetPropertyFloat("intersection angle",90*CV_PI/180);
}

cSWE_TrajectoryPlanning::~cSWE_TrajectoryPlanning()
{
}

tResult cSWE_TrajectoryPlanning::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oLines.Create("Line_Boundaries", new cMediaType(0, 0, 0, "tLineBoundaries"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oLines));

    RETURN_IF_FAILED(m_oSplines.Create("tSplineBoundaryNew", new cMediaType(0, 0, 0, "tSplineBoundaryNew"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSplines));
    RETURN_NOERROR;
}

tResult cSWE_TrajectoryPlanning::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Struct for Intersection Point Transmission
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescIntersecPoints = pDescManager->GetMediaDescription("tIntersectionsNew");
    RETURN_IF_POINTER_NULL(strDescIntersecPoints);
    cObjectPtr<IMediaType> pTypeIntersecPoints = new cMediaType(0, 0, 0, "tIntersectionsNew", strDescIntersecPoints,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeIntersecPoints->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPoints));

    RETURN_IF_FAILED(m_oIntersectionPoints.Create("tracking_point", pTypeIntersecPoints, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));

    RETURN_NOERROR;
}

tResult cSWE_TrajectoryPlanning::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
        _maxDistance = GetPropertyFloat("maximum distance",1300.0);
        _maxDirectionAngle = GetPropertyFloat("maximum direction angle",40*CV_PI/180);
        _breakAngle = GetPropertyFloat("break Angle",40*CV_PI/180);
        _insertionDistance = GetPropertyFloat("insertion distance",100.0);
        _roadWidth = GetPropertyFloat("road width",900.0);
        _maxRoadWidthDeviation = GetPropertyFloat("max road width deviation",100.0);
        _trackingPointDistance = GetPropertyFloat("tracking point distance",300.0);
        _intersectionAngle = GetPropertyFloat("intersection angle",90*CV_PI/180);
    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_TrajectoryPlanning::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_TrajectoryPlanning::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_TrajectoryPlanning::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_TrajectoryPlanning::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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

        if (pSource == &m_oLines)
        {
            // READ INPUT VALUES -------------------------------------------------------------------
            /*
            // init temporary objects
            cv::Point2d leftFront;
            cv::Point2d leftRear;
            cv::Point2d rightFront;
            cv::Point2d rightRear;

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            pCoder->Get("leftFrontX", (tVoid*)&(leftFront.x));
            pCoder->Get("leftFrontY", (tVoid*)&(leftFront.y));
            pCoder->Get("leftRearX", (tVoid*)&(leftRear.x));
            pCoder->Get("leftRearY", (tVoid*)&(leftRear.y));
            pCoder->Get("rightFrontX", (tVoid*)&(rightFront.x));
            pCoder->Get("rightFrontY", (tVoid*)&(rightFront.y));
            pCoder->Get("rightRearX", (tVoid*)&(rightRear.x));
            pCoder->Get("rightRearY", (tVoid*)&(rightRear.y));
            m_pCoderDescInputMeasured->Unlock(pCoder);

            // generate boundaries from read in values
            //SWE_cBoundary leftBoundary( leftFront, leftRear );
            //SWE_cBoundary rightBoundary( rightFront, rightRear );

            //std::vector<SWE_cBoundary> lineBoundaries;
            //lineBoundaries.push_back( leftBoundary );
            //lineBoundaries.push_back( rightBoundary );


            // CALCUALTIONS -------------------------------------------------------------------

            // calculate intersection points with boundaries
            //            cv::Point2d intersectionPoint;
            //            int intersectionIndicator = processing( intersectionPoint,
            //                                                    hasRightBoundary, hasLeftBoundary, hasMiddleBoundary,
            //                                                    rightBoundary, leftBoundary, middleBoundary );


            // TRANSMIT OUTPUT VALUES -------------------------------------------------------------------


            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleOutput;
            RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

            //allocate memory with the size given by the descriptor
            // ADAPT: m_pCoderDescPointLeft
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescPoints->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOutput->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            // ADAPT: m_pCoderDescPointLeft
            //cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescPoints->WriteLock(pMediaSampleOutput, &pCoder));
            pCoder->Set("xCoordLeft", (tVoid*)&(intersectionPoints.first.x));
            pCoder->Set("yCoordLeft", (tVoid*)&(intersectionPoints.first.y));
            pCoder->Set("xCoordRight", (tVoid*)&(intersectionPoints.second.x));
            pCoder->Set("yCoordRight", (tVoid*)&(intersectionPoints.second.y));
            //pCoder->Set("intersecIndicator", (tVoid*)&(intersecIndicator));
            m_pCoderDescPoints->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPoints.Transmit(pMediaSampleOutput));
*/


        }
        else if (pSource == &m_oSplines)
        {
            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            std::vector< cv::Point2d > rightBoundary;
            {
                tInt8 BoundaryArrayCountTemp = 0;
                pCoder->Get("rightBoundary.Count", (tVoid*)&(BoundaryArrayCountTemp));
                rightBoundary.resize( BoundaryArrayCountTemp );

                stringstream elementGetter;
                for( size_t j = 0; j < BoundaryArrayCountTemp; j++)
                {
                    elementGetter << "rightBoundary.Points[" << j << "].xCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(rightBoundary.at(j).x));
                    elementGetter.str(std::string());

                    elementGetter << "rightBoundary.Points[" << j << "].yCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(rightBoundary.at(j).y));
                    elementGetter.str(std::string());
                }
            }

            std::vector< cv::Point2d > leftBoundary;
            {
                tInt8 BoundaryArrayCountTemp = 0;
                pCoder->Get("leftBoundary.Count", (tVoid*)&(BoundaryArrayCountTemp));
                leftBoundary.resize( BoundaryArrayCountTemp );

                stringstream elementGetter;
                for( size_t j = 0; j < BoundaryArrayCountTemp; j++)
                {
                    elementGetter << "leftBoundary.Points[" << j << "].xCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(leftBoundary.at(j).x));
                    elementGetter.str(std::string());

                    elementGetter << "leftBoundary.Points[" << j << "].yCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(leftBoundary.at(j).y));
                    elementGetter.str(std::string());
                }
            }

            std::vector< cv::Point2d > middleBoundary;
            {
                tInt8 BoundaryArrayCountTemp = 0;
                pCoder->Get("middleBoundary.Count", (tVoid*)&(BoundaryArrayCountTemp));
                middleBoundary.resize( BoundaryArrayCountTemp );

                stringstream elementGetter;
                for( size_t j = 0; j < BoundaryArrayCountTemp; j++)
                {
                    elementGetter << "middleBoundary.Points[" << j << "].xCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(middleBoundary.at(j).x));
                    elementGetter.str(std::string());

                    elementGetter << "middleBoundary.Points[" << j << "].yCoord";
                    pCoder->Get(elementGetter.str().c_str(), (tVoid*)&(middleBoundary.at(j).y));
                    elementGetter.str(std::string());
                }
            }

            m_pCoderDescInputMeasured->Unlock(pCoder);

            // CALCUALTIONS -------------------------------------------------------------------

            // calculate intersection points with boundaries
            cv::Point2d intersectionPoint;
            tInt8 intersectionIndicator = processing( intersectionPoint, rightBoundary, leftBoundary, middleBoundary );

            // TRANSMIT OUTPUT VALUES -------------------------------------------------------------------

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleOutput;
            RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOutput));

            //allocate memory with the size given by the descriptor
            // ADAPT: m_pCoderDescPointLeft
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescPoints->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOutput->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            // ADAPT: m_pCoderDescPointLeft
            //cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescPoints->WriteLock(pMediaSampleOutput, &pCoder));
            pCoder->Set("intersecPoint.xCoord", (tVoid*)&(intersectionPoint.x));
            pCoder->Set("intersecPoint.yCoord", (tVoid*)&(intersectionPoint.y));
            pCoder->Set("Indicator", (tVoid*)&(intersectionIndicator));
            m_pCoderDescPoints->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPoints.Transmit(pMediaSampleOutput));

            LOG_ERROR(cString::Format( "MBMT: test2"));
            //std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe4.txt");
            //file << lineBoundaries[0] << endl << lineBoundaries[1] << endl << lineBoundaries[2] << endl << lineBoundaries[3] << endl;
            //file.close();

        }

        RETURN_NOERROR;
        // -------------------------------------------------------------------

    }
    RETURN_NOERROR;
}

int cSWE_TrajectoryPlanning::processing( cv::Point2d& returnedPoint,
                                         const std::vector< cv::Point2d >& rightBoundary,
                                         const std::vector< cv::Point2d >& leftBoundary,
                                         const std::vector< cv::Point2d >& middleBoundary)

{

    bool hasRightBoundary = rightBoundary.size() > 1;
    bool hasLeftBoundary = leftBoundary.size() > 1;
    bool hasMiddleBoundary = middleBoundary.size() > 1;

    // check if right and left boundary are not empty
    if( !hasRightBoundary && !hasLeftBoundary )
    {
        returnedPoint.x = 0.0;
        returnedPoint.y = 0.0;
        return 0;
    }

    // insert points in fixed distance on boundaries for intersection point calculation
    std::vector< std::pair< size_t, double > > segmentLengths;
    std::vector< cv::Point2d > extendedRightBoundary( splineBoundaryCalcs::insertPoints( segmentLengths, rightBoundary, _insertionDistance ));
    std::vector< cv::Point2d > extendedLeftBoundary( splineBoundaryCalcs::insertPoints( segmentLengths, leftBoundary, _insertionDistance) );
    std::vector< cv::Point2d > extendedMiddleBoundary( splineBoundaryCalcs::insertPoints( segmentLengths, middleBoundary, _insertionDistance) );
    segmentLengths.clear();

    // calculate most plausible segment for every non-empty boundary
    std::vector< cv::Point2d > plausibleRightSegment;
    splineBoundaryCalcs::extractPlausibleSegment( plausibleRightSegment, extendedRightBoundary, _breakAngle, _maxDirectionAngle, _maxDistance );
    std::vector< cv::Point2d > plausibleLeftSegment;
    splineBoundaryCalcs::extractPlausibleSegment( plausibleLeftSegment, extendedLeftBoundary, _breakAngle, _maxDirectionAngle, _maxDistance );
    std::vector< cv::Point2d > plausibleMiddleSegment;
    splineBoundaryCalcs::extractPlausibleSegment( plausibleMiddleSegment, extendedMiddleBoundary, _breakAngle, _maxDirectionAngle, _maxDistance );

    // check if there is no plausible segment usable and return if this is the case
    if( plausibleRightSegment.empty() && plausibleLeftSegment.empty() )
    {
        returnedPoint.x = 0.0;
        returnedPoint.y = 0.0;
        return 0;
    }

    // check if plausible segments have right counting order (point closest to vehicle has position 0 in vector)
    if( !plausibleRightSegment.empty() )
    {
        splineBoundaryCalcs::correctSplineDirection( plausibleRightSegment );
    }
    if( !plausibleLeftSegment.empty() )
    {
        splineBoundaryCalcs::correctSplineDirection( plausibleLeftSegment );
    }
    if( !plausibleMiddleSegment.empty() )
    {
        splineBoundaryCalcs::correctSplineDirection( plausibleMiddleSegment );
    }

    // calculate lengthes of left and right plausible segment
    double rightSegmentLength = 0.0;
    double leftSegmentLength = 0.0;

    for( size_t i = 1; i < plausibleRightSegment.size(); i++ )
    {
        rightSegmentLength += cv::norm( plausibleRightSegment[i] - plausibleRightSegment[i-1] );
    }
    for( size_t i = 1; i < plausibleLeftSegment.size(); i++ )
    {
        leftSegmentLength += cv::norm( plausibleLeftSegment[i] - plausibleLeftSegment[i-1] );
    }

    // get the longer plausible segment
    bool longestSegmentIsRight = true;
    std::vector< cv::Point2d >& longestPlausibleSegment = plausibleRightSegment;
    if( rightSegmentLength < leftSegmentLength )
    {
        longestSegmentIsRight = false;
        longestPlausibleSegment = plausibleLeftSegment;
    }

    // calculate tangents and normal vectors on every point of longest plausible segment
    std::vector< cv::Point2d > tangents( splineBoundaryCalcs::getTangents( longestPlausibleSegment ) );
    std::vector< cv::Point2d > normals;
    normals.reserve( tangents.size() );
    for( size_t i = 0; i < tangents.size(); i++ )
    {
        normals.push_back( splineBoundaryCalcs::getNormal2dVector( tangents.at( i ) ) );
    }

    // calucalte trajectory ( assuming that normals close to vehicle look to the left )
    std::vector< cv::Point2d > trajectory;
    trajectory.reserve( longestPlausibleSegment.size() );
    if( longestSegmentIsRight )
    {
        for( size_t i = 0; i < longestPlausibleSegment.size(); i++ )
        {
            trajectory.push_back( longestPlausibleSegment.at( i ) + ( normals.at( i ) * ( _roadWidth / 4 ) ) );
        }
    }
    else
    {
        for( size_t i = 0; i < longestPlausibleSegment.size(); i++ )
        {
            trajectory.push_back( longestPlausibleSegment.at( i ) + ( ( -1.0 ) * normals.at( i ) * ( _roadWidth * 3 / 4 ) ) );
        }
    }

    // if there is a middle boundary
    if( hasMiddleBoundary && !middleBoundary.empty() )
    {
        cv::Point2d intersectionPoint;
        std::vector< std::pair< bool, cv::Point2d > > middleIntersections;
        bool middleBoundaryDistanceOK = true;
        middleIntersections.resize( longestPlausibleSegment.size() );
        // for all points on the longest plausible segment (on right or left boundary)
        for( size_t i = 0; i < longestPlausibleSegment.size(); i++ )
        {
            // first assume intersections are incorrrect
            middleIntersections[i].first = false;
            // get intersections between current point on longest plausible segment
            // with middle boundary
            bool hasIntersection = splineBoundaryCalcs::getLineIntersection( intersectionPoint,
                                                                             middleBoundary,
                                                                             longestPlausibleSegment[i],
                                                                             normals[i]);

            // if there is an intersection with current point on longest plausible segment ...
            if( hasIntersection == true )
            {
                // ... and if this intersection point lies in an area where the middle boundary should lie ...
                if( ( cv::norm( intersectionPoint - longestPlausibleSegment[i] ) < ( _roadWidth/2 + _maxRoadWidthDeviation/2 ) )
                        && ( cv::norm( intersectionPoint - longestPlausibleSegment[i] ) > ( _roadWidth/2 - _maxRoadWidthDeviation/2 ) ) )
                {
                    // ... then this intersection point is added to the vector for valid middle boundary intersections ...
                    if( longestSegmentIsRight )
                    {
                        // ... and we calculate the corresponding trajectory point out of it ( right boundary)
                        middleIntersections[i].first = true;
                        middleIntersections[i].second = longestPlausibleSegment[i] + ( intersectionPoint - longestPlausibleSegment[i] ) * 0.5;
                    }
                    else
                    {
                        // ... and we calculate the corresponding trajectory point out of it (left boundary)
                        middleIntersections[i].first = true;
                        middleIntersections[i].second = longestPlausibleSegment[i] + ( intersectionPoint - longestPlausibleSegment[i] ) * 1.5;
                    }
                }
                else
                {
                    // ... if this is not the case it is assumed, that the middle boundary is not correctly detected
                    middleBoundaryDistanceOK = false;
                }
            }
        }

        // if all intersection points found with middle boundary are valid ...
        if ( middleBoundaryDistanceOK == true )
        {
            // ... we replace the corresponding points calculated before with the new ones calculated by
            // intersecting with the middle boundary (if this is not the case we leave the trajectory as it is)
            for( size_t i = 0; i < trajectory.size(); i++ )
            {
                // if we found a valid intersection with middle line
                if( middleIntersections[i].first )
                {
                    trajectory[i] = middleIntersections[i].second;
                }
            }
        }
    }

    // checking wether first point on trajectory is closer than the distance we want to calculate
    // the track control point (_trackingPointDistance)
    int intersectionIndicator = 0;
    if( !trajectory.empty() )
    {
        if( trajectory[0].x < _trackingPointDistance )
        {
            // transform trajectory so that intersection line is x-axis for intersection calculation
            std::vector< cv::Point2d > trajectory_il( splineBoundaryCalcs::ilTv( trajectory, -_intersectionAngle, _trackingPointDistance ) );
            // calculate c-spline coefficients of transformed trajectory
            std::vector< std::vector < cv::Point2d > > coeffsTrajectory_il( splineBoundaryCalcs::calcSplineCoeffs( trajectory_il, 0.5 ) );

            // calculate zeros of c-spline in intersection line coo-sys
            std::vector < cv::Point2d > zerosTrajectory_il( splineBoundaryCalcs::calcSplineZeros( coeffsTrajectory_il ) );

            // back-transform calculated zeros in front axis coo-sys
            std::vector < cv::Point2d > zerosTrajectory( splineBoundaryCalcs::vTil( zerosTrajectory_il, -_intersectionAngle, _trackingPointDistance ) );

            // indicate that two boundaries were found
            intersectionIndicator = 1;

            // return calculated tracking point
            if( !zerosTrajectory.empty() )
            {
                returnedPoint = zerosTrajectory[0];
            }
            else
            {
                returnedPoint = cv::Point2d( 0, 0 );

                // indicate that no tracking point was found
                intersectionIndicator = 0;
            }

        }
        else
        {
            // indicate that two boundaries were found
            intersectionIndicator = 1;

            // return closest point of trajectory
            returnedPoint = trajectory[0];
        }

    }
    else
    {
        // if trajectory is too far from vehicle ...
        returnedPoint = cv::Point2d( 0, 0 );

        // return no tracking point was found
        intersectionIndicator = 0;
    }

    return intersectionIndicator;
}


int cSWE_TrajectoryPlanning::doSth()
{
    //    cv::Point2d line0(1,1), line1(3,0),
    //                point(1,-2), normal1(0.9,1), normal2(2,1), normal3(-1,1);
    //    std::vector< cv::Point2d > lineVec;
    //    lineVec.push_back(line0);
    //    lineVec.push_back(line1);

    //    cv::Point2d interPt;
    //    bool hastInter = splineBoundaryCalcs::getLineIntersection( interPt, lineVec, point, normal3 );
    // test extractPlausibleSegment
    //    cv::Point2d b1(0,0), b2(1,0), b3(2,0), b4(3,0), b5(3,1),
    //            b6(4,2), b7(5,2), b8(6,2), b9(6,3), b10(6,4);
    cv::Point2d b1(0.0,-250), b2(100,-250), b3(200,-250), b4(300,-300), b5(400,-400), b6(450, -500);
    cv::Point2d c1(0.0,0.0), c2(100,0.0), c3(200,0.0), c4(300,-50), c5(400,-150), c6(450, -250);

    std::vector< cv::Point2d > rightBoundary;
    rightBoundary.push_back(b1);
    rightBoundary.push_back(b2);
    rightBoundary.push_back(b3);
    //    rightBoundary.push_back(b4);
    //    rightBoundary.push_back(b5);
    //    rightBoundary.push_back(b6);
    //    rightBoundary.push_back(b7);
    //    rightBoundary.push_back(b8);
    //    rightBoundary.push_back(b9);
    //    rightBoundary.push_back(b10);

    std::vector< cv::Point2d > middleBoundary;
    middleBoundary.push_back(c1);
    middleBoundary.push_back(c2);
    middleBoundary.push_back(c3);
    //    middleBoundary.push_back(c4);
    //    middleBoundary.push_back(c5);
    //    middleBoundary.push_back(c6);

    //std::reverse( rightBoundary.begin(), rightBoundary.end() );

    std::vector< cv::Point2d > leftBoundary;

    bool hasRightBoundary = true;
    bool hasLeftBoundary = false;
    bool hasMiddleBoundary = true;


    cv::Point2d intersectionPoint;
    int indicator = processing( intersectionPoint, hasRightBoundary, hasLeftBoundary, hasMiddleBoundary,
                                rightBoundary, leftBoundary, middleBoundary );


    return 0;
}
