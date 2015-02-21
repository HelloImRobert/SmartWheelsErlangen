#include <cmath>
#include "stdafx.h"
#include "SWE_IntersecPointCalc.h"


#include <iostream>
#include <fstream>




ADTF_FILTER_PLUGIN("SWE Intersection Point calculation", OID_ADTF_SWE_INTERSECPOINTCALC, cSWE_IntersecPointCalc)

cSWE_IntersecPointCalc::cSWE_IntersecPointCalc(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("Reference Point x-Coord",0.0);
    SetPropertyFloat("Reference Point y-Coord",0.0);
    SetPropertyFloat("Intersection Line Distance",300.0);
    SetPropertyFloat("Intersection Line Angle",0.0);
    SetPropertyFloat("Road Width",900.0);
    SetPropertyFloat("max Road Width Deviation",100);
    SetPropertyFloat("Distance Missing Boundary",490);

    //SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID");
}

cSWE_IntersecPointCalc::~cSWE_IntersecPointCalc()
{
}

tResult cSWE_IntersecPointCalc::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oLines.Create("Line_Boundaries", new cMediaType(0, 0, 0, "tLineBoundaries"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oLines));

    RETURN_IF_FAILED(m_oSplines.Create("Spline_Boundaries", new cMediaType(0, 0, 0, "tSplineBoundaries"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSplines));
    RETURN_NOERROR;
}

tResult cSWE_IntersecPointCalc::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // Struct for Intersection Point Transmission
    // TO ADAPT for new Pin/Dadatype: strDescPointLeft, "tPoint2d", pTypePointLeft, m_pCoderDescPointLeft, m_oIntersectionPointLeft, "left_Intersection_Point" !!!!!!!!!!!!!!!!!!!!
    tChar const * strDescIntersecPoints = pDescManager->GetMediaDescription("tIntersections");
    RETURN_IF_POINTER_NULL(strDescIntersecPoints);
    cObjectPtr<IMediaType> pTypeIntersecPoints = new cMediaType(0, 0, 0, "tIntersections", strDescIntersecPoints,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeIntersecPoints->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPoints));

    RETURN_IF_FAILED(m_oIntersectionPoints.Create("Intersection_Points", pTypeIntersecPoints, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIntersectionPoints));

    RETURN_NOERROR;
}

tResult cSWE_IntersecPointCalc::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {

        m_referencePoint.x = GetPropertyFloat("Reference Point x-Coord");
        m_referencePoint.y = GetPropertyFloat("Reference Point y-Coord");
        m_intersectionLineDistance = GetPropertyFloat("Intersection Line Distance");
        m_intersectionLineAngle = GetPropertyFloat("Intersection Line Angle");
        m_roadWidth = GetPropertyFloat("Road Width");
        m_maxRoadWidthDeviation = GetPropertyFloat("max Road Width Deviation");
        m_distMissingBoundary = GetPropertyFloat("Distance Missing Boundary");

    }
    else if(eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cSWE_IntersecPointCalc::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSWE_IntersecPointCalc::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSWE_IntersecPointCalc::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSWE_IntersecPointCalc::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
            SWE_cBoundary leftBoundary( leftFront, leftRear );
            SWE_cBoundary rightBoundary( rightFront, rightRear );

            std::vector<SWE_cBoundary> lineBoundaries;
            lineBoundaries.push_back( leftBoundary );
            lineBoundaries.push_back( rightBoundary );


            // CALCUALTIONS -------------------------------------------------------------------

            // calculate intersection points with boundaries
            std::pair<cv::Point2d, cv::Point2d> intersectionPoints;
            tUInt32 intersecIndicator = intersecPointCalc(intersectionPoints, lineBoundaries);
            lineBoundaries.clear();


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
            pCoder->Set("intersecIndicator", (tVoid*)&(intersecIndicator));
            m_pCoderDescPoints->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPoints.Transmit(pMediaSampleOutput));

            //std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe4.txt");
            //file << lineBoundaries[0] << endl << lineBoundaries[1] << endl << lineBoundaries[2] << endl << lineBoundaries[3] << endl;
            //file.close();

        }
        else if (pSource == &m_oSplines)
        {

            // READ INPUT VALUES -------------------------------------------------------------------

            // generate Coder object
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescInputMeasured->Lock(pMediaSample, &pCoder));

            //get values from media sample (x and y exchanged to transform to front axis coo sys)
            int BoundaryCountTemp;
            pCoder->Get("BoundaryCount", (tVoid*)&(BoundaryCountTemp));

            vector< vector<cv::Point2d> > splines;
            splines.resize(BoundaryCountTemp);

            for(int i=0; i < BoundaryCountTemp; i++)
            {
                stringstream elementSetter;

                int BoundaryArrayCountTemp;
                elementSetter << "BoundaryArray[" << i << "].Count";
                const string& tempRef3 = elementSetter.str();
                const tChar* tempPointer3 = tempRef3.c_str();
                pCoder->Get(tempPointer3, (tVoid*)&(BoundaryArrayCountTemp));
                elementSetter.str(std::string());

                splines[i].resize(BoundaryCountTemp);

                for( int j = 0; j < BoundaryArrayCountTemp; j++)
                {
                    elementSetter << "BoundaryArray[" << i << "].Points[" << j << "].xCoord";
                    const string& tempRef1 = elementSetter.str();
                    const tChar* tempPointer1 = tempRef1.c_str();
                    pCoder->Get(tempPointer1, (tVoid*)&(splines[i][j].x));
                    elementSetter.str(std::string());

                    elementSetter << "BoundaryArray[" << i << "].Points[" << j << "].yCoord";
                    const string& tempRef2 = elementSetter.str();
                    const tChar* tempPointer2 = tempRef2.c_str();
                    pCoder->Get(tempPointer2, (tVoid*)&(splines[i][j].y));
                    elementSetter.str(std::string());
                }
            }

            m_pCoderDescInputMeasured->Unlock(pCoder);
/*
            // generate boundaries from read in values
            SWE_cBoundary leftBoundary( leftFront, leftRear );
            SWE_cBoundary rightBoundary( rightFront, rightRear );

            std::vector<SWE_cBoundary> lineBoundaries;
            lineBoundaries.push_back( leftBoundary );
            lineBoundaries.push_back( rightBoundary );


            // CALCUALTIONS -------------------------------------------------------------------

            // calculate intersection points with boundaries
            std::pair<cv::Point2d, cv::Point2d> intersectionPoints;
            tUInt32 intersecIndicator = intersecPointCalc(intersectionPoints, lineBoundaries);
            lineBoundaries.clear();


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
            pCoder->Set("intersecIndicator", (tVoid*)&(intersecIndicator));
            m_pCoderDescPoints->Unlock(pCoder);

            //transmit media sample over output pin
            // ADAPT: m_oIntersectionPointLeft
            RETURN_IF_FAILED(pMediaSampleOutput->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oIntersectionPoints.Transmit(pMediaSampleOutput));

            //std::ofstream file("/home/odroid/Desktop/Ausgabe/ausgabe4.txt");
            //file << lineBoundaries[0] << endl << lineBoundaries[1] << endl << lineBoundaries[2] << endl << lineBoundaries[3] << endl;
            //file.close();
*/
        }
        else
            RETURN_NOERROR;
        // -------------------------------------------------------------------

    }
    RETURN_NOERROR;
}


tUInt32 cSWE_IntersecPointCalc::intersecPointCalc(std::pair<cv::Point2d, cv::Point2d> &intersectionPoints, std::vector<SWE_cBoundary> boundaries)
{
    int indexLeftBoundary = -1;
        bool leftBoundarySet = false;
        int indexRightBoundary = -1;
        bool rightBoundarySet = false;
        double oldDistanceLeft(std::numeric_limits<double>::max());
        double oldDistanceRight(std::numeric_limits<double>::max());
        // calculate perpenticular distance from reference point to all boundary objects
        // if no intersection point found: DirectionToReferencePoint = -1
        for(unsigned int i = 0; i < boundaries.size(); i++)
        {
            double distance;

            try
            {
                distance = boundaries[i].getPerpendicDistance(m_referencePoint);
            }
            catch (int e)
            {

            }

            if(distance >= 0.0 && ( fabs(distance) < fabs(oldDistanceLeft) || leftBoundarySet == false ) )
            {
                leftBoundarySet = true;
                indexLeftBoundary = i;
            }
            if(distance < 0.0 && ( fabs(distance) < fabs(oldDistanceRight) || rightBoundarySet == false ) )
            {
                rightBoundarySet = true;
                indexRightBoundary = i;
            }
        }


        bool hasLeftIntersectionPoint = true;
        bool hasRightIntersectionPoint = true;

        if(indexLeftBoundary != -1)
        {
            try
            {
                intersectionPoints.first = boundaries[indexLeftBoundary].getIntersection(m_intersectionLineAngle, m_intersectionLineDistance);
            }
            catch(int e)
            {
                hasLeftIntersectionPoint = false;
            }
        }
        else
        {
            hasLeftIntersectionPoint = false;
        }
        if(indexRightBoundary != -1)
        {
            try
            {
                intersectionPoints.second = boundaries[indexRightBoundary].getIntersection(m_intersectionLineAngle, m_intersectionLineDistance);
            }
            catch(int e)
            {
                hasRightIntersectionPoint = false;
            }
        }
        else
        {
            hasRightIntersectionPoint = false;
        }

        // set indicator according to the existing intersection points
        // intersecIndocator == 0: no intersection points
        // intersecIndocator == 1: two intersection points
        // intersecIndocator == 2: only left intersection point
        // intersecIndocator == 3: only right intersection points
        int intersectionIndicator = 0;
        if(hasLeftIntersectionPoint == true)
        {
            if(hasRightIntersectionPoint == true)
            {
                intersectionIndicator = 1;
            }
            else
            {
                intersectionIndicator = 2;
            }
        }
        else if(hasRightIntersectionPoint == true)
        {
            intersectionIndicator = 3;
        }
        else
        {
            intersectionIndicator = 0;
        }

        // set missing intersection points according to intersectionIndicator
        if(intersectionIndicator == 2)
        {
            double sign = 1.0;
            cv::Point2d normalUnitVector = boundaries[indexLeftBoundary].getNormalUnitVector();
            if(normalUnitVector.y >= 0.0)
            {
                sign = -1.0;
            }
            intersectionPoints.second = intersectionPoints.first + sign*m_distMissingBoundary*normalUnitVector;
        }
        else if(intersectionIndicator == 3)
        {
            double sign = 1.0;
            cv::Point2d  normalUnitVector = boundaries[indexRightBoundary].getNormalUnitVector();
            if(normalUnitVector.y <= 0.0)
            {
                sign = -1.0;
            }
            intersectionPoints.first = intersectionPoints.second + sign*m_distMissingBoundary*normalUnitVector;
        }
        else if(intersectionIndicator == 0)
        {

        }

        return intersectionIndicator;
}



/*
int cSWE_IntersecPointCalc::doSth( std::vector<cv::Point2d>& intersecPoints )
{
    cv::Point2d a1(0,200), a2(500,200), b1(0,-400), b2(500,-400);
    cv::Point2d c1(500,100), c2(500,-500), c3(500,-150);
    SWE_cBoundary bound1, bound2;
    SWE_cBoundary bound3, bound4, bound5;
    bound1.boundary.push_back(a1);
    bound1.boundary.push_back(a2);
    bound2.boundary.push_back(b1);
    bound2.boundary.push_back(b2);
    bound3.boundary.push_back(a1);
    bound3.boundary.push_back(c1);
    bound4.boundary.push_back(b1);
    bound4.boundary.push_back(c3);
    bound5.boundary.push_back(b1);
    bound5.boundary.push_back(c2);

    boundaries.clear();
    boundaries.push_back(bound1);
    boundaries.push_back(bound2);
    boundaries.push_back(bound3);
    boundaries.push_back(bound4);
    boundaries.push_back(bound5);

    //std::vector<cv::Point2d> intersecPoints;
    int isChosen = 0;
    int hasTwoIntersecs = 0;

    intersecPointCalc(intersecPoints, isChosen, hasTwoIntersecs);


    return 0;
}
*/
