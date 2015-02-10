#include "filter.h"

Filter::Filter()
{
    count = -1;
    intersectionLineAngle = 0;
    intersectionLineDistance = 300;
    roadWidth = 600;
    maxRoadWidthDeviation = 100;
    isChosen = 0;
    hasTwoIntersectionPoints = 0;
    cv::Point2d initReferencePoint(0,0);
    setReferencePoint(initReferencePoint);
}

Filter::~Filter()
{

}

/*
set count to size of boundary
*/
int Filter::setCount()
{
    if(!boundaries.empty()){
        count = boundaries.size();
        return 0;
    }
    else{
        return 1;
    }
}
/*
computes intersetion points and indicator for steering angle calculation
indicator values:
1: two intersection points found
0: less than two intersection points found
else: computation error
*/
int Filter::intersecPointCalc(std::vector<cv::Point2d>& intersectionPoints, int& isChosen, int& hasTwoIntersectionPoints)
{
    // clean necessary objects
    DistanceToReferencePoint.clear();
    DistanceToReferencePoint.resize(boundaries.size());

    DirectionToReferencePoint.clear();
    DirectionToReferencePoint.resize(boundaries.size());

    validBoundaryPairs.clear();

    intersectionPoints.clear();
    intersectionPoints.resize(2);

    // calculate perpenticular distance from reference point to all boundary objects
    // if no intersection point found: DirectionToReferencePoint = -1
    calcDistancesToReferencePoint(boundaries);

    // calculate distances from every boundary left of vehicle to every boundary right of vehicle

    // find pairs of left and right sided boundary
    // (only use boundary objects with DirectionToReferencePoint == 0 || 1 (=>valid))
    for(unsigned int i = 0; i < boundaries.size(); i++){
        if(DirectionToReferencePoint[i] == 1){
            for(unsigned int j = 0; j < boundaries.size(); j++){
                if(DirectionToReferencePoint[j] == 0){
                    // calculate intersection points of boundary pair with defined intersection line
                    std::vector<cv::Point2d> intersecPointsForBoundaryDistCalc;
                    intersecPointsForBoundaryDistCalc.resize(4);
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    double intersecAngle1, intersecDistance1, intersecAngle2, intersecDistance2;
                    intersecAngle1 = 0; intersecDistance1 = 0; intersecAngle2 = 0; intersecDistance2 = 300;
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    // if 2 intersection point pairs exist...
                    if(
                            0 == boundaries[i].getIntersection(intersecAngle1, intersecDistance1, intersecPointsForBoundaryDistCalc[0])
                            &&  0 == boundaries[j].getIntersection(intersecAngle1, intersecDistance1, intersecPointsForBoundaryDistCalc[1])
                            &&  0 == boundaries[i].getIntersection(intersecAngle2, intersecDistance2, intersecPointsForBoundaryDistCalc[2])
                            &&  0 == boundaries[j].getIntersection(intersecAngle2, intersecDistance2, intersecPointsForBoundaryDistCalc[3])
                            ){
                        // ...calculate distances between paired points
                        double pairDistance1 = norm(intersecPointsForBoundaryDistCalc[0] - intersecPointsForBoundaryDistCalc[1]);
                        double pairDistance2 = norm(intersecPointsForBoundaryDistCalc[2] - intersecPointsForBoundaryDistCalc[3]);

                        // if all pair distances are in valid intervall...
                        if(
                                (pairDistance1 > roadWidth-maxRoadWidthDeviation) && (pairDistance1 < roadWidth+maxRoadWidthDeviation)
                                && (pairDistance2 > roadWidth-maxRoadWidthDeviation) && (pairDistance2 < roadWidth+maxRoadWidthDeviation)
                                ){
                            // ...current boundary pair is valid => save it
                            std::vector<SWE_cBoundary> validPair;
                            validPair.push_back(boundaries[i]);
                            validPair.push_back(boundaries[j]);
                            validBoundaryPairs.push_back(validPair);
                        }
                    }
                }
            }
        }
    }

    // choose real road boundary from valid pairs
    if(!validBoundaryPairs.empty()){

        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // muss noch überlegt werden
        chosenBoundaryPair = validBoundaryPairs[0];
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        isChosen = 1;
    }
    else{
        isChosen = 0;
    }
    Filter::isChosen = isChosen;

    // calculate intersection points on chosen pair
    if(
            0 == chosenBoundaryPair[0].getIntersection(intersectionLineAngle, intersectionLineDistance, intersectionPoints[0])
            && 0 == chosenBoundaryPair[1].getIntersection(intersectionLineAngle, intersectionLineDistance, intersectionPoints[1])
            ){
        hasTwoIntersectionPoints = 1;
    }
    else{
        hasTwoIntersectionPoints = 0;
    }
    Filter::hasTwoIntersectionPoints = hasTwoIntersectionPoints;

    return 0;
}

/*
sets reference point: for example (0,0) is center of front axis
*/
int Filter::setReferencePoint(const cv::Point2d& referencePoint)
{
    Filter::referencePoint = referencePoint;

    return 0;
}

/*
calculates perpenticular distance from boundary to reference point
*/
int Filter::calcDistancesToReferencePoint(std::vector<SWE_cBoundary>& boundaries)
{
    for(unsigned int i = 0; i < boundaries.size(); i++){
        boundaries[i].getPerpenticDist(referencePoint, DistanceToReferencePoint[i], DirectionToReferencePoint[i]);
    }

    return 0;
}

// !!!!!!!!!!!!!!!!!!!!!!!!!
int Filter::setBoundaries(const std::vector<SWE_cBoundary>& boundaries)
{
    Filter::boundaries = boundaries;

    return 0;
}

int Filter::doSth()
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

    boundaries.push_back(bound1);
    boundaries.push_back(bound2);
    boundaries.push_back(bound3);
    boundaries.push_back(bound4);
    boundaries.push_back(bound5);

    std::vector<cv::Point2d> intersecPoints;
    int isChosen = 0;
    int hasTwoIntersecs = 0;

    intersecPointCalc(intersecPoints, isChosen, hasTwoIntersecs);

    return 0;
}
