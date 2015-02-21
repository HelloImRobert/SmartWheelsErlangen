#ifndef FILTER_H
#define FILTER_H

#include "swe_cboundary.h"


class Filter
{
public:
    Filter();
    ~Filter();

private:
    // vector containing all boundaries
    std::vector<SWE_cBoundary> boundaries;
    // vector containing boundaries that fulfill criterias for validity
    // current criterias: one boundary left, one right of vehicle; distance between boundaries in sepcified intervall
    std::vector< std::vector<SWE_cBoundary> > validBoundaryPairs;
    // boundary pair that is in deed assumed to be the real road boundary
    std::vector<SWE_cBoundary> chosenBoundaryPair;
    // number of boundary elements
    int count;
    // parameter for intersection line angle (from front axis)
    double intersectionLineAngle;
    // parameter for intersection line distance (from front axis)
    double intersectionLineDistance;
    // parameter for road width
    double roadWidth;
    // parameter for max allowed deviation of boundary from road width (for determination of valid boundary objects)
    double maxRoadWidthDeviation;
    // indicator wether a valid road boundary is found and chosen
    // 1: yes
    // 0: no
    int isChosen;
    // indicator wether 2 intersection points on chosen boundary were found
    // 1: yes
    // 0: no
    int hasTwoIntersectionPoints;
    // point to which distances to boundaries are calculated (usually front axis)
    cv::Point2d referencePoint;
    // vector containing distances from reference point to every boundary
    std::vector<double> DistanceToReferencePoint;
    // vector indicationg wether boundary is left or right of vehicle (relativ to reference point)
    std::vector<int> DirectionToReferencePoint;

protected:
    int setCount();
    int setReferencePoint(const cv::Point2d& referencePoint); // sets reference point: (0,0) is center of front axis
    int calcDistancesToReferencePoint(std::vector<SWE_cBoundary> &boundaries); // calculates perpeticular distance from boundary to reference point
    int intersecPointCalc(std::vector<cv::Point2d>& intersectionPoints, int &isChosen, int &hasTwoIntersectionPoints);
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    int setBoundaries(const std::vector<SWE_cBoundary>& boundaries);

public:
    int doSth();
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
};

#endif // FILTER_H
