#include <exception>
#include "swe_cboundary.h"
#include <iostream>
#include <fstream>


SWE_cBoundary::SWE_cBoundary(cv::Point2d firstPoint, cv::Point2d secondPoint)
{
    m_boundary.first = firstPoint;
    m_boundary.second = secondPoint;
}

SWE_cBoundary::~SWE_cBoundary()
{

}



cv::Point2d SWE_cBoundary::getIntersection(const double angle, const double distance)
{
    cv::Point2d o1(distance, 0);
    cv::Point2d o2(m_boundary.first);

    cv::Point2d p2(m_boundary.second);

    // calculate line directions
    cv::Point2d x = o2 - o1;
    cv::Point2d d1(sin(angle), cos(angle));
    cv::Point2d d2 = p2 - o2;

    // calculate intersection point
    double cross = d1.x*d2.y - d1.y*d2.x;
    if (fabs(cross) < 1e-8)
    {
        //throw std::domain_error("Lines are parallel!");
        throw 1;
    }

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    cv::Point2d intersectionPoint = (o1 + d1 * t1);

    return intersectionPoint ;
}

double SWE_cBoundary::getPerpendicDistance(const cv::Point2d& referencePoint)
{
    // calculate line directions
    cv::Point2d x = m_boundary.first - referencePoint;
    cv::Point2d directionVector = m_boundary.second - m_boundary.first;
    cv::Point2d orthogonalDirection;
    orthogonalDirection.x = -directionVector.y;
    orthogonalDirection.y = directionVector.x;

    // calculate intersection point
    double cross = orthogonalDirection.x*directionVector.y - orthogonalDirection.y*directionVector.x;
    if (fabs(cross) < 1e-8)
    {
        //throw std::domain_error("Point is on the line!");
        throw 2;
    }

    double t1 = (x.x * directionVector.y - x.y * directionVector.x)/cross;
    cv::Point2d intersecPoint = (referencePoint + orthogonalDirection * t1);

    // calculate distance
    double distance = cv::norm(intersecPoint - referencePoint);

    // calculate sign of position of intersetion point (+ left of vehicle; - right of vehicle)
    if(intersecPoint.y <= 0){
        distance = distance*(-1);
    }


    return distance;
}

cv::Point2d SWE_cBoundary::getNormalUnitVector()
{
    cv::Point2d tempVec = m_boundary.second - m_boundary.first;
    tempVec = tempVec*(1/cv::norm(tempVec));
    cv::Point2d normalUnitVector;
    normalUnitVector.x = tempVec.y;
    normalUnitVector.y = (-1)*(tempVec.x);

    return normalUnitVector;
}
