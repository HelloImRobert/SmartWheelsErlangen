#include "swe_cboundary.h"

SWE_cBoundary::SWE_cBoundary()
{
    count = -1;
    boundary.clear();
}

SWE_cBoundary::~SWE_cBoundary()
{

}

int SWE_cBoundary::getCount()
{
    return boundary.size();
}

int SWE_cBoundary::getIntersection(const double angle, const double distance, cv::Point2d& intersecPoint)
{
    cv::Point2d o1(distance, 0);
    cv::Point2d o2(boundary[0]);

    cv::Point2d p2(boundary[1]);

    // calculate line directions
    cv::Point2d x = o2 - o1;
    cv::Point2d d1(sin(angle), cos(angle));
    cv::Point2d d2 = p2 - o2;

    // calculate intersection point
    double cross = d1.x*d2.y - d1.y*d2.x;
    if (fabs(cross) < 1e-8)
        return 1;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    intersecPoint = (o1 + d1 * t1);
    return 0;
}

int SWE_cBoundary::getPerpenticDist(const cv::Point2d& referencePoint, double& distance, int& direction)
{
    const cv::Point2d& o1 = referencePoint;
    cv::Point2d o2(boundary[0]);

    cv::Point2d p2(boundary[1]);

    // calculate line directions
    cv::Point2d x = o2 - o1;
    cv::Point2d d2 = p2 - o2;
    cv::Point2d d1;
    d1.x = -d2.y;
    d1.y = d2.x;

    // calculate intersection point
    double cross = d1.x*d2.y - d1.y*d2.x;
    if (fabs(cross) < 1e-8){
        distance = -1;
        direction = -1;
        return 1;
    }

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    cv::Point2d intersecPoint = (o1 + d1 * t1);

    // calculate distance
    distance = cv::norm(intersecPoint - referencePoint);

    // calculate sign of position of intersetion point (+ left of vehicle; - right of vehicle)
    if(intersecPoint.y >= 0){
        direction = 1;
    }
    else{
        direction = 0;
    }

    return 0;
}
