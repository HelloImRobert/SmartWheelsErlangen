#ifndef SWE_CLASS_BOUNDARY_H
#define SWE_CLASS_BOUNDARY_H

#include <stdlib.h>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

class SWE_cBoundary
{
public:
    SWE_cBoundary(cv::Point2d firstPoint, cv::Point2d secondPoint);
    ~SWE_cBoundary();

    cv::Point2d getIntersection(const double angle, const double distance);
    double getPerpendicDistance(const cv::Point2d& referencePoint);
    cv::Point2d getNormalUnitVector();

private:
    std::pair<cv::Point2d, cv::Point2d> m_boundary;

};

#endif // SWE_CLASS_BOUNDARY_H
