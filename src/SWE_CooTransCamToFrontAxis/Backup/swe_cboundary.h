#ifndef SWE_CLASS_BOUNDARY_H
#define SWE_CLASS_BOUNDARY_H

#include <stdlib.h>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

class SWE_cBoundary
{
public:
    SWE_cBoundary();
    ~SWE_cBoundary();

    std::vector<cv::Point2d> boundary;

    int getCount();

    int getIntersection(const double angle, const double distance, cv::Point2d& intersecPoint);
    int getPerpenticDist(const cv::Point2d& referencePoint, double& distance, int &direction);

private:
    int count;


};

#endif // SWE_CLASS_BOUNDARY_H
