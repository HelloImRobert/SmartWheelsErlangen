#ifndef _SWE_BLOB_DESCRIPTOR_H_
#define _SWE_BLOB_DESCRIPTOR_H_

#include "opencv2/core/core.hpp"


enum Side
{
    RIGHT,
    LEFT,
    AMBIGOUS
};

class BlobDescriptor
{
public:
    std::vector< cv::Point > contour;
    Side side;
    double lengthContour;
    double areaContour;
    cv::Point centerOfGravity;
    double angleOfMainDirection;
    double distanceToReference;
    double principalAxisLengthRatio;
    std::vector<cv::Point2d> eigen_vecs;
    std::vector<double> eigen_vals;
    bool complexBoundaryIndicator;
    std::vector< cv::Point2d > directionVectors;
    std::vector< double > lengths;
    std::vector< double > angles;
    std::vector< double > curvature;
    size_t ninetyDegree;
};

#endif
