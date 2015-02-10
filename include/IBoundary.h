#ifndef IBOUNDARY_H
#define IBOUNDARY_H

// opencv header
#include <opencv/cv.h>

class IBoundary
{
public:
    virtual ~IBoundary() {}
    virtual std::vector< cv::Point2d > getIntersections( double offset , double angle ) = 0;
    virtual cv::Point2d getEndPoint() = 0;
    virtual cv::Point2d getStartPoint() = 0;
};

#endif // IBOUNDARY_H
