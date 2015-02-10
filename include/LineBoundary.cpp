#include "IBoundary.h"

class LineBoundary : IBoundary
{
public:
    virtual std::vector< cv::Point2d > getIntersections( double offset , double angle )
    {
        std::cerr<< "Not IMPLEMENTED !" << std::endl;
    }

    virtual cv::Point2d getEndPoint()
    {
        std::cerr<< "Not IMPLEMENTED !" << std::endl;
    }

    virtual cv::Point2d getStartPoint()
    {
        std::cerr<< "Not IMPLEMENTED !" << std::endl;
    }
};
