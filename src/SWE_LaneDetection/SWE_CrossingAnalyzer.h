#ifndef _SWE_CROSSING_ANALYZER_H_
#define _SWE_CROSSING_ANALYZER_H_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "SWE_BlobDescriptor.h"

enum analysisResult
{
    NORESULT,
    OPENING,
    STOPLINE,
    CROSSINGDETECTION
};

class CrossingDescriptor
{
public:
    CrossingDescriptor();

    analysisResult result;
    int type;
    bool isReal;
    std::pair< cv::Point, cv::Point > stopLine;
};

class CrossingAnalyzer
{

public:
    CrossingDescriptor searchCrossings(BlobDescriptor* leftLaneBlob, BlobDescriptor* rightLaneBlob);
    int classifyCrossings(std::vector< BlobDescriptor > &contours);
    CrossingAnalyzer();

    CrossingAnalyzer
    (
            int leftOfCenterThresh, double lowerLengthThresh , double higherLengthTresh ,
            double arcLengthThresh , double angleTolerance , double veryLineLikeThresh ,
            double minOuterBoundaryLength , double distanceLowestPointToSecondIndexThresh ,
            int lowerOffset , int upperOffset
    );
private:

    bool checkStopLine(std::vector< size_t >& smallFlatAngles , BlobDescriptor* laneBlob, CrossingDescriptor& crossing, bool side);
    bool checkOpening(std::vector< size_t >& smallFlatAngles, BlobDescriptor* laneBlob, CrossingDescriptor& crossing, bool side);
    bool checkStopLineVertical(std::pair< size_t, size_t >& indices, const std::vector< size_t >& flatAngles, const BlobDescriptor* laneBlob , bool side);
    bool checkArc(std::pair< size_t, size_t >& indices, const BlobDescriptor* laneBlob, bool side);

    CrossingDescriptor fuseSidesToCrossing(const CrossingDescriptor& leftSide, const CrossingDescriptor& rightSide);
    std::pair< cv::Point , cv::Point > getVirtualStopLine(size_t firstIndex, size_t secondIndex , BlobDescriptor *laneBlob , bool foundStopLineVertical);

    int _leftOfCenterThresh;
    double _lowerLengthTresh;
    double _higherLengthTresh;
    double _arcLengthThresh;
    double _angleTolerance;
    double _veryLineLikeThresh;
    double _minOuterBoundaryLength;
    double _distanceLowestPointToSecondIndexThresh;

    double _higherFlatAngleThresh;
    double _higherFlatAngleThresh2;
    double _lowerFlatAngleThresh2;
    double _lowerFlatAngleThresh;
    double _higherNinetyAngleThresh;
    double _lowerNinetyAngleThresh;

    int _lowerOffset;
    int _upperOffset;
};

#endif
