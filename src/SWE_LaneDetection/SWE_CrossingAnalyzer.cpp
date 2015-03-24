#include "SWE_CrossingAnalyzer.h"
#include <iostream>

CrossingDescriptor::CrossingDescriptor() : result( NORESULT ), isReal(false)
{

}

bool sort_yValContour(const cv::Point& point1, const cv::Point& point2)
 {
     return point1.y > point2.y;
 }

CrossingAnalyzer::CrossingAnalyzer()
{
    _leftOfCenterThresh = 150;
    _lowerLengthTresh = 320;
    _higherLengthTresh = 600;
    _arcLengthThresh = 50;
    _angleTolerance = 0.5;
    _higherFlatAngleThresh = _angleTolerance;
    _higherFlatAngleThresh2 = CV_PI + _angleTolerance;
    _lowerFlatAngleThresh2 = CV_PI - _angleTolerance;
    _lowerFlatAngleThresh = - _angleTolerance;
    _higherNinetyAngleThresh = 0.5 * CV_PI + _angleTolerance;
    _lowerNinetyAngleThresh = 0.5 * CV_PI - _angleTolerance;
    _veryLineLikeThresh = 600;
    _minOuterBoundaryLength = 1700;
    _distanceLowestPointToSecondIndexThresh = 200;

    _higherFlatAngleThresh = _angleTolerance;
    _higherFlatAngleThresh2 = CV_PI + _angleTolerance;
    _lowerFlatAngleThresh2 = CV_PI - _angleTolerance;
    _lowerFlatAngleThresh = - _angleTolerance;
    _higherNinetyAngleThresh = 0.5 * CV_PI + _angleTolerance;
    _lowerNinetyAngleThresh = 0.5 * CV_PI - _angleTolerance;


    _lowerOffset = 80;
    _upperOffset = 980;
}

bool CrossingAnalyzer::checkStopLineVertical(std::pair< size_t, size_t >& indices, const std::vector< size_t >& flatAngles, const BlobDescriptor* laneBlob, bool side)
{
    const std::vector< double >& angles = laneBlob->angles;

    for (size_t i = 0; i < flatAngles.size(); ++i)
    {
        int index = flatAngles[i];
        int successor = ( index + 1 ) % angles.size();

        int predecessor = index - 1;
        if (predecessor < 0)
        {
            predecessor = angles.size() - 1;
        }

        double succAngle = angles[successor];
        double predAngle = angles[predecessor];

        if (side)
        {
            succAngle *= -1;
            predAngle *= -1;
        }

        if (!( succAngle < 0 && predAngle < 0))
        {
            continue;
        }

        succAngle = fabs(succAngle);
        predAngle = fabs(predAngle);

        bool successorIsNinety =  succAngle > _lowerNinetyAngleThresh && succAngle < _higherNinetyAngleThresh;
        bool predecessorIsNinety = predAngle > _lowerNinetyAngleThresh && predAngle < _higherNinetyAngleThresh;
        if (successorIsNinety && predecessorIsNinety)
        {
            indices.first = predecessor;
            indices.second = ( successor + 1 ) %angles.size();
            return true;
        }
    }

    return false;
}

bool CrossingAnalyzer::checkStopLine( std::vector< size_t >& flatAngles , BlobDescriptor* laneBlob, CrossingDescriptor& crossing, bool side)
{
    std::vector<double>& angles = laneBlob->angles;
    std::vector< size_t > candidates;
    for (size_t i = 0; i < angles.size(); ++i)
    {
        bool longEnough = laneBlob->lengths[i] > _lowerLengthTresh;
        bool shortEnough = laneBlob->lengths[i] < _higherLengthTresh;

        double absAngle = fabs(laneBlob->angles[i]);
        bool flatAngle = (absAngle > _lowerFlatAngleThresh && absAngle < _higherFlatAngleThresh) || (absAngle > _lowerFlatAngleThresh2 && absAngle < _higherFlatAngleThresh2);

        if (flatAngle)
        {
            if (longEnough&&shortEnough)
            {
                int next = (i + 1) % (laneBlob->contour.size());
                int xPosition;
                double distance;
                if (side)
                {
                    xPosition = std::min(laneBlob->contour[i].x, laneBlob->contour[next].x);
                    distance = laneBlob->centerOfGravity.x - xPosition;
                }
                else
                {
                    xPosition = std::max(laneBlob->contour[i].x, laneBlob->contour[next].x);
                    distance = xPosition - laneBlob->centerOfGravity.x;
                }

                bool leftOfCenter = distance  > _leftOfCenterThresh;
                if (leftOfCenter)
                {
                    candidates.push_back(i);
                }
                else
                {
                    flatAngles.push_back(i);
                }
            }
            else
            {
                flatAngles.push_back(i);
            }
        }
    }

    int indexOfMin = -1;
    double lengthOfMin = 1.0E305;
    for (size_t i = 0; i < candidates.size(); i++)
    {
        double length = laneBlob->lengths[candidates[i]];
        if (length < lengthOfMin)
        {
            indexOfMin = candidates[i];
            lengthOfMin = length;
        }
    }

    if (indexOfMin != -1)
    {
        int next = (indexOfMin + 1) % (laneBlob->contour.size());
        crossing.isReal = true;
        crossing.stopLine = std::pair< cv::Point, cv::Point >(laneBlob->contour[indexOfMin], laneBlob->contour[next]);
        crossing.result = STOPLINE;

        return true;
    }

    return false;
}

bool CrossingAnalyzer::checkArc(std::pair< size_t, size_t >& indices, const BlobDescriptor* laneBlob, bool side)
{
    const std::vector< double >& angles = laneBlob->angles;
    const std::vector< double >& lengths = laneBlob->lengths;
    std::vector< size_t > ninetyAngles;

    for (size_t i = 0; i < angles.size(); ++i)
    {
        double angle = angles[ i ];
        if (side)
        {
            angle *= -1;
        }

        bool angleIsNinety = angle > _lowerNinetyAngleThresh && angle < _higherNinetyAngleThresh;
        if (angleIsNinety)
        {
            ninetyAngles.push_back(i);
        }
    }

    size_t indexOfMaxLength = -1;
    double maxLength = 0;
    for (size_t i = 0; i < ninetyAngles.size(); ++i)
    {
        int currentIndex = ninetyAngles[i];
        double length = lengths[currentIndex];

        if (length > maxLength)
        {
            indexOfMaxLength = currentIndex;
            maxLength = length;
        }
    }

    int successor = (indexOfMaxLength + 1) % angles.size();

    double absAngle = fabs(angles[successor]);
    bool successorHasRightDirection;
    if (side)
    {
        successorHasRightDirection = absAngle > _lowerFlatAngleThresh && absAngle < _higherFlatAngleThresh;
    }
    else
    {
        successorHasRightDirection = absAngle > _lowerFlatAngleThresh2 && absAngle < _higherFlatAngleThresh2;
    }

    bool successorIsLargeEnough = lengths[successor] > _arcLengthThresh;

    if (successorHasRightDirection && successorIsLargeEnough)
    {
        indices.first = indexOfMaxLength;
        indices.second = (successor + 1) % angles.size();

        std::vector< cv::Point > contour = laneBlob->contour;
        sort(contour.begin(),contour.end(),sort_yValContour);

        int testPointY;
        if(side)
        {
            testPointY = contour[contour.size()-1].y;
        }
        else
        {
            testPointY = contour[0].y;
        }

        int distanceLowestPointToSecondIndex = abs( testPointY - laneBlob->contour[successor].y );
        bool distanceToLowestPointIsHigh = distanceLowestPointToSecondIndex > _distanceLowestPointToSecondIndexThresh;

        if(distanceToLowestPointIsHigh)
        {
            return false;
        }

        return true;
    }
    return false;
}

std::pair< cv::Point , cv::Point > CrossingAnalyzer::getVirtualStopLine( size_t firstIndex, size_t secondIndex , BlobDescriptor* laneBlob , bool foundStopLineVertical)
{
    cv::Point anchor;
    if(foundStopLineVertical)
    {
        size_t succ = (firstIndex + 1) % laneBlob->angles.size();
        size_t succSucc = (succ + 1) % laneBlob->angles.size();

        anchor.x = std::floor( ( (laneBlob->contour[succ].x + laneBlob->contour[succSucc].x) / 2.0 ) + 0.5 );
        anchor.y = std::floor( ( (laneBlob->contour[succ].y + laneBlob->contour[succSucc].y) / 2.0 ) + 0.5 );
    }
    else
    {
        size_t anchorPos = (firstIndex + 1) % laneBlob->angles.size();
        anchor = laneBlob->contour[anchorPos];
    }

    if(laneBlob->side == LEFT)
    {
        if(foundStopLineVertical)
        {
            return std::pair< cv::Point , cv::Point >(cv::Point(0,anchor.y + _lowerOffset), cv::Point(2500,anchor.y + _lowerOffset));
        }
        else
        {
            return std::pair< cv::Point , cv::Point >(cv::Point(0,anchor.y + _upperOffset), cv::Point(2500,anchor.y + _upperOffset));
        }
    }
    else
    {
        if(foundStopLineVertical)
        {
            return std::pair< cv::Point , cv::Point >(cv::Point(0,anchor.y + _upperOffset), cv::Point(2500,anchor.y + _upperOffset));
        }
        else
        {
            return std::pair< cv::Point , cv::Point >(cv::Point(0,anchor.y + _lowerOffset), cv::Point(2500,anchor.y + _lowerOffset));
        }
    }
}

bool CrossingAnalyzer::checkOpening(std::vector< size_t >& flatAngles, BlobDescriptor* laneBlob, CrossingDescriptor& crossing, bool side)
{
    std::pair< size_t, size_t > indices;
    bool foundStopLineVertical = checkStopLineVertical(indices, flatAngles, laneBlob, side);
    bool foundArc = checkArc(indices, laneBlob, side);

    if (foundArc && foundStopLineVertical)
    {
        std::cerr << "inconclusive complex laneBoundary !" << std::endl;
        return false;
    }
    else if (foundStopLineVertical || foundArc)
    {
        crossing.isReal = false;
        crossing.result = OPENING;

        crossing.stopLine = getVirtualStopLine( indices.first, indices.second , laneBlob , foundStopLineVertical);

        return true;
    }
    else
    {
        return false;
    }
}

CrossingDescriptor CrossingAnalyzer::fuseSidesToCrossing(const CrossingDescriptor& leftSide, const CrossingDescriptor& rightSide)
{
    CrossingDescriptor crossing;

    // if we have a real stopline at the right side we are finished
    if (rightSide.isReal)
    {
        crossing.result = STOPLINE;
        crossing.isReal = true;
        crossing.stopLine = rightSide.stopLine;
    }
    else if (rightSide.result == OPENING || leftSide.result == OPENING)
    {
        if (rightSide.result == OPENING && leftSide.result != OPENING)
        {
            crossing.result = OPENING;
            crossing.type = 2;
            crossing.stopLine = rightSide.stopLine;
        }
        else if (leftSide.result == OPENING && rightSide.result != OPENING)
        {
            crossing.result = OPENING;
            crossing.type = 1;
            crossing.stopLine = leftSide.stopLine;
        }
        else if (rightSide.result == OPENING && leftSide.result == OPENING)
        {
            crossing.result = OPENING;
            crossing.type = 3;

            std::pair<cv::Point, cv::Point> averagedStopLine;
            averagedStopLine.first.x = (rightSide.stopLine.first + leftSide.stopLine.first).x / 2;
            averagedStopLine.first.y = (rightSide.stopLine.first + leftSide.stopLine.first).y / 2;
            averagedStopLine.second.x = (rightSide.stopLine.second + leftSide.stopLine.second).x / 2;
            averagedStopLine.second.y = (rightSide.stopLine.second + leftSide.stopLine.second).y / 2;

            crossing.stopLine = averagedStopLine;
        }
    }

    return crossing;
}

CrossingDescriptor CrossingAnalyzer::searchCrossings(BlobDescriptor* leftLaneBlob, BlobDescriptor* rightLaneBlob)
{
    CrossingDescriptor rightSide;
    CrossingDescriptor leftSide;

    bool isRealStopLine = false;

    // if we found a right LaneBoundary
    if (rightLaneBlob != NULL)
    {
        if (rightLaneBlob->complexBoundaryIndicator)
        {
            // determine if we have a real stopline
            std::vector< size_t > smallFlatAngles;
            isRealStopLine = checkStopLine(smallFlatAngles, rightLaneBlob, rightSide, true);

            // if it's not a stopline check if we have found an opening on the right side
            if (!isRealStopLine)
            {
                checkOpening(smallFlatAngles, rightLaneBlob, rightSide, true);
            }
        }
    }

    // if we found a left LaneBoundary and didn't find a real stopline on the right side
    if (leftLaneBlob != NULL && !isRealStopLine)
    {
        if (leftLaneBlob->complexBoundaryIndicator)
        {
            std::vector< size_t > smallFlatAngles;
            bool isStopLine = checkStopLine(smallFlatAngles, leftLaneBlob, leftSide, false);

            // if it's no stopline investigate if we have found an opening on the left side
            if (!isStopLine)
            {
                checkOpening(smallFlatAngles, leftLaneBlob, leftSide, false);
            }
        }
    }

    // fuse our results into one final result
    CrossingDescriptor crossing = fuseSidesToCrossing(leftSide, rightSide);

    return crossing;
}

// 1 = left and straight
// 2 = right and straight
// 3 = right, left and straight
// 4 = right and left
int CrossingAnalyzer::classifyCrossings(std::vector< BlobDescriptor >& blobs)
{
    std::vector< BlobDescriptor* > blobsToAnalyze;
    for(size_t i = 0 ; i < std::min( static_cast< int >( blobs.size() ) , 3 ) ;++i)
    {
        bool veryLineLike = blobs[i].principalAxisLengthRatio > _veryLineLikeThresh;
        bool notComplex = !blobs[i].complexBoundaryIndicator;
        bool large = blobs[i].lengthContour > _minOuterBoundaryLength;

        if( large && veryLineLike && notComplex )
        {
            blobsToAnalyze.push_back(&blobs[i]);
        }
    }

    if(blobsToAnalyze.size() > 0 )
    {
        size_t indexOfMin;
        double maxLength = 0;
        for(size_t i = 0 ; i < blobsToAnalyze.size();++i)
        {
            if(blobsToAnalyze[i]->lengthContour > maxLength)
            {
                maxLength = blobsToAnalyze[i]->lengthContour;
                indexOfMin = i;
            }
        }

        BlobDescriptor* blobToAnalyze = blobsToAnalyze[ indexOfMin ];

        double angle = fabs(blobToAnalyze->angleOfMainDirection);
        bool angleIsNinety = angle > _lowerNinetyAngleThresh && angle < _higherNinetyAngleThresh;
        if(angleIsNinety)
        {
            if(blobToAnalyze->side == RIGHT)
            {
                return 1;
            }
            else if(blobToAnalyze->side == LEFT)
            {
                return 2;
            }
        }
        else
        {
            return 3;
        }
    }
    else
    {
        return 4;
    }
    return 0;
}

