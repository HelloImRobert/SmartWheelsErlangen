#ifndef SPLINEBOUNDARYCALCS
#define SPLINEBOUNDARYCALCS

#include "math.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "Polynomial.h"
#include "PolynomialRootFinder.h"


namespace splineBoundaryCalcs
{

// transforms a vector of points from intersection line system to front axis system
std::vector< cv::Point2d > vTil( const std::vector< cv::Point2d >& ilP,
                                 const double intersecAngle,
                                 const double intersecDistance );


// transforms a vector of points from front axis system to intersection line system
std::vector< cv::Point2d > ilTv( const std::vector< cv::Point2d >& vP,
                                 const double intersecAngle,
                                 const double intersecDistance );


// calculate spline coefficients for a boundary (vector of cv::Point2d)
// structure of return value:
// 3. dim( Point2d ): coeff of x- and y-component of spline segment
// 2. dim( vector ) : iterating through order of polynom variable (=> coeff0 to coeff3)
// 1. dim( vector ) : iterating through spline segments
std::vector< std::vector< cv::Point2d > > calcSplineCoeffs( const std::vector< cv::Point2d >& boundary,
                                                            const double tau );


// calculate zeros of a boundary (zeros of a set of c-splines)
// structure of return value:
// 3. dim( Point2d ): x,y- coord of zero
// 2. dim( vector ) : iterating through max. 3 zeros per spline segment
// 1. dim( vector ) : interating through spline segments
std::vector< cv::Point2d > calcSplineZeros( const std::vector< std::vector< cv::Point2d > >& splineCoeffs );


// calculates angle between two vectors from tip to tip ( positiv for rotationg from x to y )
double angleBetweenVectors( const cv::Point2d& vec1,
                            const cv::Point2d& vec2 );


// start at a given position in a point vector and returns position of (closest) point after traveling given distance
int travelGivenDistance( const double distance,
                         const int startElement,
                         const std::vector< cv::Point2d >& pointVec );


// inserts points in spline with given distance and returns new spline and position-length-vector
// if distance set to below zero: no additional points inserted => only length of segments calculated
std::vector< cv::Point2d > insertPoints( std::vector< std::pair< size_t, double > >& segmentLengths,
                                         const std::vector< cv::Point2d >& srcSpline,
                                         const double distance );


// compare two spline segments in number of points included
bool alignLengthes( std::vector< std::pair< size_t, size_t > >& segments,
                    const std::vector< std::pair< size_t, double > >& lengthes );


// compare two spline segments in length (mm)
bool larger_splineSegment( const std::pair< size_t, double >& seg1,
                           const std::pair< size_t, double >& seg2 );


// reverses spline if first element is further away than last element
bool correctSplineDirection( std::vector< cv::Point2d >& spline );


// divides spline into segments that twist more than breakAngle between to points and returns the longest segments
// having a starting angle of less than maxDirectionAngle to x-axis of vehicle and beginning closer than
// maxDistance to vehicle
std::vector< std::pair< size_t, size_t > > extractPlausibleSegment( std::vector< cv::Point2d >& plausibleSplineSegment,
                                                                    const std::vector< cv::Point2d >& spline,
                                                                    const double breakAngle,
                                                                    const double maxDirectionAngle,
                                                                    const double maxDistance);


// return normal vector to 2D vector counterclockwise
cv::Point2d getNormal2dVector( cv::Point2d vec );


// calculate tangents on vector of points by using points before and after current point (linear approx)
std::vector< cv::Point2d > getTangents( const std::vector< cv::Point2d >& vec );



bool getLineIntersection( cv::Point2d& intersectionPoint,
                          const std::vector< cv::Point2d >& linePoints,
                          const cv::Point2d& point,
                          const cv::Point2d& normal );

}


#endif // SPLINEBOUNDARYCALCS
