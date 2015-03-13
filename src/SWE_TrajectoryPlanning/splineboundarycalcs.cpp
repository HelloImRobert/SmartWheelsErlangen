#include "splineboundarycalcs.h"
#include "stdafx.h"


// transforms a vector of points from intersection line system to front axis system
std::vector< cv::Point2d > splineBoundaryCalcs::vTil( const std::vector< cv::Point2d >& ilP,
                                                      const double intersecAngle,
                                                      const double intersecDistance )
{
    std::vector< cv::Point2d > vP;
    for( unsigned int i = 0; i < ilP.size(); i++ )
    {
        vP.push_back( cv::Point2d( ilP[i].x*cos(intersecAngle)-ilP[i].y*sin(intersecAngle)+intersecDistance,
                                   ilP[i].y*cos(intersecAngle)+ilP[i].x*sin(intersecAngle) ) );
    }


    return vP;
}



// transforms a vector of points from front axis system to intersection line system
std::vector< cv::Point2d > splineBoundaryCalcs::ilTv( const std::vector< cv::Point2d >& vP,
                                                      const double intersecAngle,
                                                      const double intersecDistance )
{
    std::vector< cv::Point2d > ilP;
    for( unsigned int i = 0; i < vP.size(); i++ )
    {
        ilP.push_back( cv::Point2d( vP[i].x*cos(intersecAngle)+vP[i].y*sin(intersecAngle)-intersecDistance*cos(intersecAngle),
                                    vP[i].y*cos(intersecAngle)-vP[i].x*sin(intersecAngle)+intersecDistance*sin(intersecAngle) ) );
    }


    return ilP;
}



// calculate spline coefficients for a boundary (vector of cv::Point2d)
// structure of return value:
// 3. dim( Point2d ): coeff of x- and y-component of spline segment
// 2. dim( vector ) : iterating through order of polynom variable (=> coeff0 to coeff3)
// 1. dim( vector ) : iterating through spline segments
std::vector< std::vector< cv::Point2d > > splineBoundaryCalcs::calcSplineCoeffs( const std::vector< cv::Point2d >& boundary,
                                                                                 const double tau )
{
    // calulate tangents on spline points for catmull rom splines
    std::vector< cv::Point2d > tangents;
    tangents.resize( boundary.size() );

    // set tangents for first and last point in boundary (here: natural boundary condition)
    tangents[0] = cv::Point2d( 0, 0 );
    tangents[boundary.size()-1] = cv::Point2d( 0, 0 );

    // calculate remaining tangents
    for( unsigned int i = 1; i < boundary.size()-1; i++ )
    {
        tangents[i] = tau*( boundary[i+1] - boundary[i-1] );
    }

    // calculate splineCoeffs (return value)
    std::vector< std::vector < cv::Point2d > > splineCoeffs;
    splineCoeffs.resize( boundary.size()-1 );

    // claculate spline coeffs
    for( unsigned int i = 0; i < splineCoeffs.size(); i++ )
    {
        splineCoeffs[i].resize(4);

        splineCoeffs[i][0] = boundary[i];
        splineCoeffs[i][1] = tangents[i];
        splineCoeffs[i][2] = 3*boundary[i+1]-3*boundary[i]-tangents[i+1]-2*tangents[i];
        splineCoeffs[i][3] = -2*boundary[i+1]+2*boundary[i]+tangents[i+1]+tangents[i];
    }


    return splineCoeffs;
}



// calculate zeros of a boundary (zeros of a set of c-splines)
// structure of return value:
// 3. dim( Point2d ): x,y- coord of zero
// 2. dim( vector ) : iterating through max. 3 zeros per spline segment
// 1. dim( vector ) : interating through spline segments
std::vector< cv::Point2d > splineBoundaryCalcs::calcSplineZeros( const std::vector< std::vector< cv::Point2d > >& splineCoeffs )
{
    // define object to store x,y-Coords of zeros of splines for all segments (return value)
    std::vector< cv::Point2d > splineZeros;
    //    splineZeros.resize( splineCoeffs.size() );

    // calculate zeros of spline segments
    // iterating through spline segments
    for( int i = 0; i < splineCoeffs.size(); i++ )
    {
        double coeffs[4] = {};
        // get coeffs of y-part of spline in current segment
        for( unsigned int j = 0; j < 4; j++ )
        {
            coeffs[j] = splineCoeffs[i][j].y;
        }

        // make y-polynomial
        int degree = 3;
        Polynomial ySplineSegment(coeffs, degree);

        // variables to real and imaginary components of polynomial roots
        std::vector<double> realRoots, imagRoots;
        int rootCount = 0;

        realRoots.resize(degree);
        imagRoots.resize(degree);

        double* realRootsPtr = &realRoots[0];
        double* imagRootsPtr = &imagRoots[0];

        // if polynomial degenerates to lower order, exclude corresponding zeros
        if( fabs( splineCoeffs[i][3].y ) < 1e-8 )
        {
            degree--;
            imagRoots[2] = 1000.0;
            if( fabs( splineCoeffs[i][2].y ) < 1e-8 )
            {
                degree--;
                imagRoots[1] = 1000.0;
                if( fabs( splineCoeffs[i][1].y ) < 1e-8 )
                {
                    degree--;
                    imagRoots[0] = 1000.0;
                }
            }
        }
        LOG_ERROR(cString::Format( "MBMT: test11.1.0"));
        // calculate roots
        ySplineSegment.FindRoots(realRootsPtr, imagRootsPtr, &rootCount);
        LOG_ERROR(cString::Format( "MBMT: test11.1.1"));
        // if polynomial degenerates to lower order, exclude corresponding zeros
        if( fabs( splineCoeffs[i][3].y ) < 1e-8 )
        {
            imagRoots[2] = 1000.0;
            if( fabs( splineCoeffs[i][2].y ) < 1e-8 )
            {
                imagRoots[1] = 1000.0;
                if( fabs( splineCoeffs[i][1].y ) < 1e-8 )
                {
                    imagRoots[0] = 1000.0;
                }
            }
        }
        LOG_ERROR(cString::Format( "MBMT: test11.1.2"));

        // for 3 possible roots of 3rd order polynomial
        for( int j = 0; j < degree; j++ )
        {
            // if imaginary part of root is small => real root => zero of polynomial
            if( fabs(imagRoots[j]) < 1e-4 )
            {
                // if zero lies within s = [0, 1[ ...
                if( (0.0 <= realRoots[j]) && (realRoots[j] < 1) )
                {
                    // ... zero lies in current spline segment => is zero on boundary
                    // => calculate and store x- and z-coords of boundary zero
                    cv::Point2d currentSplineZero;
                    currentSplineZero.x = splineCoeffs[i][0].x + splineCoeffs[i][1].x*realRoots[j]
                            + splineCoeffs[i][2].x*pow( realRoots[j], 2 ) + splineCoeffs[i][3].x*pow( realRoots[j], 3 );
                    currentSplineZero.y = splineCoeffs[i][0].y + splineCoeffs[i][1].y*realRoots[j]
                            + splineCoeffs[i][2].y*pow( realRoots[j], 2 ) + splineCoeffs[i][3].y*pow( realRoots[j], 3 );
                    //                    splineZeros[i].push_back( currentSplineZero );
                    splineZeros.push_back( currentSplineZero );
                }
            }
        }
        LOG_ERROR(cString::Format( "MBMT: test11.1.3"));
    }


    LOG_ERROR(cString::Format( "MBMT: test11.1.4"));
    return splineZeros;
}



// calculates angle between two vectors from tip to tip ( positiv for rotationg from x to y )
double splineBoundaryCalcs::angleBetweenVectors( const cv::Point2d& vec1,
                                                 const cv::Point2d& vec2 )
{
    return (atan2( vec2.y, vec2.x ) - atan2( vec1.y, vec1.x ));
}



// start at a given position in a point vector and returns position of (closest) point after traveling given distance
int splineBoundaryCalcs::travelGivenDistance( const double distance,
                                              const int startElement,
                                              const std::vector< cv::Point2d >& pointVec )
{
    double traveledDistance = 0.0;
    int position = startElement;

    while ( distance > traveledDistance )
    {
        traveledDistance += norm( pointVec[position+1] - pointVec[position]);
        position++;
    }

    if ( (position - startElement) > 1 )
    {

        return position-1;
    }
    else
    {

        return position;
    }
}



// inserts points in spline with given distance and returns new spline and position-length-vector
// if distance set to below zero: no additional points inserted => only length of segments calculated
std::vector< cv::Point2d > splineBoundaryCalcs::insertPoints( std::vector< std::pair< size_t, double > >& segmentLengths,
                                                              const std::vector< cv::Point2d >& srcSpline,
                                                              const double distance )
{
    // return value
    std::vector< cv::Point2d > extendedSpline;

    // return if source spline is empty
    if( srcSpline.empty() )
    {
        segmentLengths.clear();

        return extendedSpline;
    }

    // final size of return value
    size_t absoluteSize = srcSpline.size();
    LOG_ERROR(cString::Format( "MBMT: test2.1"));

    // store for number of points to insert in corresponding segments
    std::vector < unsigned int > insertCount;
    // store for direction of point insertion in corresponding segments
    std::vector < cv::Point2d > insertVector;
    insertCount.resize( srcSpline.size()-1 );
    insertVector.resize( srcSpline.size()-1 );

    // for all spline segments
    for( size_t i = 0; i < srcSpline.size()-1; i++ )
    {
        // get vector between the two segment points
        cv::Point2d currentVector = srcSpline[i+1] - srcSpline[i];
        // calculate length
        double currentLength = cv::norm( currentVector );

        // if distance is not greater than zero => no points to insert
        if( distance > 0.0 )
        {
            // scale current vector to length of distance
            currentVector = distance * ( currentVector * ( 1/currentLength ) );
            // store point insert direction in vector
            insertVector[i] = currentVector;

            // calculate number of points to insert
            double modulus = fmod( currentLength, distance );
            double multipleLength = currentLength - modulus;
            // store number of points to insert
            insertCount[i] = round( fabs( multipleLength / distance ) );
            // if last inserted point will lie very close to next trajectory point ...
            if( modulus < 1e-1 )
            {
                // ... do not insert this last point to avoid inf and nan values
                if( insertCount[i] > 0 )
                {
                    insertCount[i]--;
                }
            }
            // add count to return value size
            absoluteSize += insertCount[i];
        }
        else
        {
            insertCount[i] = 0;
        }
        LOG_ERROR(cString::Format( "MBMT: test2.2"));
        LOG_ERROR(cString::FromInt32( static_cast< int >( insertCount[i] ) ));
    }

    // allocate memory for return values
    extendedSpline.reserve( absoluteSize );
    segmentLengths.reserve( absoluteSize-1 );
    size_t n = 0;
    // build vector with inserted points
    for( size_t i = 0; i < srcSpline.size()-1; i++, n++ )
    {
        extendedSpline.push_back( srcSpline[i] );
        for( size_t j = 0; j < insertCount[i]; j++, n++ )
        {
            cv::Point2d pointToInsert;
            pointToInsert = extendedSpline.back() + insertVector[i];
            extendedSpline.push_back( pointToInsert );

            // insert position and length of current segment in return parameter
            segmentLengths.push_back( std::pair< size_t, double >( n, distance ) );
        }
        // insert position and length of current segment in return parameter
        segmentLengths.push_back( std::pair< size_t, double >( n, cv::norm( srcSpline[i+1] - extendedSpline.back()) ) );
    }
    // add last point of input spline
    extendedSpline.push_back( srcSpline.back() );


    // return new spline with inserted points
    return extendedSpline;
}



// sort segments defined by start and end position for geometric length (longest first)
bool splineBoundaryCalcs::alignLengthes( std::vector< std::pair< size_t, size_t > >& segments,
                                         const std::vector< std::pair< size_t, double > >& lengthes )
{
    if( segments.size() != lengthes.size() )
    {
        throw 1;
    }
    std::vector< std::pair< size_t, size_t > > tempSegments( segments );

    for( size_t i = 0; i < segments.size(); i++ )
    {
        segments[i] = tempSegments[ lengthes[i].first ];
    }


    return 1;

}



// compare two spline segments in length (mm)
bool splineBoundaryCalcs::larger_splineSegment( const std::pair< size_t, double >& seg1,
                                                const std::pair< size_t, double >& seg2 )
{
    return ( seg1.second > seg2.second );
}



// reverses spline if first element is further away than last element
bool splineBoundaryCalcs::correctSplineDirection( std::vector< cv::Point2d >& spline )
{
    if( spline.at( 0 ).x > spline.back().x )
    {
        std::reverse( spline.begin(), spline.end() );

        return 0;
    }


    return 1;
}



// divides spline into segments that twist more than breakAngle between to points and returns the longest segments
// having a starting angle of less than maxDirectionAngle to x-axis of vehicle and beginning closer than
// maxDistance to vehicle
std::vector< std::pair< size_t, size_t > > splineBoundaryCalcs::extractPlausibleSegment( std::vector< cv::Point2d >& plausibleSplineSegment,
                                                                                         const std::vector< cv::Point2d >& spline,
                                                                                         const double breakAngle,
                                                                                         const double maxDirectionAngle,
                                                                                         const double maxDistance)
{
    // generate return value
    std::vector< std::pair< size_t, size_t > > plausibleSegments;
    // return emtpy vector if input spline is empty
    if( spline.empty() )
    {
        plausibleSplineSegment.clear();
        return plausibleSegments;
    }

    {
        size_t segmentStart = 0;
        size_t position = 1;

        // set segment start and end points
        for( ; position < spline.size()-1; position++ )
        {
            double currentAngle = angleBetweenVectors( spline[position+1]-spline[position], spline[position]-spline[position-1] );
            if( fabs(currentAngle) > breakAngle )
            {
                plausibleSegments.push_back( std::pair< size_t, size_t >( segmentStart, position ));
                segmentStart = position;
            }
        }
        // set last point of spline as endpoint of last segment in spline
        plausibleSegments.push_back( std::pair< size_t, size_t >( segmentStart, position ));
    }

    // sort plausible segments for length

    // calculate length of every input spline segment
    std::vector< std::pair< size_t, double > > lengthsSegments;
    insertPoints( lengthsSegments, spline, -1.0 );
    // calculate lengthes of plausible segments

    std::vector< std::pair< size_t, double > > lengthsPlausibleSegments;
    lengthsPlausibleSegments.resize( plausibleSegments.size() );

    // for all plausible segments
    for( size_t i = 0; i < plausibleSegments.size(); i++ )
    {
        // store iterator of current segment
        lengthsPlausibleSegments[i].first = i;
        // for all spline segments in current plausible segment ...
        for( size_t j = plausibleSegments[i].first; j < plausibleSegments[i].second; j++ )
        {
            // ... add up lengthes of spline segments
            lengthsPlausibleSegments[i].second += lengthsSegments[j].second;
        }
    }

    // sort elements of vector of plausible segment lengthes for higher lenghtes
    std::sort( lengthsPlausibleSegments.begin(), lengthsPlausibleSegments.end(), larger_splineSegment );
    // sort vector of plausible segments for higher lengthes
    alignLengthes( plausibleSegments, lengthsPlausibleSegments );

    // for all plausible spline segments
    std::vector< size_t > toErase;
    for( size_t i = 0; i < plausibleSegments.size(); i++ )
    {
        size_t selectedPoint;

        bool isStart = true;
        // check wether end or start point of plausible segment is closer to vehicle
        double distanceStart = spline[plausibleSegments[i].first].x;
        double distanceEnd   = spline[plausibleSegments[i].second].x;
        if ( distanceStart <= distanceEnd )
        {
            selectedPoint = plausibleSegments[i].first;
            isStart = true;
        }
        else
        {
            selectedPoint = plausibleSegments[i].second;
            isStart = false;
        }

        // check if angle of beginning of plausible segment is less than maxDirectionAngle
        // and if beginning of plausible segment is closer than maxDistance
        double directionAngle = -1.0;
        double distancePlausibleSegment = -1.0;
        if( isStart == true )
        {
            directionAngle = fabs( angleBetweenVectors( (spline[selectedPoint+1] - spline[selectedPoint]), cv::Point2d( 1, 0 ) ) );
            distancePlausibleSegment = distanceStart;
        }
        else
        {
            directionAngle = fabs( angleBetweenVectors( (spline[selectedPoint-1] - spline[selectedPoint]), cv::Point2d( 1, 0 ) ) ) ;
            distancePlausibleSegment = distanceEnd;
        }
        if( ( directionAngle > maxDirectionAngle ) || ( distancePlausibleSegment > maxDistance ) )
        {
            toErase.push_back( i );
        }
    }

    // delete segments not fullfilling maxDirectionAngle criteria
    for( size_t i = 0; i < toErase.size(); i++ )
    {
        plausibleSegments.erase( plausibleSegments.begin() + toErase[i] );
    }

    // write plausible segment as spline in return parameter
    plausibleSplineSegment.clear();
    if( plausibleSegments.size() > 0 )
    {
        for( size_t i = plausibleSegments.at(0).first; i <= plausibleSegments.at(0).second; i++ )
        {
            plausibleSplineSegment.push_back( spline[i] );
        }
    }


    // return positions of start and end points of plausible segments of input spline
    return plausibleSegments;
}



// return normal vector to 2D vector with y direction always positive (looks to the left)
cv::Point2d splineBoundaryCalcs::getNormal2dVector( cv::Point2d vec )
{
    cv::Point2d tempVec( (-1.0)*vec.y, vec.x );
    //    if( tempVec.y < 0.0 )
    //    {
    //        tempVec = tempVec * ( -1.0 );
    //    }
    tempVec = tempVec * ( 1  / cv::norm( vec ) );

    return tempVec;
}



// calculate tangents on vector of points by using points before and after current point (linear approx)
std::vector< cv::Point2d > splineBoundaryCalcs::getTangents( const std::vector< cv::Point2d >& vec )
{
    std::vector< cv::Point2d > tangents;

    for( size_t i = 0; i < vec.size(); i++ )
    {
        cv::Point2d tempTangent;

        //calc first tangent
        if( i == 0)
        {
            tempTangent = vec[i+1] - vec[i];
        }
        //calc last tangent
        else if( i == vec.size()-1 )
        {
            tempTangent = vec[i] - vec[i-1];
        }
        //calc remaining tangents
        else
        {
            tempTangent = ( vec[i+1] - vec[i] ) + ( vec[i] - vec[i-1] );
        }

        // normalize tangent length
        tempTangent = tempTangent*( 1/cv::norm( tempTangent ) );
        // add to tangent vector
        tangents.push_back( tempTangent );
    }


    return tangents;
}

bool splineBoundaryCalcs::getLineIntersection( cv::Point2d& intersectionPoint,
                                               const std::vector< cv::Point2d >& linePoints,
                                               const cv::Point2d& point,
                                               const cv::Point2d& normal )
{
    // intitialize variables
    const cv::Point2d& o1( point );
    const cv::Point2d& d1( normal );

    cv::Point2d o2;
    cv::Point2d p2;

    cv::Point2d x;
    cv::Point2d d2;

    // temp variable
    cv::Point2d tempIntersecPoint;

    for( size_t i = 0; i < linePoints.size()-1; i++ )
    {
        // determine points of current line segment
        o2 = linePoints[i];
        p2 = linePoints[i+1];

        // calculate vector of current line segment
        x = o2 - o1;
        d2 = p2 - o2;

        // calculate intersection point
        double cross = d1.x*d2.y - d1.y*d2.x;
        if ( fabs(cross) > 1e-8 )
        {
            double t1 = (x.x * d2.y - x.y * d2.x)/cross;
            tempIntersecPoint = (o1 + d1 * t1);

            // get ratio between current line segment length and intersection point vector length
            double ratio = ( cv::norm( tempIntersecPoint - o2 ) * ( 1 / cv::norm( p2 - o2 ) ) );
            // get angle between current line segment and intersection point vector
            double angle = angleBetweenVectors( tempIntersecPoint - o2, p2 - o2 );

            // if vector goes in same direction
            if( ( ratio < 1.0 ) && ( fabs( angle ) < CV_PI/180 ) )
            {
                intersectionPoint = tempIntersecPoint;

                return true;
            }
            else
            {
                intersectionPoint = cv::Point2d( 0, 0 );

                //return false;
            }
        }
        else
        {
            intersectionPoint = cv::Point2d( 0, 0 );

            //return false;
        }
    }
    // if no intersection point is found return false
    intersectionPoint = cv::Point2d( 0, 0 );


    return false;

}
