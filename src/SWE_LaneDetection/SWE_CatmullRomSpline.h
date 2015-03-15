#ifndef _SWE_CATMULL_ROM_SPLINE_H_
#define _SWE_CATMULL_ROM_SPLINE_H_

#include "opencv2/opencv.hpp"

	using namespace cv;

	class CatMullRomSpline
	{
	public:
        CatMullRomSpline( const std::vector< Point2d >& points , double tkParameter = 0.5 );
		virtual ~CatMullRomSpline();

        void addSplinePoint( const Point2d& point);
		Point getInterpolatedSplinePoint( double t );
		size_t getNumberOfPoints();
		Point getPoint( size_t i );
	private:
        std::vector< Point2d > _Points;
		double _Delta_t;
		double _TkParameter;
	};

#endif
