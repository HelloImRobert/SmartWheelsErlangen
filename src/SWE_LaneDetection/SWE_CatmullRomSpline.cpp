#include "SWE_CatmullRomSpline.h"

    CatMullRomSpline::CatMullRomSpline(const std::vector<Point2d> &points , double tkParameter )
		: _TkParameter( tkParameter )
	{
		if (points.size() < 2)
		{
			Exception e;
			e.msg = "Catmull-Rom-Splines need at least two points to be constructed";
			throw e;
		}
		_Points = points;
		_Points.insert(_Points.begin(), _Points[0]);
		_Points.push_back(_Points[_Points.size() - 1]);
	}

	CatMullRomSpline::~CatMullRomSpline(){}

    void CatMullRomSpline::addSplinePoint(const Point2d &point )
	{
		_Points.push_back( point );
	}

	Point CatMullRomSpline::getInterpolatedSplinePoint( double t )
	{
		size_t section = floor(t);
		if (section < _Points.size() - 3)
		{
			t -= section;
		}
		else
		{
			section = _Points.size() - 4;
			t = 1.0;
		}
		

		double tCubed = pow(t,3);
		double tSquared = pow(t, 2);

		std::vector< double > coefficients( 4 , _TkParameter );
		coefficients[ 0 ] *= -tCubed + 2 * tSquared - t;
		coefficients[ 1 ] *= 3 * tCubed - 5 * tSquared + 2;
		coefficients[ 2 ] *= -3 * tCubed + 4 * tSquared + t;
		coefficients[ 3 ] *= tCubed - tSquared;

		Point2d result;
		for (size_t i = 0; i < 4; i++)
		{
			result += coefficients[i] * (Point2d) _Points[ i + section ];
		}

		return Point(std::floor(result.x + 0.5), std::floor(result.y + 0.5));
	}

	size_t CatMullRomSpline::getNumberOfPoints()
	{
		return _Points.size();
	}

	Point CatMullRomSpline::getPoint( size_t i )
	{
		return _Points.at( i );
	}
