#ifndef BEZIER_H
#define BEZIER_H
#include "XMesh.h"
#include <map>

typedef  XPoint<double, 3> Point3;
typedef  XPoint<double, 2> Point2;


template<class T_P> 
class BezierCurve
{
public:
	int m_d;
	int m_n;
	int m_dim;
	std::vector<T_P> m_P;
	BezierCurve() {};
	BezierCurve(int d);
	
	
	std::vector<int> m_map;
	std::vector<std::vector<T_P> > curves;
	T_P recursive(int i0, int ik, double t);

	void generateRC(int divN);
	
	T_P& operator [](const size_t i)
	{
		return m_P[i];
	};

	
};

typedef  BezierCurve<Point2> BezierCurve2;
typedef  BezierCurve<Point3> BezierCurve3;

#endif
