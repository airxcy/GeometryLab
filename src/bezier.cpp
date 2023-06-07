#include "bezier.h"
#include <iostream>
template<class T_P>
BezierCurve<T_P>::BezierCurve(int d)
{
	m_d = d;
	m_n = d + 1;
	m_P.resize(m_n);
	m_dim = sizeof(m_P[0]) / sizeof(m_P[0][0]);
}

template<class T_P>
T_P BezierCurve<T_P>::recursive(int i0, int ik, double t)
{
	T_P p = m_P[i0];
	if (ik > i0)
		p = recursive(i0, ik - 1, t) * (1 - t) + recursive(i0 + 1, ik, t) * t;
	

	if (m_map[i0 * m_n + ik] > 0)
		curves[m_map[i0 * m_n + ik] - 1].push_back(p);
	else
	{
		curves.push_back({ p });
		m_map[i0 * m_n + ik] = curves.size();
	}
	return  p;
}

template<class T_P>
void BezierCurve<T_P>::generateRC(int divN)
{
	m_map.resize(m_n*m_n,0);
	for (auto& curve : curves)
		curve.clear();
	double div = 1.0 / divN, t = 0;
	for (int i = 0; i < divN; i++)
	{
		recursive(0, m_d, t);
		t += div;
	}
}


template class BezierCurve<Point2>;
template class BezierCurve<Point3>;