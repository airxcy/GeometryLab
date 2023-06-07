#ifndef XMESH_H
#define XMESH_H


#include <vector>

template<class T_REAL, int DIM>
struct XPoint
{
	T_REAL coord[DIM];
	T_REAL& operator[](int i) { return coord[i]; }
	XPoint operator* (double a) 
	{ 
		XPoint<T_REAL, DIM> p;
		for (int i = 0; i < DIM; i++)
			p[i] = coord[i] * a;
		return p;
	}

	XPoint operator+ (XPoint& a)
	{
		XPoint<T_REAL, DIM> p;
		for (int i = 0; i < DIM; i++)
			p[i] = coord[i] +a[i];
		return p;
	}
};


template<class T_REAL,class T_IDX>
class XMesh
{
public:
	std::vector<std::vector<T_REAL > > V;
	std::vector<std::vector<T_IDX > > F;
};




#endif // !XMESH_H

