#ifndef NTORUS_H
#define NTORUS_H

#include "HarmonicDeformation.h"
#include "LaplacianSmoothing.h"
#include "XMesh.h"


class TorusN :  public XMesh<double,int>
{
public:
	int genus = 0;
	double R = 4;
	double r = 1;
	double cDist = 8;
	

	int nANG = 64;
	int nang = 32;
	double DistDAng;
	double DistAng;
	double dANG;
	double dang;
	double startANG;
	int halfrange;
	int range1;
	double angBorder1;
	int range2;
	double angBorder2;
	double dMid;
	std::vector< std::vector< int>  > innerBndIdx;
	std::vector<int> stdb;
	void clear();
	void buildV();
	void buildMesh(int G);
	void triBnd(std::vector<int>& bnd, double area, std::vector< std::vector< int > >& addF);
	
	
	HarmonicDeformation deformer;
	Eigen::MatrixXd deformedV;
	void HarmonicShape();

	LaplacianSmoothing Lsmoother;
	void LaplacianSmooth(double delta);
	
	Eigen::MatrixXd clrs;
};
#endif // !GENUS_N
