#ifndef GENUS_N
#define GENUS_N

#include "Manifold.h"
#include "glm/vec3.hpp"
class GenusN: public Manifold
{
public:

	int genus = 0;
	double R=4;
	double r=1;
	double cDist = 8;
	Eigen::MatrixXd centers;

	int nANG= 64;
	int nang = 32;
	double DistDAng;
	double DistAng ;
	double dANG ;
	double dang ;
	double startANG ;
	int halfrange ;
	int range1 ;
	double angBorder1 ;
	int range2;
	double angBorder2 ;
	double dMid;
	std::vector< std::vector< int>  > innerBndIdx;
	

	Eigen::MatrixXd N;
	Eigen::SparseMatrix<double> L;

	void paramFromG2(XMesh& dbTorus);
	void buildV();
	void buildMesh(int G);
	void triBnd(std::vector<int>& bnd,double area, std::vector< std::vector< int > >& addF);
	void HarmonicShape();
	void setUpLaplacian();
	void LaplacianSmooth();
	void clear();
	Eigen::MatrixXd clrs;

};	

#endif // !GENUS_N
