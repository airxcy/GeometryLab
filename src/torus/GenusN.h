#ifndef GENUS_N
#define GENUS_N

#include "Manifold.h"

class GenusN: public Manifold
{
public:

	int genus = 0;
	int nHole = 1;
	double R=4;
	double r=1;
	double cDist = 8;
	Eigen::MatrixXd centers;

	int nANG= 20;
	int nang = 20;

	void paramFromG1(XMesh& dbTorus);
	void buildMesh();
	Eigen::MatrixXd clrs;

};	

#endif // !GENUS_N
