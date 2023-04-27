#ifndef GENUS_N
#define GENUS_N

#include "Manifold.h"

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

	std::vector< std::vector< std::vector<double>  > > innerBnd;

	void paramFromG2(XMesh& dbTorus);
	void buildMesh(int G);
	void clear();
	Eigen::MatrixXd clrs;

};	

#endif // !GENUS_N
