#ifndef L_SMOOTH_H
#define L_SMOOTH_H

#include <Eigen/Sparse>

class LaplacianSmoothing
{
public:
	Eigen::SparseMatrix<double> L;
	void init();
	void deltaL(double delta);
};

#endif
