#ifndef L_SMOOTH_H
#define L_SMOOTH_H

#include <Eigen/Sparse>

class LaplacianSmoothing
{
public:
	Eigen::SparseMatrix<double> L;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	void init(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
	void deltaL(double delta);
};

#endif
