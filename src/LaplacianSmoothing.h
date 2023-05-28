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
	void init(std::vector<std::vector<double> >&V,std::vector< std::vector<int> >& F);
	void deltaL(double delta);
	void deltaLReg(double delta);
};

#endif
