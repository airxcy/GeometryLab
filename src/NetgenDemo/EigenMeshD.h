#ifndef EIGENMESHF_H
#define EIGENMESHF_H
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

//#include "Model/Shell/TopoMeshDef.hh"
//#include "BaseCore/EigenExt/MinQuadWithFixed.h"
#include "igl/min_quad_with_fixed.h"


class EigenMeshD
{
public:
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	std::vector<std::vector<double > > stdV;
	std::vector<std::vector<double > > stdTC;
	std::vector<std::vector<double > > stdN;
	std::vector<std::vector<int > > stdF;
	std::vector<std::vector<int > > stdFTC;
	std::vector<std::vector<int > > sdtFN;

	std::vector< std::vector<int> > v2f, v2v;
	unsigned char* connM{nullptr};
	//unsigned int* e2f{nullptr};

	std::vector< std::vector<int> > components;
	void loadOBJ(std::string fpath);
	void loadSTL(std::string fpath);
	void map_v2v();
	void map_v2f();
	void updateTplgy();
	void seedBndLoop(std::vector<int>& bnd, int seed, int seed2);

	Eigen::MatrixXd deformedV;

	Eigen::VectorXi boundryIdx;
	Eigen::MatrixXd boundryPos;
	std::array < Eigen::SparseMatrix<double>,3 > LCotVec, MassVec, QmatData;
	Eigen::MatrixXd bc;
	Eigen::VectorXi b;
	std::array <  igl::min_quad_with_fixed_data<double>* , 3> mq_data;
	//std::array < POP_NS::eigenext::MinQuadWithFixedData<double> ,3> mq_data;



	void bfs_components();


	void calLaplacion(int k);

	void removeBIdx(std::vector<int>& indices);

	void setUpB(int k);
	void setUpBc();
	bool solveHarmonic(int k);

};

void directDeltaMushPrecompute(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& W, Eigen::MatrixXd& Omega,
	int p = 40,
	float lambda = 1.2,
	float kappa = 1,
	float alpha = 0.5);
//int p = 40;
//float lambda = 1; // 0 < lambda
//float kappa = 0.5; // 0 < kappa < lambda
//float alpha = 0.6; // 0 <= alpha < 1

void directDeltaMush(Eigen::MatrixXd& V, const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& T_list, Eigen::MatrixXd& Omega, Eigen::MatrixXd& U);


#endif // !EIGENMESHF_H
