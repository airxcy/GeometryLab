#ifndef MANIFOLD_H
#define MANIFOLD_H
#include "XMesh.h"

#include <Eigen/Sparse>
#include <Eigen/Geometry>

//#include "Model/Shell/TopoMeshDef.hh"
//#include "BaseCore/EigenExt/MinQuadWithFixed.h"
#include "igl/min_quad_with_fixed.h"


class Manifold: public XMesh
{
public:
	Eigen::MatrixXd deformedV;
	Eigen::VectorXi boundryIdx;
	Eigen::MatrixXd boundryPos;
	std::array < Eigen::SparseMatrix<double>,3 > LCotVec, MassVec, QmatData;
	Eigen::MatrixXd bc;
	Eigen::VectorXi b;
	std::array <  igl::min_quad_with_fixed_data<double>* , 3> mq_data;
	//std::array < POP_NS::eigenext::MinQuadWithFixedData<double> ,3> mq_data;
	void calLaplacion(int k);
	void removeBIdx(std::vector<int>& indices);
	void setUpB(int k);
	void setUpBc();
	bool solveHarmonic(int k);

};

#endif // !ManifoldF_H
