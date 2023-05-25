#ifndef HARMONIC_DEFORMATION_H
#define HARMONIC_DEFORMATION_H
#include "XMesh.h"

#include <eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

//#include "Model/Shell/TopoMeshDef.hh"
//#include "BaseCore/EigenExt/MinQuadWithFixed.h"
#include "igl/min_quad_with_fixed.h"
#define MAX_HARMONIC_DEGREE 4

class HarmonicDeformation: public XMesh<double,int>
{
public:
	Eigen::MatrixXd eigenV;
	Eigen::MatrixXi eigenF;
	Eigen::VectorXi boundryIdx;
	std::array < Eigen::SparseMatrix<double>, MAX_HARMONIC_DEGREE > LCotVec, MassVec, QmatData;
	Eigen::MatrixXd bc;
	Eigen::VectorXi b;
	std::array <  igl::min_quad_with_fixed_data<double>* , MAX_HARMONIC_DEGREE> mq_data;
	//std::array < POP_NS::eigenext::MinQuadWithFixedData<double> ,3> mq_data;
	HarmonicDeformation()
	{
		for (int i = 0; i < mq_data.size(); i++)
			mq_data[i] = nullptr;
	}

	void calLaplacion(int k=2);
	void setUpB(int k=2);
	void setUpBc();
	bool solveHarmonic(Eigen::MatrixXd& deformedV, int k=2);

};

#endif // !ManifoldF_H
