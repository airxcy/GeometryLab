#include "HarmonicDeformation.h"

//#include "BaseCore/EigenExt/CotMatrix.h"
//#include "BaseCore/EigenExt/MassMatrix.h"
//#include "BaseCore/EigenExt/InvertDiag.h"
//#include "BaseCore/EigenExt/MinQuadWithFixed.h"

//#include <queue>

#include "geometrycentral/surface/halfedge_factories.h"
#include "geometrycentral/surface/simple_polygon_mesh.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/tufted_laplacian.h"

#include "igl/cotmatrix.h"
#include "igl/massmatrix.h"
#include "igl/invert_diag.h"
#include "igl/min_quad_with_fixed.h"

void HarmonicDeformation::calLaplacion(int k)
{
	deformedV.resize(eigenV.rows(), eigenV.cols());
	Eigen::SparseMatrix<double>& LCot = LCotVec[k-1];
	Eigen::SparseMatrix<double>& Mass = MassVec[k-1];
	Eigen::SparseMatrix<double>& Q = QmatData[k-1];
	igl::cotmatrix(eigenV, eigenF, LCot);
//	POP_NS::eigenext::CotMatrix(V, F, LCot);
	if (k > 1)
	{
		igl::massmatrix(eigenV, eigenF, igl::MASSMATRIX_TYPE_DEFAULT, Mass);
		//POP_NS::eigenext::MassMatrix(V, F, POP_NS::eigenext::MASSMATRIX_TYPE_DEFAULT, Mass);
		
		using namespace geometrycentral;
		using namespace geometrycentral::surface;
		float mollifyFactor = 0.;
		std::unique_ptr<SurfaceMesh> mesh;
		std::unique_ptr<VertexPositionGeometry> geometry;
		SimplePolygonMesh inputMesh;
		for (int i = 0; i < eigenV.rows(); i++)
			inputMesh.vertexCoordinates.push_back(geometrycentral::Vector3({ eigenV(i,0) , eigenV(i, 1), eigenV(i, 2) }));
		for (int i = 0; i < eigenF.rows(); i++)
			inputMesh.polygons.push_back(std::vector<size_t>({ (size_t)eigenF(i,0),(size_t)eigenF(i,1),(size_t)eigenF(i,2) }));
		std::tie(mesh, geometry) = makeGeneralHalfedgeAndGeometry(inputMesh.polygons, inputMesh.vertexCoordinates);
		std::tie(LCot, Mass) = buildTuftedLaplacian(*mesh, *geometry, mollifyFactor);
		
	}

	const int n = LCot.rows();
	assert(n == LCot.cols() && "LCot must be square");
	assert((k == 1 || n == Mass.cols()) && "Mass matrix must be same size as LCot");
	assert((k == 1 || n == Mass.rows()) && "Mass matrix must be square");
	
	Q = -LCot;
	if (k == 1) return;
	Eigen::SparseMatrix<double> Mi;

	igl::invert_diag(Mass, Mi);
	//POP_NS::eigenext::InvertDiag(Mass, Mi);
	// This is **not** robust for k>2. See KKT system in [Jacobson et al. 2010]
	// of the kharmonic function in gptoolbox
	
	for (int p = 1; p < k; p++)
	{
		
		Q = (Q * Mi * -LCot).eval();
	}
	
}

void HarmonicDeformation::removeBIdx(std::vector<int>& indices)
{
	for (auto v : indices)
		boundryIdx(v) = false;
}

void HarmonicDeformation::setUpB(int k)
{
	int n = eigenV.rows();
	int counter = 0;
	b.resize(n);
	b.setConstant(0);
	for (int i = 0; i < boundryIdx.rows(); i++)
	{
		if (boundryIdx(i))
		{
			b(counter) = i;
			counter++;
		}
	}
	b.conservativeResize(counter);
	if (mq_data[k - 1] == nullptr)
		mq_data[k - 1] = new igl::min_quad_with_fixed_data<double>();
	igl::min_quad_with_fixed_precompute(QmatData[k - 1], b, Eigen::SparseMatrix<double>(), true, *mq_data[k - 1]);

	//POP_NS::eigenext::MinQuadWithFixedPrecompute(QmatData[k - 1], b, Eigen::SparseMatrix<double>(), true, mq_data[k - 1]);
}

void HarmonicDeformation::setUpBc()
{
	int n = eigenV.rows();
	int counter = 0;
	bc.resize(b.rows(), eigenV.cols());
	bc.setConstant(0);
	for (int i = 0; i < b.rows(); i++)
			bc.row(i) = boundryPos.row(b(i));
}

bool HarmonicDeformation::solveHarmonic(int k)
{
	int n = eigenV.rows();
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXS;
	const VectorXS B = VectorXS::Zero(n, 1);
	for (int w = 0; w < bc.cols(); w++)
	{
		const VectorXS bcw = bc.col(w);
		VectorXS Ww;
		if (!igl::min_quad_with_fixed_solve(*mq_data[k - 1], B, bcw, VectorXS(), Ww))
		//if (!POP_NS::eigenext::MinQuadWithFixedSolve(mq_data[k-1], B, bcw, VectorXS(), Ww))
		{
			return false;
		}
		deformedV.col(w) = Ww;
	}
	return true;
}

