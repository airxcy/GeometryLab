#include "EigenMeshD.h"
//#include "BaseCore/EigenExt/CotMatrix.h"
//#include "BaseCore/EigenExt/MassMatrix.h"
//#include "BaseCore/EigenExt/InvertDiag.h"
//#include "BaseCore/EigenExt/MinQuadWithFixed.h"

#include <queue>

#include "geometrycentral/surface/halfedge_factories.h"
#include "geometrycentral/surface/simple_polygon_mesh.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/tufted_laplacian.h"


#include "igl/readSTL.h"
#include "igl/remove_duplicate_vertices.h"
#include "igl/readOBJ.h"
#include "igl/cotmatrix.h"
#include "igl/massmatrix.h"
#include "igl/invert_diag.h"
#include "igl/min_quad_with_fixed.h"

#include "polyscope/surface_mesh.h"

void EigenMeshD::map_v2f() {
	v2f = std::vector< std::vector<int> >(V.rows());
	for (int c = 0; c < 3; c++)
	{
		for (int r = 0; r < F.rows(); r++)
		{
			int vidx = F(r, c);
			v2f[vidx].push_back(r);
		}
	}
}

void EigenMeshD::loadOBJ(std::string fpath)
{

	igl::readOBJ(fpath, stdV, stdF);
	V.resize(stdV.size(), 3);
	F.resize(stdF.size(), 3);
	std::cout << fpath <<":" << V.rows() << "," << F.rows() << std::endl;
	for (int i = 0; i < stdV.size(); i++)
		V.row(i) << stdV[i][0], stdV[i][1], stdV[i][2];
	for (int i = 0; i < stdF.size(); i++)
	{
		F.row(i) << stdF[i][0], stdF[i][1], stdF[i][2];
	}
	//auto plym = polyscope::registerSurfaceMesh("eigen", stdV, stdF);
}
	

void EigenMeshD::loadSTL(std::string fpath)
{
	Eigen::MatrixXd N, SV;
	Eigen::MatrixXi SVI,SVJ,SF;
	igl::readSTL(fpath, V,F,N);
	igl::remove_duplicate_vertices(V,F, 1e-7, SV, SVI, SVJ, SF);
	V = SV;
	F=SF;
	//auto plym = polyscope::registerSurfaceMesh("eigen", stdV, stdF);
}

void EigenMeshD::map_v2v() {
	int nV = V.rows();
	v2v.resize(nV);
	connM = new unsigned char[nV * nV];
	memset(connM,0, nV * nV * sizeof(unsigned char));
	//e2f= new unsigned int[nV*nV];
	//memset(e2f, 0, nV * nV*sizeof(unsigned int)); 
	for (int fi = 0; fi < F.rows(); fi++)
	{
		for (int k = 0; k < 3; k++)
		{
			int vi0 = F(fi, k);
			int vi1 = F(fi, (k + 1) % 3);
			//e2f[vi0*nV+ vi1] = fi+1;
			if (!connM[vi0* nV+ vi1])
			{
				connM[vi0 * nV + vi1] = 1;
				connM[vi1 * nV + vi0] = 1;
				v2v[vi0].push_back(vi1);
				v2v[vi1].push_back(vi0);
			}
		}
	}
}


void EigenMeshD::updateTplgy()
{

	map_v2v();
	map_v2f();	
}


void EigenMeshD::bfs_components()
{
	std::vector<int> board(v2v.size(), 1);
	int sumv = board.size();
	while (sumv > 0)
	{

		std::vector<int> bnd;
		int seed = -1;
		for (int i = 0; i < v2v.size(); i++)
		{
			if (board[i] > 0)
			{
				seed = i;
				break;
			}
		}
		if (seed < 0)
			break;
		else
		{
			std::queue<int> Queue;
			std::vector<int> passed;
			Queue.push(seed);
			while (Queue.size() > 0)
			{
				int idx = Queue.front();
				Queue.pop();
				for (int ii = passed.size() - 1; ii >= 0; ii--)
				{
					if (passed[ii] == idx)
						goto bfs_break_point;
				}
				passed.push_back(idx);
				for (auto nid : v2v[idx])
				{
					Queue.push(nid);
				}
			bfs_break_point:;
			}
			components.push_back(passed);
			for (int di : passed)board[di] = 0;
		}
		sumv = 0;
		for (int vi : board)sumv += vi;
	}
}


void EigenMeshD::seedBndLoop(std::vector<int>& bnd, int seed,int seed2)
{
	bnd.clear();
	bnd.push_back(seed);
	int vid = seed2;
	int preid = seed;
	bool foundNew = false;
	do {
		std::vector<int>& nlist = v2v[vid];
		std::vector<int>& flist = v2f[vid];
		std::map<int, int> nmap;
		for (int i = 0; i < nlist.size(); i++)
		{
			nmap.emplace(nlist[i], i);
		}
		std::vector<int> zeros(nlist.size(), 0);
		for (int fid : flist)
		{
			Eigen::RowVector3i f = F.row(fid);
			for (int i = 0; i < 3; i++)
			{
				if (f(i) != vid)
				{
					zeros[nmap[f(i)]]++;
				}
			}
		}
		foundNew = false;
		for (int i = 0; i < zeros.size(); i++)
		{
			int nextid = nlist[i];
			if (zeros[i] <= 1 && nextid != preid)
			{
				bnd.push_back(vid);
				preid = vid;
				vid = nextid;
				foundNew = true;
				break;
			}
		}
	} while (foundNew && vid != seed);
}

void EigenMeshD::calLaplacion(int k)
{
	deformedV.resize(V.rows(), V.cols());
	Eigen::SparseMatrix<double>& LCot = LCotVec[k-1];
	Eigen::SparseMatrix<double>& Mass = MassVec[k-1];
	Eigen::SparseMatrix<double>& Q = QmatData[k-1];
	igl::cotmatrix(V, F, LCot);
//	POP_NS::eigenext::CotMatrix(V, F, LCot);
	if (k > 1)
	{
		igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_DEFAULT, Mass);
		//POP_NS::eigenext::MassMatrix(V, F, POP_NS::eigenext::MASSMATRIX_TYPE_DEFAULT, Mass);
		
		using namespace geometrycentral;
		using namespace geometrycentral::surface;
		float mollifyFactor = 0.;
		std::unique_ptr<SurfaceMesh> mesh;
		std::unique_ptr<VertexPositionGeometry> geometry;
		SimplePolygonMesh inputMesh;
		for (int i = 0; i < V.rows(); i++)
			inputMesh.vertexCoordinates.push_back(geometrycentral::Vector3({ V(i,0) , V(i, 1), V(i, 2) }));
		for (int i = 0; i < F.rows(); i++)
			inputMesh.polygons.push_back(std::vector<size_t>({ (size_t)F(i,0),(size_t)F(i,1),(size_t)F(i,2) }));
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

void EigenMeshD::removeBIdx(std::vector<int>& indices)
{
	for (auto v : indices)
		boundryIdx(v) = false;
}

void EigenMeshD::setUpB(int k)
{
	int n = V.rows();
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

void EigenMeshD::setUpBc()
{
	int n = V.rows();
	int counter = 0;
	bc.resize(b.rows(), V.cols());
	bc.setConstant(0);
	for (int i = 0; i < b.rows(); i++)
			bc.row(i) = boundryPos.row(b(i));
}

bool EigenMeshD::solveHarmonic(int k)
{
	int n = V.rows();
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

