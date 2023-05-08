#include "MeshIO.h"
#include "igl/readOBJ.h"
#include "igl/readSTL.h"
#include "igl/remove_duplicate_vertices.h"
#include "igl/polygon_mesh_to_triangle_mesh.h"
#include "igl/boundary_loop.h"
template<class T_REAL, class T_IDX>
void loadOBJ(std::string fpath,XMesh<T_REAL, T_IDX> * m)
{
	igl::readOBJ(fpath, m->V, m->F);
	Eigen::MatrixX<size_t> F;
	
	igl::polygon_mesh_to_triangle_mesh(m->F, F);
	m->F.clear();
	for (int i = 0; i < F.rows(); i++)
	{
		std::vector<T_IDX> f;
		for (int j = 0; j < F.cols(); j++)
			f.push_back(F(i, j));
		m->F.push_back(f);
	}
	std::vector<int> bnd;
	igl::boundary_loop(F, bnd);
	std::cout << fpath << ":" << m->V.size() << "," << m->F.size() << std::endl;
	if (bnd.size() == 0)
		std::cout << "is water tight" << std::endl;
}

template<class T_REAL, class T_IDX>
void loadSTL(std::string fpath, XMesh<T_REAL, T_IDX> * m)
{
	Eigen::MatrixXd V,N, SV;
	Eigen::MatrixXi F,SVI,SVJ,SF;
	igl::readSTL(fpath, V,F,N);
	igl::remove_duplicate_vertices(V,F, 1e-7, SV, SVI, SVJ, SF);
	m->V.resize(V.rows());
	for (int i = 0; i < SV.rows(); i++)
	{
		for (int j = 0; j < SV.cols(); j++)
			m->V[i].push_back(SV(i, j));
	}

	for (int i = 0; i < SF.rows(); i++)
	{
		std::vector<T_IDX> f;
		for (int j = 0; j < SF.cols(); j++)
			f.push_back(SF(i, j));
		m->F.push_back(f);
	}
}

template void loadOBJ(std::string fpath, XMesh<double, int>* m);
template void loadSTL(std::string fpath, XMesh<double, int>* m);