#include "MeshIO.h"
#include "igl/readOBJ.h"
#include "igl/readSTL.h"
#include "igl/remove_duplicate_vertices.h"
#include "igl/polygon_mesh_to_triangle_mesh.h"
#include "igl/boundary_loop.h"
#include "igl/matrix_to_list.h"

void loadOBJ(std::string fpath,XMesh* m)
{
	igl::readOBJ(fpath, m->V, m->F);
	Eigen::MatrixX<size_t> F;
	
	igl::polygon_mesh_to_triangle_mesh(m->F, F);
	m->F.clear();
	igl::matrix_to_list(F, m->F);
	std::vector<int> bnd;
	igl::boundary_loop(F, bnd);
	std::cout << fpath << ":" << m->V.size() << "," << m->F.size() << std::endl;
	if (bnd.size() == 0)
		std::cout << "is water tight" << std::endl;
}


void loadSTL(std::string fpath,XMesh* m)
{
	Eigen::MatrixXd V,N, SV;
	Eigen::MatrixXi F,SVI,SVJ,SF;
	igl::readSTL(fpath, V,F,N);
	igl::remove_duplicate_vertices(V,F, 1e-7, SV, SVI, SVJ, SF);
    igl::matrix_to_list(SV,m->V);
	for (int i = 0; i < SF.rows(); i++)
	{
		std::vector<size_t> f;
		for (int j = 0; j < SF.cols(); j++)
			f.push_back(SF(i, j));
		m->F.push_back(f);
	}
}