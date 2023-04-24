#include "EigenMeshD.h"
#include <queue>
#include "igl/readSTL.h"
#include "igl/remove_duplicate_vertices.h"
#include "igl/readOBJ.h"
#include "igl/polygon_mesh_to_triangle_mesh.h"
#include "igl/boundary_loop.h"
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
	
	std::cout << fpath <<":" << V.rows() << "," << F.rows() << std::endl;
	for (int i = 0; i < stdV.size(); i++)
		V.row(i) << stdV[i][0], stdV[i][1], stdV[i][2];

	igl::polygon_mesh_to_triangle_mesh(stdF, F);
	std::vector<int> bnd;
	igl::boundary_loop(F, bnd);
	if (bnd.size() == 0)
		std::cout << "is water tight" << std::endl;
	/*
	F.resize(stdF.size(), 3);
	for (int i = 0; i < stdF.size(); i++)
	{
		F.row(i) << stdF[i][0], stdF[i][1], stdF[i][2];
	}
	*/
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
