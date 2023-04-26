#ifndef XMESH_H
#define XMESH_H
#include <Eigen/Core>
#include <vector>

class XMesh
{
public:
	std::vector<std::vector<double > > stdV;
	std::vector<std::vector<int > > stdF;

	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

	std::vector< std::vector<int> > v2f, v2v;
	//unsigned int* e2f{nullptr};
	unsigned char* connM{ nullptr };

	std::vector< std::vector<int> > components;

	void loadOBJ(std::string fpath);

	void map_v2v();
	void map_v2f();
	void updateTplgy();
	void seedBndLoop(std::vector<int>& bnd, int seed, int seed2);
	void bfs_components();
};
#endif // !XMESH_H

