#ifndef EIGENMESHF_H
#define EIGENMESHF_H
#include <Eigen/Core>


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

	void bfs_components();
};

#endif // !EIGENMESHF_H
