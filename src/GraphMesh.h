#ifndef GRAPHMESH_H
#define GRAPHMESH_H
#include "XMesh.h"

class GraphMesh : public XMesh
{

	std::vector< std::vector<int> > v2f, v2v;
	//unsigned int* e2f{nullptr};
	unsigned char* connM{ nullptr };

	std::vector< std::vector<int> > components;
    void map_v2v();
	void map_v2f();
	void updateTplgy();
    void seedBndLoop(std::vector<int>& bnd, int seed, int seed2);
    void bfs_components();
};
#endif 