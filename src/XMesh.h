#ifndef XMESH_H
#define XMESH_H
#include <vector>

class XMesh
{
public:
	std::vector<std::vector<double > > V;
	std::vector<std::vector<size_t > > F;
};
#endif // !XMESH_H

