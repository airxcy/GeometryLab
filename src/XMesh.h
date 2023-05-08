#ifndef XMESH_H
#define XMESH_H
#include <vector>

template<class T_REAL,class T_IDX>
class XMesh
{
public:
	std::vector<std::vector<T_REAL > > V;
	std::vector<std::vector<T_IDX > > F;
};


#endif // !XMESH_H

