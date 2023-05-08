#ifndef VOLUMEMESH_H
#define VOLUMEMESH_H
#include "XMesh.h"
template<class T_REAL, class T_IDX>
class VolumeMesh : public XMesh<T_REAL,T_IDX>
{
public:
    T_IDX nV{0};
    T_IDX nT{0};
    T_IDX nF{0};

	std::vector<std::vector<T_IDX> > T;
	std::vector<std::vector<T_IDX> > TF;

};

#endif