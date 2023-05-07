#ifndef VOLUMEMESH_H
#define VOLUMEMESH_H
#include "XMesh.h"
class VolumeMesh : public XMesh
{
public:
    int nV{0};
    int nT{0};
    int nF{0};

	std::vector<std::vector<size_t> > T;
	std::vector<std::vector<int> > TF;

};

#endif