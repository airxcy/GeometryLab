#ifndef OOC_STEP_READER
#define OOC_STEP_READER

#include "VolumeMesh.h"
#include "TopoDS_Shape.hxx"
#include  <utility>



class occStepReader
{
public:
    TopoDS_Shape shape;
    void read(const char* fpath);
    template<class T_REAL, class T_IDX>
    void occTri(std::vector<XMesh<T_REAL,T_IDX> > & meshlist);
};

#endif
