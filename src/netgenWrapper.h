#ifndef NETGEN_DEMO
#define NETGEN_DEMO
#include "TopoDS_Shape.hxx"
#include "VolumeMesh.h"
#ifndef OCCGEOMETRY
#define OCCGEOMETRY
#endif // !OCCGEOMETRY
#include <occgeom.hpp>
#include <meshing.hpp>
namespace nglib {
#include <nglib.h>
}

typedef VolumeMesh<double, int> TetMesh;
typedef XMesh<double, int> TriMesh;


using namespace nglib;
class NetGenWrapper
{
public:
    char tetvcoding[4][3] =
    {
        {1,2,3},
        {2,0,3},
        {0,1,3},
        {1,0,2}
    };
    TetMesh m_mesh;


    netgen::OCCGeometry occgeo;
    netgen::Mesh mesh;
    netgen::MeshingParameters& meshParam();
    void addOCCSolid(TopoDS_Shape& shape);
    void tetralization();
    void addSurface(XMesh<double,int>& egm,int fmarker=0);
    void translateOutput();
};


#endif // !NETGEN_DEMO
