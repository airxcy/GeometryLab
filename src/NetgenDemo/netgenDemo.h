#ifndef NETGEN_DEMO
#define NETGEN_DEMO
#include "TopoDS_Shape.hxx"
#include "EigenMeshD.h"
#ifndef OCCGEOMETRY
#define OCCGEOMETRY
#endif // !OCCGEOMETRY
#include <occgeom.hpp>
#include <meshing.hpp>
namespace nglib {
#include <nglib.h>
}



using namespace nglib;
class NetGenDemo
{
public:
    char tetvcoding[4][3] =
    {
        {1,2,3},
        {2,0,3},
        {0,1,3},
        {1,0,2}
    };

    netgen::Mesh mesh;

    netgen::MeshingParameters& meshParam();
    void occ2Surface(TopoDS_Shape& shape);
    void tetralization();
    void fromEigen(EigenMeshD& egm);
    void toEigen(Eigen::MatrixXd& V1, Eigen::MatrixXi& F1);
    void VisVolumeWhole();
    void VisVolumeSep();
};


#endif // !NETGEN_DEMO
