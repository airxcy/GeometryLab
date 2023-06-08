#ifndef NETGEN_DEMO
#define NETGEN_DEMO
#include "TopoDS_Shape.hxx"
#include "VolumeMesh.h"
#ifndef OCCGEOMETRY
#define OCCGEOMETRY
#endif // !OCCGEOMETRY
#include <occgeom.hpp>
#include <meshing.hpp>
#include <Eigen/Dense>
namespace nglib {
#include <nglib.h>
}


/*
NetGenDemo demo;
netgen::MeshingParameters& mp = demo.meshParam();
//mp.maxh = diag;
mp.grading = 0.1;
mp.optsteps3d = 0;
mp.blockfill = false;
mp.uselocalh = false;
mp.delaunay = false;
mp.delaunay2d = false;
mp.checkoverlap = false;
mp.checkchartboundary = false;
mp.checkchartboundary = false;
nglib::Ng_Init();
demo.mesh.AddFaceDescriptor(netgen::FaceDescriptor(1, 1, 0, 1));
demo.fromEigen(egm);
auto bb = plym->boundingBox();
double diag = (std::get<0>(bb) - std::get<1>(bb)).length();
std::cout << diag << std::endl;
demo.tetralization();
demo.VisVolumeSep();
*/



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

    netgen::Mesh mesh;

    netgen::MeshingParameters& meshParam();
    void occ2Surface(TopoDS_Shape& shape);
    void tetralization();
    void fromEigen(VolumeMesh<double,int>& egm);
    void toEigen(Eigen::MatrixXd& V1, Eigen::MatrixXi& F1);
    void VisVolumeWhole();
    void VisVolumeSep();
};


#endif // !NETGEN_DEMO
