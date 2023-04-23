#include "netgenDemo.h"
#include "BRepBndLib.hxx"
#include <polyscope/surface_mesh.h>

namespace netgen
{
    extern MeshingParameters mparam;
    extern OCCParameters occparam;
    inline void NOOP_Deleter(void*) { ; }
}

netgen::MeshingParameters& NetGenDemo::meshParam()
{
    return netgen::mparam;
}


void NetGenDemo::occ2Surface(TopoDS_Shape& shape)
{
    Bnd_Box aabb;
    BRepBndLib::Add(shape, aabb, false);
    const double diag = std::sqrt(aabb.SquareExtent());

    netgen::MeshingParameters& mp = netgen::mparam;

    std::cout << diag << std::endl;
    // Parameters definition.
    mp.minh = 0.0;
    mp.maxh = diag;
    mp.uselocalh = true;
    mp.secondorder = false;
    mp.grading = 0.1;
    nglib::Ng_Init();

    Ng_OCC_Geometry;
    netgen::OCCGeometry occgeo;
    occgeo.shape = shape;
    occgeo.changed = 1;

    occgeo.BuildFMap();
    occgeo.CalcBoundingBox();

    // Resulting mesh.
    
    netgen::OCCParameters& op = netgen::occparam;

    // Mesh building
    netgen::OCCSetLocalMeshSize(occgeo, mesh, mp, op);
    mesh.SetGeometry(std::shared_ptr<netgen::NetgenGeometry>(&occgeo, &netgen::NOOP_Deleter));
    occgeo.FindEdges(mesh, mp);
    occgeo.MeshSurface(mesh, mp);
    //occgeo.OptimizeSurface(mesh, mp);
    const int nbTriangles = (int)mesh.GetNSE(); // We expect that mesh contains only triangles.
    
}

void NetGenDemo::fromEigen(EigenMeshD& egm)
{
    // Parameters definition.
    netgen::MeshingParameters& mp = netgen::mparam;
    mp.minh = 0.0;

    mp.uselocalh = true;
    mp.secondorder = false;
    nglib::Ng_Init();

    for(int i=0;i<egm.V.rows();i++)
        mesh.AddPoint(netgen::Point3d(egm.V(i,0), egm.V(i, 1), egm.V(i, 2)));

    for (int i = 0; i < egm.F.rows(); i++)
    {
        netgen::Element2d f(egm.F(i, 0) + 1, egm.F(i, 1) + 1, egm.F(i, 2) + 1);
        f.SetIndex(1);
        mesh.AddSurfaceElement(f);
    }
}
void NetGenDemo::toEigen(Eigen::MatrixXd& V1, Eigen::MatrixXi& F1)
{
    V1.resize(mesh.GetNP(), 3);
    for (int i = 1; i <= mesh.GetNP(); ++i)
    {
        const netgen::MeshPoint& point = mesh.Point(netgen::PointIndex(i));
        V1.row(i - 1) = Eigen::RowVector3d(point[0], point[1], point[2]);
    }
    auto& ope1 = mesh.SurfaceElements();
    F1.resize(ope1.Size(), 3);
    for (int i = 0; i < ope1.Size(); i++)
    {
        netgen::Element2d& elem = ope1[i];
        F1.row(i) = Eigen::RowVector3i(elem[0] - 1, elem[1] - 1, elem[2] - 1);
    }
}



void NetGenDemo::tetralization()
{
    netgen::MeshingParameters& mp = netgen::mparam;
    mesh.CalcLocalH(mp.grading);
    MeshVolume(mp, mesh);
    //RemoveIllegalElements(mesh);
    //OptimizeVolume(mp, mesh);
    const int nbNodes = (int)mesh.GetNP();
    const int nTetra = (int)mesh.GetNE();
    Eigen::MatrixXd V(nbNodes, 3);
    Eigen::MatrixXi F(nTetra * 4, 3);
    for (int i = 1; i <= nbNodes; ++i)
    {
        const netgen::MeshPoint& point = mesh.Point(netgen::PointIndex(i));
        V.row(i - 1) = Eigen::RowVector3d(point[0], point[1], point[2]);
    }
    int counter = 0;
    //char* v2vMat = new char[nbNodes * nbNodes
    //mesh.FindOpenElements();
    //auto& ope=mesh.OpenElements();
    for (int i = 1; i <= nTetra; ++i)
    {
        const netgen::Element& elem = mesh.VolumeElement(i);
        for (int j = 0; j < 4; j++)
        {
            F.row(counter * 4 + j) = Eigen::RowVector3i(elem[tetvcoding[j][0]] - 1, elem[tetvcoding[j][1]] - 1, elem[tetvcoding[j][2]] - 1);
        }
        counter++;
    }
    auto volumeVis = polyscope::registerSurfaceMesh("netgen", V, F);
    volumeVis->setEdgeWidth(1);
    volumeVis->setSmoothShade(true);
    volumeVis->setSurfaceColor({ 1,0,0 });
    //auto nQ = model->addFaceVectorQuantity("normals", model->faceNormals);
    //auto Q = model->addFaceColorQuantity("face", clrmp);
    //Q->setEnabled(true);

    std::cout << " nbNodes:" << nbNodes << " nTetra:" << nTetra << std::endl;

    //igl::writeSTL("out.stl", V, F);
}