#include "netgenWrapper.h"
#include "BRepBndLib.hxx"

namespace netgen
{
    extern MeshingParameters mparam;
    extern OCCParameters occparam;
    inline void NOOP_Deleter(void*) { ; }
}

netgen::MeshingParameters& NetGenWrapper::meshParam()
{
    return netgen::mparam;
}


void NetGenWrapper::addOCCSolid(TopoDS_Shape& shape)
{
    Bnd_Box aabb;
    BRepBndLib::Add(shape, aabb, false);
    const double diag = std::sqrt(aabb.SquareExtent());
    netgen::MeshingParameters& mp = netgen::mparam;
    netgen::multithread.percent = 0;
    std::cout <<"diag:" << diag << std::endl;
    // Parameters definition.
    mp.minh = 0.0;
    mp.maxh = 0.01*diag;
    mp.uselocalh = true;
    mp.secondorder = false;
    mp.grading = 0.6;
    nglib::Ng_Init();
    occgeo.shape = shape;
    occgeo.changed = 1;
    occgeo.BuildFMap();
    occgeo.CalcBoundingBox();
    netgen::OCCParameters& op = netgen::occparam;
    netgen::OCCSetLocalMeshSize(occgeo, mesh, mp, op);
    mesh.SetGeometry(std::shared_ptr<netgen::NetgenGeometry>(&occgeo, &netgen::NOOP_Deleter));
    occgeo.FindEdges(mesh, mp);
    occgeo.MeshSurface(mesh, mp);
    ////occgeo.OptimizeSurface(mesh, mp);
    std::cout << "nbTriangles:" << mesh.GetNSE() << std::endl;
}
void NetGenWrapper::tetralization()
{
    netgen::MeshingParameters& mp = netgen::mparam;
    //mesh.CalcLocalH(mp.grading);
    MeshVolume(mp, mesh);
    //occgeo.GenerateMesh(std::shared_ptr<netgen::Mesh>(&mesh, & netgen::NOOP_Deleter), mp);
    //RemoveIllegalElements(mesh);
    //OptimizeVolume(mp, mesh);
}

void NetGenWrapper::addSurface(XMesh<double,int>& egm,int fmarker)
{
    // Parameters definition.
    //netgen::MeshingParameters& mp = netgen::mparam;
    //mp.minh = 0.0;

    //mp.uselocalh = true;
    //mp.secondorder = false;
    

    for(int i=0;i<egm.V.size();i++)
        mesh.AddPoint(netgen::Point3d(egm.V[i][0], egm.V[i][1], egm.V[i][2]));

    for (int i = 0; i < egm.F.size(); i++)
    {
        netgen::Element2d f(egm.F[i][0] + 1, egm.F[i][1] + 1, egm.F[i][2] + 1);
        f.SetIndex(fmarker);
        mesh.AddSurfaceElement(f);
    }
}
void NetGenWrapper::translateOutput()
{
    const int nbNodes = (int)mesh.GetNP();
    const int nTetra = (int)mesh.GetNE();
    std::cout << " nbNodes:" << nbNodes << " nTetra:" << nTetra << std::endl;
    m_mesh.nV = nbNodes;
    m_mesh.V.resize(mesh.GetNP());
    for (int i = 1; i <= nbNodes; ++i)
    {
        const netgen::MeshPoint& point = mesh.Point(netgen::PointIndex(i));
        m_mesh.V[i - 1] = { point[0], point[1], point[2] };
    }
    m_mesh.nF = nTetra * 4;
    m_mesh.nT = nTetra;
    m_mesh.F.resize(nTetra * 4);
    m_mesh.T.resize(nTetra);
    m_mesh.TF.resize(nTetra, {});
    for (int i = 1; i <= nTetra; ++i)
    {
        const netgen::Element& elem = mesh.VolumeElement(i);
        m_mesh.T[i - 1] = { elem[0] - 1,elem[1] - 1,elem[2] - 1,elem[3] - 1 };
        for (int j = 0; j < 4; j++)
        {
            int fi = (i - 1) * 4 + j;
            m_mesh.F[fi] = { elem[tetvcoding[j][0]] - 1, elem[tetvcoding[j][1]] - 1, elem[tetvcoding[j][2]] - 1 };
            m_mesh.TF[i-1].push_back( fi);
        }
    }
}


