#include "occStepReader.h"
#include "STEPCAFControl_Reader.hxx"


#include "Poly_CoherentTriangulation.hxx"
#include "TopExp_Explorer.hxx"
#include "TopoDS_Iterator.hxx"
#include "TopoDS.hxx"
#include "TopoDS_Face.hxx"
#include "BRep_Tool.hxx"
#include "BRepMesh_IncrementalMesh.hxx"


#include "igl/boundary_loop.h"
#include "polyscope/surface_mesh.h"

#include <vector>

void occStepReader::read(const char* fpath)
{
    {
        STEPControl_Reader reader;
        IFSelect_ReturnStatus stat = reader.ReadFile(fpath);

        Standard_Integer NbRoots = reader.NbRootsForTransfer();
        Standard_Integer num = reader.TransferRoots();
        TopoDS_Iterator tree(reader.OneShape());

        //tree->Next();
        shape = tree.Value();
        //std::cout << shape.NbChildren() << std::endl;
    }
}

void occStepReader::occTri2Eigen()
{
    /*
    Handle(Poly_CoherentTriangulation) cohTris = new Poly_CoherentTriangulation;
    int counter = 0;
    BRepMesh_IncrementalMesh meshGen(shape, 0.01);

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) faceTris = BRep_Tool::Triangulation(face, loc);
        std::unordered_map<int, int> nodesMap;
        TetMesh m;
        m.V = Eigen::MatrixXd(faceTris->NbNodes(), 3);
        m.F = Eigen::MatrixXi(faceTris->NbTriangles(), 3);

        int v1, v2;
        for (int iNode = 0; iNode < faceTris->NbNodes(); ++iNode)
        {
            gp_Pnt P = faceTris->Node(iNode + 1);
            m.V.row(iNode) = Eigen::RowVector3d(P.X(), P.Y(), P.Z());
        }
        double minl = 100;
        for (int iTri = 0; iTri < faceTris->NbTriangles(); ++iTri)
        {
            int c[3];
            faceTris->Triangle(iTri + 1).Get(c[0], c[1], c[2]);
            c[0] = c[0] - 1;
            c[1] = c[1] - 1;
            c[2] = c[2] - 1;
            double l1 = (m.V.row(c[0]) - m.V.row(c[1])).norm();
            double l2 = (m.V.row(c[1]) - m.V.row(c[2])).norm();
            double l3 = (m.V.row(c[2]) - m.V.row(c[0])).norm();
            if (l1 < minl) { minl = l1; v1 = c[0]; v2 = c[1]; }
            if (l2 < minl) {
                minl = l2; v1 = c[1]; v2 = c[2];
            }
            if (l3 < minl) { minl = l3; v1 = c[2]; v2 = c[0]; }
            m.F.row(iTri) = Eigen::RowVector3i(c[0], c[1], c[2]);
        }
        auto model = polyscope::registerSurfaceMesh(std::to_string(fmeshlist.size()), m.V, m.F);
        //Eigen::RowVector3d cent = V.colwise().mean();
        std::vector< glm::vec3 > e;
        e.push_back({ m.V(v1,0),m.V(v1,1) ,m.V(v1,2) });
        e.push_back({ m.V(v2,0),m.V(v2,1) ,m.V(v2,2) });
        glm::vec3 midp = e[0] + e[1];
        midp /= 2;
        minE.push_back(glmE(midp, minl));

        std::vector < std::vector<glm::vec3> > ee; ee.push_back(e);
        m.updateTplgy();
        std::vector<std::vector< int > > bndlist;
        igl::boundary_loop(m.F, bndlist);
        //m.seedBndLoop(bnd, bnd[0], bnd[1]);
        auto clr = polyscope::getNextUniqueColor();
        std::cout << "+++++++++++++++++++++++++++++++" << fmeshlist.size() << "+++++++++++++++++++++++++++++++" << std::endl;
        for (auto& bnd : bndlist)
        {
            std::cout << "----------------" << std::endl;
            for (int i = 0; i < bnd.size(); i++)
            {
                //bnde.row(i) << bnd[i], bnd[(i + 1) % bnd.size()];
                //auto gQ = model->addSurfaceGraphQuantity("minE", ee);
                std::cout << bnd[i] << std::endl;
                Eigen::MatrixXd bndV = m.V({ bnd[i],bnd[(i + 1) % bnd.size()] }, { 0,1,2 });
                Eigen::RowVector2i bnde(0, 1);
                auto gQ = model->addSurfaceGraphQuantity(std::to_string(i).c_str(), bndV, bnde);
                double val = (i + 0.1) / bnd.size();
                gQ->setColor(glm::vec3(val, 1 - val, 0));
                gQ->setEnabled(true);
                gQ->setRadius(0.3, false);
            }
        }
        fmeshlist.push_back(m);
        //model->setTransparency(0.6);
        model->setEdgeWidth(1);
        model->setSmoothShade(true);
        model->setSurfaceColor(clr);


    }
    */
}
