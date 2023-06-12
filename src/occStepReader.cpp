#include "occStepReader.h"
#include "STEPCAFControl_Reader.hxx"


#include "Poly_CoherentTriangulation.hxx"
#include "TopExp_Explorer.hxx"
#include "TopoDS_Iterator.hxx"
#include "TopoDS.hxx"
#include "TopoDS_Face.hxx"
#include "BRep_Tool.hxx"
#include "BRepTools.hxx"
#include <BRep_Builder.hxx>
#include "BRepMesh_IncrementalMesh.hxx"


#include "igl/boundary_loop.h"

#include <vector>

void occStepReader::read(const char* fpath)
{
    if (BRepTools::Read(shape, fpath, BRep_Builder()))
        ;
    else
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

template<class T_REAL, class T_IDX>
void occStepReader::occTri(std::vector< XMesh<T_REAL, T_IDX> >& meshlist)
{
    
    Handle(Poly_CoherentTriangulation) cohTris = new Poly_CoherentTriangulation;
    int counter = 0;
    BRepMesh_IncrementalMesh meshGen(shape, 0.01);
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) faceTris = BRep_Tool::Triangulation(face, loc);
        std::unordered_map<int, int> nodesMap;
        XMesh<T_REAL, T_IDX> m;
        int v1, v2;
        for (int iNode = 0; iNode < faceTris->NbNodes(); ++iNode)
        {
            gp_Pnt P = faceTris->Node(iNode + 1);
            m.V.push_back({ P.X(), P.Y(), P.Z() });
        }
        double minl = 100;
        for (int iTri = 0; iTri < faceTris->NbTriangles(); ++iTri)
        {
            int c[3];
            faceTris->Triangle(iTri + 1).Get(c[0], c[1], c[2]);
            c[0] = c[0] - 1;
            c[1] = c[1] - 1;
            c[2] = c[2] - 1;
            m.F.push_back({ c[0], c[1], c[2] });

            //double l1 = (m.V.row(c[0]) - m.V.row(c[1])).norm();
            //double l2 = (m.V.row(c[1]) - m.V.row(c[2])).norm();
            //double l3 = (m.V.row(c[2]) - m.V.row(c[0])).norm();
            //if (l1 < minl) { minl = l1; v1 = c[0]; v2 = c[1]; }
            //if (l2 < minl) {
            //    minl = l2; v1 = c[1]; v2 = c[2];
            //}
            //if (l3 < minl) { minl = l3; v1 = c[2]; v2 = c[0]; }
            
        }
        meshlist.push_back(m);
    }
}
template void occStepReader::occTri<double,int>(std::vector< XMesh<double, int> >& meshlist);