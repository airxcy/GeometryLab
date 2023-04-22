




#include "STEPCAFControl_Reader.hxx"
#include "Poly_CoherentTriangulation.hxx"
#include "TopExp_Explorer.hxx"
#include "TopoDS_Iterator.hxx"
#include "TopoDS_Shape.hxx"
#include "TopoDS.hxx"
#include "TopoDS_Face.hxx"
#include "BRep_Tool.hxx"
#include "BRepMesh_IncrementalMesh.hxx"

#include "BRepBndLib.hxx"


#include "GlfwOcctView.h"
#include "igl/writeSTL.h"
#include "igl/boundary_loop.h"
#include "EigenMeshD.h"



#ifndef OCCGEOMETRY
#define OCCGEOMETRY
#endif // !OCCGEOMETRY


#include <occgeom.hpp>
#include <meshing.hpp>
namespace nglib {
#include <nglib.h>
}


//#include <ngexception.hpp>


#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>



#include <Eigen/Core>
#include <unordered_map>


Handle(GlfwOcctView) anApp;
GLFWwindow* GLFWwin;
void errorCallback(int error, const char* description)
{
    // Do nothing
}

typedef std::pair<glm::vec3, double  > glmE;
std::vector< glmE > minE;


void occTri2Eigen(TopoDS_Shape& shape)
{
    Handle(Poly_CoherentTriangulation) cohTris = new Poly_CoherentTriangulation;
    int counter = 0;
    BRepMesh_IncrementalMesh meshGen(shape, 0.01);

    std::vector<EigenMeshD> fmeshlist;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) faceTris = BRep_Tool::Triangulation(face, loc);
        std::unordered_map<int, int> nodesMap;
        EigenMeshD m;
        m.V = Eigen::MatrixXd(faceTris->NbNodes(), 3);
        m.F = Eigen::MatrixXi(faceTris->NbTriangles(), 3);

        int v1, v2;
        for (int iNode = 0; iNode < faceTris->NbNodes(); ++iNode)
        {
            gp_Pnt P = faceTris->Node(iNode+1);
            m.V.row(iNode) = Eigen::RowVector3d(P.X(), P.Y(), P.Z());
        }
        double minl = 100;
        for (int iTri = 0; iTri < faceTris->NbTriangles(); ++iTri)
        {
            int c[3];
            faceTris->Triangle(iTri+1).Get(c[0], c[1], c[2]);
            c[0] = c[0] - 1;
            c[1] = c[1] - 1;
            c[2] = c[2] - 1;
            double l1=(m.V.row(c[0]) - m.V.row(c[1])).norm();
            double l2= (m.V.row(c[1]) - m.V.row(c[2])).norm();
            double l3= (m.V.row(c[2]) - m.V.row(c[0])).norm();
            if (l1 < minl) { minl = l1; v1 = c[0]; v2 = c[1]; }
            if (l2 < minl) {
                minl = l2; v1 = c[1]; v2 = c[2];
            }
            if (l3 < minl) { minl = l3; v1 = c[2]; v2 = c[0]; }
            m.F.row(iTri) = Eigen::RowVector3i(c[0], c[1], c[2] );
        }
        auto model = polyscope::registerSurfaceMesh(std::to_string(fmeshlist.size()), m.V, m.F);
        //Eigen::RowVector3d cent = V.colwise().mean();
        std::vector< glm::vec3 > e;
        e.push_back({ m.V(v1,0),m.V(v1,1) ,m.V(v1,2) });
        e.push_back({ m.V(v2,0),m.V(v2,1) ,m.V(v2,2) });
        glm::vec3 midp = e[0] + e[1];
        midp /= 2;
        minE.push_back(glmE(midp, minl));

        std::vector < std::vector<glm::vec3> > ee;ee.push_back(e);
        m.updateTplgy();
        std::vector<std::vector< int > > bndlist;
        igl::boundary_loop(m.F, bndlist);
        //m.seedBndLoop(bnd, bnd[0], bnd[1]);
        auto clr = polyscope::getNextUniqueColor();

        for(auto& bnd: bndlist)
            for (int i = 0; i < bnd.size(); i++)
            {
                //bnde.row(i) << bnd[i], bnd[(i + 1) % bnd.size()];
                //auto gQ = model->addSurfaceGraphQuantity("minE", ee);
                Eigen::MatrixXd bndV = m.V({ bnd[i],bnd[(i + 1) % bnd.size()]}, {0,1,2});
                Eigen::RowVector2i bnde(0, 1);
                auto gQ = model->addSurfaceGraphQuantity(std::to_string(i).c_str(), bndV, bnde);
                double val = (i + 0.1) / bnd.size();
                gQ->setColor(glm::vec3(val,1-val,0));
                gQ->setEnabled(true);
                gQ->setRadius(0.3, false);
            }
        fmeshlist.push_back(m);
        model->setTransparency(0.6);
        model->setEdgeWidth(1);
        model->setSmoothShade(true);
        model->setSurfaceColor(clr);


    }
    
}

namespace netgen
{
    extern MeshingParameters mparam;
    extern OCCParameters occparam;
    inline void NOOP_Deleter(void*) { ; }
}
using namespace nglib;
char tetvcoding[4][3] =
{
    {1,2,3},
    {2,0,3},
    {0,1,3},
    {1,0,2}
};
void ng2Eigen(TopoDS_Shape& shape)
{
    
    
    Bnd_Box aabb;
    BRepBndLib::Add(shape, aabb, false);
    const double diag = std::sqrt(aabb.SquareExtent());
    
    
    
    
    netgen::MeshingParameters& mp=netgen::mparam;
    
    std::cout << diag << std::endl;
    // Parameters definition.
    mp.minh = 0.0;
    mp.maxh =  diag;
    mp.uselocalh = true;
    mp.secondorder = false;
    mp.grading = 0.01;
    nglib::Ng_Init();
    
    Ng_OCC_Geometry;
    netgen::OCCGeometry occgeo;
    occgeo.shape = shape;
    occgeo.changed = 1;
    
    occgeo.BuildFMap();
    occgeo.CalcBoundingBox();
    
    // Resulting mesh.
    netgen::Mesh mesh;
    netgen::OCCParameters& op = netgen::occparam;
    
    // Mesh building
    netgen::OCCSetLocalMeshSize(occgeo, mesh,mp,op);
    mesh.SetGeometry(std::shared_ptr<netgen::NetgenGeometry>(&occgeo,&netgen::NOOP_Deleter ));
    occgeo.FindEdges(mesh, mp);
    occgeo.MeshSurface(mesh, mp);
    //occgeo.OptimizeSurface(mesh, mp);
    const int nbTriangles = (int)mesh.GetNSE(); // We expect that mesh contains only triangles.
    std::cout << "nbTriangle : " << nbTriangles << std::endl;

    Eigen::MatrixXd V1(mesh.GetNP(), 3);
    for (int i = 1; i <= mesh.GetNP(); ++i)
    {
        const netgen::MeshPoint& point = mesh.Point(netgen::PointIndex(i));
        V1.row(i - 1) = Eigen::RowVector3d(point[0], point[1], point[2]);
    }
    auto& ope1 = mesh.SurfaceElements();
    Eigen::MatrixXi F1(ope1.Size(), 3);
    for (int i = 0; i < ope1.Size(); i++)
    {
        netgen::Element2d& elem = ope1[i];
        F1.row(i) = Eigen::RowVector3i(elem[0] - 1, elem[1] - 1, elem[2] - 1);
    }
    auto surfVis1 = polyscope::registerSurfaceMesh("SurfaceElements1", V1, F1);
    surfVis1->setSurfaceColor({ 0,1,0 });

    //Ng_OCC_Geometry* og = (Ng_OCC_Geometry*)&occgeo;
    //Ng_Mesh* me = (Ng_Mesh*) & mesh;
    //Ng_Meshing_Parameters* nmp = (Ng_Meshing_Parameters *)&mp;
    //Ng_Result result = Ng_OCC_GenerateSurfaceMesh(og, me, nmp);
    mesh.CalcLocalH(mp.grading);
    MeshVolume(mp, mesh);
    //RemoveIllegalElements(mesh);
    //OptimizeVolume(mp, mesh);
    const int nbNodes = (int)mesh.GetNP();
    const int nTetra = (int)mesh.GetNE(); 
    Eigen::MatrixXd V(nbNodes,3);
    Eigen::MatrixXi F(nTetra*4,3);
    for (int i = 1; i <= nbNodes; ++i)
    {
        const netgen::MeshPoint& point = mesh.Point(netgen::PointIndex(i));
        V.row(i-1)= Eigen::RowVector3d( point[0], point[1], point[2] );
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
            F.row(counter * 4 + j) = Eigen::RowVector3i(elem[tetvcoding[j][0]]-1, elem[tetvcoding[j][1]]-1, elem[tetvcoding[j][2]]-1);
        }
        counter++;
    }
    auto volumeVis = polyscope::registerSurfaceMesh("netgen", V, F);
    volumeVis->setTransparency(0.6);
    volumeVis->setEdgeWidth(1);
    volumeVis->setSmoothShade(true);
    //auto nQ = model->addFaceVectorQuantity("normals", model->faceNormals);
    //auto Q = model->addFaceColorQuantity("face", clrmp);
    //Q->setEnabled(true);


    auto& ope = mesh.SurfaceElements();
    Eigen::MatrixXi F2(ope.Size(), 3);
    for (int i = 0; i < ope.Size(); i++)
    {
        netgen::Element2d& elem = ope[i];
        F2.row(i)=Eigen::RowVector3i(elem[0]-1, elem[1]-1, elem[2]-1);
    }
    auto surfVis = polyscope::registerSurfaceMesh("SurfaceElements", V, F2);
    surfVis->setSurfaceColor({1,0,0});
    std::cout << "surface elem : " << ope.Size() <<" nbNodes:" << nbNodes<<" nTetra:" << nTetra << std::endl;

    //igl::writeSTL("out.stl", V, F);

    
}

int main (const int, const char**)
{
  //init();
    
    polyscope::options::programName = "Nine Cube";
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::view::style = polyscope::view::NavigateStyle::Free;
    polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
    
    polyscope::init();
    glfwSetErrorCallback(errorCallback);
    auto  misc = polyscope::registerSurfaceMesh2D("misc", Eigen::MatrixXd(), Eigen::MatrixXi());
    
      STEPControl_Reader reader;
      IFSelect_ReturnStatus stat = reader.ReadFile("D:/projects/GeometryLab/src/occSTEP/test_model_step203.stp");
      
      Standard_Integer NbRoots = reader.NbRootsForTransfer();
      Standard_Integer num = reader.TransferRoots();
      TopoDS_Iterator tree(reader.OneShape());
      
      //tree->Next();
      TopoDS_Shape shape = tree.Value();
      //std::cout << shape.NbChildren() << std::endl;
      
      
      ng2Eigen(shape);
      //occTri2Eigen(shape);
      
      polyscope::state::userCallback = [&]()
      {
          ImGui::GetStyle().AntiAliasedLines = false;

          auto viewsz = ImGui::GetMainViewport()->Size;
          auto view = polyscope::view::getCameraViewMatrix();
          auto proj = polyscope::view::getCameraPerspectiveMatrix();
          for (int i =0;i< minE.size();i++)
          {
              auto sname= std::to_string(i);
              auto s = polyscope::getSurfaceMesh(sname);
              if (s->isEnabled())
              {
                  glm::vec3 v = glm::project(minE[i].first, glm::mat4(1.0), proj * view, glm::vec4(0, 0, viewsz.x, viewsz.y));
                  ImGui::SetNextWindowPos({ v[0],  viewsz.y - v[1] });
                  ImGui::Begin(sname.c_str(), nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground);
                  ImGui::Text("%.4e", minE[i].second);
                  ImGui::End();
              }
          }

      };
      
      polyscope::show();
      
  return 0;
}


/*
void init()
{
    glfwInit();
    const bool toAskCoreProfile = true;
    if (toAskCoreProfile)
    {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#if defined (__APPLE__)
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    }
    anApp = new GlfwOcctView();
    GLFWwin = anApp->initWindow(800, 600, "step file");
    anApp->initViewer();
    // Setup Dear ImGui context
    const char* glsl_version = "#version 330";
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(GLFWwin, true);

    ImGui_ImplOpenGL3_Init(glsl_version);
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    anApp->initDemoScene();
    if (anApp->myView.IsNull())
        return ;

    anApp->myView->MustBeResized();
    anApp->myOcctWindow->Map();
}

void cleanup()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    anApp->cleanup();
}

Handle(Poly_Triangulation) triOCC(const TopoDS_Shape& shape )
{
    BRepMesh_IncrementalMesh meshGen(shape, 0.1);
    Handle(Poly_CoherentTriangulation) cohTris = new Poly_CoherentTriangulation;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) faceTris = BRep_Tool::Triangulation(face, loc);

        std::unordered_map<int, int> nodesMap;
        for (int iNode = 1; iNode <= faceTris->NbNodes(); ++iNode)
        {
            gp_Pnt P = faceTris->Node(iNode);
            const int cohNodeIndex = cohTris->SetNode(P.XYZ());

            nodesMap.insert({ iNode,cohNodeIndex });
        }

        for (int iTri = 1; iTri <= faceTris->NbTriangles(); ++iTri)
        {
            const Poly_Triangle& tri = faceTris->Triangle(iTri);

            int iNodes[3];
            tri.Get(iNodes[0], iNodes[1], iNodes[2]);

            int iCohNodes[3] = { nodesMap.at(iNodes[0]),nodesMap.at(iNodes[1]),nodesMap.at(iNodes[2]) };
            cohTris->AddTriangle(iCohNodes[0], iCohNodes[1], iCohNodes[2]);

        }
    }

    return cohTris->GetTriangulation();
}
void occGLFWLoop()
{
    while (!glfwWindowShouldClose(anApp->myOcctWindow->getGlfwWindow()))
    {
        // glfwPollEvents() for continuous rendering (immediate return if there are no new events)
        // and glfwWaitEvents() for rendering on demand (something actually happened in the viewer)
        //glfwPollEvents();
        glfwWaitEvents();
        if (!anApp->myView.IsNull())
        {
            anApp->Flush();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            Standard_Integer w, h;
            anApp->myOcctWindow->Size(w, h);
            ImGui::NewFrame();
            ImGui::SetNextWindowPos({ (float)w - 80, 0 });
            ImGui::Begin("main", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1, 0, 0, 1));
            if (ImGui::Button("classify"))
            {
                //anApp->addTri(triOCC(shape));
                //anApp->Flush();
            }
            ImGui::PopStyleColor();
            ImGui::End();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(anApp->myOcctWindow->getGlfwWindow());
        }
    }
    cleanup();
}
*/