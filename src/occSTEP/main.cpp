﻿




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


void occTri2Eigen(TopoDS_Shape& shape)
{
    Handle(Poly_CoherentTriangulation) cohTris = new Poly_CoherentTriangulation;
    int counter = 0;
    BRepMesh_IncrementalMesh meshGen(shape, 0.001);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    std::vector< std::vector<double> > vertices;
    std::vector< std::vector<int> > mesh;
    std::vector< glm::vec3> clrmp;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {


        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) faceTris = BRep_Tool::Triangulation(face, loc);

        std::unordered_map<int, int> nodesMap;
        for (int iNode = 1; iNode <= faceTris->NbNodes(); ++iNode)
        {
            gp_Pnt P = faceTris->Node(iNode);
            std::vector<double> vtx = { P.X(),P.Y(), P.Z() };
            const int cohNodeIndex = vertices.size();
            vertices.push_back(vtx);
            nodesMap.insert({ iNode, cohNodeIndex });
        }
        auto c = polyscope::getNextUniqueColor();
        for (int iTri = 1; iTri <= faceTris->NbTriangles(); ++iTri)
        {
            int iNodes[3];
            faceTris->Triangle(iTri).Get(iNodes[0], iNodes[1], iNodes[2]);
            int iCohNodes[3] = { nodesMap.at(iNodes[0]),nodesMap.at(iNodes[1]),nodesMap.at(iNodes[2]) };
            std::vector<int> tri = { iCohNodes[0], iCohNodes[1], iCohNodes[2] };
            mesh.push_back(tri);
            clrmp.push_back(c);
        }

        counter++;
        
    }

    auto model = polyscope::registerSurfaceMesh("OCC", vertices, mesh);
    model->setTransparency(0.6);
    model->setEdgeWidth(1);
    model->setSmoothShade(true);
}

namespace netgen
{
    extern MeshingParameters mparam;
    extern OCCParameters occparam;
    inline void NOOP_Deleter(void*) { ; }
}
using namespace nglib;

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
    //Ng_OCC_Geometry* og = (Ng_OCC_Geometry*)&occgeo;
    Ng_Mesh* me = (Ng_Mesh*) & mesh;
    Ng_Meshing_Parameters* nmp = (Ng_Meshing_Parameters *)&mp;
    //Ng_Result result = Ng_OCC_GenerateSurfaceMesh(og, me, nmp);
    

    
    
    
    const int nbTriangles = (int)mesh.GetNSE(); // We expect that mesh contains only triangles.
    



    mesh.CalcLocalH(mp.grading);

    MeshVolume(mp, mesh);
    //RemoveIllegalElements(mesh);
    //OptimizeVolume(mp, mesh);
    const int nbNodes = (int)mesh.GetNP();
    const int nTetra = (int)mesh.GetNE(); 

    std::cout << nbNodes << "----------------------------------------" << nTetra << std::endl;

    //Handle(Poly_Triangulation) result = new Poly_Triangulation(nbNodes, nbTriangles, false);
    
    Eigen::MatrixXd V(nbNodes,3);
    Eigen::MatrixXi F(nTetra*4,3);
    
    std::vector< std::vector<double> > vertices;
    std::vector< std::vector<int> > trimesh;
    std::vector< glm::vec3> clrmp;
    for (int i = 1; i <= nbNodes; ++i)
    {
        const netgen::MeshPoint& point = mesh.Point(netgen::PointIndex(i));
        //std::cout <<i<< ": " << point[0] << " " << point[1] << " " << point[2] << std::endl;
        V.row(i-1)= Eigen::RowVector3d( point[0], point[1], point[2] );
    }
    int counter = 0;
    for (int i = 1; i <= nTetra; ++i)
    {
        const netgen::Element& elem = mesh.VolumeElement(i);
        {
            //double val=(V.row(elem[0] - 1) + V.row(elem[1] - 1) + V.row(elem[2] - 1) + V.row(elem[3] - 1))* Eigen::Vector3d(1, 1, 1);
            //if (val < 0)
            {
                //std::cout << i << ": " << elem[0] << " " << elem[1] << " " << elem[2] << std::endl;
                F.row(counter * 4 + 0) = Eigen::RowVector3i(elem[0] - 1, elem[1] - 1, elem[2] - 1);
                F.row(counter * 4 + 1) = Eigen::RowVector3i(elem[1] - 1, elem[2] - 1, elem[3] - 1);
                F.row(counter * 4 + 2) = Eigen::RowVector3i(elem[2] - 1, elem[3] - 1, elem[0] - 1);
                F.row(counter * 4 + 3) = Eigen::RowVector3i(elem[3] - 1, elem[0] - 1, elem[1] - 1);
                counter++;
            }
        }
    }

    igl::writeSTL("out.stl", V, F);
    auto model = polyscope::registerSurfaceMesh("netgen", V, F);
    model->setTransparency(0.6);
    model->setEdgeWidth(1);
    model->setSmoothShade(true);
    //auto nQ = model->addFaceVectorQuantity("normals", model->faceNormals);
    //auto Q = model->addFaceColorQuantity("face", clrmp);
    //Q->setEnabled(true);
    
}
int main (const int, const char**)
{
  //init();
    
    polyscope::options::programName = "Nine Cube";
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::view::style = polyscope::view::NavigateStyle::Free;
    polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
    
    polyscope::init();

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
      occTri2Eigen(shape);
      /*
      //anApp.addShape(shape);
      polyscope::state::userCallback = [&]()
      {
          ImGui::GetStyle().AntiAliasedLines = false;

          auto viewsz = ImGui::GetMainViewport()->Size;
          ImGui::SetNextWindowPos({ viewsz[0] - 80, 0});
          ImGui::Begin("main", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground);
          ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1, 0, 0, 1));
          if (ImGui::Button("classify"))
          {


          }
          ImGui::PopStyleColor();
          ImGui::End();
      };
      */
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