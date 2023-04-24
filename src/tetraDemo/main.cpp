
#include "occStepReader.h"
#include "netgenDemo.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "tetgenWrapper.h"


#include <imgui.h>

#include "imguizmo/ImGuizmo.h"

//#include <imgui_impl_glfw.h>
//#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

void errorCallback(int error, const char* description){}
int main (const int, const char**)
{
    polyscope::options::programName = "Nine Cube";
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::view::style = polyscope::view::NavigateStyle::Free;
    polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };    
    polyscope::init();
    glfwSetErrorCallback(errorCallback);
    auto  misc = polyscope::registerSurfaceMesh2D("misc", Eigen::MatrixXd(), Eigen::MatrixXi());
    
    //occStepReader occStep;
    //occStep.read("D:/data/test_model_step203.stp");
    //occStep.occTri2Eigen();
    //demo.occ2Surface(occStep.shape);
    //demo.tetralization();
    //occTri2Eigen(shape);
    EigenMeshD egm;
    egm.loadOBJ("D:/projects/GeometryLab/data/Gear_Spur_16T.obj");
    //egm.loadOBJ("D:/projects/GeometryLab/data/cube.obj");
    auto plym = polyscope::registerSurfaceMesh("eigen", egm.V, egm.F);
    plym->setSurfaceColor({ 0,1,0 });
    plym->setTransparency(0.6);
    plym->setEdgeWidth(1);
    TetgenWrapper tet;
    tet.fromEigen(egm);
    tet.run();
    EigenMeshD egt;
    tet.toEigen(egt);
    
    //auto pctet = polyscope::registerPointCloud("V", egt.V);
    auto pstet= polyscope::registerSurfaceMesh("tetgen", egt.sV, egt.sF);
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
    int starti = 0;
    ImGui::GetStyle().AntiAliasedLines = false;
    float gizmoMatrix[16];
    for (int i = 0; i < 4; i++)
        gizmoMatrix[i * 4 + i] = 1;
    polyscope::state::userCallback = [&]()
    {
          
        auto viewsz = ImGui::GetMainViewport()->Size;
        
        auto view = polyscope::view::getCameraViewMatrix();
        glm::mat4 viewT = glm::transpose(view);
        auto proj = polyscope::view::getCameraPerspectiveMatrix();
        glm::mat4 projT = glm::transpose(proj);
        auto pos = polyscope::view::getCameraWorldPosition();
        ImGui::SetNextWindowPos({ 0,0 });
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, viewsz.x, viewsz.y);
        ImGuizmo::Manipulate(glm::value_ptr(viewT), glm::value_ptr(projT), ImGuizmo::TRANSLATE, ImGuizmo::WORLD, gizmoMatrix);
        //std::set<int> selset;
        //for (int i = starti; i < egt.nT; i += 2)
        //    selset.insert(egt.sTF[i].begin(), egt.sTF[i].end());
        //std::vector<std::vector<size_t> > newfaces;
        //for (int fi : selset)
        //    newfaces.push_back(egt.sF[fi]);
        //pstet->faces = newfaces;
        //pstet->updateObjectSpaceBounds();
        //pstet->computeCounts();
        //pstet->geometryChanged();
        //starti = 1 - starti;
        /*
        auto viewsz = ImGui::GetMainViewport()->Size;
        auto view = polyscope::view::getCameraViewMatrix();
        auto proj = polyscope::view::getCameraPerspectiveMatrix();
        for (int i =0;i< occStep.minE.size();i++)
        {
            auto sname= std::to_string(i);
            auto s = polyscope::getSurfaceMesh(sname);
            if (s->isEnabled())
            {
                glm::vec3 v = glm::project(occStep.minE[i].first, glm::mat4(1.0), proj * view, glm::vec4(0, 0, viewsz.x, viewsz.y));
                ImGui::SetNextWindowPos({ v[0],  viewsz.y - v[1] });
                ImGui::Begin(sname.c_str(), nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground);
                ImGui::Text("%.4e", occStep.minE[i].second);
                ImGui::End();
            }
        }
        */
    };
      
      polyscope::show();
      
  return 0;
}

