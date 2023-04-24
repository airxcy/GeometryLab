
#include "occStepReader.h"
#include "netgenDemo.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "tetgenWrapper.h"

#include <imgui.h>
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
    tet.Vis();
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
      
      polyscope::state::userCallback = [&]()
      {
          ImGui::GetStyle().AntiAliasedLines = false;
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

