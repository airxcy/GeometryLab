
#include "occStepReader.h"
#include "netgenDemo.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include <imgui.h>
//#include <imgui_impl_glfw.h>
//#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include "igl/readOBJ.h"

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
    egm.loadOBJ("D:/data/shuttle.obj");
    auto plym = polyscope::registerSurfaceMesh("eigen", egm.V, egm.F);
    
    NetGenDemo demo;
    demo.mesh.AddFaceDescriptor(netgen::FaceDescriptor(1, 1, 0, 1));
    std::cout << "fromEigen" << std::endl;
    demo.fromEigen(egm);
    EigenMeshD egm2;
    demo.toEigen(egm2.V, egm2.F);
    auto surfVis1 = polyscope::registerSurfaceMesh("SurfaceElements1", egm2.V, egm2.F);
    surfVis1->setSurfaceColor({ 0,1,0 });
    demo.tetralization();
    

      
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

