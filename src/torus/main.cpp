#include "GenusN.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"


#include <imgui.h>
#include <Eigen/Core>
#include <GLFW/glfw3.h>
#include "imguizmo/ImGuizmo.h"

#include "igl/matrix_to_list.h"


ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };
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
    XMesh example;
    example.loadOBJ("D:/projects/GeometryLab/data/double-torus.obj");
    auto ps1 = polyscope::registerSurfaceMesh("DoubleTorus", example.V, example.F);
    ps1->setTransparency(0.5);
    ps1->setSmoothShade(true);
    ps1->setEdgeWidth(1);
    GenusN torusN;
    int G=2;
    torusN.paramFromG2(example);
    torusN.buildMesh(G);
    auto pc1 = polyscope::registerPointCloud("genus", torusN.V);
    auto cQ=pc1->addColorQuantity("clr", torusN.clrs);
    cQ->setEnabled(true);
    //std::vector<std::vector<double> > vgraph;
    //igl::matrix_to_list(torusN.V, vgraph);
    //auto qG = misc->addSurfaceGraphQuantity("g", torusN.innerBnd );
    //qG->setEnabled(true);
    //std::cout << "visual" << std::endl;
    //Eigen::MatrixXd points(torusN.nHole*torusN.nANG, 3);
    //std::vector<glm::vec3> pcclr;
    //Eigen::MatrixXi circleG(torusN.nHole * torusN.nANG, 2);
    //double dANG = 2.0 * M_PI / torusN.nANG;
    //for (int i = 0; i < torusN.nHole; i++)
    //{
    //    double ang = 0;
    //    for (int j = 0; j < torusN.nANG; j++)
    //    {
    //        ang += dANG;
    //        points.row(pcclr.size()) = Eigen::RowVector3d(cos(ang) * torusN.R, 0, sin(ang) * torusN.R) + torusN.centers.row(i);
    //        circleG(pcclr.size(), 0) = j;
    //        circleG(pcclr.size(), 1) = (j + 1) % torusN.nANG;
    //        pcclr.push_back({ 0,0,1 });
    //    }
    //}
    //auto pc=polyscope::registerPointCloud("centers", points);
    //auto cQ = pc->addColorQuantity("clr", pcclr);
    //cQ->setEnabled(true);
    //auto gQ=misc->addSurfaceGraphQuantity("skel", points, circleG);
    //gQ->setEnabled(true);

    Eigen::Matrix4f gizmomat = Eigen::Matrix4f::Identity();
    polyscope::state::userCallback = [&]()
    {
          
        auto viewsz = ImGui::GetMainViewport()->Size;
        
        auto view = polyscope::view::getCameraViewMatrix();
        glm::mat4 viewT = glm::transpose(view);
        auto proj = polyscope::view::getCameraPerspectiveMatrix();
        glm::mat4 projT = glm::transpose(proj);
        auto pos = polyscope::view::getCameraWorldPosition();
        ImGui::SetNextWindowPos(ImVec2(viewsz.x - 210 , 0), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(200, 0), ImGuiCond_Once);
        
        ImGui::Begin("gizmoControl",NULL, ImGuiWindowFlags_None);
        if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        if (ImGui::SliderInt("genus",&G,1,10))
        {

            torusN.clear();
            torusN.buildMesh(G);
            pc1->remove();
            pc1=polyscope::registerPointCloud("genus", torusN.V);
            pc1->addColorQuantity("clr", torusN.clrs);

        }
        ImGui::End();
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, viewsz.x, viewsz.y);
        //ImGuizmo::DrawCubes(glm::value_ptr(view), glm::value_ptr(proj), gizmoMatrix,1);
        std::set<int> selset;
        if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), mCurrentGizmoOperation, ImGuizmo::LOCAL, gizmomat.data()))
        {
        }
        ImGuizmo::DrawGrid(glm::value_ptr(view), glm::value_ptr(proj), gizmomat.data(), 10.f);
        ImGui::End();
    };
      
      polyscope::show();
      
  return 0;
}

