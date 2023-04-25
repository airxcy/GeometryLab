#include "EigenMeshD.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"


#include <imgui.h>
#include <Eigen/Core>
#include <GLFW/glfw3.h>
#include "imguizmo/ImGuizmo.h"


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
    EigenMeshD dbTorus;
    dbTorus.loadOBJ("D:/projects/GeometryLab/data/double-torus.obj");
    auto psTorus=polyscope::registerSurfaceMesh("DoubleTorus", dbTorus.V, dbTorus.F);
    psTorus->setTransparency(0.5);
    psTorus->setSmoothShade(true);
    psTorus->setEdgeWidth(1);

    Eigen::RowVector3d maxv = dbTorus.V.colwise().maxCoeff();
    Eigen::RowVector3d minv = dbTorus.V.colwise().minCoeff();
    std::cout << minv << "||" << maxv << std::endl;
    double R = (maxv(0)-minv(0)) / 2;
    double r = R  / 4;
    double Rr = R - r;
    Eigen::RowVector3d leftC(0,0,maxv(2)-R), rightC(0, 0, minv(2) + R);
    Eigen::MatrixXd joints(2, 3);
    std::vector<glm::vec3> pcclr(joints.rows());
    joints<<leftC,rightC;
    pcclr[0] = { 1,0,0 };
    pcclr[1] = { 0,1,0 };
    int Nring = 40;
    joints.conservativeResize(2 + Nring * 2, 3);
    Eigen::MatrixXi skel(Nring * 2,2);
    for(int i=0;i<Nring;i++)
    {
        double ang = i * 2.0 * M_PI / Nring;
        joints.row(2+i)<<cos(ang) * Rr, 0, sin(ang) * Rr+leftC(2);
        skel(i, 0) = 2 + i;
        skel(i, 1) = 2 + (i + 1) % Nring;
        joints.row(2+Nring+i)<< cos(ang) * Rr, 0, sin(ang)* Rr+rightC(2);
        skel(Nring+i, 0) = 2 + Nring + i;
        skel(Nring+i, 1) = 2 + Nring + (i + 1) % Nring;
        pcclr.push_back({ 0,0,1 });
        pcclr.push_back({ 0,0,1 });
    }
    auto pc=polyscope::registerPointCloud("centers", joints);

    auto cQ = pc->addColorQuantity("clr", pcclr);
    cQ->setEnabled(true);
    auto gQ=misc->addSurfaceGraphQuantity("skel", joints, skel);
    gQ->setEnabled(true);

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

