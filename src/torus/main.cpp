#include "GenusN.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/types.h"


#include <imgui.h>
#include <Eigen/Core>
#include <GLFW/glfw3.h>
#include "imguizmo/ImGuizmo.h"

#include "igl/boundary_loop.h"
#include "igl/matrix_to_list.h"


ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };
void errorCallback(int error, const char* description){}

std::pair<polyscope::SurfaceMesh*,polyscope::PointCloud* > addGenus(GenusN& m)
{
    //std::vector< std::vector<int> >bnd;
    //igl::boundary_loop(m.F, bnd);
    //if (bnd.size() == 0)
    //    std::cout << "is water tight" << std::endl;
    Eigen::MatrixXd V2(m.V.rows(),3);
    int counter = 0;
    for (int i = 0; i < m.boundryIdx.size(); i++)
    {
        
        if (m.boundryIdx(i))
        {
            V2.row(counter++)=m.V.row(i);
        }
    }
    V2.conservativeResize(counter, 3);
    auto pc1 = polyscope::registerPointCloud("genus", V2);
    pc1->setPointColor({ 0,0,1 });
    //auto cQ = pc1->addColorQuantity("clr", m.clrs);
    pc1->setPointRadius(0.01,false);
    //cQ->setEnabled(true);
    
    auto ps=polyscope::registerSurfaceMesh("mesh", m.V, m.F);
    ps->setEdgeWidth(1);
    ps->setSurfaceColor({ 1,0.5,0 });
    ps->setBackFaceColor({ 0.5,0.5,0.5 });
    ps->setBackFacePolicy(polyscope::BackFacePolicy::Custom);
    //ps->setTransparency(0.5);
    //std::vector < std::vector< glm::vec3 > > innerBnd;
    //for (int i = 0; i < m.innerBndIdx.size(); i++)
    //{
    //    std::vector<glm::vec3> bnd;
    //    for (int vi : m.innerBndIdx[i])
    //        bnd.push_back({ m.V(vi,0),m.V(vi,1),m.V(vi,2) });
    //    innerBnd.push_back(bnd);
    //}
    //std::vector<std::vector<glm::vec3>> pos(1);
    //pos[0].push_back(glm::vec3(0,0,0));
    //pos[0].push_back(glm::vec3(m.V(0, 0), m.V(0, 1), m.V(0, 2)));
    //auto qG = ps->addSurfaceGraphQuantity("g", pos);
    //qG->setEnabled(true);
    //qG->setColor({ 0,0,1 });
    //glm::vec3 campos = std::get<1>(ps->objectSpaceBoundingBox);
    //campos *= 2;
    //polyscope::view::lookAt(campos, { 0,0,0 }, true);

    return { ps ,pc1};
}
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
    //auto ps1 = polyscope::registerSurfaceMesh("DoubleTorus", example.V, example.F);
    //ps1->setSurfaceColor({ 0,1,0 });
    //ps1->setTransform({ 0,0,1,0,0,1,0,0,-1,0,0,0,0,0,0,1 });
    //ps1->setTransparency(0.5);
    //ps1->setSmoothShade(true);
    //ps1->setEdgeWidth(1);
    GenusN torusN;
    torusN.paramFromG2(example);
    int G = 2;
    float cDist = torusN.cDist, R = torusN.R, r = torusN.r;
    torusN.buildMesh(G);
    auto psG = addGenus(torusN);
    Eigen::Matrix4f gizmomat = Eigen::Matrix4f::Identity();
    polyscope::state::userCallback = [&]()
    {
          
        auto viewsz = ImGui::GetMainViewport()->Size;
        
        auto view = polyscope::view::getCameraViewMatrix();
        glm::mat4 viewT = glm::transpose(view);
        auto proj = polyscope::view::getCameraPerspectiveMatrix();
        glm::mat4 projT = glm::transpose(proj);
        auto pos = polyscope::view::getCameraWorldPosition();
        ImGui::SetNextWindowPos(ImVec2(viewsz.x - 510 , 0), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(500, 0), ImGuiCond_Once);
        
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
            //torusN.HarmonicShape();
            //torusN.LaplacianSmooth();
            psG.first->remove();
            psG.second->remove();

            psG = addGenus(torusN);
        }

        if (ImGui::SliderFloat("cDist", &cDist, 0.5, 0.8))
        {
            torusN.cDist = cDist;
            
            torusN.buildV();
            psG.first->updateVertexPositions(torusN.V);
            Eigen::MatrixXd V2 = torusN.V(torusN.b, { 0,1,2 });
            psG.second->updatePointPositions(V2);
        }
        if (ImGui::SliderFloat("R", &R, 0.4, 0.5))
        {
            torusN.R = R;
            torusN.buildV();
            psG.first->updateVertexPositions(torusN.V);
            
            Eigen::MatrixXd V2 = torusN.V(torusN.b, { 0,1,2 });
            psG.second->updatePointPositions(V2);
        }
        if (ImGui::SliderFloat("r", &r, 0.4, 0.5))
        {
            torusN.R = r;
            torusN.buildV();
            psG.first->updateVertexPositions(torusN.V);
            Eigen::MatrixXd V2 = torusN.V(torusN.b, { 0,1,2 });
            psG.second->updatePointPositions(V2);
        }
        if (ImGui::Button("Harmonic Shape"))
        {
            torusN.HarmonicShape();
            torusN.LaplacianSmooth();
            psG.first->updateVertexPositions(torusN.V);
            Eigen::MatrixXd V2 = torusN.V(torusN.b, { 0,1,2 });
            psG.second->updatePointPositions(V2);
        }
        ImGui::SameLine();
        if (ImGui::Button("Laplacian Smooth"))
        {
            torusN.LaplacianSmooth();
            psG.first->updateVertexPositions(torusN.V);
            Eigen::MatrixXd V2 = torusN.V(torusN.b, { 0,1,2 });
            psG.second->updatePointPositions(V2);
            //psG.first->updateObjectSpaceBounds();
            //glm::vec3 campos = std::get<1>(psG.first->objectSpaceBoundingBox);
            //campos *= 2;
            //polyscope::view::lookAt(campos , {0,0,0}, true);
        }
        //ImGui::End();
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

