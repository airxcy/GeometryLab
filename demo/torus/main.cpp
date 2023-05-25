#include "NTorus.h"
#include "MeshIO.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/types.h"


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


std::pair<polyscope::SurfaceMesh*,polyscope::PointCloud* > addGenus(TorusN& m)
{
    //std::vector< std::vector<int> >bnd;
    //igl::boundary_loop(m.F, bnd);
    //if (bnd.size() == 0)
    //    std::cout << "is water tight" << std::endl;
    //std::vector<std::vector<float> > V2; 
    //for (int i : m.stdb)
    //    V2.push_back({ m.V[i][0],m.V[i][1] ,m.V[i][2] });
    //auto pc1 = polyscope::registerPointCloud("genus", V2);
    //pc1->setPointColor({ 0,0,1 });
    //auto cQ = pc1->addColorQuantity("clr", m.clrs);
    //pc1->setPointRadius(0.01,false);
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

    return { ps ,nullptr};
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
    XMesh<double,int> example;
    loadOBJ("D:/projects/GeometryLab/data/double-torus.obj",&example);
    //auto ps1 = polyscope::registerSurfaceMesh("DoubleTorus", example.V, example.F);
    //ps1->setSurfaceColor({ 0,1,0 });
    //ps1->setTransform({ 0,0,1,0,0,1,0,0,-1,0,0,0,0,0,0,1 });
    //ps1->setTransparency(0.5);
    //ps1->setSmoothShade(true);
    //ps1->setEdgeWidth(1);
    TorusN torusN;
    double maxv = -999, minv = 999;
    for (auto v : example.V)
    {
        if (maxv < v[0])maxv = v[0];
        if (minv > v[0])minv = v[0];
    }
    torusN.R = (maxv- minv) / 2;
    torusN.r = torusN.R / 4;
    maxv = -999, minv = 999;
    for (auto v : example.V)
    {
        if (maxv < v[2])maxv = v[2];
        if (minv > v[2])minv = v[2];
    }
    torusN.cDist = (maxv - minv)/2 - torusN.R- torusN.r;
    torusN.R = torusN.R - torusN.r;


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
            //psG.second->remove();

            psG = addGenus(torusN);
        }

        if (ImGui::SliderFloat("cDist", &cDist, 0.5, 0.8))
        {
            torusN.cDist = cDist;
            torusN.buildV();
            psG.first->updateVertexPositions(torusN.V);
        }
        if (ImGui::SliderFloat("R", &R, 0.4, 0.5))
        {
            torusN.R = R;
            torusN.buildV();
            psG.first->updateVertexPositions(torusN.V);
          
        }
        if (ImGui::SliderFloat("r", &r, 0.4, 0.5))
        {
            torusN.R = r;
            torusN.buildV();
            psG.first->updateVertexPositions(torusN.V);
        }
        if (ImGui::Button("Harmonic Shape"))
        {
            torusN.HarmonicShape();
            psG.first->updateVertexPositions(torusN.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Laplacian Smooth"))
        {
            torusN.LaplacianSmooth(0.5);
            psG.first->updateVertexPositions(torusN.V);
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

