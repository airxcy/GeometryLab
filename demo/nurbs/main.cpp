#include "bezier.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"

#include <imgui.h>
#include <Eigen/Dense>
#include "imguizmo/ImGuizmo.h"

#include <GLFW/glfw3.h>
void errorCallback(int error, const char* description) {}
ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
polyscope::SurfaceMesh* misc;
polyscope::PointCloud* conrtrolpoints;
polyscope::SurfaceGraphQuantity* segments;
std::vector<polyscope::SurfaceGraphQuantity* > curves;
int divN = 100;
void addCurve(BezierCurve3& bezier)
{
    std::vector<glm::vec3> pointsdata(bezier.m_n);
    std::cout << bezier.m_d << std::endl;
    for(int i=0;i<bezier.m_n;i++)
        for (int j = 0; j < bezier.m_dim; j++)
            pointsdata[i][j] = bezier[i][j];
    conrtrolpoints = polyscope::registerPointCloud("ControlPoints", pointsdata);
    conrtrolpoints->setPointRadius(0.03, false);
    conrtrolpoints->setPointColor({ 1,1,1 });
    bezier.generateRC(100);
    curves.resize(bezier.m_d+1);
    for (int d = 0; d <= bezier.m_d; d++)
    {
        std::vector< glm::vec3 > nodes;
        std::vector<std::array<size_t, 2> > edges;
        for (int i = 0; i < bezier.m_n - d; i++)
        {
            int j = i + d;
            int cid = bezier.m_map[i * bezier.m_n + j]-1;
            for (int k=0;k<bezier.curves[cid].size();k++)
            {
                auto& p = bezier.curves[cid][k];
                nodes.push_back({ p[0],p[1],p[2] });
                if (k>0)
                    edges.push_back({ nodes.size() - 2,nodes.size() - 1 });
            }
        }
        curves[d] = misc->addSurfaceGraphQuantity(std::to_string(d)+"_degree", nodes, edges);
        float val = double(d) / bezier.m_d;
        curves[d]->setColor({val,1-val,0});
        //curves[d]->setEnabled(true);
    }
    curves[bezier.m_d]->setEnabled(true);
    segments = misc->addSurfaceGraphQuantity("segments", std::vector < std::vector< glm::vec3 > >( { pointsdata }));
    segments->setColor({ 0.3,0.3,0.3 });
    segments->setEnabled(true);
}

void updateRC(BezierCurve3& bezier)
{
    bezier.generateRC(100);
    for (int d = 0; d <= bezier.m_d; d++)
    {
        auto& nodes = curves[d]->nodes;
        auto& edges = curves[d]->edges;
        nodes.clear();
        edges.clear();
        for (int i = 0; i < bezier.m_n - d; i++)
        {
            int j = i + d;
            int cid = bezier.m_map[i * bezier.m_n + j] - 1;
            bool has_1 = false;

            for (int k = 0; k < bezier.curves[cid].size(); k++)
            {
                auto& p = bezier.curves[cid][k];
                nodes.push_back({ p[0],p[1],p[2] });
                if (k > 0)
                {
                    edges.push_back({ nodes.size() - 2,nodes.size() - 1 });
                }
            }
        }
        curves[d]->refresh();
        curves[d]->setEnabled(true);
    }
}


int main(const int, const char**)
{
    polyscope::options::programName = "Nine Cube";
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::view::style = polyscope::view::NavigateStyle::Free;
    polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
    polyscope::init();
    glfwSetErrorCallback(errorCallback);

    misc = polyscope::registerSurfaceMesh2D("misc", Eigen::MatrixXd(), Eigen::MatrixXi());

    int degree=5;
    BezierCurve3 bezier(degree);
    double drad = M_PI / degree;
    double rad = 0;
    for (int i = 0; i < bezier.m_n; i++)
    {
        bezier[i].coord[0] = sin(rad);
        bezier[i].coord[1] = cos(rad);
        bezier[i].coord[2] =  0 ;
        rad += drad;
    }
    addCurve(bezier);
    float gizmomat[16];
    memset(gizmomat, 0, 16);
    for (int i = 0; i < 4; i++)gizmomat[i * 4 + i]=1;
    polyscope::state::userCallback = [&]()
    {
        auto viewsz = ImGui::GetMainViewport()->Size;

        auto view = polyscope::view::getCameraViewMatrix();
        glm::mat4 viewT = glm::transpose(view);
        auto proj = polyscope::view::getCameraPerspectiveMatrix();
        glm::mat4 projT = glm::transpose(proj);
        auto pos = polyscope::view::getCameraWorldPosition();
        ImGui::SetNextWindowPos(ImVec2(viewsz.x - 210, 0), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(200, 0), ImGuiCond_Once);

        ImGui::Begin("gizmoControl", NULL, ImGuiWindowFlags_None);
        if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        ImGui::End();
        std::pair<polyscope::Structure*, size_t> selection=polyscope::pick::getSelection();
        if (selection.first == conrtrolpoints)
        {
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::BeginFrame();
            ImGuizmo::Enable(true);
            ImGuizmo::SetRect(0, 0, viewsz.x, viewsz.y);
            Point3 p = bezier[selection.second];
            gizmomat[12] = p[0];
            gizmomat[13] = p[1];
            gizmomat[14] = p[2];
            if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), mCurrentGizmoOperation, ImGuizmo::LOCAL, gizmomat))
            {
                bezier[selection.second][0] = gizmomat[12];
                bezier[selection.second][1] = gizmomat[13];
                bezier[selection.second][2] = gizmomat[14];
                for (int i = 0; i < bezier.m_n; i++)
                    for (int j = 0; j < bezier.m_dim; j++)
                    {
                        conrtrolpoints->points[i][j] = bezier[i][j];
                        segments->nodes[i][j] = bezier[i][j];
                    }
                conrtrolpoints->refresh();
                segments->refresh();
                int divN = 100;
                double div = 1.0 / divN, t = 0;
                curves[bezier.m_d]->nodes.clear();
                for (int i = 0; i < divN; i++)
                {
                    auto p=bezier(t);
                    t += div;
                    curves[bezier.m_d]->nodes.push_back({p[0],p[1],p[2]});
                }
                curves[bezier.m_d]->refresh();
            }
        }
        
    };
    polyscope::show();
}