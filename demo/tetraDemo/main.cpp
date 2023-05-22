
#include "MeshIO.h"
#include "igl/matrix_to_list.h"
#include "igl/list_to_matrix.h"
#include "igl/barycenter.h"

#include "occStepReader.h"
#include "netgenDemo.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "tetgenWrapper.h"


#include <imgui.h>
#include <Eigen/Dense>
#include "imguizmo/ImGuizmo.h"


//#include <imgui_impl_glfw.h>
//#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };
void errorCallback(int error, const char* description){}

void computeCellCentroids(TetMesh& egm, Eigen::MatrixXd& cellCentroids)
{
    Eigen::MatrixXd eV;
    Eigen::MatrixX<int>  eT;
    igl::list_to_matrix(egm.V, eV);
    igl::list_to_matrix(egm.T, eT);
    igl::barycenter(eV, eT, cellCentroids);
    std::cout << "cellCentroids:" << eT.rows() << std::endl;
}

void addCube(TetgenWrapper& tet, Eigen::MatrixXd& m,std::vector<int>& nDiv)
{
    using namespace Eigen;
    //Vector3f xax = m.col(0);
    //Vector3f yax = m.col(1);
    //Vector3f yax = m.col(2);
    int NX = nDiv[0];
    int NY = nDiv[1];
    int NZ = nDiv[2];
    RowVector3d p0 = m.row(0);
    RowVector3d axx = m.row(1) /NX;
    RowVector3d axy = m.row(2) /NY;
    RowVector3d axz = m.row(3) /NZ;
    //axx /= NX;
    //axy /= NY;
    //axz /= NZ;
    TriMesh mesh;
    double x = m(0,0), y = m(0,1), z = (0,2);

    for (int i = 0; i < NX; i++)
    {
        RowVector3d pi = p0 + axx * i;
        for (int j = 0; j < NY; j++)
        {
            RowVector3d pj = pi + axy * j;
            for (int k = 0; k < NZ; k++)
            {
                RowVector3d pk = pj + axz * k;
            }
        }
    }
    
    for (int i = 0; i < NX-1; i++)
    {
        for (int j = 0; j < NY-1; j++)
        {
            int v1 =   i* NY +j;
            int v2 =  (i+1)* NY + j;
            int v3 = (i + 1) * NY + j+1;
            int v4 = i * NY + j + 1;
            mesh.F.push_back({ v1,v2,v3 });
            mesh.F.push_back({ v3,v4,v1 });
        }
    }

    tet.addSurface(mesh, 2);
}

void scale_move(TriMesh& m, double scale, double x=0, double y=0, double z=0)
{
    for (auto& v : m.V)
    {
        v[0] = v[0] * scale + x;
        v[1] = v[1] * scale + y;
        v[2] = v[2] * scale + z;
    }
}

int main (const int, const char**)
{
    polyscope::options::programName = "TetraDemo";
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::view::style = polyscope::view::NavigateStyle::Free;
    polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };    
    polyscope::init();
    glfwSetErrorCallback(errorCallback);
    auto  misc = polyscope::registerSurfaceMesh2D("misc", Eigen::MatrixXd(), Eigen::MatrixXi());

    TriMesh egm,egm2;
    loadOBJ("D:/projects/GeometryLab/data/spot/spot_triangulated.obj", &egm);
    loadOBJ("D:/projects/GeometryLab/data/Gear_Spur_16T.obj", &egm2);
    
    scale_move(egm,3,0,0,-1);
    TetgenWrapper tet;
    tet.addSurface(egm,1);
    tet.addSurface(egm2, 2);
    tet.convertInput();
    
    tet.addRegion(-0.2, 0.1, 0);
    tet.addRegion(0, 0, 0);
    tet.run();
    Eigen::MatrixXd cellCentroids;
    computeCellCentroids(tet.m_mesh,cellCentroids);

    auto plym = polyscope::registerSurfaceMesh("eigen", tet.plc_mesh.V, tet.plc_mesh.F);
    //plym->setSurfaceColor({ 0,1,0 });
    std::unordered_map<int , glm::vec3> clrs;
    for (int i : tet.m_FmarkerID)
    {
        std::cout << "marker: " << i << std::endl;
        clrs[i] = polyscope::getNextUniqueColor();
    }
    std::vector<glm::vec3> clrmap;
    for (int i : tet.m_FMarkers)
        clrmap.push_back(clrs[i]);
    auto clrQ= plym->addFaceColorQuantity("boundary Marker", clrmap);
    clrQ->setEnabled(true);
    plym->setTransparency(0.3);
    plym->setEdgeWidth(1);



    auto pstet = polyscope::registerSurfaceMesh("tetgen", tet.m_mesh.V, tet.m_mesh.F);
    pstet->setEdgeWidth(1);
    pstet->setSurfaceColor({ 1,1,0 });
    pstet->setEdgeColor({ 0.8,0.5,0.2 });
    //auto steinerPC = polyscope::registerPointCloud("steiner", tet.steinerVertex);
    //steinerPC->setPointColor({ 1,0,0 });



    ImGui::GetStyle().AntiAliasedLines = false;
    Eigen::Matrix4f gizmomat=Eigen::Matrix4f::Identity();
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
        std::set<int> selset;
        if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), mCurrentGizmoOperation, ImGuizmo::LOCAL, gizmomat.data()))
        {
            float matrixTranslation[3], matrixRotation[3], matrixScale[3];
            ImGuizmo::DecomposeMatrixToComponents(gizmomat.data(), matrixTranslation, matrixRotation, matrixScale);
            Eigen::RowVector3d p((double)matrixTranslation[0], (double)matrixTranslation[1], (double)matrixTranslation[2]);
            Eigen::Vector3d n= gizmomat.col(1).topRows(3).normalized().cast<double>();
            Eigen::VectorXd d = (cellCentroids.rowwise() -p)*n;
            for (int i = 0; i < tet.m_mesh.nT; i++)
            {
                if (d(i) > 0)
                    selset.insert(tet.m_mesh.TF[i].begin(), tet.m_mesh.TF[i].end());
            }
        }
        ImGuizmo::DrawGrid(glm::value_ptr(view), glm::value_ptr(proj), gizmomat.data(), 100.f);
        ImGui::End();

        if (selset.size() > 0)
        {
            std::vector<std::vector<size_t> > newfaces;
            //std::vector<glm::vec3> cmap;
            for (int fi : selset)
            {
                newfaces.push_back({ (size_t)tet.m_mesh.F[fi][0],(size_t)tet.m_mesh.F[fi][1] ,(size_t)tet.m_mesh.F[fi][2] });
                //cmap.push_back(clrs[tet.Fregionlist[fi]]);
            }
            pstet->faces = newfaces;
            //auto cQ=pstet->addFaceColorQuantity("region", cmap);
            //cQ->setEnabled(true);
            pstet->updateObjectSpaceBounds();
            pstet->computeCounts();
            pstet->geometryChanged();
        }
    };
      
      polyscope::show();
      
  return 0;
}

