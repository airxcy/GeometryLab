
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

void addPlane(Eigen::MatrixXf m,TriMesh& mesh)
{
    using namespace Eigen;
    //Vector3f xax = m.col(0);
    //Vector3f yax = m.col(1);
    //Vector3f yax = m.col(2);
    
    float x = 0, y = 0, z = 0;
    int i0 = mesh.V.size();
    for (int i = 0; i < 100; i++)
    {
        x += 0.001;
        y = 0;
        for (int j = 0; j < 100; j++)
        {
            y += 0.001;
            Vector4f p(x,y,0,1);
            Vector4f p1=m*p;
            mesh.V.push_back({ p1(0),p1(1),p1(2) });
        }
    }
    
    for (int i = 0; i < 99; i++)
    {
        for (int j = 0; j < 99; j++)
        {
            int v1 = i0 + i*100+j;
            int v2 = i0 + (i+1)*100 + j;
            int v3 = i0 + (i + 1) * 100 + j+1;
            int v4 = i0 + i * 100 + j + 1;
            mesh.F.push_back({ v1,v2,v3 });
            mesh.F.push_back({ v3,v4,v1 });
        }
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
    //occStepReader occStep;
    //occStep.read("D:/data/test_model_step203.stp");
    //occStep.occTri2Eigen();
    //demo.occ2Surface(occStep.shape);
    //demo.tetralization();
    //occTri2Eigen(shape);
    TriMesh egm;
    loadOBJ("D:/projects/GeometryLab/data/Gear_Spur_16T.obj", &egm);
    Eigen::Matrix4f planem=Eigen::Matrix4f::Identity();
    planem.col(3) << 0.1, 0, 0, 1;
    addPlane(planem, egm);
    auto plym = polyscope::registerSurfaceMesh("eigen", egm.V, egm.F);
    plym->setSurfaceColor({ 0,1,0 });
    plym->setTransparency(0.3);
    plym->setEdgeWidth(1);
    plym->setEdgeColor({ 0.2,0.5,0.8 });
    TetgenWrapper tet;
    tet.initSurfaceMesh(egm);
    tet.run();
    Eigen::MatrixXd cellCentroids;
    computeCellCentroids(tet.mesh,cellCentroids);
    auto pstet= polyscope::registerSurfaceMesh("tetgen", tet.mesh.V, tet.mesh.F);
    pstet->setEdgeWidth(1);
    pstet->setSurfaceColor({ 1,1,0 });
    pstet->setEdgeColor({0.8,0.5,0.2});
    auto steinerPC = polyscope::registerPointCloud("steiner", tet.steinerVertex);
    steinerPC->setPointColor({ 1,0,0 });
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
            for (int i = 0; i < tet.mesh.nT; i++)
            {
                if (d(i) > 0)
                    selset.insert(tet.mesh.TF[i].begin(), tet.mesh.TF[i].end());
            }
        }
        ImGuizmo::DrawGrid(glm::value_ptr(view), glm::value_ptr(proj), gizmomat.data(), 10.f);
        ImGui::End();

        if (selset.size() > 0)
        {
            std::vector<std::vector<size_t> > newfaces;
            for (int fi : selset)
                newfaces.push_back({ (size_t)tet.mesh.F[fi][0],(size_t)tet.mesh.F[fi][1] ,(size_t)tet.mesh.F[fi][2] });
            pstet->faces = newfaces;
            pstet->updateObjectSpaceBounds();
            pstet->computeCounts();
            pstet->geometryChanged();
        }
    };
      
      polyscope::show();
      
  return 0;
}

