#include "occStepReader.h"
#include "netgenWrapper.h"


#include "MeshIO.h"
#include "igl/matrix_to_list.h"
#include "igl/list_to_matrix.h"
#include "igl/barycenter.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"


#include <imgui.h>
#include <Eigen/Dense>
#include "imguizmo/ImGuizmo.h"

#include <GLFW/glfw3.h>
void errorCallback(int error, const char* description) {}
ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };

void computeCellCentroids(TetMesh& egm, Eigen::MatrixXd& cellCentroids)
{
    Eigen::MatrixXd eV;
    Eigen::MatrixX<int>  eT;
    igl::list_to_matrix(egm.V, eV);
    igl::list_to_matrix(egm.T, eT);
    igl::barycenter(eV, eT, cellCentroids);
    std::cout << "cellCentroids:" << eT.rows() << std::endl;
}


int main(const int, const char**)
{
	polyscope::options::programName = "OCC2NetGen";
	polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
	polyscope::view::style = polyscope::view::NavigateStyle::Free;
	polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
	polyscope::init();
	glfwSetErrorCallback(errorCallback);

	occStepReader occStep;
	//occStep.read("D:/data/test_model_step203.stp");
    occStep.read("D:/data/sxwy/DiancifaNewNoGap2.brep");
	std::vector< XMesh<double, int> > meshlist;
	occStep.occTri(meshlist);
    int facenum = 0;
    for (auto& m : meshlist)
    {
        auto pt=polyscope::registerSurfaceMesh(std::to_string(facenum), m.V, m.F);
        pt->setTransparency(0.7);
    }
	NetGenWrapper demo;
	demo.addOCCSolid(occStep.shape);
	demo.tetralization();
	demo.translateOutput();
	TetMesh& mesh = demo.m_mesh;
    Eigen::MatrixXd cellCentroids;
    computeCellCentroids(mesh, cellCentroids);
	auto pstet = polyscope::registerSurfaceMesh("tetgen", mesh.V, mesh.F);
	pstet->setEdgeWidth(1);
	pstet->setSurfaceColor({ 1,1,0 });
	pstet->setEdgeColor({ 0.8,0.5,0.2 });
	Eigen::Matrix4f gizmomat = Eigen::Matrix4f::Identity();
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
            Eigen::Vector3d n = gizmomat.col(1).topRows(3).normalized().cast<double>();
            Eigen::VectorXd d = (cellCentroids.rowwise() - p) * n;
            for (int i = 0; i < mesh.nT; i++)
            {
                if (d(i) > 0)
                    selset.insert(mesh.TF[i].begin(), mesh.TF[i].end());
            }
        }
        ImGuizmo::DrawGrid(glm::value_ptr(view), glm::value_ptr(proj), gizmomat.data(), 1000.f);
        ImGui::End();

        if (selset.size() > 0)
        {
            std::vector<std::vector<size_t> > newfaces;
            //std::vector<glm::vec3> cmap;
            for (int fi : selset)
            {
                newfaces.push_back({ (size_t)mesh.F[fi][0],(size_t)mesh.F[fi][1] ,(size_t)mesh.F[fi][2] });
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
}