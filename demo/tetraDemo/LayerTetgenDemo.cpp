#include "tetgenWrapper.h"
#include "NTorus.h"

#include "MeshIO.h"
#include "igl/matrix_to_list.h"
#include "igl/list_to_matrix.h"
#include "igl/barycenter.h"

//#include "occStepReader.h"
//#include "netgenDemo.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#include <imgui.h>
#include <Eigen/Dense>
#include "imguizmo/ImGuizmo.h"


#include <GLFW/glfw3.h>
ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };
void errorCallback(int error, const char* description) {}

void computeCellCentroids(TetMesh& egm, Eigen::MatrixXd& cellCentroids)
{
    Eigen::MatrixXd eV;
    Eigen::MatrixX<int>  eT;
    igl::list_to_matrix(egm.V, eV);
    igl::list_to_matrix(egm.T, eT);
    igl::barycenter(eV, eT, cellCentroids);
    std::cout << "cellCentroids:" << eT.rows() << std::endl;
}

int surfaceCount = 0;
polyscope::SurfaceMesh* addSurface(TriMesh& m)
{
    auto ps = polyscope::registerSurfaceMesh(std::to_string(surfaceCount++), m.V, m.F);
    ps->setEdgeWidth(1);
    ps->setSurfaceColor(polyscope::getNextUniqueColor());
    ps->setBackFaceColor({0.1,0.2,0.3});
    //ps->setTransparency(0.5);
    ps->setBackFacePolicy(polyscope::BackFacePolicy::Custom);
    return ps;
}

void newTorusN(TriMesh& shape)
{
    TorusN* torusN = new TorusN();
    torusN->R = 3;
    torusN->r = 1;
    torusN->cDist = 6;
    torusN->buildMesh(3);
    torusN->HarmonicShape();
    shape.V = torusN->V;
    shape.F = torusN->F;
}

int main(const int, const char**)
{
    polyscope::options::programName = "Nine Cube";
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
    polyscope::view::style = polyscope::view::NavigateStyle::Free;
    polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
    polyscope::init();
    glfwSetErrorCallback(errorCallback);


    
    TriMesh shape;
    LaplacianSmoothing smoother;
    //loadOBJ("D:/projects/GeometryLab/data/spot/spot_triangulated.obj", &shape);
    newTorusN(shape);
    smoother.init(shape.V,shape.F);
    auto psG=addSurface(shape);
    double delta = 0.00001;
    int iterN = 100;
    polyscope::state::userCallback = [&]()
    {

        auto viewsz = ImGui::GetMainViewport()->Size;

        auto view = polyscope::view::getCameraViewMatrix();
        glm::mat4 viewT = glm::transpose(view);
        auto proj = polyscope::view::getCameraPerspectiveMatrix();
        glm::mat4 projT = glm::transpose(proj);
        auto pos = polyscope::view::getCameraWorldPosition();
        ImGui::SetNextWindowPos(ImVec2(viewsz.x - 510, 0), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(500, 0), ImGuiCond_Once);

        if (ImGui::Button("Laplacian Smooth"))
        {
            for(int i=0;i<iterN;i++)
                smoother.deltaL(delta);
            if(iterN>=0)
                iterN--;
            igl::matrix_to_list(smoother.V, shape.V);
            //delta /= 0.6;
            //psG->updateVertexPositions(torusN.V);
            addSurface(shape);
        }
        ImGui::End();
    };

    polyscope::show();

    return 0;
}

