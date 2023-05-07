
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

void computeCellCentroids(VolumeMesh& egm, Eigen::MatrixXd& cellCentroids)
{
    Eigen::MatrixXd eV;
    Eigen::MatrixX<size_t> eT;
    igl::list_to_matrix(egm.V, eV);
    igl::list_to_matrix(egm.T, eT);
    igl::barycenter(eV, eT, cellCentroids);
    std::cout << "cellCentroids:" << eT.rows() << std::endl;
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
    
    //occStepReader occStep;
    //occStep.read("D:/data/test_model_step203.stp");
    //occStep.occTri2Eigen();
    //demo.occ2Surface(occStep.shape);
    //demo.tetralization();
    //occTri2Eigen(shape);
    VolumeMesh egm;
    loadOBJ("D:/projects/GeometryLab/data/Gear_Spur_16T.obj", &egm);
    //egm.loadOBJ("D:/projects/GeometryLab/data/cube.obj");
    auto plym = polyscope::registerSurfaceMesh("eigen", egm.V, egm.F);
    plym->setSurfaceColor({ 0,1,0 });
    plym->setTransparency(0.3);
    plym->setEdgeWidth(1);
    plym->setEdgeColor({ 0.2,0.5,0.8 });
    TetgenWrapper tet;
    tet.fromEigen(egm);
    tet.run();
    VolumeMesh egt;
    tet.toEigen(egt);
    Eigen::MatrixXd cellCentroids;
    computeCellCentroids(egt,cellCentroids);

    //auto pctet = polyscope::registerPointCloud("V", egt.V);
    auto pstet= polyscope::registerSurfaceMesh("tetgen", egt.V, egt.F);
    pstet->setEdgeWidth(1);
    pstet->setSurfaceColor({ 1,1,0 });
    pstet->setEdgeColor({0.8,0.5,0.2});

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
    int starti = 0;
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
        
        //ImGuizmo::DrawCubes(glm::value_ptr(view), glm::value_ptr(proj), gizmoMatrix,1);
        std::set<int> selset;
        if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), mCurrentGizmoOperation, ImGuizmo::LOCAL, gizmomat.data()))
        {
            float matrixTranslation[3], matrixRotation[3], matrixScale[3];
            ImGuizmo::DecomposeMatrixToComponents(gizmomat.data(), matrixTranslation, matrixRotation, matrixScale);
            Eigen::RowVector3d p((double)matrixTranslation[0], (double)matrixTranslation[1], (double)matrixTranslation[2]);
            Eigen::Vector3d n= gizmomat.col(1).topRows(3).normalized().cast<double>();
            Eigen::VectorXd d = (cellCentroids.rowwise() -p)*n;
            for (int i = 0; i < egt.nT; i++)
            {
                if (d(i) > 0)
                    selset.insert(egt.TF[i].begin(), egt.TF[i].end());
            }
        }
        ImGuizmo::DrawGrid(glm::value_ptr(view), glm::value_ptr(proj), gizmomat.data(), 10.f);
        ImGui::End();

        if (selset.size() > 0)
        {
            std::vector<std::vector<size_t> > newfaces;
            for (int fi : selset)
                newfaces.push_back(egt.F[fi]);
            pstet->faces = newfaces;
            pstet->updateObjectSpaceBounds();
            pstet->computeCounts();
            pstet->geometryChanged();
        }
        //EditTransform(glm::value_ptr(view), glm::value_ptr(proj), gizmoMatrix, mCurrentGizmoOperation);
        //polyscope::draw();
        //std::set<int> selset;
        //for (int i = starti; i < egt.nT; i += 2)
        //    selset.insert(egt.sTF[i].begin(), egt.sTF[i].end());
        //std::vector<std::vector<size_t> > newfaces;
        //for (int fi : selset)
        //    newfaces.push_back(egt.sF[fi]);
        //pstet->faces = newfaces;
        //pstet->updateObjectSpaceBounds();
        //pstet->computeCounts();
        //pstet->geometryChanged();
        //starti = 1 - starti;
        
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

