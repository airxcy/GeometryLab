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
int main(const int, const char**)
{
	polyscope::options::programName = "OCC2NetGen";
	polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
	polyscope::view::style = polyscope::view::NavigateStyle::Free;
	polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
	polyscope::init();
	glfwSetErrorCallback(errorCallback);

	occStepReader occStep;
	occStep.read("D:/data/test_model_step203.stp");
	std::vector< XMesh<double, int> > meshlist;
	occStep.occTri(meshlist);
	
	//demo.tetralization();
	//occStep.occTri2Eigen(shape);
	//for (int i=0;i<meshlist.size();i++)
	//{
	//	polyscope::registerSurfaceMesh(std::to_string(i), meshlist[i].V,meshlist[i].F);
	//}
	
	NetGenWrapper demo;
	//netgen::MeshingParameters& mp = demo.meshParam();
	//mp.grading = 0.1;
	//mp.optsteps3d = 0;
	//mp.blockfill = false;
	//mp.uselocalh = false;
	//mp.delaunay = false;
	//mp.delaunay2d = false;
	//mp.checkoverlap = false;
	//mp.checkchartboundary = false;
	//mp.checkchartboundary = false;
	//nglib::Ng_Init();
	//demo.mesh.AddFaceDescriptor(netgen::FaceDescriptor(1, 1, 0, 1));
	demo.occ2Surface(occStep.shape);
	
	//for(auto& m:meshlist)
	//	demo.fromEigen(m);
	demo.tetralization();
	
	VolumeMesh<double,int> mesh;
	demo.toEigen(mesh);
	polyscope::registerSurfaceMesh("netgen", mesh.V, mesh.F);
	polyscope::show();
}