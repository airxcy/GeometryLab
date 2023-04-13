
//#include "netgen/meshing.hpp"

int main(int* argc, char** argv)
{
	return 0;
}
/*
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "Eigen/Core"
#include "glad/glad.h"
#include "imgui/imgui.h"

typedef polyscope::SurfaceMesh PS_Mesh;

int main(int* argc, char** argv)
{
	polyscope::init();
	polyscope::options::programName = "2D line";
	polyscope::view::style = polyscope::NavigateStyle::Planar;
	polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
	
	polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
	Eigen::MatrixXd V(5,2);
	V << 0, 0, 
		0, 1, 
		1, 1, 
		2, 3,
		1,0;
	Eigen::MatrixXi F(1, 5);
	F << 0, 1, 2, 3,4;
	auto  misc = polyscope::registerSurfaceMesh2D("misc", V,F);
	
	//glBegin(GL_LINES);
	//	glVertex2f(0, 1);
	//	glVertex2f(1, 0);
	//glEnd();
	
	
	polyscope::state::userCallback = [&]()
	{
		ImGui::GetStyle().AntiAliasedLines=false;

		auto viewsz=ImGui::GetMainViewport()->Size;
		ImGui::SetNextWindowPos({0,0});
		ImGui::SetNextWindowSize(viewsz);
		ImGui::SetNextWindowFocus();
		ImGui::SetNextWindowBgAlpha(1);
		ImGui::Begin("Canvas",nullptr, ImGuiWindowFlags_NoDecoration);

		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		draw_list->AddRectFilled({ 0,0 }, viewsz, IM_COL32(50, 50, 50, 255));
		draw_list->AddRect({ 0,0 }, viewsz, IM_COL32(255, 255, 255, 255));
		draw_list->PushClipRect({ 0,0 }, viewsz, true);
		const float GRID_STEP = 64.0f;
		for (float x = fmodf(0, GRID_STEP); x < viewsz.x; x += GRID_STEP)
			draw_list->AddLine(ImVec2( x, 0), ImVec2( x, viewsz.y), IM_COL32(200, 200, 200, 40),1);
		for (float y = fmodf(0, GRID_STEP); y < viewsz.y; y += GRID_STEP)
			draw_list->AddLine(ImVec2(0, y), ImVec2(viewsz.x, y), IM_COL32(200, 200, 200, 40),1);
		draw_list->AddLine({ 0,0 }, viewsz, IM_COL32(255, 255,255,255),1);
		draw_list->PopClipRect();
		ImGui::End();
	};
	
	polyscope::show();

}

*/