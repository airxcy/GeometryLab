#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "Eigen/Core"
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"
#define _USE_MATH_DEFINES
#include <math.h>
typedef polyscope::SurfaceMesh PS_Mesh;
void errorCallback(int error, const char* description) {}
void psinit()
{	
	
	glfwInit();
	auto vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	std::cout << "Video mode width: " + vidmode->width << std::endl;;
	std::cout << "Video mode height: " + vidmode->height << std::endl;
	polyscope::view::windowWidth = vidmode->width;
	polyscope::view::windowHeight = vidmode->height;
	polyscope::options::programName = "view2d";
	polyscope::view::style = polyscope::NavigateStyle::Planar;
	polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
	polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };
	polyscope::init();
	glfwSetErrorCallback(errorCallback);
	Eigen::MatrixXd V(5, 2);
	V << 0, 0,
		0, 1,
		1, 1,
		2, 3,
		1, 0;
	Eigen::MatrixXi F(1, 5);
	F << 0, 1, 2, 3, 4;
	auto  misc = polyscope::registerSurfaceMesh2D("misc", V, F);
}


class Curve2D
{
public:
	int N = 0;
	std::vector<double> points;
	std::vector<double> R;
	void addP(double x, double y,double r)
	{
		points.push_back(x);
		points.push_back(y);
		R.push_back(r);
		N++;
	}

	double* operator () (int i)
	{
		return points.data() + i * 2;
	}

	void clear()
	{
		points.clear();
		N = 0;
	}
};

void hyperbolicSpiral(Curve2D& spiral)
{
	double phi = M_PI, r = 2000 / phi;
	while (spiral.N < 1000)
	{
		spiral.addP(r * cos(phi), r * sin(phi),r);
		phi += 0.01 * M_PI;
		r = 2000 / phi;
	}
}

void archimedeanSpiral(Curve2D& spiral)
{
	double phi = M_PI, r = 10* phi;
	while (spiral.N < 1000)
	{
		spiral.addP(r * cos(phi), r * sin(phi),r);
		phi += 0.01 * M_PI;
		r = 10* phi;
	}
}

void Circle(Curve2D& spiral)
{
	double phi = 0, r = 500;
	while (phi<M_PI*2)
	{
		spiral.addP(r * cos(phi), r * sin(phi),r);
		phi += 0.01 * M_PI;
	}
}

void simplifyCurve(Curve2D& out, Curve2D& curve, double angle)
{
	
	double* p1 = curve(0);
	double* p2 = curve(1);
	double dx = p2[0] - p1[0];
	double dy = p2[1] - p1[1];
	double dn = sqrt(dx * dx + dy * dy);
	double phi = 0;
	double r = sqrt(p1[0] * p1[0] + p1[1] * p1[1]);
	out.addP(p1[0], p1[1], r);
	for (int i = 0; i < curve.N-1; i+=1)
	{
		p1 = curve(i);
		p2 = curve(i+1);
		double dx1 = p2[0] - p1[0];
		double dy1 = p2[1] - p1[1];
		double dn1 = sqrt(dx1 * dx1 + dy1 * dy1);
		double cosval=(dx1 * dx + dy1 * dy) / dn / dn1;
		if (cosval > 1)cosval = 1;
		if (cosval < -1)cosval = -1;
		
		double aval = acos(cosval);
		phi += aval;
		
		if (abs(phi) > angle)
		{
			phi = 0;
			r = sqrt(p1[0] * p1[0] + p1[1] * p1[1]);
			out.addP(p1[0], p1[1], r);
		}
		dx = dx1;
		dy = dy1;
		dn = dn1;

	}
	out.addP(p2[0], p2[1], r);
}
int main(int* argc, char** argv)
{
	psinit();
	Curve2D spiral,polyline;
	//hyperbolicSpiral(spiral);
	Circle(spiral);
	int divN = 10;
	simplifyCurve(polyline, spiral, M_PI/ divN);

	bool showSimplfied=true,showOriginal=true;
	ImGui::GetStyle().AntiAliasedLines = false;
	polyscope::state::userCallback = [&]()
	{
		auto viewsz = ImGui::GetMainViewport()->Size;
		ImGui::SetNextWindowPos({0,0});
		ImGui::SetNextWindowSize(viewsz);
		ImGui::SetNextWindowFocus();
		ImGui::SetNextWindowBgAlpha(1);
		ImGui::Begin("Canvas",nullptr, ImGuiWindowFlags_NoDecoration);
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		ImVec2 ORG(viewsz.x/2, viewsz.y/2);
		draw_list->AddRectFilled({ 0,0 }, viewsz, IM_COL32(50, 50, 50, 255));
		draw_list->AddRect({ 0,0 }, viewsz, IM_COL32(255, 255, 255, 255));
		draw_list->PushClipRect({ 0,0 }, viewsz, true);
		const float GRID_STEP = 64.0f;
		for (float x = fmodf(0, GRID_STEP); x < viewsz.x; x += GRID_STEP)
			draw_list->AddLine(ImVec2(x, 0), ImVec2(x, viewsz.y), IM_COL32(200, 200, 200, 40), 1);
		for (float y = fmodf(0, GRID_STEP); y < viewsz.y; y += GRID_STEP)
			draw_list->AddLine(ImVec2(0, y), ImVec2(viewsz.x, y), IM_COL32(200, 200, 200, 40), 1);
		if (ImGui::Button("showSimplfied"))showSimplfied = !showSimplfied;
		ImGui::SameLine();
		if (ImGui::Button("showOriginal"))showOriginal = !showOriginal;
		if (ImGui::Button("Hyperbolic Spiral"))
		{
			spiral.clear();
			hyperbolicSpiral(spiral);
			polyline.clear();
			simplifyCurve(polyline, spiral, M_PI / divN);
		}
		ImGui::SameLine();
		if (ImGui::Button("Archimedian Spiral"))
		{
			spiral.clear();
			archimedeanSpiral(spiral);
			polyline.clear();
			simplifyCurve(polyline, spiral, M_PI / divN);
		}
		ImGui::SameLine();
		if (ImGui::Button("Circle"))
		{
			spiral.clear();
			Circle(spiral);
			polyline.clear();
			simplifyCurve(polyline, spiral, M_PI / divN);
		}
		
		if (ImGui::SliderInt("label", &divN, 2, 10,"%d"))
		{
			polyline.clear();
			simplifyCurve(polyline, spiral, M_PI / divN);
		}
		if(showOriginal)
		for (int i = 0; i < spiral.N-1; i++)
		{
			float val = float(i) / spiral.N;
			auto p1 = spiral(i);
			auto p2 = spiral(i + 1);
			
			draw_list->AddLine(ImVec2(p1[0]+ ORG.x,p1[1]+ ORG.y), ImVec2(p2[0] + ORG.x, p2[1] + ORG.y), ImColor(val,1-val,0.0), 1);
			
		}
		
		if(showSimplfied)
		for (int i = 0; i < polyline.N - 1; i++)
		{
			auto p1 = polyline(i);
			auto p2 = polyline(i + 1);
			draw_list->AddLine(ImVec2(p1[0] + ORG.x, p1[1] + ORG.y), ImVec2(p2[0] + ORG.x, p2[1] + ORG.y), IM_COL32(255, 255, 255, 255), 1);
		}

		ImGui::End();
	};
	
	polyscope::show();

}