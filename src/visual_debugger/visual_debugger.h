#ifndef VISUAL_DEBUGGER_H
#define VISUAL_DEBUGGER_H

#include <eigen/Core>

#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/types.h"
typedef polyscope::SurfaceMesh PS_Mesh;

class Visual
{
public:
	Eigen::MatrixXd clrs;
	std::vector<glm::vec3> cmap;
	int winw = 1920, winh = 1080;
	std::vector<polyscope::PointCloud* > points;
	PS_Mesh* misc;
	Visual()
	{
		polyscope::init();
		polyscope::view::style = polyscope::NavigateStyle::Free;
		misc = polyscope::registerSurfaceMesh("misc", Eigen::MatrixXd(), Eigen::MatrixXi());
		for (int i = 0; i < 50; i++)
			cmap.push_back(polyscope::getNextUniqueColor());
	};
	void init();

	//Thing* addMesh(EigenMeshD& m)
	//{
	//	Thing* o = new Thing();
	//	o->setmesh(m.V, m.F);
	//	ui->addObject(o);
	//	o->init_color(Eigen::RowVector3d(1, 1, 1));
	//	o->mesh->line_width = 2;
	//	o->mesh->point_size=6;
	//	return o;
	//}

	template<typename T>
	polyscope::SurfaceMesh* addMesh(T& m,const std::string& name)
	{
		return polyscope::registerSurfaceMesh(name, m.V, m.F);
	}

	void show()
	{

		
		polyscope::view::bgColor = { 0.1, 0.0, 0.2, 1.0f };

		polyscope::view::style = polyscope::view::NavigateStyle::Free;
		polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
		polyscope::options::automaticallyComputeSceneExtents = true;
		polyscope::view::processZoom(0.2);
		polyscope::show();
		//ui->launch_rendering(winw, winh, true);
	}

	glm::vec3 getScrnPos(float x,float y,float z)
	{
		//Eigen::Matrix4f modelview = ui->viewgl().view * ui->viewgl().modelmat;
		//Eigen::Vector3f p(x, y, z);
		//Eigen::Vector3f coord = igl::project(p, modelview, ui->viewgl().proj, ui->viewgl().viewport);
		glm::vec4 viewport = { 0., 0., polyscope::view::windowWidth, polyscope::view::windowHeight };
		auto coord = glm::project({ x,y,z }, polyscope::view::getCameraViewMatrix(), polyscope::view::getCameraPerspectiveMatrix(), viewport);
		return coord;
	}
	
	void addPolyLine(PS_Mesh* m,std::vector<glm::vec3>& pvec, glm::vec3 clr=polyscope::render::RGB_RED)
	{
		if (m == nullptr)
			m = polyscope::getSurfaceMesh("misc");
		std::vector< std::vector<glm::vec3> > v;
		v.push_back(pvec);
		auto pathQ = m->addSurfaceGraphQuantity(std::to_string(m->quantities.size()), v);
		pathQ->setEnabled(true);
		pathQ->setColor(clr);
		pathQ->setRadius(0.1,false);
		
	}



	void addPolyLine(PS_Mesh* m, std::vector < std::vector<glm::vec3> >& pvec, glm::vec3 clr = polyscope::render::RGB_RED)
	{
		if (m == nullptr)
			m = polyscope::getSurfaceMesh("misc");
		auto pathQ = m->addSurfaceGraphQuantity(std::to_string(m->quantities.size()), pvec);
		pathQ->setEnabled(true);
		pathQ->setColor(clr);
		pathQ->setRadius(0.1, false);

	}

	polyscope::PointCloud* addPoints(Eigen::MatrixXf& pvec, Eigen::MatrixXf& clr,float r,std::string name = "")
	{
		if (name.size() == 0)
			name=std::to_string(points.size());
		auto pc = polyscope::registerPointCloud(name, pvec);
		
		auto clrQ= pc->addColorQuantity("clr",clr);
		clrQ->setEnabled(true);
		pc->setPointRadius(r, false);
		points.push_back(pc);
		
		return pc;
	}

	polyscope::PointCloud* addPoints(std::vector<glm::vec3>& pvec, std::vector<glm::vec3>& clr, float r, std::string name = "")
	{
		if (name.size() == 0)
			name = std::to_string(points.size());
		auto pc = polyscope::registerPointCloud(name, pvec);
		auto clrQ = pc->addColorQuantity("clr", clr);
		clrQ->setEnabled(true);
		pc->setPointRadius(r, false);
		points.push_back(pc);
		return pc;
	}


	//void centerModelVec(std::vector<PS_Mesh* >& ovec)
	//{
	//	auto minrow = [](Eigen::RowVector3d& d, Eigen::RowVector3d& s)
	//	{
	//		for (int i = 0; i < d.cols(); i++)
	//		{
	//			d(i) = std::min(d(i), s(i));
	//		}
	//	};

	//	auto maxrow = [](Eigen::RowVector3d& d, Eigen::RowVector3d& s)
	//	{
	//		for (int i = 0; i < d.cols(); i++)
	//		{
	//			d(i) = std::max(d(i), s(i));
	//		}
	//	};


	//		Eigen::RowVector3d min_point(999, 999, 999);
	//		Eigen::RowVector3d max_point(-999, -999, -999);
	//		for (auto o : ovec)
	//		{
	//			Eigen::RowVector3d minp = o.V.colwise().minCoeff().eval();
	//			Eigen::RowVector3d maxp = o.V.colwise().maxCoeff().eval();
	//			minrow(min_point, minp);
	//			maxrow(max_point, maxp);
	//		}

	//		auto centroid = (0.5 * (min_point.row(0) + max_point.row(0))).eval();
	//		polyscope::view::processZoom(2.0 / (max_point - min_point).array().abs().maxCoeff());
	//		//viewgl.camera_base_translation.setConstant(0);
	//		//viewgl.camera_base_translation.head(centroid.size()) = -centroid.cast<float>();
	//		//for (int i = 0; i < 3; i++)
	//		//{
	//		//	if (abs(viewgl.camera_base_translation(i)) < 0.00001)
	//		//		viewgl.camera_base_translation(i) = ((viewgl.camera_base_translation(i) > 0) * 2 - 1) * 0.00001;
	//		//}
	//		//viewgl.camera_base_zoom = 2.0 / (max_point - min_point).array().abs().maxCoeff();
	//	}



};



#endif // !VISUAL_DEBUGGER_H
