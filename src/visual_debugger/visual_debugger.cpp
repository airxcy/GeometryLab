#include "visual_debugger.h"

#include "igl/readOBJ.h"
#include "EigenMeshD.h"
#include "visual_debugger.h"
#include <iostream>
#include <filesystem>

void Visual::init() {}


EigenMeshD load(std::string& path)
{
	EigenMeshD m;
	igl::readOBJ(path, m.V, m.F);
	return m;
}

inline bool exists_test3(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

std::vector<std::string> split(std::string s, std::string delimiter) {
	size_t pos_start = 0, pos_end, delim_len = delimiter.length();
	std::string token;
	std::vector<std::string> res;

	while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
		token = s.substr(pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		res.push_back(token);
	}

	res.push_back(s.substr(pos_start));
	return res;
}

int main(int argc, char** argv)
{
	Visual vis;
	vis.init();
	std::string dir = argv[1];
	int stepi = 0;
	auto up_o=vis.addMesh(load(dir + "/" + "up_" + std::to_string(stepi) + ".obj"), "up");
	auto down_o=vis.addMesh(load(dir + "/" + "down_" + std::to_string(stepi) + ".obj"), "down");
	std::vector < std::map<int, glm::mat4> > transformation;
	//for (const auto& entry : std::filesystem::directory_iterator(dir))
	std::string name = "step_" + std::to_string(stepi)+".txt";
	std::string spath = dir + "/" + name;
	std::vector<Eigen::MatrixXd> upvec,downvec;
	while(exists_test3(spath))
	{
		std::ifstream f(spath);
		std::string line;
		std::map<int, glm::mat4> tt;
		while (std::getline(f, line))
		{

			auto slist =split(line, ",");
			glm::mat4 m;
			int pos = 1;
			int tid = std::stoi(slist[0]);
			for(int i=0;i<4;i++)
				for (int j = 0; j < 4; j++)
				{
					m[i][j] = std::stof(slist[pos++]);
				}
			
			tt[tid] = m;
		}
		transformation.push_back(tt);
		auto upm=load(dir + "/" + "up_" + std::to_string(stepi) + ".obj");
		upvec.push_back(upm.V);
		auto downm = load(dir + "/" + "down_" + std::to_string(stepi) + ".obj");
		downvec.push_back(downm.V);
		stepi++;
		name = "step_" + std::to_string(stepi) + ".txt";
		spath = dir + "/" + name;
	}
	stepi = 0;
	std::vector<polyscope::SurfaceMesh*> otlist;
	for (int i = 0; i < 50; i++)
	{
		auto fpath = dir+"/" + std::to_string(i) + ".obj";
		if (exists_test3(fpath))
		{
			auto m = load(fpath);
			auto o = vis.addMesh(m, std::to_string(i));
			otlist.push_back(o);
			auto& mat = transformation[stepi][i];
			o->setTransform(mat);
		}
	}

	
	polyscope::state::userCallback = [&]()
	{
		ImGui::Text(dir.c_str());
		//impl.vis.misc->removeAllQuantities();
		if (ImGui::SliderInt("step", &stepi, 0, transformation.size()-1, NULL))
		{
			for (auto o : otlist)
			{
				int tid = std::stoi(o->name);
				auto& m = transformation[stepi][tid];
				o->setTransform(m);
			}
			up_o->updateVertexPositions(upvec[stepi]);
			down_o->updateVertexPositions(downvec[stepi]);
		}
	};
	vis.show();
	return 0;
}