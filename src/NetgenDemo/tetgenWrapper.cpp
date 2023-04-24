#include "tetgenWrapper.h"
#include "igl/matrix_to_list.h"

#include "polyscope/surface_mesh.h"



void TetgenWrapper::fromEigen(EigenMeshD& egm)
{
	Eigen::MatrixXd TV;
	Eigen::MatrixXi TT;
	Eigen::MatrixXi TF;

	std::vector<std::vector<REAL> > V;
	std::vector<std::vector<int> > F;
	igl::matrix_to_list(egm.V, V);
	igl::matrix_to_list(egm.F, F);
    using namespace std;
    // all indices start from 0
    in.firstnumber = 0;

    in.numberofpoints = V.size();
    in.pointlist = new REAL[in.numberofpoints * 3];
    // loop over points
    for (int i = 0; i < (int)V.size(); i++)
    {
        assert(V[i].size() == 3);
        in.pointlist[i * 3 + 0] = V[i][0];
        in.pointlist[i * 3 + 1] = V[i][1];
        in.pointlist[i * 3 + 2] = V[i][2];
    }
    
    in.numberoffacets = F.size();
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];
    tetparam.plc = 1;
    tetparam.nobisect++;
    tetparam.supsteiner_level = 0;
    tetparam.addsteiner_algo = 0;
    // loop over face
    for (int i = 0; i < (int)F.size(); i++)
    {
        in.facetmarkerlist[i] = i;
        tetgenio::facet* f = &in.facetlist[i];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = NULL;
        tetgenio::polygon* p = &f->polygonlist[0];
        p->numberofvertices = F[i].size();
        p->vertexlist = new int[p->numberofvertices];
        // loop around face
        for (int j = 0; j < (int)F[i].size(); j++)
        {
            p->vertexlist[j] = F[i][j];
        }
    }
    
}

bool TetgenWrapper::toSTD()
{

    using namespace std;
    // process points
    if (out.pointlist == NULL)
    {
        cerr << "^tetgenio_to_tetmesh Error: point list is NULL\n" << endl;
        return false;
    }
    V.resize(out.numberofpoints, vector<REAL>(3));
    // loop over points
    for (int i = 0; i < out.numberofpoints; i++)
    {
        V[i][0] = out.pointlist[i * 3 + 0];
        V[i][1] = out.pointlist[i * 3 + 1];
        V[i][2] = out.pointlist[i * 3 + 2];
    }


    // process tets
    if (out.tetrahedronlist == NULL)
    {
        cerr << "^tetgenio_to_tetmesh Error: tet list is NULL\n" << endl;
        return false;
    }

    // When would this not be 4?
    assert(out.numberofcorners == 4);
    T.resize(out.numberoftetrahedra, vector<int>(out.numberofcorners));
    int min_index = 1e7;
    int max_index = -1e7;
    // loop over tetrahedra
    F.clear();
    for (int i = 0; i < out.numberoftetrahedra; i++)
    {
        int tmpv[4];
        for (int j = 0; j < out.numberofcorners; j++)
        {
            int index = out.tetrahedronlist[i * out.numberofcorners + j];
            T[i][j] = index;
            min_index = (min_index > index ? index : min_index);
            max_index = (max_index < index ? index : max_index);
            tmpv[j] = index;
        }
        
        for (int j = 0; j < 4; j++)
        {
            std::vector<int> f = { tmpv[tetvcoding[j][0]], tmpv[tetvcoding[j][1]], tmpv[tetvcoding[j][2]]};
            F.push_back(f);
        }
    }
    assert(min_index >= 0);
    assert(max_index >= 0);
    assert(max_index < (int)V.size());

    //// When would this not be 4?
    //F.clear();
    //// loop over tetrahedra
    //for (int i = 0; i < out.numberoftrifaces; i++)
    //{
    //    if (out.trifacemarkerlist && out.trifacemarkerlist[i] >= 0)
    //    {
    //        vector<int> face(3);
    //        for (int j = 0; j < 3; j++)
    //        {
    //            face[j] = out.trifacelist[i * 3 + j];
    //        }
    //        F.push_back(face);
    //    }
    //}
    return true;
}

void TetgenWrapper::run()
{
    tetrahedralize(&tetparam, &in, &out);
    toSTD();
    std::cout<<"tetgen-----V:" << V.size()<<",T:"<<T.size() << ",F:" << F.size() << std::endl;
}

void  TetgenWrapper::Vis()
{
    
    polyscope::registerSurfaceMesh("tetgen", V, F);
}