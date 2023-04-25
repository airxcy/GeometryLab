#include "tetgenWrapper.h"
#include "igl/matrix_to_list.h"
#include "igl/list_to_matrix.h"
#include "igl/barycenter.h"
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

bool TetgenWrapper::toEigen(EigenMeshD& egm)
{
    using namespace std;
    egm.nV = out.numberofpoints;
    egm.nT = out.numberoftetrahedra;
    egm.nF = egm.nT * 4;
    egm.sV.resize(out.numberofpoints, vector<REAL>(3));
    for (int i = 0; i < out.numberofpoints; i++)
    {
        egm.sV[i][0] = out.pointlist[i * 3 + 0];
        egm.sV[i][1] = out.pointlist[i * 3 + 1];
        egm.sV[i][2] = out.pointlist[i * 3 + 2];
    }
    egm.sT.resize(out.numberoftetrahedra, vector<size_t>(4));
    egm.sF.resize(egm.nF, vector<size_t>(4));
    egm.sTF.resize(out.numberoftetrahedra, vector<int>(4));
    int fi = 0;
    for (int i = 0; i < out.numberoftetrahedra; i++)
    {
        int* tmpv = out.tetrahedronlist + i * 4;
        for (int j = 0; j < 4; j++)
            egm.sT[i][j] = out.tetrahedronlist[i * 4 + j];
        for (int j = 0; j < 4; j++)
        {
            egm.sF[fi] = { (size_t)tmpv[tetvcoding[j][0]], (size_t)tmpv[tetvcoding[j][1]], (size_t)tmpv[tetvcoding[j][2]] };
            egm.sTF[i][j]=fi;
            fi++;
        }
    }
    igl::list_to_matrix(egm.sV, egm.V);
    igl::list_to_matrix(egm.sF, egm.F);
    igl::list_to_matrix(egm.sT, egm.T);
    igl::barycenter(egm.V, egm.T, egm.B);
    //egm.B = egm.B.rowwise().homogeneous();
    //std::cout << egm.B.rows()<<","<<egm.B.cols() << std::endl;
    return true;
}

void TetgenWrapper::run()
{
    tetrahedralize(&tetparam, &in, &out);
    //toSTD();
    std::cout<<"tetgen V:" << out.numberofpoints <<",T:"<< out.numberoftetrahedra << std::endl;
}