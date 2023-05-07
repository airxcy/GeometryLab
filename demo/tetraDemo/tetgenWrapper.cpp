#include "tetgenWrapper.h"
#include <iostream>


void TetgenWrapper::fromEigen(VolumeMesh& egm)
{
	std::vector<std::vector<REAL> >& V=egm.V;
	std::vector<std::vector<size_t> >& F=egm.F;
    using namespace std;
    // all indices start from 0
    in.firstnumber = 0;

    in.numberofpoints = V.size();
    in.pointlist = new REAL[in.numberofpoints * 3];
    // loop over points
    for (int i = 0; i < (int)V.size(); i++)
    {
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
            p->vertexlist[j] = (int)F[i][j];
            
        }
    }
    
}

bool TetgenWrapper::toEigen(VolumeMesh& egm)
{
    using namespace std;
    egm.nV = out.numberofpoints;
    egm.nT = out.numberoftetrahedra;
    egm.nF = egm.nT * 4;
    egm.V.resize(out.numberofpoints, vector<REAL>(3));
    for (int i = 0; i < out.numberofpoints; i++)
    {
        egm.V[i][0] = out.pointlist[i * 3 + 0];
        egm.V[i][1] = out.pointlist[i * 3 + 1];
        egm.V[i][2] = out.pointlist[i * 3 + 2];
    }
    egm.T.resize(out.numberoftetrahedra, vector<size_t>(4));
    egm.F.resize(egm.nF, vector<size_t>(4));
    egm.TF.resize(out.numberoftetrahedra, vector<int>(4));
    int fi = 0;
    for (int i = 0; i < out.numberoftetrahedra; i++)
    {
        int* tmpv = out.tetrahedronlist + i * 4;
        for (int j = 0; j < 4; j++)
            egm.T[i][j] = out.tetrahedronlist[i * 4 + j];
        for (int j = 0; j < 4; j++)
        {
            egm.F[fi] = { (size_t)tmpv[tetvcoding[j][0]], (size_t)tmpv[tetvcoding[j][1]], (size_t)tmpv[tetvcoding[j][2]] };
            egm.TF[i][j]=fi;
            fi++;
        }
    }
    std::cout << "tetgen V:" << egm.nV << ",T:" << egm.nT << std::endl;
    //egm.B = egm.B.rowwise().homogeneous();
    //std::cout << egm.B.rows()<<","<<egm.B.cols() << std::endl;
    return true;
}

void TetgenWrapper::run()
{
    tetrahedralize(&tetparam, &in, &out);
    //toSTD();
    
}