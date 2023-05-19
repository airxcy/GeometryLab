#include "tetgenWrapper.h"
#include <iostream>


void TetgenWrapper::initSurfaceMesh(TriMesh& egm)
{
	std::vector<std::vector<REAL> >& V=egm.V;
	std::vector<std::vector<int> >& F=egm.F;
    using namespace std;
    // all indices start from 0
    in.firstnumber = 0;

    in.numberofpoints = V.size();
    in.pointlist = new REAL[in.numberofpoints * 3];
    in.pointmarkerlist = new int[in.numberofpoints];
    // loop over points
    int VmarkerID.size()+1;
    for (int i = 0; i < (int)V.size(); i++)
    {
        in.pointmarkerlist[i] = 1;
        in.pointlist[i * 3 + 0] = V[i][0];
        in.pointlist[i * 3 + 1] = V[i][1];
        in.pointlist[i * 3 + 2] = V[i][2];
    }
    
    in.numberoffacets = F.size();
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];
    tetparam.plc = 1;

    // loop over face
    for (int i = 0; i < (int)F.size(); i++)
    {
        //in.facetmarkerlist[i] = 1;
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

bool TetgenWrapper::translateOutput()
{
    using namespace std;
    size_t oldVN = in.numberofpoints;
    mesh.nV = out.numberofpoints;
    mesh.nT = out.numberoftetrahedra;
    mesh.nF = mesh.nT * 4;
    mesh.V.resize(out.numberofpoints, vector<REAL>(3));
    for (int i = 0; i < out.numberofpoints; i++)
    {
        mesh.V[i][0] = out.pointlist[i * 3 + 0];
        mesh.V[i][1] = out.pointlist[i * 3 + 1];
        mesh.V[i][2] = out.pointlist[i * 3 + 2];
    }
    mesh.T.resize(out.numberoftetrahedra, vector<int>(4));
    mesh.F.resize(mesh.nF, vector<int>(4));
    mesh.TF.resize(out.numberoftetrahedra, vector<int>(4));
    int fi = 0;
    for (int i = 0; i < out.numberoftetrahedra; i++)
    {
        int* tmpv = out.tetrahedronlist + i * 4;
        for (int j = 0; j < 4; j++)
            mesh.T[i][j] = out.tetrahedronlist[i * 4 + j];
        for (int j = 0; j < 4; j++)
        {
            mesh.F[fi] = { tmpv[tetvcoding[j][0]], tmpv[tetvcoding[j][1]], tmpv[tetvcoding[j][2]] };
            mesh.TF[i][j]=fi;
            fi++;
        }
    }
    std::cout << "tetgen V:" << mesh.nV << ",T:" << mesh.nT << std::endl;
    //egm.B = egm.B.rowwise().homogeneous();
    //std::cout << egm.B.rows()<<","<<egm.B.cols() << std::endl;
    steinerVertex.clear();
    for (int i = oldVN; i < mesh.nV; i++)
        steinerVertex.push_back(mesh.V[i]);
    return true;
}

void TetgenWrapper::run()
{
    tetrahedralize(&tetparam, &in, &out);
    translateOutput();
}