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
    int curVmarker = m_VmarkerID.size()+1;
    m_VmarkerID.push_back(curVmarker);
    for (int i = 0; i < (int)V.size(); i++)
    {
        in.pointmarkerlist[i] = curVmarker;
        in.pointlist[i * 3 + 0] = V[i][0];
        in.pointlist[i * 3 + 1] = V[i][1];
        in.pointlist[i * 3 + 2] = V[i][2];
    }
    
    in.numberoffacets = F.size();
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];
    tetparam.plc = 1;

    // loop over face
    int curFmarker = m_FmarkerID.size() + 1;
    m_FmarkerID.push_back(curFmarker);
    for (int i = 0; i < (int)F.size(); i++)
    {
        in.facetmarkerlist[i] = curFmarker;
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

void TetgenWrapper::addSurface(TriMesh& surface, int fmarker, int vmarker)
{
    auto& V = surface.V;
    auto& F = surface.F;

    int i0 = m_mesh.V.size();
    
    if (m_VmarkerID.size() == 0)
        m_VmarkerID.push_back(vmarker);
    else
    {
        auto it = m_VmarkerID.end();
        do {
            it--;
            if (*it < vmarker)
            {
                m_VmarkerID.insert(++it, vmarker);
                break;
            }

        } while (it!= m_VmarkerID.begin());
        if(it== m_VmarkerID.begin())
            m_VmarkerID.insert(it, vmarker);
    }

    for (int i = 0; i < V.size(); i++)
    {
        m_mesh.V.push_back(V[i]);
        m_VMarkers.push_back(vmarker);
    }

    for (int i = 0; i < F.size(); i++)
    {
        m_mesh.F.push_back(F[i]);
        m_FMarkers.push_back(fmarker);
    }

}


void TetgenWrapper::freeMemory()
{
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.trifacemarkerlist);
    free(in.facetlist);
    free(in.pointlist);
    free(in.pointmarkerlist);
    // out free
    free(out.pointlist);
    //free(out.pointattributelist);
    free(out.pointmarkerlist);
    //free(out.triangleattributelist);
    //free(out.trianglearealist);
    //free(out.neighborlist);
    //free(out.edgelist);
    //free(out.edgemarkerlist);
}

void TetgenWrapper::convertInput()
{
    auto& V = m_mesh.V;
    auto& F = m_mesh.F;
    in.numberofpoints = V.size();
    in.pointlist = new REAL[in.numberofpoints * 3];
    in.pointmarkerlist = new int[in.numberofpoints];
    for (int i = 0; i < (int)V.size(); i++)
    {
        in.pointmarkerlist[i] = m_VMarkers[i];
        in.pointlist[i * 3 + 0] = V[i][0];
        in.pointlist[i * 3 + 1] = V[i][1];
        in.pointlist[i * 3 + 2] = V[i][2];
    }


    in.numberoffacets = F.size();
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];
    tetparam.plc = 1;
    for (int i = 0; i < (int)F.size(); i++)
    {
        in.facetmarkerlist[i] = m_VMarkers[i];
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



void TetgenWrapper::translateOutput()
{
    using namespace std;
    size_t oldVN = in.numberofpoints;
    m_mesh.nV = out.numberofpoints;
    m_mesh.nT = out.numberoftetrahedra;
    m_mesh.nF = m_mesh.nT * 4;
    m_mesh.V.resize(out.numberofpoints, vector<REAL>(3));
    for (int i = 0; i < out.numberofpoints; i++)
    {
        m_mesh.V[i][0] = out.pointlist[i * 3 + 0];
        m_mesh.V[i][1] = out.pointlist[i * 3 + 1];
        m_mesh.V[i][2] = out.pointlist[i * 3 + 2];
    }
    m_mesh.T.resize(out.numberoftetrahedra, vector<int>(4));
    m_mesh.F.resize(m_mesh.nF, vector<int>(4));
    m_mesh.TF.resize(out.numberoftetrahedra, vector<int>(4));
    int fi = 0;
    for (int i = 0; i < out.numberoftetrahedra; i++)
    {
        int* tmpv = out.tetrahedronlist + i * 4;
        for (int j = 0; j < 4; j++)
            m_mesh.T[i][j] = out.tetrahedronlist[i * 4 + j];
        for (int j = 0; j < 4; j++)
        {
            m_mesh.F[fi] = { tmpv[tetvcoding[j][0]], tmpv[tetvcoding[j][1]], tmpv[tetvcoding[j][2]] };
            m_mesh.TF[i][j]=fi;
            fi++;
        }
    }
    std::cout << "tetgen V:" << m_mesh.nV << ",T:" << m_mesh.nT << std::endl;
    //egm.B = egm.B.rowwise().homogeneous();
    //std::cout << egm.B.rows()<<","<<egm.B.cols() << std::endl;
    steinerVertex.clear();
    for (int i = oldVN; i < m_mesh.nV; i++)
        steinerVertex.push_back(m_mesh.V[i]);
}

void TetgenWrapper::run()
{
    tetrahedralize(&tetparam, &in, &out);
    translateOutput();
}