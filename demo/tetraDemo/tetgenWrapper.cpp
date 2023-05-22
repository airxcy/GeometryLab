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

    int i0 = plc_mesh.V.size();
    std::cout << "add Surface " << fmarker << std::endl;
    
    auto insertOrder = [](std::vector<int>& vlist, int v)
    {
        if (vlist.size() == 0)
            vlist.push_back(v);
        else
        {
            auto it = vlist.end();
            do {
                it--;
                if (*it < v)
                {
                    vlist.insert(++it, v);
                    break;
                }

            } while (it != vlist.begin());
            if (it == vlist.begin())
                vlist.insert(it, v);
        }
    };
    insertOrder(m_VmarkerID, vmarker);
    insertOrder(m_FmarkerID, fmarker);

    for (auto& v:V)
    {
        plc_mesh.V.push_back(v);
        m_VMarkers.push_back(vmarker);
    }
    for (auto& f:F)
    {
        std::vector<int> ff;
        for (auto i : f)
        {
            ff.push_back(i+i0);
        }
        plc_mesh.F.push_back(ff);
        m_FMarkers.push_back(fmarker);
    }

}

void TetgenWrapper::addHole(double x,double y,double z)
{
    holepoints.push_back({x,y,z});
    in.numberofholes = holepoints.size();
    in.holelist = new REAL[in.numberofholes];
    for (int i = 0; i < holepoints.size(); i++)
    {
        in.holelist[i * 3] = holepoints[i][0];
        in.holelist[i * 3 + 1] = holepoints[i][1];
        in.holelist[i * 3 + 2] = holepoints[i][2];
    }
}

void TetgenWrapper::addRegion(double x, double y, double z)
{
    regionPoints.push_back({ x,y,z });
    in.numberofregions = regionPoints.size();
    in.regionlist = new REAL[in.numberofregions];
    for (int i = 0; i < regionPoints.size(); i++)
    {
        in.regionlist[i * 3] = regionPoints[i][0];
        in.regionlist[i * 3 + 1] = regionPoints[i][1];
        in.regionlist[i * 3 + 2] = regionPoints[i][2];
    }
    tetparam.regionattrib = 1;
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
    std::cout << "convert Input" << std::endl;
    auto& V = plc_mesh.V;
    auto& F = plc_mesh.F;
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
        in.facetmarkerlist[i] = m_FMarkers[i];
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
    regionlist.resize(out.numberoftetrahedra);
    Fregionlist.resize(m_mesh.nF);
    std::cout << "numberoftetrahedronattributes" << out.numberoftetrahedronattributes << std::endl;
    int fi = 0;
    for (int i = 0; i < out.numberoftetrahedra; i++)
    {
        int* tmpv = out.tetrahedronlist + i * 4;
        int r = (int)round(out.tetrahedronattributelist[i]);
        out.
        for (int j = 0; j < 4; j++)
            m_mesh.T[i][j] = out.tetrahedronlist[i * 4 + j];
        for (int j = 0; j < 4; j++)
        {
            m_mesh.F[fi] = { tmpv[tetvcoding[j][0]], tmpv[tetvcoding[j][1]], tmpv[tetvcoding[j][2]] };
            //Fregionlist[fi] = r;
            m_mesh.TF[i][j]=fi;
            fi++;
        }
        if(r)std::cout << r << std::endl;
        regionlist[i]=r;
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