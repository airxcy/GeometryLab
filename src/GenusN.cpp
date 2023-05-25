#include "GenusN.h"

#include <Eigen/Core>
#include <eigen/Geometry>

#define _USE_MATH_DEFINES
#include <cmath>

#include <igl/canonical_quaternions.h>
#include <igl/list_to_matrix.h>
#include <igl/per_vertex_normals.h>

#include "igl/cotmatrix.h"
#include "igl/massmatrix.h"
#include "igl/invert_diag.h"
#include "igl/min_quad_with_fixed.h"
#include "igl/doublearea.h"
#include "igl/grad.h"
#include "igl/barycenter.h"

#define REAL double
#define VOID int
extern "C"
{
    #include <triangle/triangle.h>
}
void TorusN::clear()
{
    V.clear();
    F.clear();
    clrs.resize(0, 0);
}



void TorusN::buildV()
{
    nANG = (64 / genus) * genus;
    nang = 32;
    /** outer circle part **/
    dANG = 2.0 * M_PI / nANG;
    dang = 2.0 * M_PI / nang;
    startANG = 0;
    halfrange = nang / 2;
    range1 = nANG / genus;
    angBorder1 = M_PI * 2.0 / genus;
    range2 = nANG / genus / 2 + nANG / 2;
    angBorder2 = 0;
    dMid = cDist * sqrt((1 - cos(DistDAng)) / 2) / (nANG / 2 - nANG / genus / 2);
    if (!V.size())
    {
        V.resize(genus * nANG * nang, std::vector<double>(3));
        clrs.resize(V.size(), 3);
    }
    int counter = 0;


    for (int i = 0; i < genus; i++)
    {
        double initANG = DistAng - DistDAng / 2;
        double ANG = initANG;
        Eigen::RowVector3d C0(cos(DistAng) * cDist, 0, sin(DistAng) * cDist);

        for (int j = 0; j < nANG; j++)
        {
            Eigen::RowVector3d C1 = C0 + Eigen::RowVector3d(cos(ANG) * R, 0, sin(ANG) * R);
            Eigen::Quaterniond v0(0, cos(ANG) * r, 0, sin(ANG) * r);
            Eigen::RowVector3d ax = Eigen::RowVector3d(-sin(ANG), 0, cos(ANG));
            Eigen::Quaterniond v00 = v0;
            Eigen::RowVector3d C11 = C1;
            Eigen::RowVector3d axx = ax;
            if (j > range2)
            {
                v00 = Eigen::Quaterniond(0, cos(initANG + angBorder2) * r, 0, sin(initANG + angBorder2) * r);
                axx = Eigen::RowVector3d(-sin(initANG + angBorder2), 0, cos(initANG + angBorder2));
                C11 = C0 + Eigen::RowVector3d(cos(initANG + angBorder2) * R, 0, sin(initANG + angBorder2) * R) - axx * dMid * (nANG - j);
            }
            else if (j > range1)
            {
                v00 = Eigen::Quaterniond(0, cos(initANG + angBorder1) * r, 0, sin(initANG + angBorder1) * r);
                axx = Eigen::RowVector3d(-sin(initANG + angBorder1), 0, cos(initANG + angBorder1));
                C11 = C0 + Eigen::RowVector3d(cos(initANG + angBorder1) * R, 0, sin(initANG + angBorder1) * R) + axx * dMid * (j - range1);
            }
            double ang = -M_PI / 2.0;
            double clrval = double(j) / nANG;
            for (int k = 0; k < nang; k++)
            {
                double halfang = ang / 2;
                double qcos = cos(halfang);
                double qsin = sin(halfang);
                double qw = qcos, qx = qsin * ax(0), qy = 0, qz = qsin * ax(2);
                Eigen::Quaterniond q1(qw, qx, qy, qz);
                Eigen::Quaterniond q2(qw, -qx, -qy, -qz);
                Eigen::Quaterniond v = q1 * v0 * q2;
                Eigen::RowVector3d C2 = C1 + Eigen::RowVector3d(v.x(), v.y(), v.z());

                if (j > range1 && k <= halfrange)
                {

                    qw = qcos, qx = qsin * axx(0), qy = 0, qz = qsin * axx(2);
                    q1 = Eigen::Quaterniond(qw, qx, qy, qz);
                    q2 = Eigen::Quaterniond(qw, -qx, -qy, -qz);
                    v = q1 * v00 * q2;
                    C2 = C11 + Eigen::RowVector3d(v.x(), v.y(), v.z());
                }
                clrs.row(counter) << clrval, 1 - clrval, 0;
                V[counter++] = { C2(0),C2(1) ,C2(2) };
                ang += dang;
            }
            ANG += dANG;
        }
        DistAng += DistDAng;
    }
}
void TorusN::buildMesh(int G)
{
    genus = G;
    DistDAng = 2.0 * M_PI / genus;
    DistAng = 0;
    cDist = (R) * sqrt(2 / (1 - cos(DistDAng)))+r;
    buildV();
    int counter = 0;
    F.clear();
    std::vector<int> stdb;
    for (int i = 0; i < genus; i++)
    {
        
        for (int j = -1; j <= range1; j++)
        {
            for (int k = 0; k < nang; k++)
            {
                int j0 = (j + nANG) % nANG;
                int j1 = (j + 1) % nANG;
                int k1 = (k + 1) % nang;
                int v1 = i * nANG * nang + j0 * nang + k;
                int v2 = i * nANG * nang + j0 * nang + k1;
                int v3 = i * nANG * nang + j1 * nang + k1;
                int v4 = i * nANG * nang + j1 * nang + k;
                //if (!(j > range1&&(k1==0||k==halfrange)) && !(k < halfrange && j==range2))
                F.push_back({ v1,v2,v3 });
                F.push_back({ v3,v4,v1 });

                stdb.push_back(v1);
                stdb.push_back(v2);
                stdb.push_back(v3);
                stdb.push_back(v4);

            }
        }
        for (int j = range1+1; j <nANG ; j++)
        {
            if(j!=range2)
            for (int k = 0; k < halfrange; k++)
            {
                int j0 = (j + nANG) % nANG;
                int j1 = (j + 1) % nANG;
                int k1 = (k + 1) % nang;
                int v1 = i * nANG * nang + j0 * nang + k;
                int v2 = i * nANG * nang + j0 * nang + k1;
                int v3 = i * nANG * nang + j1 * nang + k1;
                int v4 = i * nANG * nang + j1 * nang + k;
                F.push_back({ v1,v2,v3 });
                F.push_back({ v3,v4,v1 });

            }

            for (int k = halfrange+1; k < nang-1; k++)
            {
                int j0 = (j + nANG) % nANG;
                int j1 = (j + 1) % nANG;
                int k1 = (k + 1) % nang;
                int v1 = i * nANG * nang + j0 * nang + k;
                int v2 = i * nANG * nang + j0 * nang + k1;
                int v3 = i * nANG * nang + j1 * nang + k1;
                int v4 = i * nANG * nang + j1 * nang + k;
                F.push_back({ v1,v2,v3 });
                F.push_back({ v3,v4,v1 });
                if (k>=halfrange+3&&k<=nang-4)
                {
                    stdb.push_back(v1);
                    stdb.push_back(v2);
                    stdb.push_back(v3);
                    stdb.push_back(v4);
                }
            }
        }
        for (int k = 0; k < halfrange; k++)
        {
            int i1 = (i + 1) % genus;
            int j0 = range2;
            int j1 = range2+1;
            int k1 = (k + 1) % nang;
            int v1 = i * nANG * nang + j0 * nang + k;
            int v2 = i * nANG * nang + j0 * nang + k1;
            int v3 = i1 * nANG * nang + j1 * nang + k1;
            int v4 = i1 * nANG * nang + j1 * nang + k;
            //if (!(j > range1&&(k1==0||k==halfrange)) && !(k < halfrange && j==range2))
            F.push_back({ v1,v2,v3 });
            F.push_back({ v3,v4,v1 });
        }

    }

    innerBndIdx.clear();
    for (int sidei = 0; sidei < 2; sidei++)
    {
        innerBndIdx.push_back(std::vector<int>());
        int i1 = 0;
        int i2 = nang-1;
        if (sidei == 1)
        {
            i1 = halfrange;
            i2 = halfrange + 1;
        }
        for (int i = 0; i < genus; i++)
        {
            int starti = i * nANG * nang;
            for (int j = range2+1; j <nANG; j++)
            {
                innerBndIdx[sidei].push_back(starti + j * nang + i1);
            }
            for (int j = nANG-1; j >range1; j--)
            {
                innerBndIdx[sidei].push_back(starti + j * nang + i2);
            }
            for (int j = range1+1; j <= range2; j++)
            {
                innerBndIdx[sidei].push_back(starti + j * nang + i1);
            }


        }

        //innerBndIdx[sidei].push_back(innerBndIdx[sidei][0]);
    }
    
    /** middle part **/
    int r0 = V.size();
    int f0 = F.size();
    std::vector<std::vector<int> > addF;
    double area = (R * dANG) * (r * dang) / 2 * 1.1;
    triBnd(innerBndIdx[0], area, addF);

    int nAddV = V.size() - r0;
    int nAddF = F.size() - f0;
    std::vector<int>& bnd = innerBndIdx[1];
    int r1 = V.size();
    clrs.conservativeResize(r1+nAddV, 3);
    for (int i = 0; i < nAddV; i++)
    {
        V.push_back({ V[r0 + i][0] , r, V[r0 + i][ 2] });
        clrs.row(r1 + i ) << 0, 1, 1;
    }
    int f1 = F.size();
    for (int i = 0; i < addF.size(); i++)
    {
        std::vector<int> ff;
        for (int j = 0; j < 3; j++)
        {
            int vi = addF[i][j];
            if (vi < bnd.size())
                vi = bnd[vi];
            else
                vi = vi - bnd.size() + r1;
            ff[2-j] = vi;
        }
        F.push_back(ff);
    }

    igl::list_to_matrix(V, deformer.deformedV);
    igl::list_to_matrix(V, deformer.boundryPos);
    deformer.boundryIdx.resize(V.size());
    deformer.boundryIdx.setConstant(0);
    for (int i : stdb)
        deformer.boundryIdx[i] = 1;
    setUpLaplacian();
}

void TorusN::triBnd(std::vector<int>& bnd,double area, std::vector< std::vector< int > >& addF)
{
    
    using namespace std;
    using namespace Eigen;
    string flags = "q30a"+std::to_string(area) + "Y";
    Eigen::VectorXi VM, EM, VM2, EM2;
    // Prepare the flags
    string full_flags = flags + "pz" + (EM.size() || VM.size() ? "" : "B");

    typedef Map< Matrix<double, Dynamic, Dynamic, RowMajor> > MapXdr;
    typedef Map< Matrix<int, Dynamic, Dynamic, RowMajor> > MapXir;

    // Prepare the input struct

    triangulateio in;
    in.numberofpoints = bnd.size();
    in.pointlist = (double*)calloc(bnd.size()*2, sizeof(double));
    double* ptr = in.pointlist;
    for(int i=0;i<bnd.size();i++)
    {
        in.pointlist[i*2] = V[bnd[i]][ 0];
        in.pointlist[i * 2+1] =  V[bnd[i]][2];
    }
    
    in.numberofpointattributes = 0;
    in.pointmarkerlist = (int*)calloc(bnd.size(), sizeof(int));
    for (unsigned i = 0; i < bnd.size(); ++i) 
        in.pointmarkerlist[i] =  1;

    in.trianglelist = NULL;
    in.numberoftriangles = 0;
    in.numberofcorners = 0;
    in.numberoftriangleattributes = 0;
    in.triangleattributelist = NULL;

    in.numberofsegments = bnd.size();
    in.segmentlist = (int*)calloc(bnd.size()*2, sizeof(int));
    int* iptr = in.segmentlist;
    for (int i = 0; i < bnd.size(); i++)
    {
        in.segmentlist[i*2] = i;
        in.segmentlist[i*2+1]= (i+1)%bnd.size();
    }
    
    in.segmentmarkerlist = (int*)calloc(bnd.size(), sizeof(int));
    for (unsigned i = 0; i < bnd.size(); ++i) in.segmentmarkerlist[i] =  1;

    in.numberofholes =  0;
    in.holelist = NULL;
    in.numberofregions = 0;

    // Prepare the output struct
    triangulateio out;
    out.pointlist = NULL;
    out.pointattributelist = nullptr;
    out.trianglelist = NULL;
    out.segmentlist = NULL;
    out.segmentmarkerlist = NULL;
    out.pointmarkerlist = NULL;
    
    // Call triangle
    triangulate(const_cast<char*>(full_flags.c_str()), &in, &out, 0);
    
    
    // Return the mesh
    
    int r0 = V.size();
    V.resize(V.size()+out.numberofpoints- bnd.size(),std::vector<double>(3) );
    clrs.conservativeResize(V.size(), 3);
    for (int i = bnd.size(); i < out.numberofpoints; i++)
    {
        V[r0 + i - bnd.size()] = { out.pointlist[i * 2], -r, out.pointlist[i * 2 + 1] };
        clrs.row(r0 + i - bnd.size()) << 0, 0, 1;
    }

    for (int i = 0; i < out.numberoftriangles; i++)
    {
        std::vector<int> f,ff;
        for (int j = 0; j < 3; j++)
        {
            
            int vi = out.trianglelist[i * 3 + j];
            f.push_back(vi);
            if (vi < bnd.size())
                vi = bnd[vi];
            else
                vi = vi - bnd.size() + r0;
            ff.push_back(vi);
        }
        addF.push_back(f);
        F.push_back(ff);
    }
    

    // Cleanup in
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.segmentlist);
    free(in.segmentmarkerlist);
    free(in.holelist);
    // Cleanup out
    free(out.pointlist);
    free(out.trianglelist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);
    free(out.pointmarkerlist);

}

void TorusN::HarmonicShape()
{
    deformer.calLaplacion(2);
    deformer.setUpB(2);
    deformer.setUpBc();
    deformer.solveHarmonic(2);
    deformer.deformedV;
    setUpLaplacian();
}


