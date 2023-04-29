#include "GenusN.h"
#include <eigen/Geometry>
#include <igl/canonical_quaternions.h>
#include <igl/list_to_matrix.h>
#include <igl/face_areas.h>
#include <igl/doublearea.h>
#define REAL double
#define VOID int
//#include <igl/triangle/triangulate.h>
extern "C"
{
    #include <triangle/triangle.h>
}
void GenusN::clear()
{
    XMesh::clear();
    clrs.resize(0, 0);
}

void GenusN::paramFromG2(XMesh& dbTorus)
{
    genus = 2;
    centers.resize(genus, 3);
    Eigen::RowVector3d maxv = dbTorus.V.colwise().maxCoeff();
    Eigen::RowVector3d minv = dbTorus.V.colwise().minCoeff();
    R = (maxv(0) - minv(0)) / 2;
    r = R / 4;
    centers.row(0) << 0, 0, maxv(2) - R;
    centers.row(1) << 0, 0, minv(2) + R;
    R = R - r;
    cDist = (centers(0, 2) - centers(1, 2))/2;
    
}

void GenusN::buildMesh(int G)
{
    genus = G;
    nANG =  (64/genus)*genus;
    nang = 32;
    /** outer circle part **/
    double DistDAng = 2.0 * M_PI / genus;
    double DistAng = 0;
    cDist = (R + r) * sqrt(2 / (1 - cos(DistDAng)));
    double dANG = 2.0 * M_PI / nANG;
    double dang = 2.0 * M_PI / nang;
    double startANG = 0;
    int halfrange = nang / 2;
    int range1= nANG / genus;
    double angBorder1 = M_PI*2.0/genus;
    int range2 = nANG/genus/2 + nANG/2;
    double angBorder2 = 0;
    double dMid = cDist*sqrt((1-cos(DistDAng))/2)/(nANG / 2 -  nANG / genus / 2);
    V.resize(genus * nANG *nang,3);
    clrs.resize(V.rows(), 3);
    int counter = 0;

    innerBnd.resize(1);
    innerBnd[0].resize((nANG* genus - nANG)*2 );
    for (int i = 0; i < genus; i++)
    {
        double initANG = DistAng - DistDAng / 2;
        double ANG = initANG;
        Eigen::RowVector3d C0(cos(DistAng) * cDist, 0, sin(DistAng) * cDist);
        
        for (int j = 0; j < nANG; j++)
        {
            
            Eigen::RowVector3d C1 = C0+Eigen::RowVector3d(cos(ANG) * R, 0, sin(ANG) * R);
            Eigen::Quaterniond v0(0, cos(ANG)*r, 0, sin(ANG)*r);
            Eigen::RowVector3d ax= Eigen::RowVector3d(-sin(ANG) , 0, cos(ANG) );
            Eigen::Quaterniond v00 = v0;
            Eigen::RowVector3d C11 = C1;
            Eigen::RowVector3d axx = ax;
            if (j > range2)
            {
                v00 = Eigen::Quaterniond(0, cos(initANG + angBorder2) * r, 0, sin(initANG + angBorder2) * r);
                axx = Eigen::RowVector3d(-sin(initANG + angBorder2), 0, cos(initANG + angBorder2));
                C11 = C0 + Eigen::RowVector3d(cos(initANG + angBorder2) * R, 0, sin(initANG + angBorder2) * R) - axx * dMid * (nANG-j);
            }
            else if (j > range1)
            {
                v00 = Eigen::Quaterniond(0, cos(initANG + angBorder1) * r, 0, sin(initANG + angBorder1) * r);
                axx = Eigen::RowVector3d(-sin(initANG + angBorder1), 0, cos(initANG + angBorder1));
                C11 = C0 + Eigen::RowVector3d(cos(initANG + angBorder1) * R, 0, sin(initANG + angBorder1) * R) + axx * dMid * (j - range1);
            }
            double ang = -M_PI / 2.0;
            double clrval = double(j) / nANG;
            glm::vec3 v1, v2, v3, v4;
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

                if (j > range1 && k <=  halfrange)
                {
                    
                    qw = qcos, qx = qsin * axx(0), qy = 0, qz = qsin * axx(2);
                    q1 = Eigen::Quaterniond(qw, qx, qy, qz);
                    q2 = Eigen::Quaterniond(qw, -qx, -qy, -qz);
                    v = q1 * v00 * q2;
                    C2 = C11 + Eigen::RowVector3d(v.x(), v.y(), v.z());
                    //clrs.row(counter) << clrval, 1 - clrval, 0;
                    //V.row(counter++) = C2;
                    

                }

                clrs.row(counter) << clrval, 1 - clrval, 0;
                V.row(counter++) = C2;
                if (k == 0) v1 = { C2(0), C2(1), C2(2) };
                if (k == nang-1) v2 = { C2(0), C2(1), C2(2) };
                ang += dang;
            }
            //int i1 = range1 + 1;
            //int i2 = range2 + 1;
            //if (j > range2)
            //{
            //    innerBnd[0][(nANG - range1)*2 * i + nANG-j-1 + nANG+i2-2*i1] = v1;
            //    innerBnd[0][(nANG - range1) * 2 * i + j - i1 + (i2-i1)] = v2;
            //}
            //else if (j > range1)
            //{
            //    innerBnd[0][(nANG - range1) * 2 * i + range2-j ] = v1;
            //    innerBnd[0][(nANG - range1) * 2 * i + j-i1 + (i2 - i1)] = v2;
            //}
            
            ANG += dANG;
        }
        DistAng += DistDAng;
    }
    counter = 0;
    stdF.clear();
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
                stdF.push_back({ v1,v2,v3 });
                stdF.push_back({ v3,v4,v1 });
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
                stdF.push_back({ v1,v2,v3 });
                stdF.push_back({ v3,v4,v1 });
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
                stdF.push_back({ v1,v2,v3 });
                stdF.push_back({ v3,v4,v1 });
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
            stdF.push_back({ v1,v2,v3 });
            stdF.push_back({ v3,v4,v1 });
        }

    }
    igl::list_to_matrix(stdF, F);

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
    int r0 = V.rows();
    int f0 = F.rows();
    std::vector<std::vector<int> > addF;
    double area = (R * dANG) * (r * dang) / 2 * 1.1;
    triBnd(innerBndIdx[0], area, addF);

    int nAddV = V.rows() - r0;
    int nAddF = F.rows() - f0;
    std::vector<int>& bnd = innerBndIdx[1];
    int r1 = V.rows();
    V.conservativeResize(V.rows() + nAddV, 3);
    clrs.conservativeResize(V.rows(), 3);
    for (int i = 0; i < nAddV; i++)
    {
        V.row(r1 + i) << V(r0 + i,0) , r, V(r0 + i, 2);
        clrs.row(r1 + i ) << 0, 1, 1;
    }
    std::cout << r0 <<","<< r1 << std::endl;
    int f1 = F.rows();
    F.conservativeResize(F.rows() + addF.size(), 3);
    for (int i = 0; i < addF.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {

            int vi = addF[i][j];
            if (vi < bnd.size())
                vi = bnd[vi];
            else
                vi = vi - bnd.size() + r1;
            F(f1 + i, 2-j) = vi;
        }
    }

}

void GenusN::triBnd(std::vector<int>& bnd,double area, std::vector< std::vector< int > >& addF)
{
    
    using namespace std;
    using namespace Eigen;
    string flags = "q30a"+std::to_string(area)+"Y";
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
        in.pointlist[i*2] = V(bnd[i], 0);
        in.pointlist[i * 2+1] =  V(bnd[i],2);
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
    
    int r0 = V.rows();
    V.conservativeResize(V.rows()+out.numberofpoints- bnd.size(),3);
    clrs.conservativeResize(V.rows(), 3);
    for (int i = bnd.size(); i < out.numberofpoints; i++)
    {
        V.row(r0 + i - bnd.size()) << out.pointlist[i * 2], -r, out.pointlist[i * 2 + 1];
        clrs.row(r0 + i - bnd.size()) << 0, 0, 1;
    }
    int f0 = F.rows();
    F.conservativeResize(F.rows() + out.numberoftriangles, 3);

    for (int i = 0; i < out.numberoftriangles; i++)
    {
        std::vector<int> f;
        for (int j = 0; j < 3; j++)
        {
            
            int vi = out.trianglelist[i * 3 + j];
            f.push_back(vi);
            if (vi < bnd.size())
                vi = bnd[vi];
            else
                vi = vi - bnd.size() + r0;
            F(f0+i, j) = vi;
        }
        addF.push_back(f);
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
