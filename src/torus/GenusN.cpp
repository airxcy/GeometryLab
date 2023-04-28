#include "GenusN.h"
#include <eigen/Geometry>
#include <igl/canonical_quaternions.h>
#include <igl/list_to_matrix.h>

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
    //circle part
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
        
        for (int j = 0; j < nANG; j++)
        {
            for (int k = 0; k < nang; k++)
            {
                
                int j1 = (j + nANG- 1) % nANG;
                int k1 = (k + nANG - 1) % nang;
                int v1 = i * nANG * nang + j * nang + k;
                int v2 = i * nANG * nang + j * nang + k1;
                int v3 = i * nANG * nang + j1 * nang + k1;
                int v4 = i * nANG * nang + j1 * nang + k;
                if (j <= range1+1)
                {
                    stdF.push_back({ v1,v2,v3 });
                    stdF.push_back({ v3,v4,v1 });
                }
            }
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

        innerBndIdx[sidei].push_back(innerBndIdx[sidei][0]);
    }
    
    //middle part
    //double dSquare = dANG * R;
    

}


