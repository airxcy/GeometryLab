#include "GenusN.h"
#include <eigen/Geometry>
#include <igl/canonical_quaternions.h>

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

void GenusN::buildMesh()
{
    genus = 2;
    //circle part
    double DistDAng = 2.0 * M_PI / genus;
    double DistAng = M_PI/2;
    

    double dANG = 2.0 * M_PI / nANG;
    double dang = 2.0 * M_PI / nang;
    double startANG = 0;
    int halfrange = nang / 2;
    int range1= nANG / genus;
    double angBorder1 = M_PI*2.0/genus;
    int range2 = nANG/genus/2 + range1;
    double angBorder2 = 0;
    double dMid = cDist / (nANG / 4);
    V.resize(genus * nANG *nang,3);
    clrs.resize(V.rows(), 3);
    int counter = 0;

    innerBnd.resize(2);
    for (int i = 0; i < genus; i++)
    {
        
        double ANG = 0;
        Eigen::RowVector3d C0(cos(DistAng) * cDist, 0, sin(DistAng) * cDist);
        for (int j = 0; j < nANG; j++)
        {
            
            Eigen::RowVector3d C1 = C0+Eigen::RowVector3d(cos(ANG) * R, 0, sin(ANG) * R);
            Eigen::Quaterniond v0(0, cos(ANG)*r, 0, sin(ANG)*r);
            Eigen::RowVector3d ax= Eigen::RowVector3d(-sin(ANG) , 0, cos(ANG) );
            Eigen::Quaterniond v00 = v0;
            Eigen::RowVector3d C11 = C1;
            Eigen::RowVector3d axx = ax;
            if (j >= range2)
            {
                v00 = Eigen::Quaterniond(0, cos(angBorder2) * r, 0, sin(angBorder2) * r);
                axx = Eigen::RowVector3d(-sin(angBorder2), 0, cos(angBorder2));
                C11 = C0 + Eigen::RowVector3d(cos(angBorder2) * R, 0, sin(angBorder2) * R)- axx* dMid* (j- range2);
                
            }
            else if (j >= range1)
            {
                
                v00 = Eigen::Quaterniond(0, cos(angBorder1) * r, 0, sin(angBorder1) * r);
                axx = Eigen::RowVector3d(-sin(angBorder1), 0, cos(angBorder1));
                C11 = C0 + Eigen::RowVector3d(cos(angBorder1) * R, 0, sin(angBorder1) * R)+ axx * dMid * (j - range1);
                
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

                if (j >= range1 && k <=  halfrange)
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
                ang += dang;
            }
            ANG += dANG;
        }
        DistAng += DistDAng;
        dANG = -dANG;
        dMid = -dMid;
    }
    //middle part
    //double dSquare = dANG * R;


}
