#include "GenusN.h"
#include <eigen/Geometry>
#include <igl/canonical_quaternions.h>

void GenusN::paramFromG1(XMesh& dbTorus)
{
    genus = 1;
    nHole = genus + 1;
    centers.resize(nHole, 3);
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
    double dANG = 2.0 * M_PI / nANG;
    double dang = 2.0 * M_PI / nang;
    V.resize(nHole * nANG*nang,3);
    clrs.resize(V.rows(), 3);

    int counter = 0;
    double DistAng = 0;
    double DistDAng = 2.0 * M_PI / nHole;
    
    for (int i = 0; i < nHole; i++)
    {
        double ANG = 0;
        Eigen::RowVector3d C0(sin(DistAng) * cDist, 0, cos(DistAng) * cDist);
        for (int j = 0; j < nANG; j++)
        {
            double ang = 0;
            Eigen::RowVector3d C1 = C0+Eigen::RowVector3d(cos(ANG) * R, 0, sin(ANG) * R);
            Eigen::Quaterniond v0(0, cos(ANG)*r, 0, sin(ANG)*r);
            Eigen::RowVector3d ax= Eigen::RowVector3d(-sin(ANG) , 0, cos(ANG) );
            double clrval = double(j) / nANG;
            for (int k = 0; k < nang; k++)
            {
                double halfang = ang / 2;
                double qcos = cos(halfang);
                double qsin = sin(halfang);
                double qw = qcos,qx= qsin*ax(0),qy=0 ,qz= qsin * ax(2);
                Eigen::Quaterniond q1(qw,qx,qy,qz);
                Eigen::Quaterniond q2(qw, -qx, -qy, -qz);
                Eigen::Quaterniond v = q1 * v0 * q2;
                Eigen::RowVector3d C2 = C1 + Eigen::RowVector3d(v.x(), v.y(), v.z());
                clrs.row(counter) << clrval, 1 - clrval, 0;
                V.row(counter++) = C2;
                ang += dang;
            }
            ANG += dANG;
        }
        DistAng += DistDAng;
    }
}
