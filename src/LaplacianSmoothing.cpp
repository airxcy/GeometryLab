#include "LaplacianSmoothing.h"
#include "igl/cotmatrix.h"
#include "igl/massmatrix.h"
#include "igl/doublearea.h"
#include <igl/per_vertex_normals.h>
#include "igl/grad.h"
#include "igl/barycenter.h"
void LaplacianSmoothing::init(Eigen::MatrixXd& aV, Eigen::MatrixXi& aF)
{
    V = aV, F = aF;
    igl::cotmatrix(V, F, L);
    // Alternative construction of same Laplacian
    Eigen::SparseMatrix<double> G, K;
    // Gradient/Divergence
    igl::grad(V, F, G);
    // Diagonal per-triangle "mass matrix"
    Eigen::VectorXd dblA;
    igl::doublearea(V, F, dblA);
    // Place areas along diagonal #dim times
    const auto& T = 1. * (dblA.replicate(3, 1) * 0.5).asDiagonal();
    // Laplacian K built as discrete divergence of gradient or equivalently
    // discrete Dirichelet energy Hessian
    K = -G.transpose() * T * G;
    std::cout << "|K-L|: " << (K - L).norm() << std::endl;

    // Use original normals as pseudo-colors
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);
    Eigen::MatrixXd C = N.rowwise().normalized().array() * 0.5 + 0.5;

    // Initialize smoothing with base mesh
    //U = V;
}


void LaplacianSmoothing::init(std::vector<std::vector<double> >& sV, std::vector< std::vector<int> >& sF)
{
    V.resize(sV.size(),3);
    for (int i = 0; i < sV.size(); i++)
        V.row(i) << sV[i][0], sV[i][1], sV[i][2];
    F.resize(sF.size(), 3);
    for (int i = 0; i < sF.size(); i++)
        F.row(i) << sF[i][0], sF[i][1], sF[i][2];

    igl::cotmatrix(V, F, L);
    // Alternative construction of same Laplacian
    Eigen::SparseMatrix<double> G, K;
    // Gradient/Divergence
    igl::grad(V, F, G);
    // Diagonal per-triangle "mass matrix"
    Eigen::VectorXd dblA;
    igl::doublearea(V, F, dblA);
    // Place areas along diagonal #dim times
    const auto& T = 1. * (dblA.replicate(3, 1) * 0.5).asDiagonal();
    // Laplacian K built as discrete divergence of gradient or equivalently
    // discrete Dirichelet energy Hessian
    K = -G.transpose() * T * G;
    std::cout << "|K-L|: " << (K - L).norm() << std::endl;

    // Use original normals as pseudo-colors
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);
    Eigen::MatrixXd C = N.rowwise().normalized().array() * 0.5 + 0.5;

    // Initialize smoothing with base mesh
    //U = V;
}



void LaplacianSmoothing::deltaLReg(double delta)
{
    using namespace Eigen;
    SparseMatrix<double> M;
    igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);
    // Solve (M-delta*L) U = M*U
    const auto& S = (M - delta * L);
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);
    assert(solver.info() == Eigen::Success);
    V = solver.solve(M * V).eval();
    // Compute centroid and subtract (also important for numerics)
    VectorXd dblA;
    igl::doublearea(V, F, dblA);
    double area = 0.5 * dblA.sum();
    MatrixXd BC;
    igl::barycenter(V, F, BC);
    RowVector3d centroid(0, 0, 0);
    for (int i = 0; i < BC.rows(); i++)
    {
        centroid += 0.5 * dblA(i) / area * BC.row(i);
    }
    V.rowwise() -= centroid;

    //// Normalize to unit surface area (important for numerics)
    V.array() /= sqrt(area);
}

void LaplacianSmoothing::deltaL(double delta)
{
    using namespace Eigen;
    SparseMatrix<double> M;
    igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);
    // Solve (M-delta*L) U = M*U
    const auto& S = (M - delta * L);
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);
    assert(solver.info() == Eigen::Success);
    V = solver.solve(M * V).eval();
    // Compute centroid and subtract (also important for numerics)
    VectorXd dblA;
    igl::doublearea(V, F, dblA);
    double area = 0.5 * dblA.sum();
    MatrixXd BC;
    igl::barycenter(V, F, BC);
    RowVector3d centroid(0, 0, 0);
    for (int i = 0; i < BC.rows(); i++)
    {
        centroid += 0.5 * dblA(i) / area * BC.row(i);
    }
    //V.rowwise() -= centroid;
    
    //// Normalize to unit surface area (important for numerics)
    //deformedV.array() /= sqrt(area);
    //for (int i = 0; i < boundryIdx.size(); i++)
    //    if (!boundryIdx(i))
    //    {
    //        deformedV.row(i) -= centroid;
    //        deformedV.row(i) /= sqrt(area);
    //    }
    //V = deformedV;
}