#ifndef MESHLAPLACIANSOLVER_H
#define MESHLAPLACIANSOLVER_H
#include "../mesh.h"
#include <Eigen/SparseLU>
#include <../utility/basedefine.h>

class MeshLaplacianSolver
{
public:
    void set(MyMesh& mesh,vector<int>& selectVertexId){this->mesh=mesh;this->selectVertexId=selectVertexId;}
    ~MeshLaplacianSolver(){}
    Eigen::VectorXd& LaplacainSolve();
    void ComputeVertexLaplacianWeight();
    void ComputeLaplacianMatrix();
    OpenMesh::VPropHandleT<vector<double>> vertexLPLWeight;
    void setRightHand(Eigen::VectorXd& rightHand){B=rightHand;}
private:

    MyMesh mesh;
    vector<int> selectVertexId;
    Eigen::SparseLU<Eigen::SparseMatrix<double>,
        Eigen::COLAMDOrdering<int> > solver;
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd B,X;
};

#endif // MESHLAPLACIANSOLVER_H
