#ifndef POISSONDEFORMATION_H
#define POISSONDEFORMATION_H
#include "../mesh.h"
#include "meshlaplaciansolver.h"
#include "../utility/quaternion.h"
#include "../utility/pointvector.h"
#include <vector>
class PoissonDeformation
{
private:
    MeshObj* pMeshObj;
    VectorXd divMatrixX;
    VectorXd divMatrixY;
    VectorXd divMatrixZ;
    MeshLaplacianSolver LPsolver;
    QQuaternion m_quater_fixed;
    QQuaternion m_quater_hand;
    Matrix4d m_handTransMat;

    MyMesh mesh;
    vector<int> selectVertexId;
    vector<int> fixVertexId;
    MyMesh m_static_mesh;
    std::vector<double> freeVertexWeight;

public:
    void ComputeFreeVertexWeight();
    PoissonDeformation(){}
    void setObj(MeshObj& mesh);
    void ComputeCoefficientMatrix();
    void ComputeDivergence();
    Vector3D ComputeTriangleDiv(const Point3D& source,const Point3D& vleft,const Point3D& vright,int l,int r);
    void TriangleLocalTransform(MyMesh::VertexHandle vh_s,MyMesh::VertexHandle vh_l,MyMesh::VertexHandle vh_r,
                                Point3D& source,Point3D& left,Point3D& right);

    void deform();
    vector<bool> isControlVertex();

    void InterTransform(const Matrix4d &mat);
};



#endif // POISSONDEFORMATION_H
