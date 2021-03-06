#ifndef POISSONDEFORMATION_H
#define POISSONDEFORMATION_H
#include "../mesh.h"
#include "meshlaplaciansolver.h"

#include "../utility/quaternion.h"

#include <map>
#include <vector>

typedef Vector3d Vector3D;
typedef Vector3d Point3D;




class PoissonDeformation
{
private:
    MeshObj* pMeshObj;
    VectorXd divMatrixX;
    VectorXd divMatrixY;
    VectorXd divMatrixZ;
    MeshLaplacianSolver LPsolver;
    QQuaternion m_quater_fixed;
    //QQuaternion m_quater_hand;
    Matrix4d m_handTransMat;

    MyMesh mesh;
    vector<int> selectVertexId;
    vector<int> fixVertexId;
    MyMesh m_static_mesh;
    std::vector<double> freeVertexWeight;

    //
    map<int, QQuaternion> rotations;

public:
    void testMeshs();
    void ComputeFreeVertexWeight();
    //void ComputeFree();
    PoissonDeformation(){}
    void setObj(MeshObj& mesh);
    void ComputeCoefficientMatrix();
    void ComputeDivergence();
    Vector3D ComputeTriangleDiv(const Point3D& source,const Point3D& vleft,const Point3D& vright,int l,int r);
    void TriangleLocalTransform(MyMesh::VertexHandle vh_s,MyMesh::VertexHandle vh_l,MyMesh::VertexHandle vh_r,
                                Point3D& source,Point3D& left,Point3D& right);

    void deform();
    vector<bool> isControlVertex();

    void InterTransform(const vector<int> &selectedVertexIds, const QQuaternion &rotate);
};



#endif // POISSONDEFORMATION_H
