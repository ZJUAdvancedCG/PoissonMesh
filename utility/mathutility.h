#ifndef MATHUTILITY_H
#define MATHUTILITY_H
#include "pointvector.h"
#include "../utility/basedefine.h"
#include "Eigen/Core"
#include <vector>
#include "mesh.h"
using namespace std;
using namespace Eigen;

class MathUtility
{
public:
    MathUtility(){}
    static Point3D ComputeMatrixMultiPoint(Matrix4d Mt, Point3D point);
    static Point3D ComputeMatrixMultiPoint(Matrix3d Mt, Point3D point);
    static Vector3D ComputeMatrixMutiVector(Matrix3d Mt, Vector3D vec);
    static double GetTriangleArea(const Point3D& v1,  const Point3D& v2, const Point3D& v3);
    static Vector3D GetTriangleVertexGradient( Vector3D a,  Vector3D b);
    static Matrix4d Rotation2Matrix(double angle,int x,int y,int z);
    static Matrix4d Rotation2Matrix(double angle_x, double angle_y, double angle_z, bool x, bool y, bool z);
    static Matrix4d Translate2Matrix(double x,double y,double z);
    static Matrix4d Scale2Matrix(double x,double y,double z);
    static Matrix4d MatrixMultNum(const Matrix4d mat, double factor);
    static Vector3D getNormalOfTri(const Point3D p1,const Point3D p2,const Point3D p3);
    static double ComputeMeshVolume(MyMesh& desMesh);
    static void  EvaluateVNormalByArea(MyMesh& desMesh,vector<Vector3D>& vNormal);
};

#endif // MATHUTILITY_H
