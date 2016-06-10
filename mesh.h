#ifndef MESH_H
#define MESH_H
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <QOpenGLFunctions_1_1>
#include <QDebug>
#include <string>
#include <QMatrix4x4>
#include <QVector4D>
#include <QRect>
#include <algorithm>
#include <QDebug>
using namespace std;
struct MeshTraits : OpenMesh::DefaultTraits
{
    // let point and normal be a vector of doubles
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
};


//typedef OpenMesh::DefaultTraits MeshTraits;
typedef OpenMesh::TriMesh_ArrayKernelT<MeshTraits> MyMesh;

class MeshObj
{
private:
    MyMesh mesh;
    GLuint drawList;
    QRect rect;
    vector<int> selectVertexIds;
    MyMesh copy;

    //QPoint win_coord(MyMesh::Point point);

public:
    MeshObj(string filename);
    void draw(QOpenGLFunctions_1_1 *functions);
    void select(QMatrix4x4 modelview, QMatrix4x4 project, const int viewport[4], int viewheight);
    void setRegion(QRect rect);
    void changeSelectedPosition(float dx, float dy, float dz);
    void rotateSelected(float rx, float ry, float rz);
    void deformation();
    vector<int>& getSelectVertexIds(){return selectVertexIds;}
    MyMesh& getMesh(){return mesh;}
    void loadObj(string filename);
    void Reset();




};

#endif // MESH_H
