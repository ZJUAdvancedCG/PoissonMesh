#include "mesh.h"
#include <fstream>
#include <QQuaternion>
#include <QVector>
using namespace std;

MeshObj::MeshObj()
{
    /*try{
    ifstream in(filename);
    string line;
    while(getline(in, line)){
        qDebug() << line.data();
    }
    }
    catch(std::ios_base::failure& e) {
        qDebug() << e.what();
    }*/

}

struct MyVertex{
    float x, y, z;
};

void MeshObj::draw(QOpenGLFunctions_1_1 *functions)
{	
    //functions->glNewList(drawList, GL_COMPILE);
    functions->glPushMatrix();
    //functions->glScalef(0.2,0.2,0.2);
	functions->glLineWidth(1);
	functions->glDisable(GL_LIGHTING);

	functions->glBegin(GL_POINTS);

    //draw selected points
    functions->glColor3f(0,1,0);
    functions->glPointSize(100);
    for(int id:selectVertexIds)
    {
        MyMesh::VertexHandle vhl(id);
        MyMesh::Point point = mesh.point(vhl);
        functions->glVertex3f(point[0], point[1], point[2]);
    }
    functions->glEnd();

    functions->glBegin(GL_POINTS);

    //draw fix points
    functions->glColor3f(0,0,1);
    functions->glPointSize(100);
    for(int id:fixVertexIds)
    {
        MyMesh::VertexHandle vhl(id);
        MyMesh::Point point = mesh.point(vhl);
        functions->glVertex3f(point[0], point[1], point[2]);
    }
    functions->glEnd();

    functions->glBegin(GL_POINTS);
    functions->glColor3f(1, 0, 0);
    functions->glPointSize(5);

    for (MyMesh::VertexIter vit = mesh.vertices_begin(); vit != mesh.vertices_end(); ++vit)
	{
        //MyMesh::Point point = mesh.point(vit.handle());
        //functions->glVertex3f(point[0], point[1], point[2]);
        functions->glVertex3dv(mesh.point(*vit).data());
    }

	functions->glEnd();
    functions->glPointSize(1);
    //functions->glEndList();

    //functions->glCallList(drawList);
    //functions->glFlush();
    mesh.request_face_normals();
    mesh.request_vertex_normals();
    mesh.update_normals();

    functions->glPushAttrib(GL_ALL_ATTRIB_BITS);
    functions->glEnable(GL_POLYGON_OFFSET_FILL);
    functions->glPolygonOffset(1, 1);
    functions->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    functions->glDisable(GL_COLOR_MATERIAL);

    functions->glEnable(GL_LIGHTING);

    for(MyMesh::FaceIter fit = mesh.faces_begin();fit!=mesh.faces_end();fit++)
    {
        functions->glBegin(GL_POLYGON);

        functions->glNormal3dv(mesh.normal(*fit).data());
        for(MyMesh::FaceVertexIter fvit=mesh.fv_iter(*fit);fvit.is_valid();fvit++)
        {
            functions->glNormal3dv(mesh.normal(*fvit).data());
            functions->glVertex3dv(mesh.point(*fvit).data());
        }
        functions->glEnd();
    }
    functions->glPopAttrib();

    functions->glPopMatrix();

}

void MeshObj::setRegion(QRect rect)
{
    this->rect = rect;
}

void MeshObj::fix(QMatrix4x4 modelview, QMatrix4x4 project, const int viewport[4], int viewheight)
{
    fixVertexIds.clear();
    //qDebug() << viewport;

    for (MyMesh::VertexIter vit = mesh.vertices_begin(); vit != mesh.vertices_end(); ++vit)
    {
        MyMesh::Point point = mesh.point(*vit);
        int id = vit->idx();
        QVector4D vertex(point[0], point[1], point[2], 1.0);
        //qDebug() << vertex;
        vertex = vertex*modelview;
        vertex = vertex*project;

        if(fabs(vertex[3]-0.0)>=0.0000001f)
        {
            float div = 1.0f/vertex[3];
            vertex[0] *= div;
            vertex[1] *= div;
        }

        float x = float(viewport[0])+(1.0+vertex[0])*viewport[2]/2.0f;
        float y = float(viewport[1])+(1.0+vertex[1])*viewport[3]/2.0f;
        //qDebug() << "x:" << x << "y:" << y;
        y = viewheight-y;
        QPoint pos(x,y);
        //qDebug() << pos;

        if(rect.contains(pos))
        {
            fixVertexIds.push_back(id);
            //qDebug() << "Fix: Contains" << id;
        }
    }
}


void MeshObj::select(QMatrix4x4 modelview, QMatrix4x4 project, const int viewport[4], int viewheight)
{
    selectVertexIds.clear();
    //qDebug() << viewport;

    for (MyMesh::VertexIter vit = mesh.vertices_begin(); vit != mesh.vertices_end(); ++vit)
    {
        MyMesh::Point point = mesh.point(*vit);
        int id = vit->idx();
        QVector4D vertex(point[0], point[1], point[2], 1.0);
        //qDebug() << vertex;
        vertex = vertex*modelview;
        vertex = vertex*project;

        if(fabs(vertex[3]-0.0)>=0.0000001f)
        {
            float div = 1.0f/vertex[3];
            vertex[0] *= div;
            vertex[1] *= div;
        }

        float x = float(viewport[0])+(1.0+vertex[0])*viewport[2]/2.0f;
        float y = float(viewport[1])+(1.0+vertex[1])*viewport[3]/2.0f;
        //qDebug() << "x:" << x << "y:" << y;
        y = viewheight-y;
        QPoint pos(x,y);
        //qDebug() << pos;

        if(rect.contains(pos))
        {
            selectVertexIds.push_back(id);
            //qDebug() << "Contains" << id;
        }
    }

    //force debug
   /*selectVertexIds.clear();
    selectVertexIds.push_back(0);
    selectVertexIds.push_back(1);
    selectVertexIds.push_back(2);
    selectVertexIds.push_back(3);*/


    //qDebug() << rect;
    //qDebug() << "size " << selectVertexIds.size();
}

void MeshObj::changeSelectedPosition(float dx, float dy, float dz)
{
    //qDebug() << dx << dy << dz;
    for(int id:selectVertexIds)
    {
        MyMesh::VertexHandle vhl(id);
        MyMesh::Point point = mesh.point(vhl);
        MyMesh::Point d(dx, dy, dz);
        //qDebug() << point[0] << " " << point[1] << " " << point[2];

        mesh.set_point(vhl, point+d);
        //
        //point = mesh.point(vhl);
        //qDebug() << point[0] << " " << point[1] << " " << point[2];
    }
}

void MeshObj::rotateSelected(float rx, float ry, float rz)
{
    QQuaternion rotate = QQuaternion::fromEulerAngles(rx, ry, rz);
    QMatrix4x4 matrix(rotate.toRotationMatrix());

    for(int id:selectVertexIds)
    {
        MyMesh::VertexHandle vhl(id);
        QVector4D position(mesh.point(vhl)[0], mesh.point(vhl)[1], mesh.point(vhl)[2], 1.0);
        position = matrix*position;
        MyMesh::Point newposition(position.x(), position.y(), position.z());
        mesh.set_point(vhl, newposition);
        //
        //point = mesh.point(vhl);
        //qDebug() << point[0] << " " << point[1] << " " << point[2];
    }
}

void MeshObj::loadObj(string filename)
{
    if (!OpenMesh::IO::read_mesh(mesh, filename))
    {
        qDebug() << "Error: Cannot read mesh from " << filename.data();
        //qDebug() << filename
    }
    else{
        copy = mesh;
        //qDebug() << "Succ";
    }

}

void MeshObj::Reset()
{
    mesh = copy;
    selectVertexIds.clear();
    fixVertexIds.clear();
}

#include <QMessageBox>

void MeshObj::deformation()
{
}

void MeshObj::clearSelect()
{
    selectVertexIds.clear();
    fixVertexIds.clear();
}

#include <queue>
#include <algorithm>

struct PathNode{
    int id;
    double distance;

    PathNode(int _id, double _distance)
        :id(_id), distance(_distance){}
    bool operator<(const PathNode&n) const{
        if(distance == n.distance)
            return id<n.id;
        return distance < n.distance;
    }
};

void MeshObj::TestGeode()
{
    int start = 0;
    vector<bool> visited(mesh.n_vertices(),false);
    vector<double> dist(mesh.n_vertices(),1<<30);
    priority_queue<PathNode> q;
    q.push(PathNode(start, 0));
    while(!q.empty())
    {
        PathNode n = q.top();
        q.pop();
        if(visited[n.id])
            continue;
        visited[n.id] = true;
        //circulate around neigborhood
        MyMesh::VertexHandle vhl(n.id);
        MyMesh::Point current = mesh.point(vhl);
        for (MyMesh::VertexVertexIter vv_it=mesh.vv_iter(vhl); vv_it.is_valid(); ++vv_it)
        {
            int neighborId = vv_it->idx();
            MyMesh::Point neigbor = mesh.point(vv_it);
            double newdist = n.distance+(neigbor-current).length();
            if(newdist < dist[neighborId])
            {
                //update
                dist[neighborId] = newdist;
                q.push(PathNode(neighborId, newdist));
            }
        }
    }

    //test result
    QVector<double> dist2select;
    for(int i:selectVertexIds)
    {
        dist2select.push_back(dist[i]);
    }
    qDebug() << dist2select;
}
