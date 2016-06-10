#include "mesh.h"
#include <fstream>
using namespace std;

MeshObj::MeshObj(string filename)
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

     if (!OpenMesh::IO::read_mesh(mesh, filename))
     {
         qDebug() << "Error: Cannot read mesh from " << filename.data();
         //qDebug() << filename
     }
     else{
         qDebug() << "Succ";
     }


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
    glPointSize(100);
    for(int id:selectVertexIds)
    {
        MyMesh::VertexHandle vhl(id);
        MyMesh::Point point = mesh.point(vhl);
        //qDebug() << point[0] << point[1] << point[2];
        functions->glVertex3f(point[0], point[1], point[2]);
    }

    functions->glColor3f(1, 0, 0);
    glPointSize(5);

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
        for(MyMesh::FaceVertexIter fvit=mesh.fv_begin(fit);fvit!=mesh.fv_end(fit);fvit++)
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

void MeshObj::select(QMatrix4x4 modelview, QMatrix4x4 project, const int viewport[4], int viewheight)
{
    selectVertexIds.clear();
    //qDebug() << viewport;

    for (MyMesh::VertexIter vit = mesh.vertices_begin(); vit != mesh.vertices_end(); ++vit)
    {
        MyMesh::Point point = mesh.point(vit);
        int id = vit.handle().idx();
        QVector4D vertex(point[0], point[1], point[2], 1.0);
        //qDebug() << vertex;
        vertex = vertex*modelview;
        vertex = vertex*project;

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
    //qDebug() << rect;
    //qDebug() << "size " << selectVertexIds.size();
}

void MeshObj::changeSelectedPosition(float dx, float dy, float dz)
{
    qDebug() << dx << dy << dz;
    for(int id:selectVertexIds)
    {
        MyMesh::VertexHandle vhl(id);
        MyMesh::Point point = mesh.point(vhl);
        MyMesh::Point d(dx, dy, dz);
        qDebug() << point[0] << " " << point[1] << " " << point[2];

        mesh.set_point(vhl, point+d);
        //
        point = mesh.point(vhl);
        qDebug() << point[0] << " " << point[1] << " " << point[2];
    }
}

#include <QMessageBox>

void MeshObj::deformation()
{
}
