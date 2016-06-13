/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtWidgets>
#include <QtOpenGL>
#include <math.h>
#include <Eigen/Eigen>

#include "glwidget.h"
//#include "qtlogo.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

//! [0]
GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
      obj()
   // ,rubberband(QRubberBand::Rectangle, this)
{
    //logo = 0;
    selectMode = true;
    xRot = 0;
    yRot = 0;
    zRot = 0;

    qtGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
    qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);

    scale = 1.0f;
}
//! [0]

//! [1]
GLWidget::~GLWidget()
{
}
//! [1]

//! [2]
QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}
//! [2]

//! [3]
QSize GLWidget::sizeHint() const
//! [3] //! [4]
{
    return QSize(400, 400);
}
//! [4]

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

//! [5]
void GLWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
        update();
    }
}
//! [5]

void GLWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
        update();
    }
}

void GLWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
        update();
    }
}

void GLWidget::setSelectedPosition(float dx, float dy, float dz)
{
    obj.changeSelectedPosition(dx, dy, dz);
    Affine3d transform(Translation3d(dx, dy, dz));
    Matrix4d mat = transform.matrix();
    pd.InterTransform(mat);
    update();
}

void GLWidget::rotateSelected(float x, float y, float z)
{
    obj.rotateSelected(x, y, z);
    QQuaternion rotate = QQuaternion::fromEulerAngles(x, y, z);
    qDebug() << "rotate Matrix";
    qDebug() << rotate.toRotationMatrix();
    QVector4D qrotate = rotate.toVector4D();
    Matrix3d mat3 = Quaterniond(qrotate.w(),qrotate.x(), qrotate.y(), qrotate.z()).toRotationMatrix();
    Matrix4d mat4 = Matrix4d::Identity();
    mat4.block(0,0,3,3) = mat3;

    pd.InterTransform(mat4);

    qDebug() << "update!!";
    update();
}

void GLWidget::loadObj(string filename)
{
    obj.loadObj(filename);
}

//! [6]
void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    qglClearColor(QColor(0, 0, 0));

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_MULTISAMPLE);
    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}
//! [6]

void GLWidget::setLight()
{
    GLfloat ambient[] = {0.0, 0.0, 0.0, 1.0};
    GLfloat diffuse[] = {1.0, 1.0, 1.0, 1.0};
    //GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat position[] = {0.0, 3.0, 2.0, 0.0};
    //GLfloat lmodel_ambient[] = {0.4, 0.4, 0.4, 1.0};

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    GLfloat mat_ambient[] = {1.0, 1.0, 1.0, 1.0};
    //GLfloat mat_ambient_color[] = {0.8,0.8,0.2, 1.0};
    GLfloat mat_diffuse[] = {0.1, 0.5, 0.9, 1.0};
    GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat high_shininess[] = {100.0};

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

}

//! [7]
void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    //set light
    setLight();

    glTranslatef(0.0, 0.0, -10.0f);
    glScalef(1/scale, 1/scale, 1/scale);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
    //logo->draw(static_cast<QOpenGLFunctions_1_1 *>(this));
    //glPushMatrix();
    if(select)
    {
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        float modelview_data[16];
        float project_data[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview_data);
        glGetFloatv(GL_PROJECTION_MATRIX, project_data);
        //qDebug() << "select!";
        if(selectMode)
            obj.select(QMatrix4x4(modelview_data), QMatrix4x4(project_data), viewport, viewHeight);
        else
            obj.fix(QMatrix4x4(modelview_data), QMatrix4x4(project_data), viewport, viewHeight);
        pd.setObj(this->obj);
        select = false;
    }
    obj.draw(static_cast<QOpenGLFunctions_1_1 *>(this));

    //glPopMatrix();
}
//! [7]

//! [8]
void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    viewWidth = width;
    viewHeight = height;
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
#ifdef QT_OPENGL_ES_1
    glOrthof(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#else
    glOrtho(-0.5, +0.5, -0.5, +0.5, 0.1, 50.0);
    //glFrustum(-0.5, 0.5, -0.5, 0.5, 1.5, 40.0);
#endif
    glMatrixMode(GL_MODELVIEW);
}
//! [8]

//! [9]
void GLWidget::mousePressEvent(QMouseEvent *event)
{
    //regison selection
    origin = event->pos();
    //qDebug() << "mouse Press";
    //qDebug() << origin;
    /*if(!rubberband)
        rubberband = new QRubberBand(QRubberBand::Rectangle, this);
    rubberband->setGeometry(QRect(lastPos, QSize()));
    rubberband->show();
    update();*/
}
//! [9]

//! [10]
void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    //qDebug() << "Pressing...";
    //
   /* if(!rubberband->isHidden())
    {
        rubberband->setGeometry(QRect(origin, event->pos()).normalized());
        update();
    }*/
}
//! [10]
//!

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    //rubberband->hide();
    //qDebug() << "Start select";
    QPoint pos = event->pos();
    //qDebug() << "End " << pos;
    if(pos == origin)
    {
        selectRegion = QRect(origin,QSize(5,5));
    }
    else
    {
        int left = std::min(origin.x(), pos.x());
        int right = std::max(origin.x(), pos.x());

        int top = std::min(origin.y(), pos.y());
        int down = std::max(origin.y(), pos.y());

        selectRegion = QRect(left, top, right-left, down-top);
    }

    select = true;
    obj.setRegion(selectRegion);


    update();
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    if(scale+event->delta()/40<=0)
        return;
    scale+=(event->delta()/40);
    update();
}

void GLWidget::keyPressEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Equal)
        scale += 1;
    else if(event->key()==Qt::Key_Minus)
        scale -= 1;

    //handle select mode change
    if(event->key()==Qt::Key_Shift)
    {
        selectMode = !selectMode;
        qDebug() << "selectMode" << selectMode;
        emit changeSelectMode(selectMode);
    }
    update();
}
