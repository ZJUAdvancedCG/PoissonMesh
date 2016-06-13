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
#include <QMenuBar>
#include "glwidget.h"
#include "window.h"

//! [0]
Window::Window()
{
    QAction *load = new QAction("Load", this);
    QAction *reset = new QAction("Reset", this);
    QMenuBar *menuBar = new QMenuBar;
    QMenu *menu;
    menu = menuBar->addMenu("Menu");
    menu->addAction(load);
    menu->addAction(reset);
    glWidget = new GLWidget;

    connect(load, &QAction::triggered, [=]{
        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Load Obj"), "../../../../PoissonMesh/obj", tr("Obj Files (*.obj)"));
        glWidget->loadObj(fileName.toStdString());
    });
    connect(reset, &QAction::triggered, [=]{
        glWidget->obj.Reset();
        glWidget->update();
    });

    xSlider = createSlider();
    ySlider = createSlider();
    zSlider = createSlider();

    connect(xSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setXRotation(int)));
    connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider, SLOT(setValue(int)));
    connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
    connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
    connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
    connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));

//! [0]

//! [1]
    QVBoxLayout *globalLayout = new QVBoxLayout;
    globalLayout->addWidget(menuBar);
    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addWidget(glWidget);
    mainLayout->addWidget(xSlider);
    mainLayout->addWidget(ySlider);
    mainLayout->addWidget(zSlider);
    QWidget *w = new QWidget;
    w->setLayout(mainLayout);

    selectModeLabel = new QLabel("Change");
    globalLayout->addWidget(selectModeLabel);
    globalLayout->addWidget(w);

    QVBoxLayout *bottom = new QVBoxLayout;
        QWidget *position = new QWidget;
            QHBoxLayout *controller = new QHBoxLayout;
            px = LineEdit();
            py = LineEdit();
            pz = LineEdit();
            QLabel *pl = new QLabel("Position");
            controller->addWidget(pl);
            controller->addWidget(px);
            controller->addWidget(py);
            controller->addWidget(pz);
        position->setLayout(controller);

        QWidget *rotate = new QWidget;
            QHBoxLayout *rotater = new QHBoxLayout;
            rx = LineEdit();
            ry = LineEdit();
            rz = LineEdit();
            QLabel *rl = new QLabel("Rotate");
            rotater->addWidget(rl);
            rotater->addWidget(rx);
            rotater->addWidget(ry);
            rotater->addWidget(rz);
       rotate->setLayout(rotater);

        QPushButton *button = new QPushButton("Deformation");
    bottom->addWidget(position);
    bottom->addWidget(rotate);
    bottom->addWidget(button);
    QWidget* bw = new QWidget;
    bw->setLayout(bottom);
    globalLayout->addWidget(bw);
    setLayout(globalLayout);

    connect(px, &QLineEdit::editingFinished, [=](){
        QString content = px->text();
        bool ok;
         float value = content.toFloat(&ok);
        if(ok)
        glWidget->setSelectedPosition(value, 0, 0);
        px->clear();
        //glWidget->setFocus();
    });

    connect(py, &QLineEdit::editingFinished, [=](){
        QString content = py->text();
        bool ok;
        float value = content.toFloat(&ok);
        if(ok)
        glWidget->setSelectedPosition(0, value, 0);
        py->clear();
       // glWidget->setFocus();
    });

    connect(pz, &QLineEdit::editingFinished, [=](){
        QString content = pz->text();
        bool ok;
        float value = content.toFloat(&ok);
        if(ok)
        glWidget->setSelectedPosition(0, 0, value);
        pz->clear();
       // glWidget->setFocus();
    });

    connect(rx, &QLineEdit::editingFinished, [=](){
        QString content = rx->text();
        bool ok;
         float value = content.toFloat(&ok);
        if(ok)
        glWidget->rotateSelected(value, 0, 0);
        rx->clear();
       //glWidget->setFocus();
    });

    connect(ry, &QLineEdit::editingFinished, [=](){
        QString content = ry->text();
        bool ok;
        float value = content.toFloat(&ok);
        if(ok)
        glWidget->rotateSelected(0, value, 0);
        ry->clear();
       // glWidget->setFocus();
    });

    connect(rz, &QLineEdit::editingFinished, [=](){
        QString content = rz->text();
        bool ok;
        float value = content.toFloat(&ok);
        if(ok)
        glWidget->rotateSelected(0, 0, value);
        rz->clear();
       // glWidget->setFocus();
    });

    connect(button, &QPushButton::clicked, [=](){
        glWidget->pd.deform();
        glWidget->obj.deformation();
        glWidget->pd.setObj(glWidget->obj);
        qDebug()<<"finish!";
        glWidget->updateGL();
    });

    connect(glWidget, &GLWidget::changeSelectMode, [=](){
        if(glWidget->getSelectMode())
            selectModeLabel->setText("Change");
        else
            selectModeLabel->setText("Fix");
    });

    xSlider->setValue(15 * 16);
    ySlider->setValue(345 * 16);
    zSlider->setValue(0 * 16);
    setWindowTitle(tr("PossionDeformation"));
}
//! [1]

//! [2]
QSlider *Window::createSlider()
{
    QSlider *slider = new QSlider(Qt::Vertical);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    slider->setValue(0);
    return slider;
}

QLineEdit *Window::LineEdit()
{
    QLineEdit *line = new QLineEdit;
    return line;
}

QSlider *Window::createRotater()
{
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

//! [2]

void Window::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}
