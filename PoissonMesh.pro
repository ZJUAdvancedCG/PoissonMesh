
CONFIG += c++11
HEADERS       = glwidget.h \
                window.h \
    mesh.h \
    utility/basedefine.h \
    Deformation/meshlaplaciansolver.h \
    Deformation/poissondeformation.h \
    utility/mathutility.h \
    utility/pointvector.h \
    utility/quaternion.h

SOURCES       = glwidget.cpp \
                main.cpp \
                window.cpp \
                mesh.cpp \
    Deformation/meshlaplaciansolver.cpp \
    Deformation/poissondeformation.cpp \
    utility/mathutility.cpp \
    utility/pointvector.cpp \
    utility/quaternion.cpp

QT           += opengl widgets

# install
#target.path = $$[QT_INSTALL_EXAMPLES]/opengl/legacy/hellogl
#INSTALLS += target

contains(QT_CONFIG, opengles.|angle):error("This example requires Qt to be configured with -opengl desktop")



macx: LIBS += -L$$PWD/lib/ -lOpenMeshCore

INCLUDEPATH += $$PWD
INCLUDEPATH += $$PWD/eigen3
# DEPENDPATH += $$PWD/.

# macx: PRE_TARGETDEPS += $$PWD/lib/libOpenMeshCore.a
