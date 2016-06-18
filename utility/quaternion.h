#ifndef QUATERNION_H
#define QUATERNION_H
#include "basedefine.h"
#include <QQuaternion>



void quaterToMatrix(QQuaternion& quater,Matrix4d& q_r);
QQuaternion matrixToQuater(const Matrix4d& rotation);

#endif // CQUATERNION_H
