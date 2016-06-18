#include "quaternion.h"
#include <cmath>


//将一个四元数转化为一个3*3的旋转矩阵，稍加修改矩阵可以扩展为4*4
void quaterToMatrix(QQuaternion& quater,Matrix4d& q_r)
{
    double x = quater.x();
    double y = quater.y();
    double z = quater.z();
    double w = quater.scalar();

    q_r(0,0) = 1 - 2*y*y - 2*z*z;
    q_r(0,1) = 2*x*y - 2*z*w;
    q_r(0,2) = 2*y*w + 2*x*z;
    q_r(0,3) = 0;

    q_r(1,0) = 2*x*y + 2*z*w;
    q_r(1,1) = 1- 2*x*x - 2*z*z;
    q_r(1,2) = -2*x*w + 2*y*z;
    q_r(1,3) = 0;

    q_r(2,0) = -2*y*w + 2*x*z;
    q_r(2,1) = 2*x*w + 2*y*z;
    q_r(2,2) = 1-2*x*x - 2*y*y;
    q_r(2,3) = 0;

    q_r(3,0) = 0;
    q_r(3,1) = 0;
    q_r(3,2) = 0;
    q_r(3,3) = 1;
}

//旋转矩阵（4*4）转换为四元数
QQuaternion matrixToQuater(const Matrix4d& rotation)
{
    double e_1_2 = rotation(1,2);
    double e_2_0 = rotation(2,0);
    double angle = 0;
    if(e_1_2 != 0)
    {
        angle = asin(-e_1_2)/PI  * 180;
        return QQuaternion::fromAxisAndAngle(1,0,0,angle);
        //R2Q(angle,1,0,0);
    }
    else if(e_2_0 != 0)
    {
        angle = asin(-e_2_0)/PI * 180;
        return QQuaternion::fromAxisAndAngle(0,1,0,angle);
        //R2Q(angle,0,1,0);
    }
}

