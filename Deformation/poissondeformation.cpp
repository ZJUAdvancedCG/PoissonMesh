#include "poissondeformation.h"
#include "../utility/mathutility.h"

void PoissonDeformation::setObj(MeshObj& meshObj)
{
    this->pMeshObj=&meshObj;
    this->mesh = pMeshObj->getMesh();
    this->selectVertexId=pMeshObj->getSelectVertexIds();
    m_handTransMat = Matrix4d::Identity();
    m_quater_fixed.R2Q(0,1,0,0);

    LPsolver.set(mesh,selectVertexId);
}


void PoissonDeformation::ComputeCoefficientMatrix()
{
    LPsolver.ComputeLaplacianMatrix();
    ComputeFreeVertexWeight();
}

void PoissonDeformation::ComputeFreeVertexWeight(){
    MeshLaplacianSolver tmpLPsolver;
    tmpLPsolver.set(this->pMeshObj->getMesh(),this->pMeshObj->getSelectVertexIds());
    tmpLPsolver.ComputeLaplacianMatrix();

    int nvertex = mesh.n_vertices();
    VectorXd b(nvertex); b.setZero();

    for(int i = 0; i < selectVertexId.size(); i++)
    {
        int handIndex = selectVertexId[i];
        b[handIndex]  =  1;
    }
    tmpLPsolver.setRightHand(b);

    VectorXd  x = tmpLPsolver.LaplacainSolve();
    for(int i = 0; i < nvertex; i++)
        freeVertexWeight.push_back(1-x[i]);
}

void PoissonDeformation::ComputeDivergence()
{

    MyMesh& mesh = this->pMeshObj->getMesh();
    m_static_mesh = mesh;
    divMatrixX = VectorXd::Zero(mesh.n_vertices());
    divMatrixY = VectorXd::Zero(mesh.n_vertices());
    divMatrixZ = VectorXd::Zero(mesh.n_vertices());
    vector<int> selectVertexId= this->pMeshObj->getSelectVertexIds();


    int vid = 0,l = 0,r = 0;
    MyMesh::VertexHandle vh0,vh1,vh2;
    for(MyMesh::VertexIter v_it = m_static_mesh.vertices_begin(); v_it != m_static_mesh.vertices_end(); v_it++)
    {
        vid = v_it.handle().idx();
        if (find(selectVertexId.begin(),selectVertexId.end(),vid)!=selectVertexId.end())
        {
            divMatrixX[vid] = mesh.point(v_it)[0];
            divMatrixY[vid] = mesh.point(v_it)[1];
            divMatrixZ[vid] = mesh.point(v_it)[2];
            continue;
        }

        for(MyMesh::VertexFaceIter vf_it = m_static_mesh.vf_begin(v_it); vf_it != m_static_mesh.vf_end(v_it); vf_it++)
        {
            MyMesh::FaceVertexIter fv_it = m_static_mesh.fv_begin(vf_it);
            int tri_vid0 = fv_it.handle().idx(); fv_it++;
            int tri_vid1 = fv_it.handle().idx(); fv_it++;
            int tri_vid2 = fv_it.handle().idx();

            if(tri_vid0 == vid)		{ l = tri_vid1; r = tri_vid2;}
            else if(tri_vid1 == vid){ l = tri_vid0; r = tri_vid2;}
            else					{ l = tri_vid0; r = tri_vid1;}
            vh0 = m_static_mesh.vertex_handle(vid);
            vh1 = m_static_mesh.vertex_handle(l);
            vh2 = m_static_mesh.vertex_handle(r);

            //triangle local transform
            Point3D source_,right_,left_;
            TriangleLocalTransform(vh0,vh1,vh2,source_,left_,right_);

            //compute divergence
            Vector3D W = ComputeTriangleDiv(source_,left_,right_,l,r);
            divMatrixX[vid] += W.m_x;
            divMatrixY[vid] += W.m_y;
            divMatrixZ[vid] += W.m_z;
        }
    }

    //vertex at hand
    for(int id:selectVertexId)
    {
        MyMesh::VertexHandle vhl(id);
        MyMesh::Point point = mesh.point(vhl);
        divMatrixX[vid] = point[0];
        divMatrixY[vid] = point[1];
        divMatrixZ[vid] = point[2];
    }
}

Vector3D PoissonDeformation::ComputeTriangleDiv(const Point3D& source,const Point3D& vleft,const Point3D& vright,int l,int r)
{
    Vector3D s_l = source - vleft;
    Vector3D s_r = source - vright;
    Vector3D l_s = s_l*(-1);
    Vector3D l_r = vleft - vright;
    Vector3D r_s = s_r*(-1);
    Vector3D r_l = vright - vleft;

    //▽ΦiT
    Vector3D ha =  MathUtility::GetTriangleVertexGradient(s_l,s_r);
    Vector3D hb =  MathUtility::GetTriangleVertexGradient(l_r,l_s);
    Vector3D hc =  MathUtility::GetTriangleVertexGradient(r_s,r_l);

    //gradient field
    Vector3D wx = hb*(l_s.m_x) + hc*(r_s.m_x);
    Vector3D wy = hb*(l_s.m_y) + hc*(r_s.m_y);
    Vector3D wz = hb*(l_s.m_z) + hc*(r_s.m_z);

    //S△
    double area = MathUtility::GetTriangleArea(source,vleft,vright);

    //divergence
    Vector3D div = Vector3D(wx*ha*area,wy*ha*area,wz*ha*area);
    return   div;
}

void PoissonDeformation::TriangleLocalTransform(MyMesh::VertexHandle vh_s,MyMesh::VertexHandle vh_l,MyMesh::VertexHandle vh_r,
                            Point3D& source,Point3D& left,Point3D& right)
{
    MyMesh& mesh = this->pMeshObj->getMesh();
    Matrix4d triangleTransMatrix = Matrix4d::Identity();
    Matrix4d interpMat= Matrix4d::Identity();

    int s = vh_s.idx(), l = vh_l.idx(), r = vh_r.idx();
    Point3D v      =  Point3D(mesh.point(vh_s)[0],mesh.point(vh_s)[1],mesh.point(vh_s)[2]);
    Point3D	vleft  =  Point3D(mesh.point(vh_l)[0],mesh.point(vh_l)[1],mesh.point(vh_l)[2]);
    Point3D vright =  Point3D(mesh.point(vh_r)[0],mesh.point(vh_r)[1],mesh.point(vh_r)[2]);
    Point3D center((v.m_x+vleft.m_x+vright.m_x)/3.0, (v.m_y+vleft.m_y+vright.m_y)/3.0, (v.m_z+vleft.m_z+vright.m_z)/3.0);

    //ensure local transform
    Matrix4d  t1 = MathUtility::Translate2Matrix(center.m_x,center.m_y,center.m_z);
    Matrix4d  t2 = MathUtility::Translate2Matrix(-center.m_x,-center.m_y,-center.m_z);

    //interpolation with bc`s geodesic distance
    double factor_s = freeVertexWeight[s];
    double factor_l = freeVertexWeight[l];
    double factor_r = freeVertexWeight[r];
    double factor = (factor_s+factor_l+factor_r)/3.0;

    //quaternuon  interpolation
    CQuaternion quater;
    quater.Slerp(m_quater_hand,m_quater_fixed,factor);
    quater.Q2R(interpMat);

    //local transform
    triangleTransMatrix = t1*interpMat*t2;

    source = MathUtility::ComputeMatrixMultiPoint(triangleTransMatrix,v);
    left   = MathUtility::ComputeMatrixMultiPoint(triangleTransMatrix,vleft);
    right  = MathUtility::ComputeMatrixMultiPoint(triangleTransMatrix,vright);
}

void PoissonDeformation::deform()
{
    ComputeCoefficientMatrix();
    ComputeDivergence();

    //solve X
    this->LPsolver.setRightHand(this->divMatrixX);
    VectorXd x = LPsolver.LaplacainSolve();

    //solve y
    this->LPsolver.setRightHand(this->divMatrixY);
    VectorXd y = LPsolver.LaplacainSolve();

    //solve z
    this->LPsolver.setRightHand(this->divMatrixZ);
    VectorXd z = LPsolver.LaplacainSolve();

    //update mesh
    MyMesh::VertexHandle vh;
    MyMesh& mesh = this->pMeshObj->getMesh();
    for(int i = 0; i < mesh.n_vertices(); i++)
    {
        vh  = mesh.vertex_handle(i);

        mesh.point(vh)[0] = x[i];
        mesh.point(vh)[1] = y[i];
        mesh.point(vh)[2] = z[i];
    }

}
