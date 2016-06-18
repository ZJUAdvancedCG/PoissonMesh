#include "meshlaplaciansolver.h"
#include "../utility/pointvector.h"
#include <algorithm>
#include <QDebug>
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
void MeshLaplacianSolver::ComputeVertexLaplacianWeight()
{
    mesh.add_property(vertexLPLWeight);
    for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it){
        MyMesh::Point point = mesh.point(*v_it);
        Vector3D currentVertex = Vector3D(point[0],point[1],point[2]);
        vector<Vector3D> neighborVertices;
        for(MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it);vv_it.is_valid();++vv_it)
        {
            MyMesh::Point point = mesh.point(*vv_it);
            neighborVertices.push_back(Vector3D(point[0],point[1],point[2]));
        }

        bool isBorderVertex = mesh.is_boundary(*v_it);
        int numNeighbors = (int)neighborVertices.size();
        vector<double> weights(numNeighbors,0);
        for (int i=0; i<numNeighbors; i++)
        {
            double w1 = CotValue(neighborVertices[(i + numNeighbors -1)%numNeighbors], currentVertex, neighborVertices[i]);
            double w2 = CotValue(neighborVertices[(i+1)%numNeighbors], currentVertex, neighborVertices[i]);

            if(isBorderVertex)
            {
                if(i == 0)
                    w1 = CotValue(neighborVertices[i], currentVertex, neighborVertices[i]);
                if(i == numNeighbors - 1)
                    w2 = CotValue(neighborVertices[i], currentVertex, neighborVertices[i]);
            }
            weights[i] = 0.5*(w1+w2);
            weights[i] = max(weights[i], EPSILON);
        }
        mesh.property(vertexLPLWeight, *v_it) = weights;
    }

}

#include <iostream>
void MeshLaplacianSolver::ComputeLaplacianMatrix()
{
    ComputeVertexLaplacianWeight();
    //assert(!selectVertexId.empty());
    assert(!isControlVertex.empty());

    int numVertices = mesh.n_vertices();
    A.resize(numVertices,numVertices);

    for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {
        int vindex = v_it->idx();

       // if (find(selectVertexId.begin(),selectVertexId.end(),vindex)!=selectVertexId.end())
        if(isControlVertex[vindex])
        {
            // control points
            A.coeffRef(vindex,vindex) = 1.0;
        }
        else
        {
            double sum = 0.0;
            vector<double> weights = mesh.property(vertexLPLWeight, *v_it);
            int id = 0;
            for (MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it);  vv_it.is_valid();  ++vv_it)
            {

                int vvindex = vv_it->idx();
                A.coeffRef(vindex, vvindex) = -weights[id];
                sum +=  weights[id++];
            }
            A.coeffRef(vindex,vindex) = sum;
            //qDebug()<<sum<<" ";
        }
    }

    //pre QR decompose
    //std::cout << "---A---" << std::endl;
    //std::cout << A << std::endl;
    //std::cout <<"--compressed A--" <<std::endl;
    A.makeCompressed();
    //std::cout << A << std::endl;
//    solver.analyzePattern(A);
    //solver.compute(A);
    bc.compute(A);
}

Eigen::VectorXd& MeshLaplacianSolver::LaplacainSolve()
{
//    X = solver.solve(B);
    X = bc.solve(B);
    return X;
}
