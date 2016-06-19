#ifndef MESHUTILITY_H
#define MESHUTILITY_H

#include <unordered_map>
#include <vector>
#include <../mesh.h>

using namespace std;

unordered_map<int, double> geodesic_distance(MyMesh& mesh, MyMesh::VertexHandle source,
                                             vector<MyMesh::VertexHandle> const &sinks);


unordered_map<int, double> geodesic_distance(MyMesh& mesh,vector<MyMesh::VertexHandle> const &sources,
                                                                     vector<MyMesh::VertexHandle> const &sinks);
#endif // MESHUTILITY_H
