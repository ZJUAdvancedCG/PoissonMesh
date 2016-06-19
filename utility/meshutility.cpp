#include "meshutility.h"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <../mesh.h>
#include <geode/utility/prioritize.h>

#include <float.h>
#include <iostream>
using namespace std;
using namespace geode;
using namespace OpenMesh;
#define inf DBL_MAX

unordered_map<int, double> geodesic_distance(MyMesh& mesh, MyMesh::VertexHandle source,
                                             vector<MyMesh::VertexHandle> const &sinks) {
    return geodesic_distance(mesh,vector<MyMesh::VertexHandle>(1,source), sinks);
}



unordered_map<int, double> geodesic_distance(MyMesh& mesh,vector<MyMesh::VertexHandle> const &sources,
                                                                     vector<MyMesh::VertexHandle> const &sinks){

  // initialize distance map and unassigned set
  unordered_map<int, double> dist;
  for(auto& vh : sources)
    dist[vh.idx()] = 0.;
  unordered_set<int> unassigned;
  for(auto& vv:sinks){
      unassigned.insert(vv.idx());
  }

  for(auto& vh : sources)
    unassigned.erase(vh.idx());

  //cout << "computing geodesic distance from " << source << " to " << sinks.size() << " sinks, " << unassigned.size() << " distances unassigned." << endl;

  if (unassigned.empty())
    return dist;

  std::priority_queue<Prioritize<MyMesh::VertexHandle>, vector<Prioritize<MyMesh::VertexHandle> >, std::greater<Prioritize<MyMesh::VertexHandle> > > queue;
  //小顶堆
  for(auto& vh : sources)
    queue.push(prioritize(vh, 0.));
  //初始为0
  while (!queue.empty()) {
    MyMesh::VertexHandle current = queue.top().a;
    double d = queue.top().p;
    queue.pop();

    // assign distance if not set yet
    if (unassigned.count(current.idx())) {
      dist[current.idx()] = d;
      unassigned.erase(current.idx());

      // nothing else to do? leave.
      if (unassigned.empty()) {
        break;
      }
    }
    for (MyMesh::ConstVertexVertexIter vv = mesh.cvv_iter(current); vv; ++vv) {

      double l = (mesh.point(vv.handle()) - mesh.point(current)).length();
      //auto point = (mesh.point(vv.handle()) - mesh.point(current));
      //double l = point[0]*point[0] + point[1]*point[1] + point[2]*point[3];
      double newdist = d + l;
      assert(isfinite(newdist));

      double olddist = inf;
      if (dist.count(vv.handle().idx())) {
        olddist = dist[vv.handle().idx()];
      }

      if (newdist < olddist) {
        dist[vv.handle().idx()] = newdist;
        queue.push(prioritize(vv.handle(), newdist));
      }
    }
  }

  return dist;
}
