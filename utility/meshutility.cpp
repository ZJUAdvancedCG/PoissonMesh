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
  for(auto& vh : sources)
    queue.push(prioritize(vh, 0.));

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
    for (MyMesh::ConstVertexVertexIter vv = mesh.cvv_iter(current); vv.is_valid(); ++vv) {

      double l = (mesh.point(*vv) - mesh.point(current)).length();
      double newdist = d + l;
      assert(isfinite(newdist));

      double olddist = inf;
      if (dist.count(vv->idx())) {
        olddist = dist[vv->idx()];
      }

      if (newdist < olddist) {
        dist[vv->idx()] = newdist;
        queue.push(prioritize(vv.handle(), newdist));
      }
    }
  }

  return dist;
}
