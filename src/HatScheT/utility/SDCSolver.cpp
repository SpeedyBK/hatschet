//
// Created by bkessler on 14/01/20.
//

#include <vector>
#include <climits>
#include "SDCSolver.h"
#include "HatScheT/utility/FibonacciHeap.h"

////////////////////////////////
/// Constraint Graph related ///
////////////////////////////////

void HatScheT::SDCSolver::ConstraintGraph::removeEdge(HatScheT::Vertex &srcVertex, HatScheT::Vertex &dstVertex) {
  for (auto &it : this->edges){
    if (it->getVertexSrc().getId() == srcVertex.getId() && it->getVertexDst().getId() == dstVertex.getId()){
      edges.erase(it);
      return;
    }
  }
  cerr << "SDCSolver::ConstraintGraph: Trying to remove a non-existing Edge!" << endl;
}

bool HatScheT::SDCSolver::ConstraintGraph::doesEdgeExistID(HatScheT::Vertex *src, HatScheT::Vertex *dst) {
  for(auto &it : this->edges){
    if (it->getVertexSrc().getId() == src->getId() && it->getVertexDst().getId() == dst->getId()){
      return true;
    }
  }
  return false;
}

HatScheT::Edge &
HatScheT::SDCSolver::ConstraintGraph::createEdgeSDC(Vertex &Vsrc, Vertex &Vdst, int distance, HatScheT::Edge::DependencyType dependencyType) {

  bool idExists = false;
  for (int i = 0; i <= edges.size(); i++) {
    for (auto &it : Edges()) {
      if (i == it->getId()) {
        idExists = true;
      }
      if (it->getVertexDst().getId() == Vdst.getId() && it->getVertexSrc().getId() == Vsrc.getId()){
        if (it->getDistance() > distance){
          it->setDistance(distance);
        }
        return *it;
      }
    }
    if (!idExists) {
      Edge *e = new Edge(Vsrc, Vdst, distance, dependencyType, i);
      edges.insert(e);
      return *e;
    }
    idExists = false;
  }
  throw HatScheT::Exception("SDCSolver::ConstraintGraph::createEdgeSDC: Failed to create edge - this should never happen");
}

HatScheT::Vertex &HatScheT::SDCSolver::ConstraintGraph::createVertexSDC() {
  return createVertexSDC(++maxVertexId);
}

HatScheT::Vertex &HatScheT::SDCSolver::ConstraintGraph::createVertexSDC(int id) {
  for(auto &it : vertices){
    Vertex* v = it;
    if(v->getId()==id) throw HatScheT::Exception("ConstraintGraph.createVertexSDC: Error! This id is already occupied: " + to_string(id) + "( " + v->getName() +" )");
  }

  auto *v = new Vertex(id);
  vertices.insert(v);
  vertex_index[id] = v;

  //keep maxVertexId consistent
  if(this->maxVertexId < v->getId()) this->maxVertexId = this->getMaxVertexId()+1;

  return *v;
}

HatScheT::Vertex &HatScheT::SDCSolver::ConstraintGraph::getVertexbyIdSDC(int id){
  auto *v = vertex_index[id];
  return *v;
}



//////////////
/// Solver ///
//////////////

HatScheT::SDCSolver::SDCSolver() {
  this->solver_status = 0;
  auto &v = cg.createVertexSDC(-1);
  v.setName("Startvertex");
  startvertex = &v;
}


void HatScheT::SDCSolver::add_sdc_constraint(SDCConstraint constr) {
  bool vsrc_exist = false;
  bool vdst_exist = false;

  for (auto &it : this->cg.Vertices()){
    if (it->getId() == constr.VSrc->getId()){
      vsrc_exist = true;
    }
    if (it->getId() == constr.VDst->getId()){
      vdst_exist = true;
    }
  }
  if (!vsrc_exist) {
    Vertex &V = this->cg.createVertexSDC(constr.VSrc->getId());
    V.setName(constr.VSrc->getName());
  }
  if (!vdst_exist) {
    Vertex &V = this->cg.createVertexSDC(constr.VDst->getId());
    V.setName(constr.VDst->getName());
  }
  this->cg.createEdgeSDC(cg.getVertexbyIdSDC(constr.VSrc->getId()), cg.getVertexbyIdSDC(constr.VDst->getId()), constr.constraint, HatScheT::Edge::Data);
}


void HatScheT::SDCSolver::remove_sdc_constraint(Vertex &Vsrc, Vertex &Vdst) {
  this->cg.removeEdge(Vsrc, Vdst);
}

void HatScheT::SDCSolver::set_start_vertex() {
  for (auto &it: cg.Vertices()){
    if (it->getId() != -1) {
      cg.createEdgeSDC(*startvertex, *it, 0, HatScheT::Edge::Data);
    }
  }
}

void HatScheT::SDCSolver::compute_inital_solution() {
  /*!
   * Setup a Startvertex and create edges from this startvertex to all other vertices.
   */
  set_start_vertex();
  /*!
   * Initialisation of Bellman-Ford-Algorithm
   */
  for (auto &it : this->cg.Vertices()){
    this->solution[it] = INT_MAX;
  }
  this->solution[&this->cg.getVertexbyIdSDC(startvertex->getId())] = 0;

  /*!
   * Finding Shortest Paths
   */
  for (int i = 1; i < this->cg.getNumberOfVertices(); i++){
    for (auto &it : this -> cg.Edges()){
      if ((this->solution[&it->getVertexSrc()] < INT_MAX) && (this->solution[&it->getVertexSrc()] + it->getDistance() < this->solution[&it->getVertexDst()])) {
        this->solution[&it->getVertexDst()] = this->solution[&it->getVertexSrc()] + it->getDistance();
      }
    }
  }

  /*!
   * Checking for negative Cycles
   */
  for (auto &it : this->cg.Edges()){
    if ((this->solution[&it->getVertexSrc()] < INT_MAX) && (this->solution[&it->getVertexSrc()] + it->getDistance() < this->solution[&it->getVertexDst()])){
      solver_status = 11;
      return;
    }
  }
  solver_status = 10;
}


map<HatScheT::Vertex *, int> HatScheT::SDCSolver::get_solution() {
  return this->solution;
}

HatScheT::SDCConstraint
HatScheT::SDCSolver::create_sdc_constraint(HatScheT::Vertex *src, HatScheT::Vertex *dst, int c) {
  SDCConstraint C = {.VSrc = src, .VDst= dst, .constraint = c};
  return C;
}

void HatScheT::SDCSolver::add_to_feasible(SDCConstraint constraint) {
  /*!
   * Local Variables
   */
  auto pQueue = new FibonacciHeap<int>;
  map <Vertex*, int> loc_solution;

  /*!
   * Begin of the Algorithm
   */
  // 1
  add_sdc_constraint(constraint);
  // 3
  loc_solution = solution;
  // 5
  pQueue->push(0, &this->cg.getVertexbyIdSDC(constraint.VDst->getId()));
  // 6
  while (!pQueue->empty()){
    // 7
    auto x = fetch_from_heap(*pQueue);
    // 8
    if (solution[constraint.VSrc] + constraint.constraint + (solution[x.first] + x.second - solution[constraint.VDst]) < solution[x.first]){
      // 9
      if (x.first->getId() == constraint.VSrc->getId()){
        //cout << "Infeasible System!" << endl;
        // 11
        remove_sdc_constraint(*constraint.VSrc, *constraint.VDst);
        // 12
        solver_status = 21;
        return;
        // 13
      }else{
        // 14
        loc_solution[x.first] = solution[constraint.VSrc] + constraint.constraint + (solution[x.first] + x.second - solution[constraint.VDst]);
        // 15
        for (auto &it : cg.getSuccessors(x.first)){
          // 16
          int scaled_path_length = x.second + (solution[x.first] + constraint.constraint - solution[constraint.VDst]);
          // 17
          if (scaled_path_length < key_of(*pQueue, it)){
            // 18
            adjust_Heap(*pQueue, it, scaled_path_length);
          }
        }
      }
    }
  }
  // 24
  //cout << "Feasible System" << endl;
  // 25
  solution = loc_solution;
  // 26
  solver_status = 20;
}

pair<HatScheT::Vertex *, int> HatScheT::SDCSolver::fetch_from_heap(FibonacciHeap<int> &H) {
  auto v = (Vertex*) H.topNode()->payload;
  auto k = H.topNode()->key;
  H.pop();
  return make_pair(v, k);
}

void HatScheT::SDCSolver::adjust_Heap(HatScheT::FibonacciHeap<int> &H, HatScheT::Vertex *v, int k) {

  auto val_from_heap = H.findNode_by_payload(v);
  if (val_from_heap.first){
    auto v = (Vertex*) val_from_heap.second->payload;
    cout << "val_from_heap: " << v->getName() << ", " << val_from_heap.second->key << endl;
    H.decrease_key(val_from_heap.second, k);
  }else {
    H.push(k, v);
  }
}

int HatScheT::SDCSolver::key_of(HatScheT::FibonacciHeap<int> &H, HatScheT::Vertex *v) {

  auto val_from_heap = H.findNode_by_payload(v);

  if (val_from_heap.first){
    return val_from_heap.second->key;
  }else {
    return INT_MAX;
  }
}

void HatScheT::SDCSolver::set_initial_solution(map<HatScheT::Vertex *, int> &known_solution) {
  solution.clear();
  for (auto &it : known_solution){
    this -> solution[&this -> cg.getVertexbyIdSDC(it.first->getId())] = it.second;
  }
}


