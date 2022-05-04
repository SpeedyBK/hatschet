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
  cerr << "SDCSolver::ConstraintGraph: Trying to remove a non-existing Edge!" << endl; //ToDo: Replace with "throw"...
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
  SDCConstraint C = { .VSrc = src, .VDst= dst, .constraint = c};
  return C;
}

void HatScheT::SDCSolver::add_to_feasible(SDCConstraint constraint) {
  /*!
   * Local Variables
   */
  FibonacciHeap<int> pQueue;
  map <Vertex*, int> loc_solution;

  for (auto &it : cg.Vertices()){
    vertex_to_FibNode_Map[it] = nullptr;
  }

  int j = 0;

  /*!
   * Begin of the Algorithm
   */
  // 1
  add_sdc_constraint(constraint);
  print_Constraint_Graph();
  // 3
  loc_solution = solution;
  // 5
  auto vptr = &this->cg.getVertexbyIdSDC(constraint.VDst->getId());
  vertex_to_FibNode_Map[vptr] = pQueue.push(0, vptr);
  cout << vptr->getName() << ": " << vertex_to_FibNode_Map[vptr]->key << endl;
  // 6
  while (!pQueue.empty()){
    // 7
    auto x = fetch_from_heap(pQueue);
    vertex_to_FibNode_Map[x.first] = nullptr;
    // 8
    if (solution[constraint.VSrc] + constraint.constraint + (solution[x.first] + x.second - solution[constraint.VDst]) < solution[x.first]){
      // 9
      //cout << x.first->getName() << " : " << constraint.VSrc->getName() << endl;
      if (x.first->getId() == constraint.VSrc->getId()){
        // cout << "Infeasible System!" << endl;
        // 11
        remove_sdc_constraint(*constraint.VSrc, *constraint.VDst);
        // 12
        solver_status = 21;
        vertex_to_FibNode_Map.clear();
        return;
        // 13
      }else{
        // 14
        loc_solution[x.first] = solution[constraint.VSrc] + constraint.constraint + (solution[x.first] + x.second - solution[constraint.VDst]);
        // 15
        for (auto &it : cg.getSuccessors(x.first)){
          // 16
          int scaled_path_length = x.second + (solution[x.first] + cg.getEdges(x.first, it).front()->getDistance() - solution[it]);
          // 17
          if (scaled_path_length < key_of(pQueue, it)){
            // 18
            adjust_Heap(pQueue, it, scaled_path_length);
            cout << it->getName() << ": " << scaled_path_length << endl;
            j++;
            if (j > 10){
              //throw HatScheT::Exception("Intentional Error!");
            }
          }
        }
      }
    }
  }
  // 24
  // << "Feasible System" << endl;
  // 25
  solution = loc_solution;
  // 26
  solver_status = 20;
  vertex_to_FibNode_Map.clear();
}

pair<HatScheT::Vertex *, int> HatScheT::SDCSolver::fetch_from_heap(FibonacciHeap<int> &H) {
  auto v = (Vertex*) H.topNode()->payload;
  auto k = H.topNode()->key;
  H.pop();
  return make_pair(v, k);
}

void HatScheT::SDCSolver::adjust_Heap(HatScheT::FibonacciHeap<int> &H, HatScheT::Vertex *v, int k) {

  //If v is not in the heap, it inserts v with key k into the heap. If v is in the Heap, it sets key of v to k.

  if (vertex_to_FibNode_Map[v] == nullptr){
    vertex_to_FibNode_Map[v] = H.push(k, v);
  }else {
    //cout << "Else-Case: " << endl;
    //cout << "Old Key: " << key_of(H, v) << " New Key: " << k << endl;
    //cout << ((Vertex*)vertex_to_FibNode_Map[v]->payload)->getName() << endl;
    //cout << "Size of Heap: " << H.size() << endl;
    /*while (!H.empty()){
      cout << "From Heap: " << ((Vertex*)H.topNode()->payload)->getName() << " - " << H.topNode()->degree << " - " << H.topNode()->key << endl;
      if (H.topNode()->child != nullptr){
        cout << ((Vertex*)H.topNode()->child->payload)->getName() << endl;
      }
      H.pop();
    }*/
    //throw HatScheT::Exception ("Intentional Error!");
    H.decrease_key(vertex_to_FibNode_Map[v], k);
  }
}

int HatScheT::SDCSolver::key_of(HatScheT::FibonacciHeap<int> &H, HatScheT::Vertex *v) {
  if (vertex_to_FibNode_Map[v] == nullptr) {
    return INT_MAX;
  }else
    return vertex_to_FibNode_Map[v]->key;
}

void HatScheT::SDCSolver::set_initial_solution(map<HatScheT::Vertex *, int> &known_solution) {
  solution.clear();
  for (auto &it : known_solution){
    this -> solution[&this -> cg.getVertexbyIdSDC(it.first->getId())] = it.second;
  }
}

void HatScheT::SDCSolver::add_Constraint(SDCConstraint constr) {
  // 1
  if (this->unProcessed.empty()){
    // 2
    add_to_feasible(constr);
    if (this->solver_status == 21){
      // 3
      this->unProcessed.insert(constr);
      this->solver_status = 31;
    }
    else {
      this->solver_status = 30;
    }
  }//5
  else{
    this->unProcessed.insert(constr);
  }
}

/*!
  * Declare:
  * - C : Constraint to be added to the system
  * - unProcessed : a set of constraints
  * 1.  if (C is in unProcessed) then
  * 2.    Remove C from unProcessed
  * 3.  else
  * 4.    Remove C from the constraint Graph
  * 5.    while (unProcessed != empty) do
  * 6.        select a constraint C1 from unProcessed
  * 7.        if (add_to_feasible(C1) then
  * 8.            remove C1 from unProcessed
  * 9.        else
  * 10.           exit loop
  * 11.       end if
  * 12.   end while
  * 13. end if
  */
void HatScheT::SDCSolver::delete_Constraint(SDCConstraint constr) {

  for (auto &it : this->unProcessed){
    cout << it.VDst->getName() << " - " << it.VSrc->getName() << " <= " << it.constraint << endl;
    cout << constr.VDst->getName() << " - " << constr.VSrc->getName() << " <= " << constr.constraint << endl;
  }

  if (this->unProcessed.count(constr) != 0){
    cout << "Bums" << endl;
    this->unProcessed.erase(constr);
    if (!this->unProcessed.empty()){
      this->solver_status = 31;
    }else {
      this->solver_status = 30;
    }
  }
  else{
    this->remove_sdc_constraint(*constr.VDst, *constr.VSrc);
    while (!this->unProcessed.empty()){
      auto it = this->unProcessed.begin();
      add_to_feasible(*it);
      if (this->solver_status == 20){
        this->unProcessed.erase(it);
      }
      else{
        this->solver_status = 31;
        return;
      }
    }
    this->solver_status = 30;
  }
}


