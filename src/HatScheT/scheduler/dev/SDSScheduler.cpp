//
// Created by bkessler on 18/10/19.
//

//UNDER DEVELOPEMENT

#ifdef USE_CADICAL
#include "SDSScheduler.h"
#include <cassert>
#include <iomanip>
#include <climits>

namespace HatScheT {

  HatScheT::SDSScheduler::SDSScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel) : SchedulerBase(g,
                                                                                                                   resourceModel) {
    this->silent = true;
    this->firstTime = true;
  }

  void HatScheT::SDSScheduler::schedule() {

    if (!this->silent) {
      cout << "--------------------------------------------------------" << endl;
      cout << "Creating Binding Variables..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    createBindingVariables();

    //Debug
    if (!this->silent) {
      cout << "Index : Resource ID / Vertex / Resource Instance / Boolean Binding Variable" << endl;
    }
    for (auto &it : bindingVariables) {
      if (!this->silent) {
        if (it.binding) {
          cout << it.index << ": " << it.resourceID << " / " << it.vertex->getName() << " / " << it.resourceInstance
               << " / True" << endl;
        } else {
          cout << it.index << ": " << it.resourceID << " / " << it.vertex->getName() << " / " << it.resourceInstance
               << " / False" << endl;
        }
      }
    }

    if (!this->silent) {
      cout << "--------------------------------------------------------" << endl;
      cout << "Setting Binding Variables..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    setBindingVariables();

    //Debug
    if (!this->silent) {
      cout << "Index : Resource ID / Vertex / Resource Instance / Boolean Binding Variable" << endl;
    }
    for (auto &it : bindingVariables) {
      if (!this->silent) {
        if (it.binding) {
          cout << it.index << ": " << it.resourceID << " / " << it.vertex->getName() << " / " << it.resourceInstance
               << " / True" << endl;
        } else {
          cout << it.index << ": " << it.resourceID << " / " << it.vertex->getName() << " / " << it.resourceInstance
               << " / False" << endl;
        }
      }
    }

    if (!this->silent) {
      cout << "--------------------------------------------------------" << endl;
      cout << "Creating and Setting Sharing Variables..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    if (bindingType == 'S') {
      sharingVariables = createShVarsMaxSpeed();
    } else if (bindingType == 'R') {
      sharingVariables = createShVarsMinRes();
    }

    if (!this->silent) {
      for (auto &it:sharingVariables) {
        if (it.second) {
          cout << it.first.first->getName() << ", " << it.first.second->getName() << ": true" << endl;
        } else {
          cout << it.first.first->getName() << ", " << it.first.second->getName() << ": false" << endl;
        }
      }
    }


    if (!this->silent) {
      cout << "--------------------------------------------------------" << endl;
      cout << "Passing Sharing Variable to SAT..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    vector<vector<int>> emptyVector;
    resourceConstraintsSDC = passToSATSolver(sharingVariables, emptyVector);

    if (!this->silent) {
      cout << endl;
      cout << "Resource SDC Constraints: " << endl;
      for (auto &it : resourceConstraintsSDC) {
        cout << "S" << it.first.first->getId() << " - S" << it.first.second->getId() << " <= " << it.second << endl;
      }
      cout << endl;
    }

    if (!this->silent) {
      cout << "--------------------------------------------------------" << endl;
      cout << "Getting Dependency Constraints..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    dependencyConstraintsSDC = createDependencyConstraints();

    if (!this->silent) {
      cout << "Data Dependency SDC Constraints: " << endl;
      for (auto &it : dependencyConstraintsSDC) {
        cout << "S" << it.first.first->getId() << " - S" << it.first.second->getId() << " <= " << it.second << endl;
      }
      cout << endl;
    }

    if (!this->silent) {
      cout << "--------------------------------------------------------" << endl;
      cout << "Creating Constraint Graph..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    /*!
     * Call SDC without Resource Constraints (always feasible);
     */
    SDCSolver sdc(dependencyConstraintsSDC);
    sdc.printConstraintGraph();
    cout << endl;
    sdc.dijkstra(&g.getVertexById(0));

    /*!
     * Add Timing (Chaining) Constraints: ToDo...
     */

    //(5) - (1) <= -1
    //(5) - (2) <= -1

    sdc.addConstrainttoGraph(make_pair((const Vertex*)&g.getVertexById(5), (const Vertex*)&g.getVertexById(1)), -1);
    sdc.addConstrainttoGraph(make_pair((const Vertex*)&g.getVertexById(5), (const Vertex*)&g.getVertexById(2)), -1);
    sdc.printConstraintGraph();
    /*!
     * Add Resource Constraints
     */
    sdc.addConstraints(resourceConstraintsSDC);
    sdc.printConstraintGraph();


  }

  void HatScheT::SDSScheduler::createBindingVariables() {

    list<Resource *> localResources;
    for (auto it = resourceModel.resourcesBegin(); it != resourceModel.resourcesEnd(); ++it) {
      if (!(*it)->isUnlimited()) {
        localResources.push_back(*it);
      }
    }

    this->numOfLimitedResources = localResources.size();

    bindingVariable bv;
    int idCount = 0;
    int index = 0;

    for (auto &it:localResources) {
      set<const Vertex *> verticiesOfResource = resourceModel.getVerticesOfResource(it);
      for (auto &vORIt:verticiesOfResource) {
        for (int i = 0; i < it->getLimit(); i++) {
          bv.index = index;
          bv.resource = it;
          bv.resourceID = idCount;
          bv.vertex = vORIt;
          bv.resourceInstance = i;
          bv.binding = false;
          bindingVariables.push_back(bv);
          index++;
        }
      }
      idCount++;
    }
  }

  void SDSScheduler::setBindingVariables() {

    int lastVertexId = -1;
    int count = 0;

    for (int i = 0; i < numOfLimitedResources; i++) {
      for (auto &it : bindingVariables) {
        if (it.resourceID == i) {
          if (it.vertex->getId() != lastVertexId) {
            count++;
            lastVertexId = it.vertex->getId();
          }
          if ((count % it.resource->getLimit()) == it.resourceInstance) {
            it.binding = true;
          }
        }
      }
      count = 0;
    }
  }

  map<pair<const Vertex *, const Vertex *>, bool> SDSScheduler::createShVarsMaxSpeed() {

    map<pair<const Vertex *, const Vertex *>, bool> shared;
    list<bindingVariable *> tempBinVars;
    list<list<bindingVariable *>> templists;

    for (int i = 0; i < numOfLimitedResources; i++) {
      for (auto &it : bindingVariables) {
        if (it.resourceID == i) {
          tempBinVars.push_back(&it);
        }
      }
      templists.push_back(tempBinVars);
      tempBinVars.clear();
    }

    for (auto &lit : templists) {
      int loops = 1;
      for (auto &ibvIt : lit) {
        if (ibvIt->binding) {
          auto jbvIt = lit.begin();
          for (advance(jbvIt, loops); jbvIt != lit.end(); ++jbvIt) {
            if (ibvIt->vertex->getId() != (*jbvIt)->vertex->getId() &&
                (ibvIt->resourceInstance == (*jbvIt)->resourceInstance)) {
              if (ibvIt->binding && (*jbvIt)->binding) {
                auto vpair = make_pair(ibvIt->vertex, (*jbvIt)->vertex);
                shared.insert(make_pair(vpair, true));
              } else {
                auto vpair = make_pair(ibvIt->vertex, (*jbvIt)->vertex);
                shared.insert(make_pair(vpair, false));
              }
            }
          }
        }
        loops++;
      }
    }
    return shared;
  }

  map<pair<const Vertex *, const Vertex *>, bool> SDSScheduler::createShVarsMinRes() {

    map<pair<const Vertex *, const Vertex *>, bool> shared;
    list<bindingVariable *> tempBinVars;
    list<list<bindingVariable *>> templists;

    for (int i = 0; i < numOfLimitedResources; i++) {
      for (auto &it : bindingVariables) {
        if (it.resourceID == i) {
          tempBinVars.push_back(&it);
        }
      }
      templists.push_back(tempBinVars);
      tempBinVars.clear();
    }

    for (auto &lit : templists) {
      int loops = 1;
      for (auto &ibvIt : lit) {
        if (ibvIt->binding) {
          auto jbvIt = lit.begin();
          for (advance(jbvIt, loops); jbvIt != lit.end(); ++jbvIt) {
            if (ibvIt->vertex->getId() != (*jbvIt)->vertex->getId() &&
                (ibvIt->resourceInstance == (*jbvIt)->resourceInstance)) {
              auto vpair = make_pair(ibvIt->vertex, (*jbvIt)->vertex);
              shared.insert(make_pair(vpair, true));
            }
          }
        }
        loops++;
      }
    }
    return shared;
  }


  map<pair<const Vertex*, const Vertex*>, int> SDSScheduler::passToSATSolver(map<pair<const Vertex *, const Vertex *>, bool> &shareVars,
                                            vector<vector<int>> &confClauses) {

    /*!
     * Instance of SAT-Solver
     */
    CaDiCaL::Solver *solver = new CaDiCaL::Solver;

    /*!
     * Converting Sharing Variables to SAT-Literals and pass to SAT-Solver.
     */
    int litCounter = 1;
    if (!this->silent) {
      cout << "Problem: " << "p cnf " << shareVars.size() * 2 << " " << shareVars.size() * 2 + confClauses.size()
           << endl << endl;
    }
    for (auto &It : shareVars) {
      if (It.second) {
        solver->add(litCounter);
        solver->add(litCounter + 1);
        solver->add(0);
        if (!this->silent) { cout << setw(3) << litCounter << setw(3) << litCounter + 1 << setw(3) << " 0" << endl; }
        solver->add(litCounter * -1);
        solver->add((litCounter + 1) * -1);
        solver->add(0);
        if (!this->silent) {
          cout << setw(3) << litCounter * -1 << setw(3) << (litCounter + 1) * -1 << setw(3) << " 0" << endl;
        }
        litCounter += 2;
      }
    }

    /*!
     * Adding Conflict Clauses
     */
    for (auto &It : confClauses) {
      for (auto &Itr : It) {
        solver->add(Itr);
        cout << setw(3) << Itr;
      }
      if (!It.empty()) {
        solver->add(0);
        cout << setw(3) << "0";
      }
      cout << endl;
    }

    /*!
     * Solve Problem
     */
    int res = solver->solve();

    /*!
     * Check if satisfiable:
     */
    if (res == 10) {
      cout << "CaDiCaL: Problem Satisfiable" << endl;
    } else if (res == 0) {
      cout << "CaDiCaL: Problem Unsolved" << endl;
    } else if (res == 20) {
      cout << "CaDiCaL: Problem Unsatisfiable" << endl;
    }

    /*!
     * Getting Solution
     * Solver variable starts at 1, since negated variables are shown as negative numbers.
     * The Solution from SAT is than mapped to an SDC-Format like (srcVertex - dstVertex <= -1)
     */
    map<pair<const Vertex*, const Vertex*>, int> solutionMap;
    auto mIt = sharingVariables.begin();
    if (res == 10) {
      for (int i = 0; i < litCounter - 1; i++) {
        cout << solver->val(i+1) << " ";
        if (i % 2 == 0){
          if(solver->val(i+1) > 0) {
            solutionMap.insert(make_pair(mIt->first, -1));
            sdcToSATMapping.insert(make_pair(mIt->first, i+1));
          }
        }else {
          if(solver->val(i+1) > 0) {
            solutionMap.insert(make_pair(swapPair(mIt->first), -1));
            sdcToSATMapping.insert(make_pair(swapPair(mIt->first), i+1));
          }
          ++mIt;
        }
      }
      cout << endl;
    }

    /*!
     * Delete the solver.
     */
    delete solver;

    return solutionMap;

  }

  pair<const Vertex *, const Vertex *> SDSScheduler::swapPair(pair<const Vertex *, const Vertex *> inPair) {
    return make_pair(inPair.second, inPair.first);
  }

  map<pair<const Vertex *, const Vertex *>, int> SDSScheduler::createDependencyConstraints() {

    map < pair < const Vertex*, const Vertex* >, int > dependencyConstraints;

    for (auto &it : this-> g.Edges()){
      auto sVertex = &it->getVertexSrc();
      auto dVertex = &it->getVertexDst();

      auto constVertexPair = make_pair((const Vertex*) dVertex, (const Vertex*) sVertex);
      int distance = it->getDistance();

      dependencyConstraints.insert(make_pair(constVertexPair, distance));

    }

    return dependencyConstraints;
  }

  bool SDSScheduler::doesVertexExist(Graph *gr, int vertexID) {
    for (auto &it : gr->Vertices()){
      if(it->getId() == vertexID){
        return true;
      }
    }
    return false;
  }

  void SDSScheduler::displaySDCSolution() {
    if (!this->silent){
      cout << endl;
      if (!this->isUnsolvableSolution.second){
        cout << "SDC-Solution found :) Returning Schedule:" << endl;
        cout << "Vertex:  / Starttime:" << endl;
        for (auto &it : this->isUnsolvableSolution.first){
          cout << it.first->getName() << ": " << it.second << endl;
        }
      }else {
        cout << "SDC not solvable :( Reporting Konflikt-Klaus to SAT" << endl;
      }
    }
  }

  vector<vector<int>> SDSScheduler::satfromSDC(pair<const Vertex*, const Vertex*> sdctosat) {

    vector<int> conflict;

    for (auto &it : this->sdcToSATMapping){
      if (it.first.first->getId() == sdctosat.first->getId() && it.first.second->getId() == sdctosat.second->getId()){
        conflict.push_back(sdcToSATMapping[sdctosat]*(-1));
      }
    }

    conflictClauses.push_back(conflict);

    return conflictClauses;
  }


  ///////////////////////////////////////////////
  // Additional functionallity for Graph Class //
  ///////////////////////////////////////////////

  void SDSScheduler::ConstraintGraph::removeEdge(Vertex *srcVertex, Vertex *dstVertex) {

    for (auto &it : edges){
      cout << "VSRC: " << it->getVertexSrc().getId() << " VDST: " << it->getVertexDst().getId() << " || " << srcVertex->getId() << " " << dstVertex->getId() << endl;
      if (it->getVertexSrc().getId() == srcVertex->getId() && it->getVertexDst().getId() == dstVertex->getId()){
        edges.erase(it);
      }
    }
  }

  Edge &SDSScheduler::ConstraintGraph::createEdgeSDS(Vertex &Vsrc, Vertex &Vdst, int distance,
                                                  Edge::DependencyType dependencyType) {

    bool idExists = false;
    for (int i = 0; i <= edges.size(); i++) {
      for (auto &it : edges) {
        if (i == it->getId()) {
          idExists = true;
        }
      }
      if (!idExists) {
        Edge *e = new Edge(Vsrc, Vdst, distance, dependencyType, i);
        edges.insert(e);
        return *e;
      }
      idExists = false;
    }
  }

  bool SDSScheduler::ConstraintGraph::doesEdgeExistID(Vertex *src, Vertex *dst) {

    for(auto &it : Edges()){
      if (it->getVertexSrc().getId() == src->getId() && it->getVertexDst().getId() == dst->getId()){
        return true;
      }
    }
    return false;
  }



  ////////////////////////
  // SDC - Solver Class //
  ////////////////////////
  SDSScheduler::SDCSolver::SDCSolver(SDSScheduler::ConstraintGraph cg) {
    this -> cg = cg;
  }

  SDSScheduler::SDCSolver::SDCSolver(map<pair<const Vertex *, const Vertex *>, int> &constraints) {
    /*!
     * Create Verticies:
     */
    for (auto &it: constraints) {
      if (!doesVertexExist(&this->cg, it.first.first->getId())) {
        this->cg.createVertex(it.first.first->getId());
      }
      if (!doesVertexExist(&this->cg, it.first.second->getId())) {
        this->cg.createVertex(it.first.second->getId());
      }
    /*!
     * Create Edges:
     */
      if (!this->cg.doesEdgeExistID((Vertex *) it.first.first, (Vertex *) it.first.second)) {
        this->cg.createEdgeSDS(this->cg.getVertexById(it.first.first->getId()),
                               this->cg.getVertexById(it.first.second->getId()), it.second);
      }
    }
  }

  /*!
   * Adding a single Constraint to the Constraint Graph
   */
  void SDSScheduler::SDCSolver::addConstrainttoGraph(pair<const Vertex *,const Vertex *> constraint, int weight) {

    /*!
     * Checking Constraint
     */
    if(!doesVertexExist(&this->cg, constraint.first->getId()) || !doesVertexExist(&this->cg, constraint.second->getId())){
      throw HatScheT::Exception ("Constraint does not fit in the current constraint Graph!");
    }
    /*!
     * Create Edges:
     */
    if(!this->cg.doesEdgeExistID((Vertex*)constraint.first, (Vertex*)constraint.second)){
      this->cg.createEdgeSDS(this->cg.getVertexById(constraint.first->getId()), this->cg.getVertexById(constraint.second->getId()), weight);
    }
  }

  /*!
   *
   */
  void SDSScheduler::SDCSolver::addConstraints(map<pair<const Vertex *, const Vertex *>, int> &constraints) {
    for (auto &it : constraints) {
      this->additionalConstraints.insert(it);
    }
  }

  void SDSScheduler::SDCSolver::dijkstra(Vertex *startVertex) {

    /*!
     * Initializing the Algorithm by setting the cost of all Vertices to infinity except the Start-Vertex
     * and set all Vertices to uncheckt.
     */
    dijkstraInit(startVertex);

    /*!
     * Dijkstras Algorithm
     */
    while (!this->queue.empty()) {
      int minCost = INT_MAX;
      Vertex* minCostVertex = nullptr;
      /*!
       * Finding Vertex with the lowest costs.
       */
      for (auto &it : this->queue) {
        if (this->costofVertex[it] < minCost) {
          minCost = this->costofVertex[it];
          minCostVertex = it;
        }
      }
      /*!
       * Remove Vertex with lowest cost from queue, because Shortest Path is computed.
       */
      this->queue.remove(minCostVertex);
      set<Vertex*> neighbors = this->cg.getSuccessors(minCostVertex);
      for (auto &it : neighbors){
        for (auto &qIt : this->queue){
          if (qIt == it){
            updateDistance();
          }
        }
      }

    }

  }

  /*!
   * Initializing the Dijkstra Algorithm
   * @param startVertex
   */
  void SDSScheduler::SDCSolver::dijkstraInit(Vertex *startVertex) {
    for (auto &it : cg.Vertices()){
      this->costofVertex[it] = INT_MAX;
      this->predecessorInShortestPath[it] = nullptr;
    }
    this->costofVertex[startVertex] = 0;

    /*!
     * Setting up the priority queue
     */
    for (auto &it : cg.Vertices()){
      this->queue.push_back(it);
    }
  }


  void SDSScheduler::SDCSolver::printConstraintGraph() {
    cout << endl;
    for (auto &it : cg.Edges()){
      cout << it->getVertexSrc().getName() << " -- " << it->getDistance() << " -- " << it->getVertexDst().getName() << endl;
    }
  }
}

#endif //USE_CADICAL