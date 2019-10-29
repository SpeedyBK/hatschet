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

    resourceConstraintsSDC = passToSATSolver(sharingVariables, {{}});

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

    //Call SDC without Resource Constraints (always feasible);
    /*!
     * Create Constraint Graph
     * Add Dependency Constraints: Done
     */
    createBasicConstraintGraph();

    /*!
     * Add Timing (Chaining) Constraints: ToDo...
     */

    addToConstraintGraph(make_pair(&constraintGraph.getVertexById(5), &constraintGraph.getVertexById(1)), -1);
    addToConstraintGraph(make_pair(&constraintGraph.getVertexById(5), &constraintGraph.getVertexById(2)), -1);

    rceIt = resourceConstraintsSDC.begin();

    int bums = 0;
    while (rceIt != resourceConstraintsSDC.end()){
      addToConstraintGraph(swapPair(rceIt->first), rceIt->second);
      isUnsolvableSolution = solveSDC();
      displaySDCSolution();
      if (!isUnsolvableSolution.second) {
        rceIt++;
      }else {
        for (auto &it:constraintGraph.Edges()){
          cout << it->getId() << "";
        }
        cout << endl;
        constraintGraph.removeEdge((Vertex*)rceIt->first.second, (Vertex*)rceIt->first.first);
        for (auto &it:constraintGraph.Edges()){
          cout << it->getId() << "";
        }
        cout << endl;
        resourceConstraintsSDC = passToSATSolver(sharingVariables, satfromSDC(rceIt->first));
        rceIt = resourceConstraintsSDC.begin();
      }
      if (bums > 5){
        break;
      }
      bums++;
    }

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
                                            vector<vector<int>> confClauses) {

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
        cout << Itr;
      }
      if (!It.empty()) {
        solver->add(0);
      }
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

  void SDSScheduler::addToConstraintGraph(pair<const Vertex *, const Vertex *> constraintsSDCVer, int weight) {

      /*!
       * Create Verticies:
       */
      if(!doesVertexExist(&this->constraintGraph, constraintsSDCVer.first->getId())){
        this->constraintGraph.createVertex(constraintsSDCVer.first->getId());
      }
      if(!doesVertexExist(&this->constraintGraph, constraintsSDCVer.second->getId())){
        this->constraintGraph.createVertex(constraintsSDCVer.second->getId());
      }
      /*!
       * Create Edges:
       */
      if(!this->constraintGraph.edgeExists(constraintsSDCVer.first, constraintsSDCVer.second)){
        this->constraintGraph.createEdgeSDS(constraintGraph.getVertexById(constraintsSDCVer.first->getId()), constraintGraph.getVertexById(constraintsSDCVer.second->getId()), weight);
      }
  }

  bool SDSScheduler::doesVertexExist(Graph *gr, int vertexID) {
    for (auto &it : gr->Vertices()){
      if(it->getId() == vertexID){
        return true;
      }
    }
    return false;
  }

  void SDSScheduler::createBasicConstraintGraph() {
    for (auto &it:dependencyConstraintsSDC) {
      addToConstraintGraph(it.first, it.second);
    }
  }

  pair<map<Vertex*, int>, bool> SDSScheduler::solveSDC() {

    /*!
     * Detect Source- and Sinkvertices
     */
    list <Vertex*> sinks;
    list <Vertex*> sources;
    for (auto &it:this->g.Vertices()){
      if (this->g.isSinkVertex(it)){
        sinks.push_back(it);
      }
      if (this->g.isSourceVertex(it)){
        sources.push_back(it);
      }
    }

    /*!
     * Creating an Edge to limit the Shedule to x Clock Cycles.
     */
    if (firstTime) {
      addToConstraintGraph(make_pair(sources.front(), sinks.front()), maxLatencyConstraint);
      firstTime = false;
    }
    displayGraph();

    /*!
     * Bellman-Ford Algorithm to find shortest Paths in the Constraint-Graph
     * If the Algorithm detects a negative cycle the Algorithm stopps and reports a conflict clause to the SAT-Solver
     */
    BellmanFord bell = BellmanFord(&this->constraintGraph);
    return bell.getPath(0);
  }

  void SDSScheduler::displayGraph() {
    /*!
     * Display Graph:
     */
    if (!this->silent) {
      cout << "Vertices of Constraint Graph:" << endl;
      for (auto &it:this->constraintGraph.Vertices()){
        cout << it->getName() << endl;
      }
      cout << endl;
      cout << "Edges of Constraint Graph:" << endl;
      for (auto &it:this->constraintGraph.Edges()){
        cout << it->getVertexSrc().getName() << "--(" << it->getDistance() << ")--" << it->getVertexDst().getName() << endl;
      }
    }
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
    vector<vector<int>> conflictClauses;

    for (auto &it : this->sdcToSATMapping){
      if (it.first.first->getId() == sdctosat.first->getId() && it.first.second->getId() == sdctosat.second->getId()){
        conflict.push_back(sdcToSATMapping[sdctosat]*(-1));
      }
    }

    conflictClauses.push_back(conflict);

    return conflictClauses;
  }

  ////////////////////////////
  // Bellman-Ford Algorithm //
  ////////////////////////////

  SDSScheduler::BellmanFord::BellmanFord(Graph *golfRomeo) {
    this->g = golfRomeo;
    hasNegativeCycle = false;
    bellmanFordAlgorithm(0);
  }

  void SDSScheduler::BellmanFord::bellmanFordAlgorithm(int startID) {
    /*!
     * Initialisation of Algorithm
     */
    for (auto &it : this->g->Vertices()){
      vertexCosts.insert(make_pair(it, INT_MAX));
    }
    vertexCosts[&this->g->getVertexById(startID)] = 0;

    /*!
     * Finding Shortest Paths
     */
    for (int i = 1; i < this->g->getNumberOfVertices(); i++){
      for (auto &it : this -> g->Edges()){
        if ((vertexCosts[&it->getVertexSrc()] < INT_MAX) && (vertexCosts[&it->getVertexSrc()] + it->getDistance() < vertexCosts[&it->getVertexDst()]))
          vertexCosts[&it->getVertexDst()] = vertexCosts[&it->getVertexSrc()] + it->getDistance();
      }
    }
    /*!
     * Checking for negative Cycles
     */
    for (auto &it : this->g->Edges()){
      if ((vertexCosts[&it->getVertexSrc()] < INT_MAX) && (vertexCosts[&it->getVertexSrc()] + it->getDistance() < vertexCosts[&it->getVertexDst()])){
        hasNegativeCycle = true;
        break;
      }else {
        hasNegativeCycle = false;
      }
    }
  }

  pair<map<Vertex *, int>, bool> SDSScheduler::BellmanFord::getPath(int idofStartVertex) {

    bellmanFordAlgorithm(idofStartVertex);

    if (hasNegativeCycle){
      cout << "SDC-System unsolvable" << endl;
      map<Vertex*, int> emptyMap;
      return make_pair(emptyMap, hasNegativeCycle);
    }else {
      return make_pair(vertexCosts, hasNegativeCycle);
    }
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
}



#endif //USE_CADICAL