//
// Created by bkessler on 18/10/19.
//

//UNDER DEVELOPEMENT

#ifdef USE_CADICAL
#include "SDSScheduler.h"
#include <cassert>
#include <iomanip>
#include <climits>
#include <cmath>

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

    BellmanFordSDC bfsdc(dependencyConstraintsSDC, &g);
    bfsdc.setSilent(this->silent);
    auto times = bfsdc.getfirstPath();

    cout << endl;
    for (auto &it : times.first) {
      cout << it.first->getName() << ": " << it.second << endl;
    }

    for (int i = 0; i < 50; i++) {
      //Start:
      //Add Resource Constraints
      bfsdc.addConstraints(resourceConstraintsSDC);
      //Call Solver again
      auto sdcSolution = bfsdc.getPathIncremental();
      if (sdcSolution.second) {
        auto conflicts = bfsdc.getConflicts();
        cout << "Conflicts: " << endl;
        for (auto &it : conflicts) {
          cout << "Conflict: ";
          cout << it.first.first->getName() << " -- " << it.second << " -- " << it.first.second->getName() << endl;
        }
        getSATClausesFromSDCConflicts(conflicts);
        resourceConstraintsSDC = passToSATSolver(sharingVariables, conflictClauses);
        if (!this->silent) {
          cout << endl;
          cout << "Resource SDC Constraints: " << endl;
          for (auto &it : resourceConstraintsSDC) {
            cout << "S" << it.first.first->getId() << " - S" << it.first.second->getId() << " <= " << it.second << endl;
          }
          cout << endl;
        }
      } else {
        cout << endl << "Schedule:" << endl;
        for (auto &it : sdcSolution.first) {
          cout << it.first->getName() << "; " << it.first << ": " << it.second << endl;
        }
        break;
      }
    }

    //If feasible return Schedule, else report conflicts to SAT remove Edges, go to Start.

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
      //ToDo Set a Flag to tell the Scheduler to increase the min Latency Constraint.
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
      int distance = (it->getDistance() + resourceModel.getVertexLatency(&it->getVertexSrc())) * -1;

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

  void SDSScheduler::getSATClausesFromSDCConflicts(map<pair<Vertex*, Vertex*>, int> &conflicts) {
    vector <int> conflictSAT;
    for (auto &it : conflicts) {
      conflictSAT.push_back(sdcToSATMapping[it.first]*-1);
    }
    conflictClauses.push_back(conflictSAT);
    for (auto &it :conflictClauses){
      for (auto &itr : it){
        cout << itr << " ";
      }
      cout << endl;
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

  bool SDSScheduler::ConstraintGraph::doesEdgeExistID(Vertex *src, Vertex *dst) {

    for(auto &it : Edges()){
      if (it->getVertexSrc().getId() == src->getId() && it->getVertexDst().getId() == dst->getId()){
        return true;
      }
    }
    return false;
  }

  Edge& SDSScheduler::ConstraintGraph::getEdge(const Vertex *srcV, const Vertex *dstV){
    for(auto e:this->edges){
      if(&e->getVertexSrc()==srcV && &e->getVertexDst()==dstV) return *e;
    }

    throw HatScheT::Exception("Graph::getEdge: Edge not found!");
  }

  ////////////////////////////
  // BellmanFord SDC-Solver //
  ////////////////////////////

  SDSScheduler::BellmanFordSDC::BellmanFordSDC(SDSScheduler::ConstraintGraph cg, Graph* originalGraph) {
    this -> cg = cg;
    this->silent = true;
    this->hasNegativeCycle = false;
    this->origGraph = originalGraph;
  }

  SDSScheduler::BellmanFordSDC::BellmanFordSDC(map<pair<const Vertex *, const Vertex *>, int> &constraints, Graph* originalGraph) {
    this->silent = true;
    this->hasNegativeCycle = false;
    this->origGraph = originalGraph;
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

  void SDSScheduler::BellmanFordSDC::bellmanFordAlgorithm() {
    /*!
     * Initialisation of Algorithm
     */
    for (auto &it : this->cg.Vertices()){
      vertexCosts.insert(make_pair(it, INT_MAX));
    }
    vertexCosts[&this->cg.getVertexById(this->startID)] = 0;

    /*!
     * Finding Shortest Paths
     */
    for (int i = 1; i < this->cg.getNumberOfVertices(); i++){
      for (auto &it : this -> cg.Edges()){
        if ((vertexCosts[&it->getVertexSrc()] < INT_MAX) && (vertexCosts[&it->getVertexSrc()] + it->getDistance() < vertexCosts[&it->getVertexDst()])) {
          vertexCosts[&it->getVertexDst()] = vertexCosts[&it->getVertexSrc()] + it->getDistance();
        }
      }
    }

    /*!
     * Checking for negative Cycles
     */
    for (auto &it : this->cg.Edges()){
      if ((vertexCosts[&it->getVertexSrc()] < INT_MAX) && (vertexCosts[&it->getVertexSrc()] + it->getDistance() < vertexCosts[&it->getVertexDst()])){
        hasNegativeCycle = true;
        break;
      }else {
        hasNegativeCycle = false;
      }
    }
  }

  pair<map<Vertex *, int>, bool> SDSScheduler::BellmanFordSDC::getfirstPath() {

    this->startID = determineStartVertex();
    bellmanFordAlgorithm();

    if (hasNegativeCycle){
      if (!this->silent) {
        cout << "SDC-System unsolvable" << endl;
      }
      map<Vertex*, int> emptyMap;
      return make_pair(emptyMap, hasNegativeCycle);
    }else {
      int minimum = 0;
      for (auto &it : vertexCosts){
        if (it.second < minimum){
          minimum = it.second;
        }
      }
      for (auto &it : vertexCosts){
        it.second += abs(minimum);
      }

      return make_pair(vertexCosts, hasNegativeCycle);
    }
  }

  pair<map<Vertex *, int>, bool> SDSScheduler::BellmanFordSDC::getPathIncremental(){

    for (auto &it : additionalConstraints) {
      //Add Constraint to Graph.
      if (!this->cg.doesEdgeExistID((Vertex *) it.first.first, (Vertex *) it.first.second)) {
        this->cg.createEdgeSDS(this->cg.getVertexById(it.first.first->getId()),
                               this->cg.getVertexById(it.first.second->getId()), it.second);
      }

      cout << endl << "Graph: " << endl;
      for (auto &itr : cg.Edges()){
        cout << itr->getVertexSrc().getName() << " -- " << itr->getDistance() << " -- " << itr->getVertexDst().getName() << endl;
      }
      //Add Constraint to potential Conflicts.
      conflicts.insert(it);
      //Try to solve.
      vertexCosts.clear();
      bellmanFordAlgorithm();

      //If unsolvable report Conflicts. Else return solution;
      if (hasNegativeCycle) {
        if (!this->silent) {
          cout << "SDC-System unsolvable" << endl;
        }
        map<Vertex *, int> emptyMap;
        return make_pair(emptyMap, hasNegativeCycle);
      }
    }
    auto ptr = *minLatEdges.begin();
    for (auto &it : vertexCosts){
      it.second += ptr->getDistance();
    }
    return make_pair(vertexCosts, hasNegativeCycle);
  }

  int SDSScheduler::BellmanFordSDC::determineStartVertex() {

    struct longestpath{
      map<Vertex*, int> costs;
      Vertex* startvertex = nullptr;
      int minimum = 0;
      list <Vertex*> minimumVertices;

      void find_minimum(){
        for (auto &it : this->costs){
          if (it.second < minimum){
            minimum = it.second;
          }
        }
      }

      void find_minimum_vertices(){
        for (auto &it : costs){
          if (it.second == minimum){
            minimumVertices.push_back(it.first);
          }
        }
      }
    };

    list <struct longestpath> longestpaths;
    longestpath startpoint;

    if (this->silent) {
      cout << "Start:" << endl;
    }
    for (auto &it : cg.Vertices()) {
      if (this->silent) {
        cout << endl << it->getName() << ": " << endl;
      }
      this->startID = it->getId();
      bellmanFordAlgorithm();
      struct longestpath lp;
      lp.costs = vertexCosts;
      lp.startvertex = it;
      lp.find_minimum();
      longestpaths.push_back(lp);
      vertexCosts.clear();
    }

    int min = 0;
    for (auto &it : longestpaths){
      if (it.minimum < min){
        min = it.minimum;
      }
    }

    for (auto &it : longestpaths){
      if (it.minimum == min){
        startpoint = it;
      }
    }

    startpoint.find_minimum_vertices();

    for (auto &it : startpoint.minimumVertices) {
      this->minLatEdges.push_back(&cg.createEdge(*it, *startpoint.startvertex, abs(startpoint.minimum)));
    }

    if (!this->silent){
      cout << "Constraint Graph after adding a min Latency Constraint." << endl;
      for (auto &it : cg.Edges()){
        cout << it->getVertexSrc().getName() << " -- " << it->getDistance() << " -- " << it->getVertexDst().getName() << endl;
       }
    }

    return startpoint.startvertex->getId();
  }

  void SDSScheduler::BellmanFordSDC::addConstraints(map<pair<const Vertex *, const Vertex *>, int> &constraints) {
    additionalConstraints.clear();
    for (auto &it : constraints) {
      this->additionalConstraints.insert(
          make_pair(make_pair((Vertex *) it.first.first, (Vertex *) it.first.second), it.second));
    }
  }

  map<pair<Vertex *, Vertex *>, int> SDSScheduler::BellmanFordSDC::getConflicts() {
    auto tempConflicts = conflicts;
    for (auto &it : conflicts){
      cg.removeEdge(it.first.first, it.first.second);
    }
    conflicts.clear();

    for (auto &it : cg.Edges()){
      cout << it->getVertexSrc().getName() << " -- " << it->getVertexDst().getName() << endl;
    }

    return tempConflicts;
  }

}

#endif //USE_CADICAL