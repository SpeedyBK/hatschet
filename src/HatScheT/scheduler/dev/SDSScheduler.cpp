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

    //Start:
    //Add Resource Constraints

    //Call Solver again

    //If feasible return Schedule, else report conflicts to SAT remove Edges, go to Start.



    cout << endl;
    for (auto &it : times.first){
      cout << it.first->getName() << ": " << it.second << endl;
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
        if ((vertexCosts[&it->getVertexSrc()] < INT_MAX) && (vertexCosts[&it->getVertexSrc()] + it->getDistance() < vertexCosts[&it->getVertexDst()]))
          vertexCosts[&it->getVertexDst()] = vertexCosts[&it->getVertexSrc()] + it->getDistance();
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
    struct longestpath startpoint;

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


  ////////////////////////
  // SDC - Solver Class //
  ////////////////////////
  SDSScheduler::SDCSolver::SDCSolver(SDSScheduler::ConstraintGraph cg) {
    this -> cg = cg;
    this->silent = true;
  }

  SDSScheduler::SDCSolver::SDCSolver(map<pair<const Vertex *, const Vertex *>, int> &constraints) {
    this->silent = true;
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
     * Initializing the Algorithm by setting the cost of all Vertices to infinity except the Start-Vertex.
     */
    dijkstraInit(startVertex);

    /*!
     * Dijkstras Algorithm
     */
    int i = 0;
    while (!this->queue.empty()) {
      i++;
      if (i > 100){
        break;
      }
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
            updateCost(minCostVertex, it);
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
    this->costofVertex[&cg.getVertexById(startVertex->getId())] = 0;

    /*!
     * Setting up the priority queue
     */
    for (auto &it : cg.Vertices()){
      this->queue.push_back(it);
    }
  }

  void SDSScheduler::SDCSolver::updateCost(Vertex* u, Vertex* v) {
    int alternative = costofVertex[u] + cg.getEdge(u, v).getDistance();
    if (alternative < costofVertex[v]){
      costofVertex[v] = alternative;
      predecessorInShortestPath[v] = u;
    }
  }


  void SDSScheduler::SDCSolver::printConstraintGraph() {
    cout << endl;
    for (auto &it : cg.Edges()){
      cout << it->getVertexSrc().getName() << " -- " << it->getDistance() << " -- " << it->getVertexDst().getName() << endl;
    }
  }

  bool SDSScheduler::SDCSolver::addToFeasible(pair<pair<const Vertex *, const Vertex *>, int> additionalConstraint) {

    if (!silent){
      cout << "Additional Constraint: " << additionalConstraint.first.first->getName() << " -- " << additionalConstraint.second << " -- " << additionalConstraint.first.second->getName() << endl;
    }

    /*!
     * Adding new Constraint to the Constraint-Graph
     */
    this->cg.createEdgeSDS (this->cg.getVertexById(additionalConstraint.first.first->getId()),
                            this->cg.getVertexById(additionalConstraint.first.second->getId()),
                            additionalConstraint.second);

    /*!
     * Saving old costs and init. of the Fibonacci Heap
     */
    auto oldCostofVertex = costofVertex;
    FibonacciHeap priorityQueue;

    priorityQueue.insert((Vertex*)additionalConstraint.first.first, 0);

    if (!this->silent) {
      priorityQueue.display();
    }

    /*!
     * Dijkstras Algorithm with EdgeTransformation.
     */
    while (!priorityQueue.isempty()){
      auto minpair = priorityQueue.extract_min();
      if (costofVertex[(Vertex*)additionalConstraint.first.second]+additionalConstraint.second + (costofVertex[minpair.first]+additionalConstraint.second - costofVertex[(Vertex*)additionalConstraint.first.first]) < costofVertex[minpair.first]){
        if (minpair.first->getId() == additionalConstraint.first.second->getId()){
          /*!
           * Infeasible System: Reject Constraint, Return false
           */
           auto vSrc = &this->cg.getVertexById(additionalConstraint.first.first->getId());
           auto vDst = &this->cg.getVertexById(additionalConstraint.first.first->getId());
           cg.removeEdge(vSrc, vDst);
           return false;
        } else{
          oldCostofVertex[minpair.first] = costofVertex[(Vertex*)additionalConstraint.first.second] + additionalConstraint.second + (costofVertex[minpair.first]+minpair.second - costofVertex[(Vertex*)additionalConstraint.first.first]);
        }
      }

    }

    /*!
     * Feasible System
     */
    costofVertex = oldCostofVertex;
    return true;
  }

  ///////////////////////////////
  // Fibonacci Heap functions  //
  ///////////////////////////////

  SDSScheduler::FibonacciHeap::FibonacciHeap() {
    this -> mini = nullptr;
    this -> numberofNodes = 0;
  }

  void SDSScheduler::FibonacciHeap::insert(Vertex *V, int key) {

    auto new_node = (struct node*)malloc(sizeof(struct node));

    new_node->key = key;
    new_node->parent = nullptr;
    new_node->child = nullptr;
    new_node->left = new_node;
    new_node->right = new_node;
    new_node->mark = false;
    new_node->flag = false;
    new_node->storedVertex = V;

    if (mini != nullptr) {
      (mini->left)->right = new_node;
      new_node->right = mini;
      new_node->left = mini->left;
      mini->left = new_node;
      if (new_node->key < mini->key)
        mini = new_node;
    }
    else {
      mini = new_node;
    }
    this -> numberofNodes++;
  }

  void SDSScheduler::FibonacciHeap::fibonacci_link(struct node* ptr2, struct node* ptr1){
    (ptr2->left)->right = ptr2->right;
    (ptr2->right)->left = ptr2->left;
    if (ptr1->right == ptr1) {
      mini = ptr1;
    }
    ptr2->left = ptr2;
    ptr2->right = ptr2;
    ptr2->parent = ptr1;
    if (ptr1->child == nullptr) {
      ptr1->child = ptr2;
    }
    ptr2->right = ptr1->child;
    ptr2->left = (ptr1->child)->left;
    ((ptr1->child)->left)->right = ptr2;
    (ptr1->child)->left = ptr2;
    if (ptr2->key < (ptr1->child)->key) {
      ptr1->child = ptr2;
    }
    ptr1->degree++;
  }

  void SDSScheduler::FibonacciHeap::consolidate(){
    int temp1;
    float temp2 = (log(numberofNodes)) / (log(2));
    int temp3 = (int) temp2;
    struct node* arr[temp3+1];
    for (int i = 0; i <= temp3; i++)
      arr[i] = nullptr;
    node* ptr1 = mini;
    node* ptr2;
    node* ptr3;
    node* ptr4 = ptr1;
    do {
      ptr4 = ptr4->right;
      temp1 = ptr1->degree;
      while (arr[temp1] != nullptr) {
        ptr2 = arr[temp1];
        if (ptr1->key > ptr2->key) {
          ptr3 = ptr1;
          ptr1 = ptr2;
          ptr2 = ptr3;
        }
        if (ptr2 == mini)
          mini = ptr1;
        fibonacci_link(ptr2, ptr1);
        if (ptr1->right == ptr1)
          mini = ptr1;
        arr[temp1] = nullptr;
        temp1++;
      }
      arr[temp1] = ptr1;
      ptr1 = ptr1->right;
    } while (ptr1 != mini);
    mini = nullptr;
    for (int j = 0; j <= temp3; j++) {
      if (arr[j] != nullptr) {
        arr[j]->left = arr[j];
        arr[j]->right = arr[j];
        if (mini != nullptr) {
          (mini->left)->right = arr[j];
          arr[j]->right = mini;
          arr[j]->left = mini->left;
          mini->left = arr[j];
          if (arr[j]->key < mini->key)
            mini = arr[j];
        }
        else {
          mini = arr[j];
        }
        if (mini == nullptr)
          mini = arr[j];
        else if (arr[j]->key < mini->key)
          mini = arr[j];
      }
    }
  }

  pair<Vertex*, int> SDSScheduler::FibonacciHeap::extract_min(){
    pair<Vertex*, int> returnValues{nullptr, INT_MAX};
    if (mini == nullptr)
      cout << "The heap is empty" << endl;
    else {
      node* temp = mini;
      returnValues = make_pair(mini->storedVertex, mini->key);
      node* pntr;
      pntr = temp;
      node* x = nullptr;
      if (temp->child != nullptr) {

        x = temp->child;
        do {
          pntr = x->right;
          (mini->left)->right = x;
          x->right = mini;
          x->left = mini->left;
          mini->left = x;
          if (x->key < mini->key)
            mini = x;
          x->parent = nullptr;
          x = pntr;
        } while (pntr != temp->child);
      }
      (temp->left)->right = temp->right;
      (temp->right)->left = temp->left;
      mini = temp->right;
      if (temp == temp->right && temp->child == nullptr)
        mini = nullptr;
      else {
        mini = temp->right;
        consolidate();
      }
      numberofNodes--;
    }
    return returnValues;
  }

  void SDSScheduler::FibonacciHeap::cut(struct node* found, struct node* temp){
    if (found == found->right)
      temp->child = nullptr;

    (found->left)->right = found->right;
    (found->right)->left = found->left;
    if (found == temp->child)
      temp->child = found->right;

    temp->degree = temp->degree - 1;
    found->right = found;
    found->left = found;
    (mini->left)->right = found;
    found->right = mini;
    found->left = mini->left;
    mini->left = found;
    found->parent = nullptr;
    found->mark = true;
  }

  void SDSScheduler::FibonacciHeap::cascase_cut(struct node* temp){
    node* ptr5 = temp->parent;
    if (ptr5 != nullptr) {
      if (!temp->mark) {
        temp->mark = true;
      }
      else {
        cut(temp, ptr5);
        cascase_cut(ptr5);
      }
    }
  }

  void SDSScheduler::FibonacciHeap::decrease_key(struct node* found, int val){
    if (mini == nullptr)
      cout << "The Heap is Empty" << endl;

    if (found == nullptr)
      cout << "Node not found in the Heap" << endl;

    found->key = val;

    struct node* temp = found->parent;
    if (temp != nullptr && found->key < temp->key) {
      cut(found, temp);
      cascase_cut(temp);
    }
    if (found->key < mini->key)
      mini = found;
  }

  void SDSScheduler::FibonacciHeap::find(struct node* _mini, int old_val, int val){
    struct node* found = nullptr;
    node* temp5 = _mini;
    temp5->flag = true;
    node* found_ptr = nullptr;
    if (temp5->key == old_val) {
      found_ptr = temp5;
      temp5->flag = false;
      found = found_ptr;
      decrease_key(found, val);
    }
    if (found_ptr == nullptr) {
      if (temp5->child != nullptr)
        find(temp5->child, old_val, val);
      if (!(temp5->right)->flag)
        find(temp5->right, old_val, val);
    }
    temp5->flag = false;
    found = found_ptr;
  }

  void SDSScheduler::FibonacciHeap::deletion(int val){
    if (mini == nullptr)
      cout << "The heap is empty" << endl;
    else {

      // Decreasing the value of the node to 0
      find(mini, val, 0);

      // Calling Extract_min function to
      // delete minimum value node, which is 0
      extract_min();
      cout << "Key Deleted" << endl;
    }
  }

  void SDSScheduler::FibonacciHeap::display(){
    node* ptr = mini;
    if (ptr == nullptr)
      cout << "The Heap is Empty" << endl;
    else {
      cout << "The root nodes of Heap are: " << endl;
      do {
        cout << ptr->key;
        ptr = ptr->right;
        if (ptr != mini) {
          cout << "-->";
        }
      } while (ptr != mini && ptr->right != nullptr);
      cout << endl << "The heap has " << numberofNodes << " nodes" << endl;
    }
  }

  void SDSScheduler::FibonacciHeap::find_min() {
    cout << "min of heap is: " << mini->key << endl;
  }

  bool SDSScheduler::FibonacciHeap::isempty() {
    node* ptr = mini;
    if (ptr == nullptr) {
      return true;
    }else {
      return false;
    }
  }

}

#endif //USE_CADICAL