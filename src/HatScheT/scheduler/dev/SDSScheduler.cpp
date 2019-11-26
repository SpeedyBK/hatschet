//
// Created by bkessler on 18/10/19.
//

//UNDER DEVELOPEMENT

#ifdef USE_CADICAL
#include "SDSScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/utility/Utility.h"
#include <cassert>
#include <iomanip>
#include <climits>
#include <cmath>

namespace HatScheT {

  HatScheT::SDSScheduler::SDSScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel, std::list<std::string> &sw) : SchedulerBase(g,resourceModel), ILPSchedulerBase(sw) {

    this->unsatisiable = false;
    computeMinII(&this->g, &resourceModel);
    this->minII = ceil(this->minII);
    computeMaxII(&g, &resourceModel);
    if (minII >= maxII) maxII = (int) minII + 1;
    this->II = minII;
    this->swishlist = sw;
    this->bindingType = 'S';
    this->numOfLimitedResources = 0;

    //ToDo: Has to be removed
    cout << "Constructor SDS:" << endl;
    cout << "Graph generated with " << g.getNumberOfVertices() << " Vertices." << endl;
    cout << "MinII is: " << minII << endl;
    cout << "MaxII is: " << maxII << endl;
    cout << "II is: " << II << endl;
  }

  void HatScheT::SDSScheduler::schedule() {

    if (sdcS == ScaLP) {
      createScaLPVariables();

      createBindingVariables();

      setBindingVariables();

      if (bindingType == 'S') {
        sharingVariables = createShVarsMaxSpeed();
      } else if (bindingType == 'R') {
        sharingVariables = createShVarsMinRes();
      }

      checkfeasibilityScaLP({}, -1);

      this -> r = this -> solver->getResult();
      int maxLatConst = (int) r.values[newVar];
      if (!this->quiet) {
        cout << "Max LAT Const: " << maxLatConst << endl;
      }
      bool feasible = false;
      vector<vector<int>> conflict;
      int i = 0;
      while (!feasible) {
        if (i > 1000){
          break;
        }
        i++;
        passToSATSolver(sharingVariables, conflict);
        if(unsatisiable){
          conflict.clear();
          //this->II++;
          maxLatConst++;
          passToSATSolver(sharingVariables, conflict);
          unsatisiable = false;
        }
        std::vector<pair<ScaLP::Constraint, ScaLP::Constraint>> cstr;
        list <orderingVariabletoSDCMapping> possibleConflicts;
        if (!resourceConstraints.empty()) {
          for (auto &it : resourceConstraints) {
            cstr.emplace_back(createadditionalScaLPConstraits(it));
            possibleConflicts.push_back(it);
            feasible = checkfeasibilityScaLP(cstr, maxLatConst);
            if (!feasible) {
              for (auto &itr : possibleConflicts) {
                cout << "SAT-Variable:" << itr.satVariable << endl;
              }
              conflict.push_back(getSATClausesfromScaLP(possibleConflicts));
              possibleConflicts.clear();
              break;
            }
          }
        }else {
          feasible = checkfeasibilityScaLP(cstr, maxLatConst);
          if (!feasible) {
            this -> II++;
          }
        }
      }

      this-> r = this->solver->getResult();
      cout <<"Objektive Value: " << r.objectiveValue << endl;
      startTimes = findstarttimes(r);

    }


    if (sdcS == BellmanFord) {

      createBindingVariables();

      setBindingVariables();

      if (bindingType == 'S') {
        sharingVariables = createShVarsMaxSpeed();
      } else if (bindingType == 'R') {
        sharingVariables = createShVarsMinRes();
      }

      //Setup SDC and check if feasible without Resource Constraints.
      createDependencyConstraints();
      auto initialSolution = getFirstSDCSolution();
      BellmanFordSDC bfsdc(this->dependencyConstraintsSDC, &g, initialSolution);
      bfsdc.setquiet(this->quiet);
      bfsdc.printConstraintGraph();
      if (bfsdc.solveSDC()==unfeasible){
        throw HatScheT::Exception ("SDSScheduler: Can't find an initial schedule for unlimited Resources.");
      }

      //If Initial Schedule found, get Resource Constraints via SAT, and check Schedule against Resource Constraints.

      //ToDo Ab hier While Schleife.
      vector<vector<int>> conflict;
      passToSATSolver(this->sharingVariables, conflict);

      bfsdc.setadditionlaConstraints(resourceConstraints);
      while (bfsdc.solveSDC()==unfeasible){
        conflict = bfsdc.getConflicts();
        passToSATSolver(this->sharingVariables, conflict);
        if(unsatisiable){
          //Do some shit...
          conflict.clear();
          cout << "********************************************************************" << endl;
          passToSATSolver(this->sharingVariables, conflict);
          cout << "********************************************************************" << endl;
          bfsdc.increaseLatency();
          bfsdc.setadditionlaConstraints(resourceConstraints);
          if (bfsdc.solveSDC()==feasible){
            break;
          }else{
            unsatisiable = false;
            conflict = bfsdc.getConflicts();
            passToSATSolver(this->sharingVariables, conflict);
          }
        }else{
          bfsdc.setadditionlaConstraints(resourceConstraints);
        }
      }
      startTimes = bfsdc.getVertexCosts();
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
    if (!this->quiet) {
      cout << "Problem: " << "p cnf " << shareVars.size() * 2 << " " << shareVars.size() * 2 + confClauses.size()
           << endl << endl;
    }
    for (auto &It : shareVars) {
      if (It.second) {
        solver->add(litCounter);
        solver->add(litCounter + 1);
        solver->add(0);
        if (!this->quiet) { cout << setw(3) << litCounter << setw(3) << litCounter + 1 << setw(3) << " 0" << endl; }
        solver->add(litCounter * -1);
        solver->add((litCounter + 1) * -1);
        solver->add(0);
        if (!this->quiet) {
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
        if (!this->quiet) {
          cout << setw(3) << Itr;
        }
      }
      if (!It.empty()) {
        solver->add(0);
        if (!this->quiet) {
          cout << setw(3) << "0";
        }
      }
      if (!this->quiet) {
        cout << endl;
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
      unsatisiable = true;
    }

    /*!
     * Getting Solution
     * Solver variable starts at 1, since negated variables are shown as negative numbers.
     * The Solution from SAT is than mapped to an SDC-Format like (srcVertex - dstVertex <= -1)
     */
    map<pair<const Vertex*, const Vertex*>, int> solutionMap;
    resourceConstraints.clear();
    auto mIt = sharingVariables.begin();
    if (res == 10) {
      for (int i = 0; i < litCounter - 1; i++) {
        if (!this->quiet) {
          cout << solver->val(i + 1) << " ";
        }
        if (i % 2 == 0){
          if(solver->val(i+1) > 0) {
            orderingVariabletoSDCMapping mapping;
            mapping.satVariable = i+1;
            mapping.constraintOneVertices = mIt->first;
            mapping.constraintOne = - resourceModel.getResource(mIt->first.first)->getBlockingTime();
            //mapping.createConstraintTwo((int) this->II);
            resourceConstraints.push_back(mapping);
          }
        }else {
          if(solver->val(i+1) > 0) {
            orderingVariabletoSDCMapping mapping;
            mapping.satVariable = i+1;
            mapping.constraintOneVertices = swapPair(mIt->first);
            mapping.constraintOne = - resourceModel.getResource(mIt->first.first)->getBlockingTime();
            mapping.createConstraintTwo((int) this->II);
            resourceConstraints.push_back(mapping);
          }
          ++mIt;
        }
      }
      if (!this->quiet) {
        cout << endl;
        for (auto &it : resourceConstraints){
          cout << "Sat-Variable " << it.satVariable << endl;
          cout << it.constraintOneVertices.first->getName() << " - " << it.constraintOneVertices.second->getName() << " <= " << it.constraintOne << endl;
          //cout << it.constraintTwoVertices.first->getName() << " - " << it.constraintTwoVertices.second->getName() << " <= " << it.constraintTwo << endl << endl;
        }
        cout << endl;
      }
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

  void SDSScheduler::createDependencyConstraints() {

    map < pair < const Vertex*, const Vertex* >, int > dependencyConstraints;

    for (auto &it : this-> g.Edges()){
      auto sVertex = &it->getVertexSrc();
      auto dVertex = &it->getVertexDst();

      auto constVertexPair = make_pair((const Vertex*) sVertex, (const Vertex*) dVertex);
      //int distance = ((int)this->II*it->getDistance() - this->resourceModel.getVertexLatency(&it->getVertexSrc())+it->getDelay());
      int distance;
      if (it->getDistance() == 0) {
        distance = (it->getDelay() - this->resourceModel.getVertexLatency(&it->getVertexSrc()));

        if (sdcS == BellmanFord) {
          dependencyConstraints.insert(make_pair(constVertexPair, distance));
        } else if (sdcS == ScaLP) {
          this->solver->addConstraint(this->scalpVariables[sVertex] - this->scalpVariables[dVertex] <= distance);
        }
      }
    }

    if (sdcS == BellmanFord) {
      dependencyConstraintsSDC = dependencyConstraints;
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


  void SDSScheduler::createScaLPVariables() {

    if (this->scalpVariables.empty()) {
      for (auto &it : g.Vertices()) {
        auto v = it;
        this->scalpVariables[v] = ScaLP::newIntegerVariable(v->getName(), 0, ScaLP::INF());
      }
    }

  }

  pair <ScaLP::Constraint, ScaLP::Constraint> SDSScheduler::createadditionalScaLPConstraits(orderingVariabletoSDCMapping order) {
    ScaLP::Constraint cA, cB;
    cA = (this->scalpVariables[order.constraintOneVertices.first]-this->scalpVariables[order.constraintOneVertices.second] <= order.constraintOne);
    cB = (this->scalpVariables[order.constraintTwoVertices.first]-this->scalpVariables[order.constraintTwoVertices.second] <= order.constraintTwo);
    return make_pair(cA, cB);
  }

  bool SDSScheduler::checkfeasibilityScaLP(vector<pair<ScaLP::Constraint, ScaLP::Constraint>> scstr, int maxLatConstr) {

    cout << "Checking feasibility" << endl;

    this -> solver->reset();
    createDependencyConstraints();

    for (auto &it : this->scalpVariables) {
      ScaLP::Constraint c = (this->newVar - it.second >= this->resourceModel.getResource(it.first)->getLatency());
      this -> solver->addConstraint(c);
    }

    for (auto &it : scstr){
      this->solver->addConstraint(it.first);
      this->solver->addConstraint(it.second);
    }

    if (maxLatConstr > -1){
      ScaLP::Constraint cs = (this -> newVar <= maxLatConstr); //ToDo nochmal sauber aufschreiben und verstehen.
      this -> solver->addConstraint(cs);
    }

    ScaLP::Term o = 1 * newVar;
    this->solver->setObjective(ScaLP::minimize(o));

    solver->writeLP("scalpLOG.txt");

    auto status = this->solver->solve();
    cout << "ScaLP Status: " << status << endl;

    return (status == ScaLP::status::FEASIBLE or status == ScaLP::status::OPTIMAL or status == ScaLP::status::TIMEOUT_FEASIBLE);
  }

  map<Vertex *, int> SDSScheduler::findstarttimes(ScaLP::Result &r) {

    map <Vertex*, int> times;

    for (auto &it : r.values) {
      auto *v = (Vertex*)this->getVertexFromVariable(it.first);
      if (v != nullptr) times[v] = (int) it.second;
    }

    return times;
  }

  const Vertex *SDSScheduler::getVertexFromVariable(const ScaLP::Variable &sv) {
    for (auto &it : this->scalpVariables) {
      if (it.second == sv) return it.first;
    }
    return nullptr;
  }

  vector<int> SDSScheduler::getSATClausesfromScaLP(list<orderingVariabletoSDCMapping> &conflicts) {
    vector <int> conflictSAT;
    conflictSAT.reserve(conflicts.size());
    for (auto &it : conflicts) {
      conflictSAT.push_back(it.satVariable * -1);
    }
    return conflictSAT;
  }

  map<Vertex *, int> SDSScheduler::getFirstSDCSolution() {

    HatScheT::ResourceModel rmTemp;

    for (auto &rIt : resourceModel.Resources()){
      rmTemp.makeResource(rIt->getName(),-1,rIt->getLatency(),rIt->getBlockingTime());
    }

    for (auto &it : g.Vertices()){
      rmTemp.registerVertex(it, rmTemp.getResource(resourceModel.getResource(it)->getName()));
    }

    ASAPScheduler asa (this->g, rmTemp);
    asa.schedule();
    return asa.getSchedule();
  }



  ///////////////////////////////////////////////
  // Additional functionallity for Graph Class //
  ///////////////////////////////////////////////

  void SDSScheduler::ConstraintGraph::removeEdge(Vertex *srcVertex, Vertex *dstVertex) {

    for (auto &it : edges){
      if (!this->silent) {
        cout << "VSRC: " << it->getVertexSrc().getId() << " VDST: " << it->getVertexDst().getId() << " || "
             << srcVertex->getId() << " " << dstVertex->getId() << endl;
      }
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

  SDSScheduler::BellmanFordSDC::BellmanFordSDC(SDSScheduler::ConstraintGraph cg, Graph* g, map<Vertex*, int> &initsolution) {
    this -> cg = cg;
    this-> g = g;
    this-> hasNegativeCycle = false;
    this-> initsol = initsolution;
    ajustConstraintGraph();
  }

  SDSScheduler::BellmanFordSDC::BellmanFordSDC(map<pair<const Vertex *, const Vertex *>, int> &constraints, Graph* g, map<Vertex*, int> &initsolution) {
    this-> hasNegativeCycle = false;
    this-> g = g;
    this-> initsol = initsolution;
    /*!
     * Create Vertices:
     */
    for (auto &it: constraints) {
      if (!doesVertexExist(&this->cg, it.first.first->getId())) {
        this->cg.createVertex(it.first.first->getId());
        this->cg.getVertexById(it.first.first->getId()).setName(it.first.first->getName());
      }
      if (!doesVertexExist(&this->cg, it.first.second->getId())) {
        this->cg.createVertex(it.first.second->getId());
        this->cg.getVertexById(it.first.second->getId()).setName(it.first.second->getName());
      }
      /*!
       * Create Edges:
       */
      if (!this->cg.doesEdgeExistID((Vertex *) it.first.first, (Vertex *) it.first.second)) {
        this->cg.createEdgeSDS(this->cg.getVertexById(it.first.second->getId()),
                               this->cg.getVertexById(it.first.first->getId()), it.second);
      }
    }
    cout << "Blubb" << endl;
    ajustConstraintGraph();
    cout << "Blubber" << endl;
  }

  void SDSScheduler::BellmanFordSDC::bellmanFordAlgorithm() {
    /*!
     * Initialisation of Algorithm
     */
    for (auto &it : this->cg.Vertices()){
      vertexCosts[it] = INT_MAX;
    }
    vertexCosts[&this->cg.getVertexById(this->startID)] = 0;

    /*
    if(!quiet){
      cout << "Vertexcosts 1: " << endl;
      for (auto &it : vertexCosts){
        cout << it.first->getId() << ": " << it.second << endl;
      }
    }
     */

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


    if(!quiet){
      cout << "Vertexcosts 2: " << endl;
      for (auto &it : vertexCosts){
        cout << it.first->getName() << ": " << it.second << endl;
      }
    }

    /*!
     * Checking for negative Cycles
     */
    for (auto &it : this->cg.Edges()){
      if ((vertexCosts[&it->getVertexSrc()] < INT_MAX) && (vertexCosts[&it->getVertexSrc()] + it->getDistance() < vertexCosts[&it->getVertexDst()])){
        cout << "Cost SRC: " << vertexCosts[&it->getVertexSrc()] << " + Distance: " << it->getDistance() << " < Cost DST: " <<  vertexCosts[&it->getVertexDst()] << " : ";
        hasNegativeCycle = true;
        cout << "infeasible" << endl;
        break;
      }else {
        cout << "Cost SRC: " << vertexCosts[&it->getVertexSrc()] << " + Distance: " << it->getDistance() << " < Cost DST: " <<  vertexCosts[&it->getVertexDst()] << " : ";
        hasNegativeCycle = false;
        cout << "feasible" << endl;
      }
    }
  }

  void SDSScheduler::BellmanFordSDC::printConstraintGraph() {
    if (!this->quiet) {
      cout << "Constraint Graph with " << cg.getNumberOfVertices() << " Vertices and " << cg.getNumberOfEdges() << " Edges found." << endl;
      for (auto &it : this->cg.Edges()) {
        cout << it->getVertexSrcName() << " -- " << it->getDistance() << " -- " << it->getVertexDstName() << endl;
      }
    }
  }

  void SDSScheduler::BellmanFordSDC::setquiet(bool silence) {
    this->quiet = silence;
  }

  void SDSScheduler::BellmanFordSDC::ajustConstraintGraph() {

    list <pair<Vertex*, int>> outputVertices;
    auto gT = Utility::transposeGraph(g);
    cout << "Pandabär" << endl;
    for (auto &it : gT.first->Vertices()){
      if (Utility::isInput(gT.first, it)){
        outputVertices.emplace_back(make_pair(&cg.getVertexById(it->getId()),this->initsol[&g->getVertexById(it->getId())]));
      }
    }
    cout << "Pampelmuse" << endl;

    if(!quiet) {
      cout << "Outputvertices: " << endl;
    }
    for (auto &it : outputVertices){
      if(!quiet) {
        cout << it.first->getName() << ": " << it.second << endl;
      }
      startID = it.first->getId();
    }

    list<Vertex*> inputVertices;
    for (auto &it : g->Vertices()){
      if (Utility::isInput(g, it)){
        inputVertices.push_back(it);
      }
    }

    if(!quiet) {
      cout << "Inputvertices: " << endl;
      for (auto &it : inputVertices) {
        cout << it->getName() << endl;
      }
    }
    /*
    for (auto &oIt : outputVertices){
      for (auto &iIt : inputVertices){
        if(!cg.doesEdgeExistID(iIt, oIt.first)) {
          this -> latencyEdges.insert(&cg.createEdge(cg.getVertexById(iIt->getId()), cg.getVertexById(oIt.first->getId()), oIt.second));
        }
      }
    }
     */
  }

  map<Vertex *, int> SDSScheduler::BellmanFordSDC::getVertexCosts() {
    map <Vertex*, int> schedule;
    int min = INT_MAX;
    for (auto &it : vertexCosts){
      if (it.second < min){
        min = it.second;
      }
    }
    for (auto &it : vertexCosts){
      schedule.insert(make_pair(&g->getVertexById(it.first->getId()), it.second-min));
    }
    return schedule;
  }

  void SDSScheduler::BellmanFordSDC::setadditionlaConstraints(list<orderingVariabletoSDCMapping> &constraints) {
    this -> additionalConstraints = constraints;
  }

  sdcStatus SDSScheduler::BellmanFordSDC::solveSDC() {

    vector <int> potentialConflicts;

    if(additionalConstraints.empty()){
      bellmanFordAlgorithm();
      if (this -> hasNegativeCycle) {
        return unfeasible;
      }else {
        return feasible;
      }
    }else{
      vector<Edge*> newEdges;
      newEdges.reserve(additionalConstraints.size());
      for (auto &it : additionalConstraints){
        cout << "Adding: " << it.constraintOneVertices.second->getName() << " - " << it.constraintOneVertices.first->getName() << " <= " << it.constraintOne << endl;
        newEdges.emplace_back(&cg.createEdgeSDS(cg.getVertexById(it.constraintOneVertices.second->getId()), cg.getVertexById(it.constraintOneVertices.first->getId()), it.constraintOne));
        //newEdges.emplace_back(&cg.createEdgeSDS(cg.getVertexById(it.constraintTwoVertices.first->getId()), cg.getVertexById(it.constraintTwoVertices.second->getId()), it.constraintTwo));
        potentialConflicts.emplace_back(it.satVariable * (-1));
        printConstraintGraph();
        bellmanFordAlgorithm();
        //printConstraintGraph();
        if(this -> hasNegativeCycle){
          this->conflictClauses.push_back(potentialConflicts);
          for(auto &eIt : newEdges){
            cg.removeEdge(&eIt->getVertexSrc(), &eIt->getVertexDst());
          }
          newEdges.clear();
          this->additionalConstraints.clear();
          return unfeasible;
        }
      }
      return feasible;
    }
  }

  vector<vector<int>> SDSScheduler::BellmanFordSDC::getConflicts() {
    return this->conflictClauses;
  }

  void SDSScheduler::BellmanFordSDC::increaseLatency() {

    for (auto &eIt : cg.Edges()){
      for (auto &it : latencyEdges){
        if (it->getVertexSrc().getId() == eIt->getVertexSrc().getId() && it->getVertexDst().getId() == eIt->getVertexDst().getId()){
        eIt->setDistance(eIt->getDistance() + 1);
        }
      }
    }
  }

}

#endif //USE_CADICAL