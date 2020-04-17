//
// Created by bkessler on 18/10/19.
//

//UNDER DEVELOPEMENT

#ifdef USE_CADICAL
#include "SDSScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/SDCSolver.h"
#include <cassert>
#include <iomanip>
#include <climits>
#include <cmath>
#include <chrono>

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

    if (sdcS == BellmanFord) {

      double timer = 0;
      std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

      /*!
       * Setup of Variables. (1)
       */
      //Create Binding Variables.
      createBindingVariables();

      //Set Values for Binding Variables.
      setBindingVariables();

      //Create Sharing Variables.
      if (bindingType == 'S') {
        sharingVariables = createShVarsMaxSpeed();
      } else if (bindingType == 'R') {
        sharingVariables = createShVarsMinRes();
      }

      //Create Data Dependency Constraints.
      createDependencyConstraints();

      //Print Depandency Constraints for Debugging.
      cout << "Data Dependency Constraints:" << endl;
      for (auto &it : dependencyConstraintsSDC){
        static int i = 0;
        cout << ++i << ". " << it.first.first->getName() << " - " << it.first.second->getName() << " < " << it.second << endl;
      }
      cout << endl;

      std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
      std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
      timer += (((double) timeSpan.count()) / 1000000000.0);
      cout  << "Creating Variables Done after " << timer << " Seconds" << endl << endl;

      /*!
       * Calculate a Solution for the SDC-Problem without Ressource Constraints. (2)
       */
      //Setup an SDC-Solver
      auto *s = new SDCSolver;

      //Add Dependency Constraints to the SDC-Solver
      for (auto &it : dependencyConstraintsSDC) {
        s->add_sdc_constraint(
            s->create_sdc_constraint((Vertex *) it.first.second, (Vertex *) it.first.first, it.second));
      }

      //Calculate a Shedule with Data Dependencys only.
      s->compute_inital_solution();

      //Check is SDC-System is feasible. (3)
      if (s->get_solver_status() == 10) {
        //If feasible save it.
        startTimes = map_SDC_solution_to_Graph(s->get_solution());
      } else {
        //Else terminate with error.
        throw HatScheT::Exception("1. System of Constraints not feasible.");
      }
      //Delete SDC-Solver
      delete s;

      std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
      timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2);
      timer += (((double) timeSpan.count()) / 1000000000.0);
      cout << "Initial SDC-Problem solved after " << timer << " Seconds" << endl;

      //Calculate Rssource Constraints with SAT, from Sharing Variable, conflictClauses is an empty Vector for now.
      passToSATSolver(sharingVariables, conflictClauses);

      //Print Resource Constraints for Debugging.
      cout << endl << "Resource Constraints: " << endl;
      for (auto &it : resourceConstraints){
        static int i = 0;
        cout << ++i << ". " << it.constraintOneVertices.first->getName() << " - " << it.constraintOneVertices.second->getName() << " < " << it.constraintOne << endl;
      }
      cout << endl;

      std::chrono::high_resolution_clock::time_point t4= std::chrono::high_resolution_clock::now();
      timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3);
      timer += (((double) timeSpan.count()) / 1000000000.0);
      cout << "Initial SAT-Problem solved after " << timer << " Seconds" << endl;

      //Add Dependency Constraints to an SDC-Solver.
      s = new SDCSolver;
      for (auto &it : dependencyConstraintsSDC) {
        s->add_sdc_constraint(s->create_sdc_constraint((Vertex *) it.first.second, (Vertex *) it.first.first, it.second));
      }
      //Add Resource Constraints to SDC-Solver
      for (auto &it : resourceConstraints){
        static int counter = 0;
        s->add_sdc_constraint(s->create_sdc_constraint((Vertex*)it.constraintOneVertices.first, (Vertex*)it.constraintOneVertices.second, it.constraintOne));
        s->compute_inital_solution();
        if (s->get_solver_status() != 10){
          cout << counter << endl;
          //terminate with error.
          std::chrono::high_resolution_clock::time_point t6= std::chrono::high_resolution_clock::now();
          timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t6 - t4);
          timer += (((double) timeSpan.count()) / 1000000000.0);
          cout << "SDC-Problem with Resource Constraints done after " << timer << " Seconds" << endl;
          throw HatScheT::Exception("2. System of Constraints not feasible.");
        }else {
          startTimes = map_SDC_solution_to_Graph(s->get_solution());
        }
        counter++;
      }

      std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
      timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t5 - t4);
      timer += (((double) timeSpan.count()) / 1000000000.0);
      cout << "SDC-Problem with Resource Constraints solved after " << timer << " Seconds" << endl;

      //Print Constraint Graph for Debugging.
      cout << "Constraint Graph: " << endl;
      s->print_Constraint_Graph();

      //Delete SDC-Solver
      delete s;
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
      cout << "CaDiCaL: Problem: " << "p cnf " << shareVars.size() * 2 << " " << shareVars.size() * 2 + confClauses.size() << endl;
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
      if (!quiet) {
        cout << endl << "CaDiCaL: Problem Satisfiable" << endl;
      }
    } else if (res == 0) {
        if (!quiet) {
          cout << endl << "CaDiCaL: Problem Unsolved" << endl;
        }
    } else if (res == 20) {
        if (!quiet) {
          cout << endl << "CaDiCaL: Problem Unsatisfiable" << endl;
        }
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
        //if (!this->quiet) {
          //cout << solver->val(i + 1) << " ";
        //}
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
            //mapping.createConstraintTwo((int) this->II);
            resourceConstraints.push_back(mapping);
          }
          ++mIt;
        }
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
    this->initScheduleLength = asa.getScheduleLength();
    return asa.getSchedule();
  }

  map<Vertex*, int> SDSScheduler::map_SDC_solution_to_Graph(map<Vertex *, int> solution) {

    int min = INT_MAX;
    for (auto &it : solution){
      it.second < min ? min = it.second : min = min;
    }

    map<Vertex*, int> mapped_solution;
    for (auto &it : solution){
      if (it.first->getId() != -1){
        mapped_solution[&g.getVertexById(it.first->getId())] = it.second + abs(min);
      }
    }
    return mapped_solution;
  }
}

#endif //USE_CADICAL