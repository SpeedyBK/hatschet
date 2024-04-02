//
// Created by bkessler on 4/1/24.
//

#include "SMTMODIncrementalScheduler.h"

namespace HatScheT{

  SMTMODIncrementalScheduler::SMTMODIncrementalScheduler(Graph &g, ResourceModel &resourceModel, int II) : IterativeModuloSchedulerLayer(g, resourceModel, II) {

      lastLength = 0;
      actualLength = 0;

  }

  void SMTMODIncrementalScheduler::scheduleInit() {
      cout << "scheduleInit(): Doing Nothing for now..." << endl;
  }

  void SMTMODIncrementalScheduler::scheduleIteration() {

      generateTVariables();

      addNonNegativeConstraints();

      addDependencyConstraints();

      bool breakCondition = false;
      do {
          cout << actualLength << ": " << getLatestStarttime() << endl;
          actualLength = getLatestStarttime();
          generateBVariables();

          cout << "Variable Connections ..." << endl;
          addVariableConnections();

          cout << "Resource Constraints ..." << endl;
          addResourceConstraints();
          lastLength = actualLength;
          breakCondition = actualLength >= getLatestStarttime();
      } while (!breakCondition);

      cout << "Modell: " << endl;
      cout << getZ3Result() << endl;

      if (getZ3Result() == z3::sat) {
          cout << m << endl;
      }

      exit(-1);
  }

  void SMTMODIncrementalScheduler::generateTVariables() {
      tVariables.clear();
      for (auto &vIt : g.Vertices()){
          std::stringstream name;
          name << vIt->getName();
          z3::expr e(c.int_const(name.str().c_str()));
          tVariables.insert(std::make_pair(vIt, e));
      }
  }

  z3::expr *SMTMODIncrementalScheduler::getTVariable(Vertex* vPtr) {
      try {
          return &tVariables.at(vPtr);
      }catch(std::out_of_range&){
          //cout << "Out_of_Range: " << vPtr->getName() << endl;
          throw (HatScheT::Exception("SMTMODIncrementalScheduler: getTVariable() std::out_of_range"));
      }
  }

  z3::check_result SMTMODIncrementalScheduler::addNonNegativeConstraints() {

      for (auto &vIt : g.Vertices()){
          s.add(tVariables.at(vIt) >= 0);
      }

      return z3Check();
  }

  z3::check_result SMTMODIncrementalScheduler::addDependencyConstraints() {

      for(auto &eIt : g.Edges()) {
          Vertex* src = &(eIt->getVertexSrc());
          Vertex* dst = &(eIt->getVertexDst());

          auto ti = *getTVariable(src);
          auto tj = *getTVariable(dst);

          this->s.add(ti - tj + this->resourceModel.getVertexLatency(src) + eIt->getDelay() - (int)this->II * (eIt->getDistance()) <= 0);
      }

      return z3Check();

  }

  int SMTMODIncrementalScheduler::getLatestStarttime() {
      int tMax = 0;
      for (auto &vIt: g.Vertices()) {
          if (resourceModel.getResource(vIt)->isUnlimited()){
              continue;
          }
          if (tMax < m.eval(*getTVariable(vIt)).get_numeral_int()){
              tMax = m.eval(*getTVariable(vIt)).get_numeral_int();
          }
      }
      return tMax;
  }

  void SMTMODIncrementalScheduler::generateBVariables() {

      auto tMax = getLatestStarttime();

      for (auto &rIt: resourceModel.Resources()) {
          if (rIt->isUnlimited()) {
              continue;
          }
          for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {

              for (int i = 0; i <= tMax; ++i) {

                  try {
                      getBvariable((Vertex*)vIt, i);
                  }catch (HatScheT::Exception&){

                      std::stringstream name;
                      name << "B_" << vIt->getName() << "_" << i;
                      z3::expr e(c.bool_const(name.str().c_str()));
                      auto key = std::make_pair((Vertex *) vIt, i);
                      bVariables.insert(std::make_pair(key, e));

                  }
              }
          }
      }
  }

  z3::expr *SMTMODIncrementalScheduler::getBvariable(Vertex *v, int i) {
      auto key = std::make_pair(v, i);
      try {
          return &bVariables.at(key);
      }catch(std::out_of_range&){
          //cout << "Out_of_Range: " << v->getName() << "_" << i << endl;
          throw (HatScheT::Exception("smtbased Scheduler: getBvariable std::out_of_range"));
      }
  }

  z3::check_result SMTMODIncrementalScheduler::addVariableConnections() {

      for (auto &rIt : resourceModel.Resources()){
          if (rIt->isUnlimited()){
              continue;
          }

          for (auto &vIt : resourceModel.getVerticesOfResource(rIt)){
              for (int i = lastLength; i < actualLength; ++i) {
                  z3::expr constraint(c);
                  constraint = (*getTVariable((Vertex *) vIt) == i) == *getBvariable((Vertex *) vIt, i);
                  s.add(constraint);
              }
          }
      }
      return z3Check();
  }

  z3::check_result SMTMODIncrementalScheduler::addResourceConstraints() {

      for (auto &rIt : resourceModel.Resources()){
          if (rIt->isUnlimited()){
              continue;
          }
          for (int i = lastLength; i < actualLength; ++i) {
              z3::expr_vector ev(c);
              for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {
                  ev.push_back(*getBvariable((Vertex *) vIt, i));
              }
              this->s.add(z3::atmost(ev, rIt->getLimit()));
          }
      }
      return z3Check();

  }

}
