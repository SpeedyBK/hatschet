//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTSmartieScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include "HatScheT/scheduler/ASAPScheduler.h"

#include <z3++.h>

#include <iostream>
#include <cmath>

namespace HatScheT {

  SMTSmartieScheduler::SMTSmartieScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      // reset previous solutions
      II = -1;
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
      quiet = true;
  }

  void SMTSmartieScheduler::schedule() {

      II = 3;

      generate_b_variables();

      for (auto &it:g.Vertices()){
          for (int i = 0; i < II; i++){
              cout << *get_b_variable(it, i) << endl;
          }
      }

  }

  z3::expr *SMTSmartieScheduler::get_b_variable(Vertex *v, int i) {
      auto key = std::make_pair(v, i);
      return &b_variables.at(key);
  }

  void SMTSmartieScheduler::generate_b_variables() {
      for (auto &it : g.Vertices()){
          for (int i = 0; i < II; i++){
              auto key = std::make_pair(it, i);
              std::stringstream name;
              name << it->getName() << "-" << resourceModel.getResource(it)->getName() << "-" << i;
              z3::expr e(c.bool_const(name.str().c_str()));
              b_variables.insert({key, e});
          }
      }
  }

  void SMTSmartieScheduler::set_b_variables() {

      int upperBoundry = 0;
      for (auto &it : g.Vertices()){
          upperBoundry += resourceModel.getResource(it)->getLatency();
      }

      for (auto &it : g.Edges()){
          //ti - tj >= Li - II * dij
          /*int iUpperBoundry = upperBoundry - resourceModel.getVertexLatency((const*)it);
          for (int i = 0; i < upperBoundry; i++){
              for (int j = 0; j )
          }*/
      }
  }
}
#endif

