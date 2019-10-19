//
// Created by bkessler on 18/10/19.
//

//UNDER DEVELOPEMENT


#include "SDSScheduler.h"

namespace HatScheT {

  HatScheT::SDSScheduler::SDSScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {

  }

  void HatScheT::SDSScheduler::schedule() {

    this -> bindingVariables = createBindingVariables();

  }

  map<pair<const Vertex *, int>, bool> HatScheT::SDSScheduler::createBindingVariables() {

    list<Resource *> localResources;
    for (auto it = resourceModel.resourcesBegin(); it != resourceModel.resourcesEnd(); ++it) {
      if (!it.operator*()->isUnlimited()) {
        localResources.push_back(*it);
      }
    }

    map<pair<const Vertex *, int>, bool> bindingsVariables;
    for (auto &it:localResources) {
      if (!this->silent) { cout << it->getName() << " / " << it->getLimit() << "x" << endl; }
      set<const Vertex *> verticiesOfResource = resourceModel.getVerticesOfResource(it);
      for (auto &vORIt:verticiesOfResource) {
        if (!this->silent) { cout << vORIt->getName() << endl; }
        for (int i = 0; i < it->getLimit(); i++) {
          pair<const Vertex *, int> a = make_pair(vORIt, i);
          bindingsVariables.emplace(a, false);
        }
      }
    }

    if (!this->silent) { cout << "Vertex / ResourceInstance / Binding Variable" << endl; }
    for (auto &it:bindingsVariables) {
      if (!this->silent) {
        if (it.second) {
          cout << it.first.first->getName() << " / " << it.first.second << " / " << "true" << endl;
        } else {
          cout << it.first.first->getName() << " / " << it.first.second << " / " << "false" << endl;
        }
      }
    }

    return bindingsVariables;
  }
}