//
// Created by bkessler on 18/10/19.
//

//UNDER DEVELOPEMENT


#include "SDSScheduler.h"

namespace HatScheT {

  HatScheT::SDSScheduler::SDSScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
    this->silent = true;
  }

  void HatScheT::SDSScheduler::schedule() {

    if (!this->silent){
      cout << "--------------------------------------------------------" << endl;
      cout << "Creating Binding Variables..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    createBindingVariables();

    //Debug
    if(!this -> silent){cout << "Resource ID / Resource / Vertex / Resource Instance / Boolean Binding Variable" << endl;}
    for (auto &it : bindingVariables){
      if (!this->silent) {
        if (it.binding) {
          cout << it.resourceID <<": " << it.resource->getName() << " / " << it.vertex->getName() << " / " << it.resourceInstance << " / True" << endl;
        }else {
          cout << it.resourceID <<": " << it.resource->getName() << " / " << it.vertex->getName() << " / " << it.resourceInstance << " / False" << endl;
        }
      }
    }

    if (!this->silent){
      cout << "--------------------------------------------------------" << endl;
      cout << "Setting Binding Variables..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    setBindingVariables();

    //Debug
    if(!this -> silent){cout << "Resource ID / Resource / Vertex / Resource Instance / Boolean Binding Variable" << endl;}
    for (auto &it : bindingVariables){
      if (!this->silent) {
        if (it.binding) {
          cout << it.resourceID <<": " << it.resource->getName() << " / " << it.vertex->getName() << " / " << it.resourceInstance << " / True" << endl;
        }else {
          cout << it.resourceID <<": " << it.resource->getName() << " / " << it.vertex->getName() << " / " << it.resourceInstance << " / False" << endl;
        }
      }
    }

  }

  void HatScheT::SDSScheduler::createBindingVariables() {

    list<Resource*> localResources;
    for (auto it = resourceModel.resourcesBegin(); it != resourceModel.resourcesEnd(); ++it) {
      if (!(*it)->isUnlimited()) {
        localResources.push_back(*it);
      }
    }

    this -> numOfLimitedResources = localResources.size();

    bindingVariable bv;
    int idCount = 0;

    for (auto &it:localResources) {
      set<const Vertex *> verticiesOfResource = resourceModel.getVerticesOfResource(it);
      for (auto &vORIt:verticiesOfResource) {
        for (int i = 0; i < it->getLimit(); i++) {
          bv.resource = it;
          bv.resourceID = idCount;
          bv.vertex = vORIt;
          bv.resourceInstance = i;
          bv.binding = false;
          bv.isSet = false;
          bindingVariables.push_back(bv);
        }
      }
      idCount++;
    }
  }

  void SDSScheduler::setBindingVariables() {

    int lastVertexId =-1;
    int count = 0;

    for (int i = 0; i < numOfLimitedResources; i++){
      for (auto &it : bindingVariables){
        if (it.resourceID == i){
          if(it.vertex->getId() != lastVertexId){
            count++;
            lastVertexId = it.vertex->getId();
          }
          if ((count % it.resource->getLimit()) == it.resourceInstance){
            it.binding = true;
          }
        }
      }
      count = 0;
    }
  }

}