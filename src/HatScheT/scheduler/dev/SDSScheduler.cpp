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
    if(!this -> silent){cout << "Index : Resource ID / Vertex / Resource Instance / Boolean Binding Variable" << endl;}
    for (auto &it : bindingVariables){
      if (!this->silent) {
        if (it.binding) {
          cout << it.index << ": " << it.resourceID <<" / " << it.vertex->getName() << " / " << it.resourceInstance << " / True" << endl;
        }else {
          cout << it.index << ": " << it.resourceID <<" / " << it.vertex->getName() << " / " << it.resourceInstance << " / False" << endl;
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
    if(!this -> silent){cout << "Index : Resource ID / Vertex / Resource Instance / Boolean Binding Variable" << endl;}
    for (auto &it : bindingVariables){
      if (!this->silent) {
        if (it.binding) {
          cout << it.index << ": " << it.resourceID <<" / " << it.vertex->getName() << " / " << it.resourceInstance << " / True" << endl;
        }else {
          cout << it.index << ": " << it.resourceID <<" / " << it.vertex->getName() << " / " << it.resourceInstance << " / False" << endl;
        }
      }
    }

    if (!this->silent){
      cout << "--------------------------------------------------------" << endl;
      cout << "Creating and Setting Sharing Variables..." << endl;
      cout << "--------------------------------------------------------" << endl;
      cout << endl;
    }

    sharingVariables = createSharingVariables();

    if (!this->silent) {
      for (auto &it:sharingVariables) {
        if (it.second) {
          cout << it.first.first->getName() << ", " << it.first.second->getName() << ": true" << endl;
        } else {
          cout << it.first.first->getName() << ", " << it.first.second->getName() << ": false" << endl;
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

  map <pair<const Vertex*, const Vertex*>, bool> SDSScheduler::createSharingVariables() {

    map <pair<const Vertex*, const Vertex*>, bool> shared;
    list <bindingVariable*> tempBinVars;
    list <list <bindingVariable*>>templists;

    for (int i = 0; i < numOfLimitedResources; i++){
      for (auto &it : bindingVariables){
        if (it.resourceID == i){
          tempBinVars.push_back(&it);
        }
      }
      templists.push_back(tempBinVars);
      tempBinVars.clear();
    }

    for (auto &lit : templists){
      int loops = 1;
      for (auto &ibvIt : lit){
        if (ibvIt->binding) {
          auto jbvIt = lit.begin();
          for (advance(jbvIt, loops); jbvIt != lit.end(); ++jbvIt) {
            if(ibvIt->vertex->getId() != (*jbvIt)->vertex->getId() && (ibvIt->resourceInstance == (*jbvIt)->resourceInstance)){
              if (ibvIt->binding && (*jbvIt)->binding) {
                auto vpair = make_pair(ibvIt->vertex, (*jbvIt)->vertex);
                shared.insert(make_pair(vpair, true));
              }else {
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
}