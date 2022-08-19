//
// Created by bkessler on 8/11/22.
//

#ifndef HATSCHET_SMTSCCSCHEDULER_H
#define HATSCHET_SMTSCCSCHEDULER_H

#pragma once
#ifdef USE_Z3

#include <iostream>
#include <memory>
#include <deque>

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include "HatScheT/utility/subgraphs/SCC.h"

namespace HatScheT {

  class SMTSCCScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:

    SMTSCCScheduler(Graph &g, ResourceModel &resourceModel);

    ~SMTSCCScheduler() override;

    void schedule() override;

  private:

    map<SCC*, int> inversePriority;

    struct priocmp {
      bool operator() (const pair<SCC*, int> x, const pair<SCC*, int> y) const {
          if (x.second < y.second){
              return true;
          }
          if (x.second == y.second){
              return x.first < y.first;
          }
          return false;
      }
    };

    void computeSCCs();

    void modifyResourceModel();

    void resetResourceModel();

    void combineRelScheds();

    map<SCC*, int> computeTopologicalSCCOrder(vector<SCC*>&tempsccs);
    map<Vertex*, SCC*> vertexToSCC;

    map<Vertex*, int> computeBasicSchedules(SCC* sc);
    map<Vertex*, int> sdcSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&sccVertexToVertex);
    list<pair<SCC*, map<Vertex*, int>>> basicRelSchedules;

    map<Vertex*, int> computeComplexSchedule(deque<SCC*> &complexSCCs);
    map<Vertex*, int> smtSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&sccVertexToVertex);
    map<Vertex*, int> complexRelSchedule;

    map<Resource*, int>resourceLimits;

    set<pair<SCC*,int>, priocmp> topoSortedSccs;
    set<Edge*> connectingEdges;
  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTSCCSCHEDULER_H
