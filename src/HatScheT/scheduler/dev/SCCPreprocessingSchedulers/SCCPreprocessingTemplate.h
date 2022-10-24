//
// Created by bkessler on 10/5/22.
//
#ifndef HATSCHET_SCCPREPROCESSINGTEMPLATE_H
#define HATSCHET_SCCPREPROCESSINGTEMPLATE_H
#pragma once

#include <iostream>
#include <memory>
#include <deque>

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include "HatScheT/utility/subgraphs/SCC.h"

namespace HatScheT {

  class SCCPreprocessingTemplate : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:

    enum class schedule_t {optimal, fast, automatic};

    enum class schedulerSel {MOOVAC, ED97, SMT, SAT};

    SCCPreprocessingTemplate(Graph &g, ResourceModel &resourceModel);

    SCCPreprocessingTemplate(Graph &g, ResourceModel &resourceModel, double II);

    ~SCCPreprocessingTemplate() override;

    void schedule() override;

    void setSolverTimeout(int seconds);

    void setMode(schedule_t schedulemode);

    int getTimeBudget() const { return timeBudget; }

  private:

    schedulerSel scheduler = schedulerSel::MOOVAC;

    void getSolverStatus();

    bool scalpAvail;
    bool z3Avail;

    map<SCC*, int> inversePriority;

    struct priocmp {
      bool operator() (const pair<SCC*,int> x, const pair<SCC*,int> y) const {
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

    int expandSCC(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm);

    void modifyResourceModel();

    void resetResourceModel();

    void combineRelScheds();

    int timeLimit;
    int timeBudget;

    map<SCC*, int> computeTopologicalSCCOrder(vector<SCC*>&tempsccs);
    map<Vertex*, SCC*> vertexToSCC;

    map<Vertex*, int> computeBasicSchedules(SCC* sc);
    map<Vertex*, int> sdcSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&sccVertexToVertex);
    map<SCC*, map<Vertex*, int>> basicRelSchedules;

    void computeComplexSchedule(deque<SCC*> &complexSCCs);
    map<Vertex*, int> computeSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&sccVertexToVertex);
    map<SCC*, map<Vertex*, int>> complexRelSchedules;

    map<Resource*, int>resourceLimits;
    map<pair<Resource *, int>, int> usedFUsInModSlot;

    set<pair<SCC*,int>, priocmp> topoSortedSccs;
    set<Edge*> connectingEdges;
    bool iigiven;

    schedule_t mode;
    int numOfCmplxSCCs;

    map<Vertex*, int> scheduleWithScheduler(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm, shared_ptr<SchedulerBase> &sbPtr);

    map<Vertex*, int> scheduleWithSchedulerGivenII(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm, shared_ptr<SchedulerBase> &sbPtr);
  };

}

#endif //HATSCHET_SCCPREPROCESSINGTEMPLATE_H
