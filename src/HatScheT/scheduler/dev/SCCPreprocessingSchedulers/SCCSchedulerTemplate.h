//
// Created by bkessler on 10/10/22.
//
#ifndef HATSCHET_SCCSCHEDULERTEMPLATE_H
#define HATSCHET_SCCSCHEDULERTEMPLATE_H
#ifdef USE_Z3
#ifdef USE_SCALP

#include <iostream>
#include <memory>
#include <deque>

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include "HatScheT/utility/subgraphs/SCC.h"

namespace HatScheT {

  class SCCSchedulerTemplate : public IterativeModuloSchedulerLayer {

  public:

        enum class scheduler {MOOVAC, ED97, SMT, SAT, SH11, MODSDC, NONE};

        enum class sccExpandMode{optimal, fast, automatic};

        SCCSchedulerTemplate(Graph &g, ResourceModel &resourceModel, scheduler sccScheduler, scheduler finalScheduler, double II = -1);

        void setExpandMode(sccExpandMode expandMode);

        void setSolverTimeout(double seconds) override;

        void setThreads( int t ) { this->threads = t; }

        string getName() override { return "SCC-Scheduler"; }

  protected:

    void scheduleInit() override;

    void scheduleIteration() override;

  private:
    /*!
     * Function to cast shared pointers of a base class to shared pointers of derived classes to set specific settings
     * @param ssptr pointer of base class
     */
    void sharedPointerCastAndSetup(shared_ptr<IterativeModuloSchedulerLayer>& ssptr);

    void calcStarttimesPerComplexSCC();

    void modifyResourceModel();

    void resetResourceModel();

    void computeSCCs();

    void sortSCCsByType();

    void buildComplexGraph();

    map<Vertex *, int> scheduleComplexGraph();

    void finalizeComplexSchedule();

    map<Vertex*, int> computeBasicSchedules(SCC* sc);

    map<Vertex*, int> sdcSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&_sccVertexToVertex);

    void combineRelativeSchedules();

    int expandSCC();

    shared_ptr<IterativeModuloSchedulerLayer> selectSccScheduler (scheduler schedEnum, Graph& gr, ResourceModel& rm);

    map<SCC*, int> computeTopologicalSCCOrder(vector<SCC*>&tempsccs);

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

    set<pair<SCC*,int>, priocmp> topoSortedSccs;

    scheduler sccScheduler;

    scheduler finalScheduler;

    map<SCC*, int> inversePriority;

    map<Vertex*, SCC*> vertexToSCC;

    map<SCC*, map<Vertex*, int>> basicRelSchedules;

    map<SCC*, map<Vertex*, int>> complexRelSchedules;

    map<Resource*, int>resourceLimits;
    map<pair<Resource *, int>, int> usedFUsInModSlot;

    set<Edge*> connectingEdges;

    set<int>maxTimesSCC;

    map<Vertex*, int> earliestStarttimes;
    map<Vertex*, int> latestStarttimes;

    deque<SCC*> complexSCCs;
    deque<SCC*> basicSCCs;

    shared_ptr<Graph> complexGraph;
    shared_ptr<ResourceModel> complexRm;
    map<Vertex*, Vertex*> sccVertexToVertex;
    map<Vertex*, Vertex*> vertexToSccVertex;
    map<Vertex*, int> bigComplexSchedule;

    sccExpandMode sccMode;
    int numOfCmplxSCCs;

    int threads;

  };
}

#endif // USE_SCALP
#endif // USE_Z3
#endif //HATSCHET_SCCSCHEDULERTEMPLATE_H
