//
// Created by bkessler on 8/25/22.
//

#ifndef HATSCHET_SMTCDCLSCHEDULER_H
#define HATSCHET_SMTCDCLSCHEDULER_H

#ifdef USE_Z3

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <HatScheT/base/Z3SchedulerBase.h>

namespace HatScheT {

  class SMTCDCLScheduler : public IterativeModuloSchedulerLayer, public Z3SchedulerBase {

  public:

    SMTCDCLScheduler(Graph &g, ResourceModel &resourceModel, double II = -1);

    void scheduleOLD();

    void setSolverTimeout(double seconds) override;

    string getName() override { return "SMT-CDCL-Scheduler"; }

  private:

    void scheduleInit() override;
    void scheduleIteration() override;

    void modifyResourceModel();
    void resetResourceModel();
    void setInitialStartTimes();

    void calculateStartimes(int candidateLatency);
    void reduceLatency(z3::solver &s);
    void createBooleanVariables();
    void addOneSlotConstraintToSolver();
    void addResourceContraintsToSolver();

    int getScheduleLatency (map<Vertex*, int> &sched);

    void fixDependencyConstraints(stack<Edge*> &violatedEdges, z3::solver &s);

    void addConflictClause(Edge* e);

    void addConflictClauseNextTry(stack<Edge*> &eStack);

    void compareModels (z3::model &m1, z3::model &m2);

    map<Vertex*, z3::expr> timeVariables;

    map<pair<Vertex*, int>, z3::expr> booleanVariables;

    Utility::LatencyEstimation latEst;

    map<Vertex*, int>latestStartTimesWithOffset;

    map<Resource*, int>resourceLimits;

    static stack<Edge *> checkSchedule(Graph &g, ResourceModel &rm, map<Vertex *, int> &schedule, int II, bool quiet);

    void parseSMTModel();

    void printbooleanVeriables();
    void printSMTModel();

    void endTimeTracking() override;

  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTCDCLSCHEDULER_H
