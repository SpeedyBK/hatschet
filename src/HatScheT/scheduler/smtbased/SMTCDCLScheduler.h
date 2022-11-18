//
// Created by bkessler on 8/25/22.
//

#ifndef HATSCHET_SMTCDCLSCHEDULER_H
#define HATSCHET_SMTCDCLSCHEDULER_H

#ifdef USE_Z3

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>

#include <z3++.h>

namespace HatScheT {

  class SMTCDCLScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:

    SMTCDCLScheduler(Graph &g, ResourceModel &resourceModel);

    void schedule() override;

  private:

    void modifyResourceModel();
    void resetResourceModel();
    void setInitialStartTimes();

    void calculateStartimes(int candidateLatency);
    void reduceLatency(z3::solver &s);
    void createBooleanVariables();
    void addOneSlotConstraintToSolver(z3::solver &s);
    void addResourceContraintsToSolver(z3::solver &s);

    int getScheduleLatency (map<Vertex*, int> &sched);

    void fixDependencyConstraints(stack<Edge*> &violatedEdges, z3::solver &s);

    void addConflictClause(Edge* e, z3::solver &s);

    void addConflictClauseNextTry(stack<Edge*> &eStack, z3::solver &s);

    void compareModels (z3::model &m1, z3::model &m2);


    z3::context c;

    map<Vertex*, z3::expr> timeVariables;

    map<pair<Vertex*, int>, z3::expr> booleanVariables;

    Utility::LatencyEstimation latEst;

    map<Vertex*, int>latestStartTimesWithOffset;

    map<Resource*, int>resourceLimits;

    int32_t timeBudget;
    int32_t timeLimit;

    void setSolverTimeout(unsigned int seconds);

    static stack<Edge *> checkSchedule(Graph &g, ResourceModel &rm, map<Vertex *, int> &schedule, int II, bool quiet);

    void parseSMTModel(z3::model &m);

    void printbooleanVeriables();
    void printSMTModel(z3::model &m);
  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTCDCLSCHEDULER_H
