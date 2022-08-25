//
// Created by bkessler on 8/25/22.
//

#ifndef HATSCHET_SMTSCHEDULEOPTIMIZER_H
#define HATSCHET_SMTSCHEDULEOPTIMIZER_H

#ifdef USE_Z3

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>

#include <z3++.h>

namespace HatScheT {

  class SMTScheduleOptimizer : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:

    SMTScheduleOptimizer(Graph &g, ResourceModel &resourceModel, map<Vertex*, int> &initialSchedule, double II);

    void schedule() override;

  private:

    void createTimeVariables();

    void createBooleanVariables();

    void addDependencyConstraints(z3::optimize& opt);

    Vertex* createSuperSink();

    map<Vertex*, int> initialSchedule;

    z3::context c;

    map<Vertex*, z3::expr> timeVariables;

    map<pair<Vertex*, int>, z3::expr> booleanVariables;

  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTSCHEDULEOPTIMIZER_H
