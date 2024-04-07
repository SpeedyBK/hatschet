//
// Created by bkessler on 4/1/24.
//

#ifndef HATSCHET_SMTSIMPLESCHEDULER_H
#define HATSCHET_SMTSIMPLESCHEDULER_H

#include "HatScheT/layers/IterativeModuloSchedulerLayer.h"
#include "HatScheT/base/Z3SchedulerBase.h"

namespace HatScheT {

  class SMTSimpleScheduler : public IterativeModuloSchedulerLayer, public Z3SchedulerBase {

  public:

    SMTSimpleScheduler(Graph &g, ResourceModel &resourceModel, int II=-1);

    void setSolverTimeout (double seconds) override;

  private:

    void scheduleInit() override;

    void scheduleIteration() override;

    void generateTVariables();

    void generateBVariables();

    z3::expr *getBvariable(Vertex *v, int i);

    int getLatestStarttime();

    z3::expr* getTVariable(Vertex* vPtr);

    z3::check_result addNonNegativeConstraints();

    z3::check_result addDependencyConstraints();

    z3::check_result addVariableConnections();

    z3::check_result addResourceConstraints(int candidateII);

    z3::check_result addMaxLatencyConstraint();

    void z3CheckWithTimeTracking();

    map<Vertex*, z3::expr> tVariables;

    map<std::pair<Vertex*, int>, z3::expr> bVariables;

    int actualLength;

    int lastLength;

  };

}


#endif //HATSCHET_SMTSIMPLESCHEDULER_H
