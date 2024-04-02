//
// Created by bkessler on 4/1/24.
//

#ifndef HATSCHET_SMTMODINCREMENTALSCHEDULER_H
#define HATSCHET_SMTMODINCREMENTALSCHEDULER_H

#include "HatScheT/layers/IterativeModuloSchedulerLayer.h"
#include "HatScheT/base/Z3SchedulerBase.h"

namespace HatScheT {

  class SMTMODIncrementalScheduler : public IterativeModuloSchedulerLayer, public Z3SchedulerBase {

  public:

    SMTMODIncrementalScheduler(Graph &g, ResourceModel &resourceModel, int II=-1);

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

    map<Vertex*, z3::expr> tVariables;

    map<std::pair<Vertex*, int>, z3::expr> bVariables;

    int lastLength;

    int actualLength;

  };

}


#endif //HATSCHET_SMTMODINCREMENTALSCHEDULER_H
