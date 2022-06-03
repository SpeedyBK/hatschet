#ifndef HATSCHET_SMTSMARTIESCHEDULER_H
#define HATSCHET_SMTSMARTIESCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <utility>

#include <z3++.h>

namespace HatScheT {

  class SMTSmartieScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    SMTSmartieScheduler(Graph &g, ResourceModel &resourceModel);

    void schedule() override;

  protected:

    z3::context c;

    map<std::pair<Vertex*, int>, z3::expr> b_variables;

    z3::expr* get_b_variable(Vertex* v, int i);

    void generate_b_variables();

    void set_b_variables();

  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTSMARTIESCHEDULER_H
