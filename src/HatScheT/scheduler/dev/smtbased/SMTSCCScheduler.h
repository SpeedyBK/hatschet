//
// Created by bkessler on 8/11/22.
//

#ifndef HATSCHET_SMTSCCSCHEDULER_H
#define HATSCHET_SMTSCCSCHEDULER_H

#pragma once
#ifdef USE_Z3

#include <iostream>

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include "HatScheT/utility/subgraphs/SCC.h"

namespace HatScheT {

  class SMTSCCScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:

    SMTSCCScheduler(Graph &g, ResourceModel &resourceModel);

    void schedule() override;

  private:

    struct cmp {
      bool operator() (const SCC* x, const SCC* y) const {
          return *x < *y;
      }
    };

    void computeSCCs();

    set<SCC*, cmp> trivialSCCs;

    set<SCC*, cmp> basicSCCs;

    set<SCC*, cmp> complexSCCs;

  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTSCCSCHEDULER_H
