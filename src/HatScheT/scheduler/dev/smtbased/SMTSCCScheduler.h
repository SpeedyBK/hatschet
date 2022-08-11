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

namespace HatScheT {

  class SMTSCCScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:

    SMTSCCScheduler(Graph &g, ResourceModel &resourceModel);

    void schedule() override;

  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTSCCSCHEDULER_H
