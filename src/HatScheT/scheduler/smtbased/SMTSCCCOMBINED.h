//
// Created by bkessler on 9/1/22.
//

#ifndef HATSCHET_SMTSCCCOMBINED_H
#define HATSCHET_SMTSCCCOMBINED_H

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>

#include "SMTUnaryScheduler.h"
#include "SMTSCCScheduler.h"

namespace HatScheT{

class SMTSCCCOMBINED : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

public:

  SMTSCCCOMBINED (Graph &g, ResourceModel &resourceModel);

  void schedule() override;

  void setSolverTimeout(int seconds);

private:

  int timeLimit;

  };
}
#endif //USE_Z3

#endif //HATSCHET_SMTSCCCOMBINED_H
