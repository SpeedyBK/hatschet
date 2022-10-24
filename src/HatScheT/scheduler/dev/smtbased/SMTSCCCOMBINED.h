//
// Created by bkessler on 9/1/22.
//

#ifndef HATSCHET_SMTSCCCOMBINED_H
#define HATSCHET_SMTSCCCOMBINED_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>

#include "HatScheT/scheduler/dev/smtbased/SMTBinaryScheduler.h"
#include "HatScheT/scheduler/dev/smtbased/SMTSCCScheduler.h"

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

#endif //HATSCHET_SMTSCCCOMBINED_H
