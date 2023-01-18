//
// Created by bkessler on 9/1/22.
//
//ToDo: SCC-Template does the same, but better... Remove this shit...

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
/*!
 * Scheduler with two stages. At first SMTSCCScheduler is used to compute a Schedule for Complex SCCs who determine the
 * minimum feasible II. If this is found, then SMT-Unary is used to search for a better Latency
 * @param Graph g
 * @param resourceModel
 */
  SMTSCCCOMBINED (Graph &g, ResourceModel &resourceModel);

  /*!
   * Main Schedule Function
   */
  void schedule() override;

  /*!
   * Function to set a z3 solver timeout in seconds.
   * @param seconds
   */
  void setSolverTimeout(int seconds);

private:

  /*!
   * Variable that stores the available Time.
   */
  int timeLimit;

  };
}
#endif //USE_Z3

#endif //HATSCHET_SMTSCCCOMBINED_H
