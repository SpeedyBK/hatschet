//
// Created by bkessler on 5/7/22.
//

#ifndef HATSCHET_SMTMODSCHEDULER_H
#define HATSCHET_SMTMODSCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <vector>

namespace HatScheT {

  class SMTModScheduler : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    SMTModScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    /*!
     * \brief Attempts to schedule the given instances. The candidate II is incremented until a feasible schedule is found.
     */
    virtual void schedule();

    /*!
     * Handmade Schedule for development.
     */
    void handmadeSchedule();

  protected:
    /*!
     * not needed
     */
    virtual void setObjective(){}
    virtual void resetContainer(){}
    virtual void constructProblem() {/* unused */}
  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTMODSCHEDULER_H
