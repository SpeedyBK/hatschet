//
// Created by bkessler on 23/07/19.
//

//UNDER DEVELOPEMENT

#ifndef HATSCHET_GRAPHREDUCTION_H
#define HATSCHET_GRAPHREDUCTION_H

#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <vector>

namespace HatScheT {

  class GraphReduction : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
  public:
    GraphReduction(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

    /*!
     * main method to do the scheduling
     */
    virtual void schedule();


    /*!
     * not needed
     */
    virtual void setObjective() {/* unused */}
    virtual void resetContainer(){/* unused */}
    virtual void constructProblem() {/* unused */}
  private:

  };

}
#endif //HATSCHET_GRAPHREDUCTION_H
