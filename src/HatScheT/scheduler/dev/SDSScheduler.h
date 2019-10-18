//
// Created by bkessler on 18/10/19.
//

#ifndef HATSCHET_SDSSCHEDULER_H
#define HATSCHET_SDSSCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>

#include "/opt/cadical/include/cadical.hpp"

namespace HatScheT {

  class SDSScheduler : public SchedulerBase, public ModuloSchedulerBase {

  public:

    /*!
		 * The SDSScheduler is a SDC and SAT based scheduler, it is based on the paper : "A Scalable Approach to Exact
     * Resource Constraint Scheduling Based on a Joint SDC and SAT Formulation" by Steve Dai, Gai Liu and Zhiru Zhang.
		 * @param g
		 * @param resourceModel
		 */
    SDSScheduler(Graph& g, ResourceModel &resourceModel);

    /*!
     * main method to do the scheduling
     */
    void schedule() override ;

    /*!
     * not needed
     */
    virtual void setObjective() {/* unused */}
    virtual void resetContainer(){/* unused */}
    virtual void constructProblem() {/* unused */}

  private:

  };


}

#endif //HATSCHET_SDSSCHEDULER_H
