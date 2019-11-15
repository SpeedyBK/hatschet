//
// Created by sittel on 15/11/19.
//

#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <vector>

namespace HatScheT {

  /*!
   * @brief options for the used standard integer II scheduler
   */
  enum SchedulerType {MOOOVAC, MODULOSDC, ED97};

  class UnrollRationalIIScheduler : public SchedulerBase, public ILPSchedulerBase, public RationalIISchedulerLayer, public IterativeSchedulerBase {
  public:
    UnrollRationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

    /*!
    * the main function of the scheduler. The rational II scheduler tries to identify high throughput schedules on
    * the theoretical min II boundary. For this the variables s / m are used
     * Then the graph is unrolled s times and a standard integer II scheduler is chosen
     * default scheduler: EichenbergerDavidson97
    */
    virtual void schedule();

  private:
    SchedulerType scheduler;

    void unroll(int s);
  };
}


