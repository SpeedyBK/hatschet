//
// Created by bkessler on 10/12/22.
//

#ifndef HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H
#define HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H

#include <chrono>
#include "HatScheT/base/SchedulerBase.h"
#include "HatScheT/base/ModuloSchedulerBase.h"
#include "HatScheT/base/IterativeSchedulerBase.h"

namespace HatScheT {
  class IterativeModuloSchedulerLayer : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:

    IterativeModuloSchedulerLayer(Graph &g, ResourceModel &resourceModel, double II = -1);

    /*!
     * schedule function - since the core of this function is the same for all iterative modulo schedulers,
     * this is only implemented once! All schedulers must only overload scheduleIteration,
     * where the loop body is implemented
     */
    void schedule() final;

    /*!
     * enable/disable the secondary objective (Latency minimization)
     * @param d
     */
    void disableSecObjective(bool d) { this->disableSecObj = d; }

    /*!
     * Mainly for debugging.
     * @return Name of the scheduler
     */
    virtual string getName() { return "Unnamed Scheduler"; }

  protected:
   /*!
    * each scheduler should overload this function for one schedule iteration
    * II is already set automatically by this class
    */
    virtual void scheduleIteration()
    {
        throw HatScheT::Exception("IterativeModuloSchedulerLayer::scheduleIteration should never be called!");
    }

    /*!
     * if a scheduler needs to setup stuff before iterative scheduling, than this function has to be overloaded.
     * Per default it just does nothing.
     */
    virtual void scheduleInit()
    {
        if (!this->quiet) { cout << "IterativeModuloSchedulerLayer: nothing to init..." << endl; }
    }

    /*!
     * \brief use this flag to disable the secondary objective (No Latency minimization)
     */
    bool disableSecObj;
  };

}
#endif //HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H
