//
// Created by bkessler on 10/12/22.
//

#ifndef HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H
#define HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H

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
     * enable/disable objective function
     * @param d
     */
    void disableObjective(bool d) { this->disableObj = d; }

    /*!
     * Getter for solvingtime of the current iteration.
     * @return Solvingtime needed for the current iteration.
     */
    int getIterationSolvingTime() const { return solvingTimePerIteration; }

    /*!
     * Sets the amount of time which is available for each iteration.
     * Default is INT_MAX/2 seconds which is about 34 Years.
     * @param seconds
     */
    void setTimeBudget (int seconds) { this->timeBudget = seconds; }

  protected:
   /*!
    * each scheduler should overload this function for one schedule iteration
    * II is already set automatically by this class
    */
    virtual void scheduleIteration() {
        throw HatScheT::Exception("IterativeModuloSchedulerLayer::scheduleIteration should never be called!");
    }

    /*!
     * \brief use this flag to disable the minimizeObjective
     */
    bool disableObj;

    /*!
     * \brief Time available for an iteration in seconds
     */
    int timeBudget;

    /*!
     * \brief Time, that one schedule iteration took
     */
    int solvingTimePerIteration;

  };

}
#endif //HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H
