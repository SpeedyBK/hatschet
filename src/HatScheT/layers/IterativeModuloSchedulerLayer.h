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
     * Sets the amount of time which is available for each iteration.
     * Default is INT_MAX/2 seconds which is about 34 Years.
     * @param seconds
     */
    void setTimeBudget (double seconds) { this->timeBudget = seconds; }

    /*!
     * Getter for the time which was spent in solvers during the latest iteration.
     * @return Time spent in Solvers
     */
    double getTimeUsed() const { return this->timeUsed; }

    /*!
     * Getter for the
     * @return
     */
    double getTimeRemaining () const { return this->timeBudget - this->timeUsed; }

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

    /*!
     * Starts the time messurement, should be called directly before solver->solve() function.
     */
    void startTimeTracking();

    /*!
     * End of time messurement, should be called directly after solver->solve() function.
     * It calculates the results and stores them in "timeRemaining" and "timeUsed".
     */
    void endTimeTracking();

    /*!
     * Starttime of Timemessurement
     */
    std::chrono::high_resolution_clock::time_point start_t;

    /*!
     * Endtime of Timemessurement
     */
    std::chrono::high_resolution_clock::time_point end_t;

    /*!
     * \brief Time available for an iteration in seconds
     */
    double timeBudget;

    /*!
     * Time spent in solvers during the latest schedule Iteration
     */
    double timeUsed;

    /*!
     * timeBudget - timeUsed
     */
    double timeRemaining;
  };

}
#endif //HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H
