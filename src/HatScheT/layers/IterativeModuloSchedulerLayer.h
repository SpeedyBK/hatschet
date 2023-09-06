//
// Created by bkessler on 10/12/22.
//

#ifndef HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H
#define HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H

#include <chrono>
#include "HatScheT/base/SchedulerBase.h"
#include "HatScheT/base/ModuloSchedulerBase.h"
#include "HatScheT/base/IterativeSchedulerBase.h"
#include <HatScheT/utility/ILPScheduleLengthEstimation.h>

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
     * enable/disable couts from layerclass
     * @param q
     */
    void setLayerQuiet(bool q) { this->layerQuiet = q; }

    void getDebugPrintouts();
		/*!
		 * setter for this->boundSL
		 * @param b new value
		 */
		void setBoundSL(bool b) { this->boundSL = b; }

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
	 * if a scheduler needs to setup stuff after iterative scheduling, than this function has to be overloaded.
	 * Per default it just does nothing.
	 */
	virtual void scheduleCleanup()
	{
		if (!this->quiet) { cout << "IterativeModuloSchedulerLayer: nothing to clean up..." << endl; }
	}

    /*!
     * \brief use this flag to disable the secondary objective (No Latency minimization)
     */
    bool disableSecObj;

    /*!
     * Surpresses couts from layerclass.
     */
    bool layerQuiet;
	/*!
	 * calculate bounds for the schedule length if this is set
	 * -> this might help the schedulers in their search procedure and (massively) speed up scheduling
	 */
	bool boundSL = false;
	/*!
	 * a value for the optimal schedule length (SL) with minSL <= SL
	 */
	int minSL = -1;
	/*!
	 * a value for the optimal schedule length (SL) with maxSL >= SL
	 */
	int maxSL = -1;
	/*!
	 * object to estimate the schedule length
	 */
	std::unique_ptr<ILPScheduleLengthEstimation> scheduleLengthEstimation;
	/*!
	 * store earliest start times based on the min SL estimation
	 */
	std::map<Vertex*, int> earliestStartTimes;

	private:
	/*!
	 * Calculate upper/lower bounds for the schedule length if wanted by the user
	 */
	void calculateScheduleLengthEstimation();
  };

}
#endif //HATSCHET_ITERATIVEMODULOSCHEDULERLAYER_H
