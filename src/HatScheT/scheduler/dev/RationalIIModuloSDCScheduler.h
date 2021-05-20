//
// Created by nfiege on 17/02/21.
//

#ifndef HATSCHET_RATIONALIIMODULOSDCSCHEDULER_H
#define HATSCHET_RATIONALIIMODULOSDCSCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>

namespace HatScheT {
	class RationalIIModuloSDCScheduler : public RationalIISchedulerLayer, public ILPSchedulerBase {
	public:
		/*!
		 * constructor
		 * @param g graph
		 * @param resourceModel
		 * @param solverWishlist
		 */
		RationalIIModuloSDCScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

	protected:
		/*!
		 * each scheduler should overload this function for one schedule iteration
		 * M/S are already set automatically by RationalIISchedulerLayer::schedule
		 *
		 */
		void scheduleIteration() override;
		/*!
		 * constructProblem Using the graph, resource model, an II and solver settings, the problem is constructed
		 */
		void constructProblem() override;
		/*!
		 * setObjective currently asap
		 */
		void setObjective() override;
		/*!
		 * reset containers
		 */
		void resetContainer() override;
	};
}

#endif //HATSCHET_RATIONALIIMODULOSDCSCHEDULER_H
