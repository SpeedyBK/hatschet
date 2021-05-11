//
// Created by nfiege on 29/04/21.
//

#ifndef HATSCHET_COMBINEDRATIONALIISCHEDULER_H
#define HATSCHET_COMBINEDRATIONALIISCHEDULER_H

#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <ctime>

namespace HatScheT {
	class CombinedRationalIIScheduler : public RationalIISchedulerLayer, public ILPSchedulerBase {
	public:
		CombinedRationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

	protected:
		virtual void constructProblem() override {};
		virtual void setObjective() override {};
		virtual void resetContainer() override {};
		/*!
		 * each scheduler should overload this function for one schedule iteration
		 * M/S are already set automatically by RationalIISchedulerLayer::schedule
		 *
		 */
		void scheduleIteration() override;

	private:
		std::list<std::string> solverWishlist;
	};
}


#endif //HATSCHET_COMBINEDRATIONALIISCHEDULER_H
