//
// Created by nfiege on 2/15/23.
//

#ifndef HATSCHET_ILPSCHEDULELENGTHSWEEPER_H
#define HATSCHET_ILPSCHEDULELENGTHSWEEPER_H

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <HatScheT/utility/ILPScheduleLengthEstimation.h>

namespace HatScheT {
	class ILPScheduleLengthSweeper : public IterativeModuloSchedulerLayer {
	public:
		ILPScheduleLengthSweeper(Graph& graph, ResourceModel& resourceModel, const std::list<std::string> &solverWishlist, const std::string &resultFilePath);

	protected:
		void scheduleIteration() override;

	private:
		std::list<std::string> sw;
		std::string resultFilePath;
		ILPScheduleLengthEstimation estimator;
	};
}

#endif //HATSCHET_ILPSCHEDULELENGTHSWEEPER_H
