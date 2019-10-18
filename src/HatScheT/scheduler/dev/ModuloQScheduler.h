//
// Created by nfiege on 17/10/19.
//

#ifndef HATSCHET_MODULOQSCHEDULER_H
#define HATSCHET_MODULOQSCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>

namespace HatScheT {
	class ModuloQScheduler : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase {
	public:
		/*!
		 *
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		ModuloQScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * To Be Updated: II has to be rational or a vector for this scheduler to work
		 * @return
		 */
		double getII() override { return this->II;}
		/*!
		 *
		 */
		void schedule() override;
	protected:
		/*!
		 * not needed
		 */
		void constructProblem() override {}
		void setObjective() override {}
		void resetContainer() override {}
	};
}



#endif //HATSCHET_MODULOQSCHEDULER_H
