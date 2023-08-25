//
// Created by nfiege on 7/14/23.
//

#ifndef HATSCHET_CLOCKGATINGMODULOSCHEDULER_H
#define HATSCHET_CLOCKGATINGMODULOSCHEDULER_H

#ifdef USE_SCALP

#include "HatScheT/base/ILPSchedulerBase.h"
#include "HatScheT/layers/IterativeModuloSchedulerLayer.h"

namespace HatScheT {
	class ClockGatingModuloScheduler : public IterativeModuloSchedulerLayer, public ILPSchedulerBase {
	public:
		ClockGatingModuloScheduler(Graph &g, ResourceModel &resourceModel, const std::list<std::string> &solverWishlist, double II = -1);

		/*!
		 * Mainly for debugging.
		 * @return Name of the scheduler
		 */
		string getName() override { return "CGMS"; }

		/*!
		 * what the name suggests...
		 * @param newNumClockDomains
		 */
		void setNumberOfClockDomains(const int &newNumClockDomains);
		/*!
		 * define the energy per sample for the given resource
		 * @param r
		 * @param newEnergyPerSample
		 */
		void setEnergyPerSample(const Resource* r, const double &newEnergyPerSample);
		/*!
		 * define the maximum allowed clock gating delay for the given resource
		 * @param r
		 * @param newMaxClockGatingDelay
		 */
		void setMaxClockGatingDelay(const Resource* r, const int &newMaxClockGatingDelay);
		/*!
		 * getter
		 */
		[[nodiscard]] std::map<int, std::vector<bool>> getClockOffTimes() const { return this->clockOffTimes; }
		/*!
		 * getter
		 */
		[[nodiscard]] std::map<std::pair<const Resource*, int>, int> getClockDomainBinding() const { return this->clockDomainBinding; }

	protected:
		/*!
		 * \brief Schedule Iteration for one II.
		 */
		void scheduleIteration() override;
		/*!
		 * Initialize stuff before II-Search-Loop starts.
		 */
		void scheduleInit() override;
		/*!
		 * tell the solver what to optimize
		 */
		void setObjective() override;
		/*!
		 * reset all relevant containers before solving
		 */
		void resetContainer() override;
		/*!
		 * call a few functions to set up the solving process
		 */
		void constructProblem() override;

	private:
		/*!
		 * map < clock domain -> vector <size II> in which modulo slots it is turned off >
		 */
		std::map<int, std::vector<bool>> clockOffTimes;
		/*!
		 * binding from an operator instance to its associated clock domain
		 */
		std::map<std::pair<const Resource*, int>, int> clockDomainBinding;
		/*!
		 * integer-valued candidate II for a solving attempt
		 */
		int candidateII = -1;
		/*!
		 * decision variables as named in the paper
		 */
		std::map<int, ScaLP::Variable> t;
		std::map<int, ScaLP::Variable> y;
		std::map<int, ScaLP::Variable> n;
		std::map<std::pair<int, int>, ScaLP::Variable> m;
		std::map<std::pair<int, int>, ScaLP::Variable> b;
		//std::map<std::pair<int, int>, ScaLP::Variable> k;
		std::map<std::pair<int, int>, ScaLP::Variable> sigma;
		std::map<std::pair<int, int>, ScaLP::Variable> psi;
		std::map<std::pair<int, int>, ScaLP::Variable> z;
		std::map<std::tuple<const Resource*, int, int>, ScaLP::Variable> f;
		std::map<std::tuple<const Resource*, int, int>, ScaLP::Variable> h;
		std::map<std::tuple<const Resource*, int, int, int>, ScaLP::Variable> alpha;
		/*!
		 * store the number of implemented functional units for each resource type here
		 * this number is equal to the number of operations of that type for unlimited resources
		 */
		std::map<const Resource*, int> functionalUnits;
		/*!
		 * store the maximum allowed delay due to clock gating for each resource
		 */
		std::map<const Resource*, int> maxClockGatingDelay;
		/*!
		 * store the energy per sample for each resource
		 */
		std::map<const Resource*, double> energyPerSample;
		/*!
		 * limit on the number of available clock domains
		 */
		int clockDomainLimit = 1;
		/*!
		 * init ILP solver
		 */
		void setUpSolverSettings();
		/*!
		 * fill all ILP variable containers
		 */
		void constructDecisionVariables();
		/*!
		 * put constraints into solver object
		 */
		void constructConstraints();
		/*!
		 * fill result containers once we found a solution
		 */
		void fillSolutionStructure(ScaLP::status stat);
	};
}
#endif //USE_SCALP
#endif //HATSCHET_CLOCKGATINGMODULOSCHEDULER_H
