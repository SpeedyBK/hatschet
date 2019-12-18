//
// Created by nfiege on 04/11/19.
//

#ifndef HATSCHET_SCCQSCHEDULER_H
#define HATSCHET_SCCQSCHEDULER_H


#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <HatScheT/utility/subgraphs/SCC.h>
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>
#include <vector>

namespace HatScheT {

	class SCCQScheduler : public RationalIISchedulerLayer, public ILPSchedulerBase {
	public:
		/*!
		 *
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		SCCQScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * To Be Updated: II has to be rational or a vector for this scheduler to work
		 * @return
		 */
		double getII() override { return this->II; }
		/*!
		 *
		 * @return initiation intervals for the latency sequence
		 */
		std::vector<int> getInitiationIntervals() const { return this->initiationIntervals; }
		/*!
		 * print start times
		 * @param startTimes
		 */
		static void printStartTimes(std::map<Vertex *, int> startTimes);

		/*!
		 * this scheduler does not use the ILP solver directly
		 * and it is a heuristic approach which usually does not find the minimum latency
		 * @return the unknown status
		 */
		ScaLP::status getScaLPStatus() override { return ScaLP::status::UNKNOWN; }

	protected:
		/*!
		 * each scheduler should overload this function for one schedule iteration
		 * M/S are already set automatically by RationalIISchedulerLayer::schedule
		 *
		 */
		void scheduleIteration() override;
		/*!
		 * not needed
		 */
		void constructProblem() override {}
		/*!
		 * not needed
		 */
		void resetContainer() override {}
		/*!
		 * not needed
		 */
		void setObjective() override {}

	private:
		/*!
		 * It is possible that the SCCs are scheduled in a weird way such that the first operation does not start at cycle 0
		 * => subtract from all control steps the minimum control step
		 */
		void optimizeStartTimes();
		/*!
		 * create scheduling queue based on topological sort of SCC graph
		 * @param sccs
		 * @param sccSchedule
		 * @return
		 */
		std::pair<std::list<int>,Graph*> getScheduleQueue(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule);
		/*!
		 * determine start times based on relative schedule of SCCs and DFG
		 * @param sccs
		 * @param sccSchedule
		 */
		void determineStartTimes(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule);
		/*!
		 * perform relative schedule on SCCs
		 * @param sccs
		 * @return
		 */
		std::pair<bool,std::map<Vertex *, pair<int, int>>> getSCCSchedule(std::vector<SCC *> &sccs);
		/*!
		 * fill MRT based on SCC schedule
		 * @param sccs
		 * @param sccSchedule
		 */
		void determineMRT(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule);
		/*!
		 * the minimum interger II that is possible
		 */
		int integerMinII;
		/*!
		 * initiation intervals for the found schedule
		 */
		std::vector<int> initiationIntervals;

		/*!
		 * complete modulo reservation table containing all samples
		 */
		ModuloQMRT mrt;
		/*!
		 * solver wishlist for ILP based ModuloQScheduler
		 */
		std::list<std::string> solverWishlist;
	};
}


#endif //HATSCHET_SCCQSCHEDULER_H
