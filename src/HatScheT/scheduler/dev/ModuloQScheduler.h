//
// Created by nfiege on 17/10/19.
//

#ifndef HATSCHET_MODULOQSCHEDULER_H
#define HATSCHET_MODULOQSCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/utility/subgraphs/SCC.h>
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
		 * Main schedule function
		 */
		void schedule() override;
		/*!
		 * Calculate all possible latency sequences for the given M and S values
		 * @param M
		 * @param S
		 * @return
		 */
		static std::vector<std::vector<unsigned int>> getAllLatencySequences(int M, int S);
	protected:
		/*!
		 * not needed
		 */
		void constructProblem() override {}
		void setObjective() override {}
		void resetContainer() override {}

	private:
		/*!
		 * latency sequence found by ratII scheduler when scheduling SCCs
		 */
		std::vector<int> latencySequence;
		/*!
		 * perform relative schedule of SCCs
		 * @return map from Vertex* of the original graph to pair <relative start time, scc-ID>
		 * (all SCCs must be offset by the same number of cycles)
		 */
		std::map<Vertex*, pair<int,int>> getSCCSchedule(std::vector<SCC*> &sccs);
		/*!
		 * the minimum interger II that is possible
		 */
		int integerMinII;
		/*!
 		* @brief the s value for the iteration start
 		*/
		int S;
		/*!
		 * @brief the m value for the iteration start
		 */
		int M;
		/*!
		 * solver wishlist needed for rat II scheduler to schedule SCCs
		 */
		std::list<std::string> solverWishlist;
	};
}



#endif //HATSCHET_MODULOQSCHEDULER_H
