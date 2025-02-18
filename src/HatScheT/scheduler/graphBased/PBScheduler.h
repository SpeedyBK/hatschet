//
// Created by nfiege on 7/9/21.
//

#ifndef HATSCHET_PBSCHEDULER_H
#define HATSCHET_PBSCHEDULER_H


#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <vector>
#include <HatScheT/utility/subgraphs/SCC.h>

/*!
 * ATTENTION: This is still a work-in-progress. Do not use this class, yet!
 */

namespace HatScheT {
	class PBScheduler : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
	public:
		/*!
		 * Partitioning-based scheduler that attempts to partition the graph and the mrts
		 * and then schedules them individually
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		PBScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, int II=-1);
		/*!
		 * \brief Attempts to schedule the given instances
		 */
		void schedule() override;
		/*!
		 * desired maximum subgraph size
		 * this size is not exceeded by the partitioning algorithm if possible (long recurrences may break this rule!)
		 * if it is set to 0 (or any negative value) the algorithm tries to generate
		 * apporiximately sqrt(|V|) subgraphs with approximately sqrt(|V|) vertices
		 */
		int maximalSubgraphSize = 15;
		/*!
         * Function to set the solver Timeout
         * @param seconds
         */
      void setSolverTimeout(double timeoutInSeconds) override;

	protected:
		/*!
		 * unused
		 */
		void constructProblem() override {}
		/*!
		 * unused
		 */
		void setObjective() override {}
		/*!
		 * unused
		 */
		void resetContainer() override {}

	private:
		/*!
		 * solver wishlist for backbone scheduler
		 */
		std::list<std::string> sw;
		/*!
		 * free memory after scheduling attempt
		 */
		void freeMemory();
		/*!
		 * tries to find a schedule for the candidate II
		 * \param candidateII
		 */
		void scheduleAttempt(int candidateII);
		/*!
		 * this function uses a utility function to partition the graph into sccs to remove recurrences
		 */
		void createSCCs();
		/*!
		 * sccs are ordered based on a topological sort to determine which ones can be combined into subgraphs
		 */
		void orderSCCs();
		/*!
		 * sccs are merged into subgraphs
		 * \param candidateII
		 */
		void partitionSCCGraph(int candidateII);
		/*!
		 * assign (enough) resources to all subgraphs
		 * \param candidateII
		 */
		void partitionMRTs(int candidateII);
		/*!
		 * subgraphs are scheduled individually for a relative schedule
		 * \param candidateII
		 */
		void scheduleSubgraphs(int candidateII);
		/*!
		 * relative schedule are offset by n*II such that all dependency constraints hold
		 * \param candidateII
		 */
		void sortSubgraphs(int candidateII);
		/*!
		 * post-processing step to further optimize the schedule
		 */
		void postProcessSchedule();
		/*!
		 * container to hold relative schedules for all subgraphs
		 */
		std::map<Graph*, std::map<Vertex*, int>> subgraphSchedule;
		/*!
		 * all subgraphs
		 */
		std::vector<Graph*> subgraphs;
		/*!
		 * a resource model for each subgraph
		 */
		std::map<Graph*, ResourceModel*> resourceModels;
		/*!
		 * a map from original vertex to the subgraph it belongs to
		 */
		std::map<Vertex*, Graph*> vertexToSubgraphMap;
		/*!
		 * a map from original vertex to vertex in subgraph
		 */
		std::map<Vertex*, Vertex*> vertexToSubgraphVertexMap;
		/*!
		 * a map from vertex in subgraph to original vertex
		 */
		std::map<Vertex*, Vertex*> subgraphVertexToVertexMap;
		/*!
		 * topologically sorted SCCs
		 */
		std::vector<std::vector<SCC*>> topoSortedSCCs;
		/*!
		 * (unsorted) container of SCCs
		 */
		std::vector<SCC*> sccs;

	};
}


#endif //HATSCHET_PBSCHEDULER_H
