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
	class ModuloQMRT {
	public:
		/*!
		 * default constructor - should not be used
		 */
		ModuloQMRT();
		/*!
		 *
		 * @param rm resource model
		 * @param II number of columns
		 */
		ModuloQMRT(ResourceModel &rm, unsigned int II);
		/*!
		 * insert a vertex into this MRT
		 * @param v address of the vertex
		 * @param moduloSlot column where it should be inserted
		 * @return if it was successfully inserted, i.e. MRT is full at this slot => return false
		 */
		bool insertVertex(Vertex* v, unsigned int moduloSlot);
		/*!
		 * remove all instances of this vertex from MRT
		 * @param v
		 * @return if it was successfully removed
		 */
		bool removeVertex(Vertex* v);
		/*!
		 * print contents of MRT
		 */
		void print();
		/*!
		 *
		 * @param v vertex
		 * @return all modulo slots where v appears
		 */
		std::vector<int> getModuloSlots(Vertex* v);

	private:
		/*!
		 * this is where everything is stored
		 */
		std::map<const Resource*,std::vector<std::vector<Vertex*>>> mrt;
		/*!
		 * resource model for this MRT
		 */
		ResourceModel* rm;
		/*!
		 * number of columns
		 */
		unsigned int II;
	};


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
		 * @return S and M (samples and modulo)
		 */
		std::pair<int,int> getSM();
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
		/*!
		 *
		 * @return initiation intervals
		 */
		std::vector<int> getInitiationIntervals() const { return this->initiationIntervals; }
		/*!
		 *
		 * @return latency sequence
		 */
		std::vector<int> getLatencySequence() const { return this->latencySequence; }
		/*!
		 * streams start times to cout
		 * @param startTimes
		 */
		static void printStartTimes(std::map<Vertex*,int> startTimes);
		/*!
		 *
		 * @return rat II start times
		 */
		std::vector<std::map<Vertex*,int> >& getStartTimeVector() { return this->startTimesVector; }
		/*!
   * dont use this function fo rational II modulo schedules
   * @return
   */
		std::map<const Vertex*,int> getBindings() override { throw HatScheT::Exception("Use getRationalIIBindings for rational II schedulers"); }
		/*!
		 * generate a binding using the determined rational II schedule
		 * TODO figure out the best binding method (ILP?)
		 * @return
		 */
		vector<std::map<const Vertex*,int> > getRationalIIBindings() { throw HatScheT::Exception("getRationalIIBindings not yet supported"); }
	protected:
		/*!
		 * not needed
		 */
		void constructProblem() override {}
		/*!
		 * not needed
		 */
		void setObjective() override {}
		/*!
		 * not needed
		 */
		void resetContainer() override {}

	private:
		/*!
		 * get scheduling queue based on topological sort of the DFG with SCCs
		 * @param sccSchedule
		 * @return pair of {queue = vector of SCC-IDs; graph used to generate queue}
		 */
		std::pair<std::list<int>,Graph*> getScheduleQueue(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule);
		/*!
		 * fill start times vector based on MRT and DFG
		 * @param sccSchedule see getSCCSchedule
		 */
		void determineStartTimes(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule);
		/*!
		 * determine MRT based on the scc schedule
		 * @param sccs see getSCCSchedule
		 * @param sccSchedule see getSCCSchedule
		 */
		void determineMRT(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule);
		/*!
		 * perform relative schedule of SCCs
		 * @return map from Vertex* of the original graph to pair <relative start time, scc-ID>
		 * (all SCCs must be offset by the same number of cycles)
		 */
		std::map<Vertex*, pair<int,int>> getSCCSchedule(std::vector<SCC*> &sccs);
		/*!
		 * latency sequence found by ratII scheduler when scheduling SCCs
		 */
		std::vector<int> latencySequence;
		/*!
		 * initiation intervals resulting from latency sequence
		 */
		std::vector<int> initiationIntervals;
		/*!
		 * the final rational II schedule
		 */
		vector<std::map<Vertex*,int> > startTimesVector;
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
		/*!
		 * the number of operations in the DFG of the resource type
		 */
		std::map<const Resource*,int> numberOperations;
		/*!
		 * complete modulo reservation table containing all samples
		 */
		ModuloQMRT completeMrt;
		/*!
		 * MRT only for the first sample
		 */
		ModuloQMRT firstSampleMrt;
	};
}



#endif //HATSCHET_MODULOQSCHEDULER_H
