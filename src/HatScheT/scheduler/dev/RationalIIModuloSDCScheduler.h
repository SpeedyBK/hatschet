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
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>
#include <HatScheT/scheduler/dev/ModSDC.h>

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
		/*!
		 * destructor
		 */
		~RationalIIModuloSDCScheduler() override;

	protected:
		/*!
     * @brief schedule instruction 'I' at time 't'
     * @param I
     * @param time
     */
		void scheduleInstruction(Vertex *I, int t);
		/*!
     * @brief check if scheduling operation 'I' at time 't' has a resource conflict
     * @param I operation
     * @param t time
     * @return
     */
		bool hasResourceConflict(Vertex *I, int t);
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

	private:
		/*!
     * @param i start of an edge (j is the destination, that doesn't have to be passed)
     * @param newStartTime_i new start time of instruction i
     * @param newStartTime_j new start time of instruction j
     * @param edgeDelay delay on edge between operation i and j
     * @param distance on edge between operation i and j
     * @return end_i - start_j <= II * distance(i,j)
     */
		bool dependencyConflictForTwoInstructions(Vertex *i, int newStartTime_i, int newStartTime_j,
																							int edgeDelay, int edgeDistance);
		/*!
     * @param I instruction
     * @param evictTime
     * @return true if scheduling instruction 'I' at time 'evictTime' creates data dependency conflict
     */
		bool dependencyConflict(Vertex *I, int evictTime);
		/*!
     * @brief remove instruction 'evictInst' from this->mrt, clear additional constraints for this vertex and add to schedule queue
     * @param evictInst
     */
		void unscheduleInstruction(Vertex *evictInst);
		/*!
     * @param I Instruction
     * @param evictTime
     * @return all instructions scheduled to the time slot evictTime mod this->II
     */
		std::list<Vertex *> getResourceConflicts(Vertex *I, int evictTime);
		/*!
     * @brief get previously scheduled time for a given vertex; needed for backtracking
     * @param v vertex
     * @return -1 if prevSched has no entry for the given vertex
     */
		int getPrevSched(Vertex *v);
		/*!
		 * do everything needed when a scheduling attempt fails
		 */
		void failSchedulingAttempt();
		/*!
		 * @brief this does everything needed to solves the problem specified in this->solver
		 * and stores results in this->sdcTimes
		 * @return if the solver found a feasible solution
		 */
		bool solveSDCProblem();
		/*!
		 * @brief Backtracking algorithm from paper
		 * @param I instruction
		 * @param time time slot
		 */
		void backtracking(Vertex *I, int time, bool mrtBacktracking = false);
		/*!
		 * @param I instruction to be scheduled
		 * @param time control step in which it can NOT be scheduled
		 * @return offset, where time+offset is the next control step in which 'I' can be scheduled
		 */
		int getNextFreeTimeSlot(Vertex* I, int time);
		/*!
     * @brief previously scheduled times (see paper for details)
     */
		std::map<Vertex *, int> prevSched;
		/*!
		 * @brief schedQueue from paper
		 */
		std::list<Vertex *> schedQueue;
		/*!
		 * @brief creates scheduling queue based on perturbation and ALAP times
		 * @param scheduleMe asap scheduling
		 */
		void createSchedulingQueue(std::list<Vertex *> scheduleMe);
		/*!
		 * @return all vertices with limit != -1
		 */
		list<Vertex *> getResourceConstrainedVertices();
		/*!
		 * @brief manages the time budget between solving ilps
		 */
		bool manageTimeBudgetSuccess();
		/*!
		 * @brief reset all timers etc
		 */
		void handleTimeout();
		/*!
		 * @brief additional constraints based on lines 7 and 10 (ModSDC algorithm)
		 */
		std::map<Vertex *, ScaLP::Constraint *> additionalConstraints;
		/*!
		 * @brief this basically just catches std::out_of_range errors when accessing additionalConstraints.at(v) to check if a constraint for the given vertex exists
		 * @return the scalp constraint associated to vertex v
		 */
		ScaLP::Constraint *getAdditionalConstraint(Vertex *v);
		/*!
		 * @brief this clears the constraints associated to the given vertexptr
		 * @param v
		 */
		void clearConstraintForVertex(Vertex *v);
		/*!
		 * clear additional constraints of all vertices
		 */
		void clearAllAdditionalConstraints();
		/*!
		 * maximum number of allowed algorithm iterations
		 */
		int budget;
		/*!
		 * @brief is used to calculate this->timeBudget
		 */
		std::chrono::high_resolution_clock::time_point timeTracker;
		/*!
		 * @brief keep track of the overall time budget, so the algorithm runs only as long as specified by the user
		 */
		double timeBudget;
		/*!
		 * @brief initialize solver (i.e. reset etc.)
		 */
		void initSolver();
		/*!
		 * @brief this sets up scalp for the current II
		 */
		void setUpScalp();
		/*!
		 * @brief equals SDC Times from paper
		 */
		std::map<Vertex *, int> sdcTimes;
		/*!
		 * @brief asap times for a schedule without resource constraints
		 */
		std::map<Vertex *, int> asapTimes;
		/*!
		 * @brief getInitialSchedule
		 * @return an asap schedule without resource constraints
		 */
		void createInitialSchedule();
		/*!
		 * @brief get number of vertices which depend on the result of vertex 'v'
		 * @param v
		 * @return
		 */
		int getNoOfSubsequentVertices(Vertex *v);
		/*!
		 * @brief map to store priority for scheduling queue
		 */
		map<Vertex *, PriorityHandler *> priorityForSchedQueue;

		/*!
		 * @return resource model in which every resource is unlimited
		 */
		ResourceModel *getUnlimitedResourceModel();
		/*!
		 * @return ALAP schedule without resource constraints
		 */
		map<Vertex *, int> getALAPScheduleWithoutResourceConstraints();
		/*!
		 * @return ASAP schedule without resource constraints
		 */
		map<Vertex *, int> getASAPScheduleWithoutResourceConstraints();
		/*!
		 * calculate priority metric for each vertex in the input graph
		 */
		void calculatePriorities();
		/*!
		 * construct ILP time variables of the operations in the input graph
		 */
		void fillTContainer();
		/*!
		 * minimum deltas according to the latency sequence
		 */
		std::map<unsigned int,unsigned int> deltaMins;
		/*!
		 * for each distance in the graph, calculate the minimum delta according to the latency sequence
		 */
		void calcDeltaMins();
		/*!
		 * modulo reservation table
		 */
		ModuloQMRT mrt;
		/*!
		 * initiation intervals for the found schedule
		 */
		std::vector<int> initiationIntervals;
		/*!
		 * container for ILP time variables for all operations
		 */
		std::map<Vertex*,ScaLP::Variable> tVariables;
	};
}

#endif //HATSCHET_RATIONALIIMODULOSDCSCHEDULER_H
