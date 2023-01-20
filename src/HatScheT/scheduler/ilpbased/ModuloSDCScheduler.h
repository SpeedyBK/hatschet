
//
// Created by nfiege on 22/11/18.
//
// 22/10/22 bkessler: Integration of IterativeModuloSchedulerLayer - Class

#ifndef HATSCHET_MODULOSDCSCHEDULER_H
#define HATSCHET_MODULOSDCSCHEDULER_H

#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>

#include <vector>
#include <string>
#include <chrono>

namespace HatScheT {
  /*!
   * @brief The TimeoutException class implements exceptions thrown by HatScheT
   */
  class TimeoutException : public std::exception {
  public:
    std::string msg;

    TimeoutException(std::string s) : msg(s) {
    }

    virtual const char *what() const noexcept override;
  };

  std::ostream &operator<<(std::ostream &oss, HatScheT::TimeoutException &e);

  class PriorityHandler {
  public:
    /*!
     * @brief this is used to determine the order of the schedule queue
     */
    enum priorityType {
      RANDOM, // random ordering
      ASAP, // based on inverse asap times
      ALAP, // based on inverse alap times
      PERTUBATION, // number of subsequent vertices (recommended in paper)
      MOBILITY_HIGH, // based on difference between alap and alap times; HIGH mobility vertices are scheduled FIRST
      MOBILITY_LOW, // based on difference between alap and alap times; LOW mobility vertices are scheduled FIRST
      MOBLAP, // combine ALAP and MOBILITY_LOW; LOW mobility vertices with LOW alap time are scheduled FIRST
      ALABILITY, // combine ALAP and MOBILITY_LOW; LOW alap time vertices with LOW mobility are scheduled FIRST
      SUBSEQUALAP, // combine the number of subsequent vertices with ALAP time
      ALASUB, // combine ALAP time with the number of subsequent vertices
      CUSTOM, // priority must be set manually (only feasible if HatScheT is used as library)
      NONE
    };

    inline static std::list<priorityType> getAllAutomaticPriorityTypes() {
      return {PERTUBATION, SUBSEQUALAP, ALASUB, ALABILITY, MOBLAP, ALAP, ASAP, MOBILITY_LOW, MOBILITY_HIGH, RANDOM};
    }

    static priorityType getPriorityTypeFromString(std::string priorityTypeStr);

    PriorityHandler(priorityType p, int prio1, int prio2 = 0);

    inline priorityType getPriorityType() const { return this->pType; }

    inline int getFirstPriority() const { return this->firstPriority; }

    inline int getSecondPriority() const { return this->secondPriority; }

    static void putIntoSchedQueue(Vertex *v, const priorityType &p, const map<Vertex *, PriorityHandler *> *pHandlers,
                                  std::list<Vertex *> *schedQ);

    static std::string getPriorityTypeAsString(const priorityType &p);

  protected:
    priorityType pType;
    int firstPriority;
    int secondPriority;
  };

  class ModuloSDCScheduler
    : public IterativeModuloSchedulerLayer, public ILPSchedulerBase {
  public:
    /*!
     * @brief ModuloSDCScheduler for scheduling in hatschet a graph (g), a resource model (rm) is needed
     * for solving the SDC problems, a solver wishlist (sw) to enable the ScaLP ILP backend is needed
     * (for example "Gurobi", "SCIP", "CPLEX")
     * @param g graph
     * @param rm resource model
     * @param sw solver wishlist
     */
    ModuloSDCScheduler(Graph &g, ResourceModel &rm, std::list<std::string> sw, int II=-1);

    /*!
     * @brief destructor
     */
    ~ModuloSDCScheduler() override;

    /*!
     * @brief schedule main method that runs the algorithm and determines a schedule
     */
    void scheduleOLD(); //ToDo: Remove!

    /*!
     * Mainly for debugging.
     * @return Name of the scheduler
     */
    string getName () override { return "ModSDCFiege"; }

    /*!
     * Function to set the solver Timeout
     * @param seconds
     */
    void setSolverTimeout(double timeoutInSeconds) override;

    /*!
     * @brief override SchedulerBase::getBindings()
     * @return
     */
    std::map<const Vertex *, int> getBindings() override;

    /*!
     * @brief calculate life times considering II instead of schedule length
     * @return
     */
    std::map<Edge *, int> getLifeTimes() override;

    /*!
     * the status of the ilp solver does not provide any information
     * about the quality of the solution, because many ilp problems are
     * solved for MRTs
     * @return the unknown status
     */
    ScaLP::status getScaLPStatus() override { return this->scalpStatus; }

    /*!
     * @brief defines a new budget
     * @param newBudget
     */
    void setBudgetMultiplier(unsigned int newBudget) { this->budgetMultiplier = newBudget; }

    /*!
     * @brief getNumberOfConstrainedVertices
     * @param g graph which contains vertices
     * @param rm resource model that specifies for each vertex if it's limited
     * @return the number of resource constrained vertices
     */
    static int getNumberOfConstrainedVertices(Graph &g, ResourceModel &rm);

    /*!
     * @brief
     * @return the total time spent in ilp solvers
     */
    double getTimeInILPSolvers() const { return this->timeInILPSolvers; }

    /*!
     * @brief set variable for this->fastObjective (see below for details)
     * @param b
     */
    void setFastObjective(bool b) { this->fastObjective = b; }

    /*!
     * @brief set priority type for scheduling queue ordering
     * @param p
     */
    void setPriorityType(PriorityHandler::priorityType p) { this->pType = p; }

    /*!
     * @brief use this function to set the priority for each vertex (if you want to set them all manually)
     * only use this if priorityType == CUSTOM, otherwise this has no effect
     * @param v
     * @param p
     */
    void setPriority(Vertex *v, PriorityHandler p);

    /*!
     * @brief this forces all vertices without outgoing edges on the same control step (which is equal to the schedule length)!
     */
    void setOutputsOnLatestControlStep();
    /*!
     * @brief get the number of failed schedule attempts due to empty budget
     * @return
     */
    int getEmptyBudgetCounter() const {
      return this->budgedEmptyCounter;
    }
    /*!
     * @brief get initial starting budget
     * @return
     */
    int getInitialBudged() const {
      return this->initialBudget;
    }

  protected:

    /*!
     * \brief Schedule Iteration for one II.
     */
    void scheduleIteration() override;
    /*!
     * Initialize stuff before II-Search-Loop starts.
     */
    void scheduleInit() override;

  private:
    /*!
     * status is timeout_infeasible for a timeout where no solution was found
     * in all other cases it is unknown
     */
    ScaLP::status scalpStatus;
    /*!
     * schedule length
     */
    int scheduleLength;
    /*!
     * @brief schedule times of all vertices without outgoing edges are equal to the schedule length
     */
    bool outputsEqualScheduleLength;
    /*!
     *
     */
    std::map<Vertex *, bool> vertexHasOutgoingEdges;

    /*!
     * @brief create a resource model with unlimited resources for each resource type
     * @return the address (don't forget to delete it when it's not needed anymore!)
     */
    ResourceModel *getUnlimitedResourceModel();

    /*!
     * @brief create asap schedule of this->g without resource constraints
     * @return map of start times
     */
    map<Vertex *, int> getASAPScheduleWithoutResourceConstraints();

    /*!
       * @brief create alap schedule of this->g without resource constraints
       * @return map of start times
       */
    map<Vertex *, int> getALAPScheduleWithoutResourceConstraints();

    /*!
     * @brief track time spent in ilp solvers
     */
    double timeInILPSolvers;

    /*!
     * @brief manages the time budget between solving ilps
     */
    bool manageTimeBudgetSuccess();

    /*!
     * @brief reset all timers etc
     */
    void handleTimeout();

    /*!
     * @brief is used to calculate this->solverTimeout
     */
    std::chrono::high_resolution_clock::time_point timeTracker;
    /*!
     * @brief keep track of the overall time budget, so the algorithm runs only as long as specified by the user
     */
    double timeBudget;
    /*!
     * @brief schedQueue from paper
     */
    std::list<Vertex *> schedQueue;
    /*!
     * @brief map to store priority for scheduling queue
     */
    map<Vertex *, PriorityHandler *> priorityForSchedQueue;

    /*!
     * @brief fill this->priorityForSchedQueue based on method specified in this->pType
     */
    void calculatePriorities();

    /*!
     * @brief see enum for details
     */
    PriorityHandler::priorityType pType;

    /*!
     * @brief get number of vertices which depend on the result of vertex 'v'
     * @param v
     * @return
     */
    int getNoOfSubsequentVertices(Vertex *v);

    /*!
     * @brief one iteration of Modulo SDC algorithm
     * @param II the II to try and find a scheduling for
     * @param budget the number of max iterations to try for this II
     * @param initSchedule asap schedule without resource constraints as initiation for this algorithm
     * @return a pointer to a scheduling map; nullptr: no scheduling found for given II
     */
    bool modSDCIteration(const int &II, int budget);

    /*!
     * @brief creates scheduling queue based on initial asap scheduling
     * @param scheduleMe asap scheduling
     */
    void createSchedulingQueue(std::list<Vertex *> scheduleMe);

    /*!
     * @brief getInitialSchedule
     * @return an asap schedule without resource constraints
     */
    void createInitialSchedule();

    /*!
     * @brief initialize solver (i.e. reset etc.)
     */
    void initSolver();

    /*!
     * @brief
     */
    void constructProblem() override;

    /*!
     * @brief setObjective
     */
    void setObjective() override;

    /*!
     * @brief instead of minimizing last start time, minimize sum of all start time
     * => objective function is more complex, but the number of constraints is nearly halfed!
     * 	=> that is faster but the latency is not always optimal!
     */
    bool fastObjective;
    /*!
     * @brief this is needed for this->setObjective()
     */
    string uniqueVariableName;

    /*!
     * @brief find a name for this->uniqueVariableName
     */
    void setUniqueVariableName();

    /*!
     * @brief the number of iterations per II
     */
    int budget;
    int initialBudget;
    int budgedEmptyCounter;

    /*!
     * @brief this sets a default budget according to metric in paper
     */
    void setDefaultBudget();

    /*!
     * @brief check if scheduling operation 'I' at time 't' has a resource conflict
     * @param I operation
     * @param t time
     * @return
     */
    bool hasResourceConflict(const Vertex *I, const int &t) const;

    /*!
     * @brief modulo reservation table; is only valid if modSDCIteration returned true
     */
    std::map<Vertex *, int> mrt;
    /*!
     * @brief equals SDC Times from paper
     */
    std::map<Vertex *, int> sdcTimes;
    /*!
     * @brief asap times for a schedule without resource constraints
     */
    std::map<Vertex *, int> asapTimes;
    /*!
     * @brief this map stores the scalp variables associated to each vertex inside the graph
     */
    std::map<Vertex *, ScaLP::Variable> scalpVariables;

    /*!
     * @brief search this->scalpVariables for sv and return the associated key
     * @param sv
     * @return
     */
    Vertex *getVertexFromVariable(const ScaLP::Variable &sv);

    /*!
     * @brief this fills scalpVariables according to input graph
     */
    void createScalpVariables();

    /*!
     * @brief this creates ILP constraints based on data dependency (end_i - start_j <= II * distance(i,j))
     */
    void createDataDependencyConstraints();

    /*!
     * @brief this creates all additional constraints stored in this->additionalConstraints
     * @brief v only create additional constraint for vertex v (create all if nullptr)
     */
    void createAdditionalConstraints();

    /*!
     * @brief additional constraints based on lines 7 and 10 (ModuloSDCScheduler algorithm)
     */
    std::map<Vertex *, ScaLP::Constraint *> additionalConstraints;

    /*!
     * @brief this basically just catches std::out_of_range errors when accessing additionalConstraints.at(v) to check if a constraint for the given vertex exists
     * @return the scalp constraint associated to vertex v
     */
    ScaLP::Constraint *getAdditionalConstraint(Vertex *v);

    /*!
     * @brief creates or overwrites an additional constraint (c) for a given vertexptr (v)
     * @param v
     * @param c
     */
    inline void createAdditionalConstraint(Vertex *v, ScaLP::Constraint &c);

    /*!
     * @brief this clears the constraints associated to the given vertexptr
     * @param v
     */
    void clearConstraintForVertex(Vertex *v);

    /*!
     * @brief this clears all additional constraints in this->additionalConstraints
     */
    void clearAllAdditionalConstraints();

    /*!
     * @brief this sets up scalp for the current II
     * @param v only respect additional constraint for this vertex (if nullptr, respect all additional constraints!)
     */
    void setUpScalp();

    /*!
     * @brief this does everything needed to solves the problem specified in this->solver
     * and stores results in this->sdcTimes
     * @return if the solver found a feasible solution
     */
    bool solveSDCProblem();

    /*!
     * @brief previously scheduled times (see paper for details)
     */
    std::map<Vertex *, int> prevSched;

    /*!
     * @brief get previously scheduled time for a given vertex; needed for backtracking
     * @param v vertex
     * @return -1 if prevSched has no entry for the given vertex
     */
    int getPrevSched(Vertex *v);

    /*!
     * @brief Backtracking algorithm from paper
     * @param I instruction
     * @param time time slot
     */
    void backtracking(Vertex *I, const int &time);

    /*!
     * @param I Instruction
     * @param evictTime
     * @return all instructions scheduled to the time slot evictTime mod this->II
     */
    std::list<Vertex *> getResourceConflicts(Vertex *I, const int &evictTime);

    /*!
     * @brief schedule instruction 'I' at time 't'
     * @param I
     * @param time
     */
    void scheduleInstruction(Vertex *I, int t);

    /*!
     * @brief remove instruction 'evictInst' from this->mrt, clear additional constraints for this vertex and add to schedule queue
     * @param evictInst
     */
    void unscheduleInstruction(Vertex *evictInst);

    /*!
     * @param I instruction
     * @param evictTime
     * @return true if scheduling instruction 'I' at time 'evictTime' creates data dependency conflict
     */
    bool dependencyConflict(Vertex *I, const int &evictTime);

    /*!
     * @param i start of an edge (j is the destination, that doesn't have to be passed)
     * @param newStartTime_i new start time of instruction i
     * @param newStartTime_j new start time of instruction j
     * @param edgeDelay delay on edge between operation i and j
     * @param distance on edge between operation i and j
     * @return end_i - start_j <= II * distance(i,j)
     */
    bool dependencyConflictForTwoInstructions(Vertex *i, const int &newStartTime_i, const int &newStartTime_j,
                                              const int &edgeDelay, const int &edgeDistance);

    /*!
     * @brief clear all unnecessary data between iterations (e.g. mrt, schedQueue, ...)
     */
    virtual void resetContainer();

    /*!
     * @brief print all additional constraints for solver
     */
    void printAdditionalSolverConstraints();

    /*!
     * @brief
     * @return all vertices with limit != -1
     */
    list<Vertex *> getResourceConstrainedVertices();

    /*!
     * @brief is used to determine budget per II
     */
    unsigned int budgetMultiplier;

    time_t t;

  };
}

#endif //HATSCHET_MODULOSDCSCHEDULER_H
