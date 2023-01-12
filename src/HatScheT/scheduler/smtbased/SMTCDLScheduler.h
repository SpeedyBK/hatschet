//
// Created by bkessler on 8/25/22.
//

#ifndef HATSCHET_SMTCDLSCHEDULER_H
#define HATSCHET_SMTCDLSCHEDULER_H

#ifdef USE_Z3

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <HatScheT/base/Z3SchedulerBase.h>

namespace HatScheT {

  class SMTCDLScheduler : public IterativeModuloSchedulerLayer, public Z3SchedulerBase {

  public:
    /*!
     * @brief smtbased-Scheduler for scheduling in hatschet a graph (g), a resource model (rm). Uses Z3-Theorem-Prover as
     * backend.
     * @param g graph
     * @param rm resource model
     * @param II given II
     */
    SMTCDLScheduler(Graph &g, ResourceModel &resourceModel, double II = -1);
    /*!
     * Sets a timeout for Z3 for each check.
     * @param seconds
     */
    void setSolverTimeout(double seconds) override;
    /*!
     * Obvious
     */
    string getName() override { return "SMT-CDCL-Scheduler"; }

  private:
    /*!
     * Initialize stuff before II-Search-Loop starts.
     */
    void scheduleInit() override;
    /*!
     * \brief Schedule Iteration for one II.
     */
    void scheduleIteration() override;
    /*!
     * Function to modify Resource Limits, sets Ressources with FUs(r) > Operations O(r) to unlimited.
     */
    void modifyResourceModel();
    /*!
     * Resets Resourse Limits to original Values.
     */
    void resetResourceModel();
    /*!
     * Sets earliest possible Starttimes to Asap-Starttimes obtained from latency estimation.
     */
    void setInitialStartTimes();
    /*!
     * Adds an offset to latest possible starttimes from latency-estimaltion.
     * @param candidateLatency
     */
    void calculateStartimes(int candidateLatency);
    /*!
     * \brief Creates boolean z3 expressions for each vertex and start time.
     */
    void createBooleanVariables();
    /*!
     * \brief Adds constraints to solver s, that a vertex is scheduled in exactly one timeslot.
     */
    void addOneSlotConstraintToSolver();
    /*!
     * \brief Adds a constraint to solver s, that no resource limits are violated.
     */
    void addResourceContraintsToSolver();
    /*!
     * Calculates the schedule-length.
     * @param map<Vertex*, int> Schedule
     * @return length of schedule.
     */
    int getScheduleLatency (map<Vertex*, int> &sched);
    /*!
     * Checks the proposed Schedule from SMT-Solver and returns all violated edges of the DFG.
     * @param g
     * @param rm
     * @param schedule
     * @param II
     * @param quiet
     * @return violated edges
     */
    static stack<Edge *> checkSchedule(Graph &g, ResourceModel &rm, map<Vertex *, int> &schedule, int II, bool quiet);
    /*!
     * Adds conflict clauses from violates edges to the SMT-Solver
     * @param eStack Stack of violated edges.
     */
    void addConflictClauseNextTry(stack<Edge*> &eStack);
    /*!
     * Parses SMT-Model and extracts a schedule from it.
     */
    void parseSMTModel();
    /*!
     * Overrides endTimeTracking Method from SchedulerBase since there are multiple spots where time has to be tracked.
     */
    void endTimeTracking() override;

    /*!-----------*/
    /*! Variables */
    /*!-----------*/

    /*!
     * \brief z3 boolean expression for each pair of vertex* and start time. Will be true if z3 schedules a vertex to the start
     * time, and false if it does not.
     */
    map<pair<Vertex*, int>, z3::expr> booleanVariables;
    /*!
     * Result from Latency Estimation (Min Latency, MaxLatency, earliest and latest Starttimes)
     */
    Utility::LatencyEstimation latEst;
    /*!
     * Latest Starttimes with offset caused by candidate Latency
     */
    map<Vertex*, int>latestStartTimesWithOffset;
    /*!
     * Stores original resource limits when ResourceModel is modified.
     */
    map<Resource*, int>resourceLimits;

  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTCDLSCHEDULER_H
