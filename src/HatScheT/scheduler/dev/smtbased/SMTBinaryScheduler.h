#ifndef HATSCHET_SMTBINARYSCHEDULER_H
#define HATSCHET_SMTBINARYSCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>

#include <utility>
#include <deque>
#include <queue>
#include <cmath>

#include <z3++.h>

namespace HatScheT {

  class SMTBinaryScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    struct SmtVertexIntComp {
      constexpr bool operator()(
          pair<Vertex*, int> const &a,
          pair<Vertex*, int> const &b)
      const noexcept {
          return a.second < b.second;
      }
    };
    struct SmtIntIntComp {
      bool operator()(
          pair<int, int> const &a,
          pair<int, int> const &b)
      const noexcept {
          return a.second < b.second;
      }
    };

    /*!
     * @brief smtbased-Scheduler for scheduling in hatschet a graph (g), a resource model (rm). Uses Z3-Theorem-Prover as
     * backend.
     * @param g graph
     * @param rm resource model
     */
    SMTBinaryScheduler(Graph &g, ResourceModel &resourceModel);

    /*!
     * \brief schedule main method that runs the algorithm and determines a schedule
     */
    void schedule() override;
    /*!
     * Sets a timeout for Z3 for each check.
     * @param seconds
     */
    void setSolverTimeout(unsigned seconds);
    /*!
     * Enum to set the method to find the best possible latency.
     * linear: Incremental starting at min. latency. Checking after each increment.
     * - Good for designs when known, that they can be scheduled with MinII and a Low Latency.
     * binary: Binary search for the optimal latency
     * - Needs less checks than the linear methode to show, that a design can not be scheduled for a given II
     */
    enum class latSearchMethod { LINEAR, BINARY , REVERSE_LINEAR };
    /*!
     * Function to set latency-search-method, default is linear.
     * @param lsm (latency-search-method)
     */
    void setLatencySearchMethod(latSearchMethod lsm) { this->latSM = lsm; }

    enum class schedulePreference {MOD_ASAP, MOD_ALAP};
    void setSchedulePreference(schedulePreference pref) { this->sPref = pref; }

  protected:

    /*!------------------------
     * latency related stuff:
     *------------------------*/
    /*!
     * \brief Discribed above:
     */
    latSearchMethod latSM;
    /*!
     * \brief All possible latencys are stored in this vector.
     */
    vector<int>latencySpace;
    /*!
     * \brief Current index to an element in latency_space, used for linear- and binary-latency-search
     */
    int latencySpaceIndex;
    /*!
     * \brief Minimal possible Latency
     */
    int minLatency;
    /*!
     * \brief Upper latency bound. It makes no sence to increase the latency futher, if no schedule has been found to
     * that point.
     */
    int maxLatency;
    /*!
     * \brief Latency that has to be ckecked.
     */
    int candidateLatency;
    /*!
     * \brief Determines the latest possible start-times for each vertex by creating an ALAP-Schedule without
     * ressource-constraints.
     */
    void updateLatestStartTimes(int oldLatency);
    /*!
     * \brief Calculates the max latency by counting the vertices and multiplying with the max-vertex-latency.
     */
    void calcLatencyEstimation();
    void calcMaxLatencyEstimation(int currentII);
    bool calcMinLatencyEstimation(pair<map<Vertex*, int>, map<Vertex*, int>> &aslap, int currentII);
    pair <map<Vertex*,int>, map<Vertex*, int>> calcAsapAndAlapModScheduleWithSdc(Graph &g, ResourceModel &resM);
    schedulePreference sPref;
    int modAsapLength;
    int modAlapLength;
    bool verifyModuloScheduleSMT(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int II);

    int getScheduleLatency(unordered_map<Vertex*, int> &vertexLatency, unordered_map<Vertex*, Vertex*> &newToOld);
    /*!
     * \brief Creates the latency space vector.
     */
    void calcLatencySpace();
    /*!
     * Binary search for the optimal latency.
     * @param result from the previous z3 check. (SAT, UNSAT, UNKOWN)
     * @return next candidate latency, or -1 if the search is completed.
     */
    int latBinarySearch(z3::check_result result);
    /*!
     * Containts the information, if the binary search is called the first time.
     */
    bool binarySearchInit;
    int leftIndex;
    int rightIndex;
    /*!
     * Linear (incremental) search for the optimal latency.
     * @param result from the previous z3 check. (SAT, UNSAT, UNKOWN)
     * @return next candidate latency, or -1 if the search is completed.
     */
    int latLinearSearch(z3::check_result result);

    int latReverseLinearSearch(z3::check_result result);

    bool reverseSearchInit;

    /*!------------------
     * II related stuff
     *------------------*/
    /*!
     * \brief Vector which contains all possible IIs, created by the constructor.
     */
    vector<int>iiSpace;
    /*!
     * \brief Index of an element in the II-Space. Used by II-linear-search.
     */
    int iiSpaceIndex;
    /*!
     * Linear (incremental) search for the optimal II.
     * @param result from the previous z3 check. (SAT, UNSAT, UNKOWN)
     * @return next candidate II, or -1 if the search is completed or search space is exhausted.
     * ToDo: Add something if search space is exhausted.
     */
    int iiLinearSearch(z3::check_result result);
    /*!
     * Containts the information, if the II-Search is called the first time.
     */
    bool iiSearchInit;

    /*!---------------------------
     * Stuff needed to set up z3.
     *---------------------------*/
    /*!
     * Problem context for z3
     */
    z3::context c;
    /*!
     * \brief Earliest possible start times for each operation, determined by an ASAP-Schedule without ressource constraints
     */
    map<Vertex*, int> earliestStartTimes;
    /*!
     * \brief Latest possible start times for each operation, determined by an ALAP-Schedule without ressource constraints +
     * candidate latency.
     */
    map<Vertex*, int> latestStartTimes;
    /*!
     * \brief Maps each pair of Vertex and start time to a bool. The bool is true if the pair of Vertex and start time is
     * possible and false if it is impossible. The map is used to simplify the cnf for z3 and is created by
     * prohibitToEarlyStartsAndAdd() and prohibitToLateStartsAndAdd()
     */
    map<pair<Vertex*, int>, bool> startTimesSimplification;
    /*!
     * \brief Function prohibits start times which violate the ASAP-Schedule without ressource constraints and adds them
     * to the solver s.
     * @param reference to solver s
     */
    void prohibitToEarlyStartsAndAdd(z3::solver &s);
    /*!
     * \brief Function prohibits start times which violate the ALAP-Schedule without ressource constraints and adds them
     * to the solver s.
     * @param reference to solver s
     */
    void prohibitToLateStartsAndAdd(z3::solver &s);
    /*!
     * \brief z3 boolean expression for each pair of vertex* and start time. Will be true if z3 schedules a vertex to the start
     * time, and false if it does not.
     */
    map<std::pair<Vertex*, int>, z3::expr> bVariables;
    /*!
     * \brief Creates boolean z3 expressions for each vertex and start time and inserts them to bVariables.
     */
    void generateBvariables();
    /*!
     * \brief Getter for bVariables
     * @param v - Vertex* v, first part of the key.
     * @param i - Start time i, second part of the key.
     * @return corresponding b_variable to the pair of Vertex* v and start time i.
     */
    z3::expr* getBvariable(Vertex* v, int i);
    /*!
     * \brief This function inserts the dependency constraints to solver s. It checks if a combination of a vertex* and start
     * time violates dependency constraints. If it does it prohibits this combination.
     * @param Reference to solver s
     * @param candidateII
     */
    z3::check_result setDependencyConstraintsAndAddToSolver(z3::solver& s, const int &candidateII);
    /*!
     * \brief Checks an obvious unsat condition without starting z3. (prohibitToEarlyStartsAndAdd() and
     * prohibitToLateStartsAndAdd() prohibiting all time slots for one Vertex)
     * @return True if unsat and false if possibly sat.
     */
    bool unsatCheckShortcut();
    /*!
     * \brief Adds a constraint to solver s, that a vertex is scheduled in exactly one timeslot.
     * @param Reference to solver s
     */
    z3::check_result addOneSlotConstraintsToSolver(z3::solver &s);
    /*!
     * \brief Adds a constraint to solver s, that no ressource limits are violated.
     * @param Reference to solver s
     */
    z3::check_result addResourceLimitConstraintToSolver(z3::solver &s, int candidateII);

    int timeBudget;
    /*!-------------------------------
     *  Print and Debugging Methods
     *-------------------------------*/
    void print_solution(z3::model &m);
    void print_ASAP_ALAP_restictions();
    void print_latency_space(int l_index, int r_index);
    void print_b_variables();
    void printPossibleStarttimes(map<pair<Vertex*, int>, bool>& vertex_timeslot);
    static z3::check_result test_binary_search(int value_to_check, int target_value);
    void set_design_name(string s){ this->designName = s; }

    string designName;
    void writeSolvingTimesToFile(deque<double> &times, int x);
    /*!
     * Gets model m if z3 returns sat and extracts the schedule from this model.
     * @param z3::model &m
     */
    void parseSchedule(z3::model &m);

  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTBINARYSCHEDULER_H
