#ifndef HATSCHET_SMTBINARYSCHEDULER_H
#define HATSCHET_SMTBINARYSCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <HatScheT/base/Z3SchedulerBase.h>

#include <utility>
#include <deque>
#include <queue>
#include <cmath>

namespace HatScheT {

  class SMTBinaryScheduler : public IterativeModuloSchedulerLayer, public Z3SchedulerBase{

  public:
    /*!
     * @brief smtbased-Scheduler for scheduling in hatschet a graph (g), a resource model (rm). Uses Z3-Theorem-Prover as
     * backend.
     * @param g graph
     * @param rm resource model
     * @param II given II
     */
    SMTBinaryScheduler(Graph &g, ResourceModel &resourceModel, double II = -1);
    /*!
     * Obvious
     */
    string getName() override { return "SMT-Binary-Scheduler"; }
    /*!
     * Sets a timeout for Z3 for each check.
     * @param seconds
     */
    void setSolverTimeout(double seconds) override;
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
    /*!
     * Look below.
     */
    enum class schedulePreference {MOD_ASAP, MOD_ALAP};
    /*!
     * In case SDC finds valid ASAP- and ALAP-schedules the pefered schedule can be selected here.
     */
    void setSchedulePreference(schedulePreference pref) { this->sPref = pref; }
    /*!
     * for debugging to set a filename for debug files.
     * @param s name
     */
    void set_design_name(string s){ this->designName = s; }
    /*!
     * Function to set a specific latency to search a schedule for.
     * @param latency
     */
    void setTargetLatency(int latency) { this->targetLatency = latency; }

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
     * Comperator for priority queue in latency estimation
     */
    struct SmtVertexIntCompLess {
      constexpr bool operator()(
          pair<Vertex*, int> const &a,
          pair<Vertex*, int> const &b)
      const noexcept {
          return a.second < b.second;
      }
    };
    /*!
     * Comperator for priority queue in latency estimation
     */
    struct SmtIntIntComp {
      bool operator()(
          pair<int, int> const &a,
          pair<int, int> const &b)
      const noexcept {
          return a.second < b.second;
      }
    };

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
     * \brief Latency which is set by the user.
     */
    int targetLatency;
    /*!
     * \brief Latency that has to be checked.
     */
    int candidateLatency;
    /*!
     * \brief Determines the latest possible start-times for each vertex by creating an ALAP-schedule without
     * ressource-constraints.
     */
    void updateLatestStartTimes();
    /*!
     * \brief Wrapper for latency-estimation.
     */
    void calcLatencyEstimation();
    /*!
     * Calculates the maximum latency estimation based on earliest and latest possible starttimes.
     * @param currentII
     */
    void calcMaxLatencyEstimation(int currentII);
    /*!
     * Calculates the minimum latency based on earliest and latest starttimes.
     * @param aslap earliest and latest starttimes for each vertex
     * @param currentII
     * @return a boolean value with the information, if min latency-estimation is valid. If not, the wrapper-function
     * will increase latency, and try again.
     */
    bool calcMinLatencyEstimation(pair<map<Vertex*, int>, map<Vertex*, int>> &aslap, int currentII);
    /*!
     * Calculates earliest and latest starttimes for each vertex.
     * @param g Graph
     * @param resM ResourceModel
     * @return 2 maps with earliest and latest starttimes.
     */
    pair <map<Vertex*,int>, map<Vertex*, int>> calcAsapAndAlapModScheduleWithSdc(Graph &g, ResourceModel &resM);
    /*!
     * Length of SDC ASAP Schedule
     */
    int modAsapLength;
    /*!
     * Length of SDC ALAP Schedule
     */
    int modAlapLength;
    /*!
     * Value by which minimum latency has to be increased if it is to small
     */
    int increment;
    /*!
     * Verifies if found SDC-Shedules are allready valid modulo schedules.
     * @param g Graph
     * @param rm Resource Model
     * @param schedule found Schedule
     * @param II current II
     * @return boolean which says if schedule is valid.
     */
    bool verifyModuloScheduleSMT(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int II);
    /*!
     * Used to switch between ASAP and ALAP SDC schedules in case they are valid.
     */
    schedulePreference sPref;
    /*!
     * Calculates the schedule-length of an SDC-schedule.
     * @param vertexLatency mapping between Vertex in SDC-Graph and the latency of original vertex.
     * @param newToOld mapping between Vertex in SDC-Graph and original vertex.
     * @return length of SDC-schedule.
     */
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
     * Contains the information, if the binary search is called the first time.
     */
    bool binarySearchInit;
    /*!
     * Indices for binary search.
     */
    int leftIndex;
    int rightIndex;
    /*!
     * Linear (incremental) search for the optimal latency.
     * @param result from the previous z3 check. (SAT, UNSAT, UNKOWN)
     * @return next candidate latency, or -1 if the search is completed.
     */
    int latLinearSearch(z3::check_result result);
    /*!
     * Contains the information, if the linear search is called the first time.
     */
    bool linearSearchInit;

    /*!---------------------------
     * Stuff needed to set up z3.
     *---------------------------*/
    /*!
     * \brief Earliest possible start times for each operation, determined by an ASAP-Schedule without ressource constraints
     */
    map<Vertex*, int> earliestStartTimes;
    /*!
     * \brief Latest possible start times for each operation, determined by an ALAP-Schedule without ressource constraints +
     * candidate latency.
     */
    map<Vertex*, int> latestStartTimes;
    map<Vertex*, int> latestStartTimesUpdated;
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
    void prohibitToEarlyStartsAndAdd();
    /*!
     * \brief Function prohibits start times which violate the ALAP-Schedule without ressource constraints and adds them
     * to the solver s.
     * @param reference to solver s
     */
    void prohibitToLateStartsAndAdd();
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
    void setDependencyConstraintsAndAddToSolver(const int &candidateII);
    /*!
     * \brief This function inserts the dependency constraints to solver s. Only used if there are to many constraints to
     * get in with the other function.
     * @param candidateII
     */
    void setDependencyConstraintsAndAddToSolverBIG(const int &candidateII);
    /*!
     * \brief Adds a constraint to solver s, that a vertex is scheduled in exactly one timeslot.
     * @param Reference to solver s
     */
    z3::check_result addOneSlotConstraintsToSolver();
    /*!
     * \brief Adds a constraint to solver s, that no ressource limits are violated.
     * @param Reference to solver s
     */
    z3::check_result addResourceLimitConstraintToSolver(int candidateII);
    /*!
     * Gets model m if z3 returns sat and extracts the schedule from this model.
     * @param z3::model &m
     */
    void parseSchedule(z3::model &m);
    /*!-------------------------------
     *  Print and Debugging Methods
     *-------------------------------*/
    void print_solution(z3::model &m);
    void print_ASAP_ALAP_restictions();
    void print_latency_space(int l_index, int r_index);
    void print_b_variables();
    void printPossibleStarttimes(map<pair<Vertex*, int>, bool>& vertex_timeslot);
    static z3::check_result test_binary_search(int value_to_check, int target_value);

    string designName;
    void writeSolvingTimesToFile(deque<double> &times, int x);

    void endTimeTracking() override;

  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTBINARYSCHEDULER_H
