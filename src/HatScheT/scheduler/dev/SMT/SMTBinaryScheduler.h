#ifndef HATSCHET_SMTBINARYSCHEDULER_H
#define HATSCHET_SMTBINARYSCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <utility>
#include <deque>
#include <cmath>

#include <z3++.h>

namespace HatScheT {

  class SMTBinaryScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    /*!
     * @brief SMT-Scheduler for scheduling in hatschet a graph (g), a resource model (rm). Uses Z3-Theorem-Prover as
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
    enum class latSearchMethod { linear, binary , reverse_linear};
    /*!
     * Function to set latency-search-method, default is linear.
     * @param lsm (latency-search-method)
     */
    void setLatencySearchMethod(latSearchMethod lsm) { this->latSM = lsm; }

    void set_design_name(string s){ this->design_name = s; }

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
    vector<int>latency_Space;
    /*!
     * \brief Current index to an element in latency_space, used for linear- and binary-latency-search
     */
    int latency_space_index;
    /*!
     * \brief Minimal possible Latency
     */
    int min_Latency;
    /*!
     * \brief Upper latency bound. It makes no sence to increase the latency futher, if no schedule has been found to
     * that point.
     */
    int max_Latency;
    /*!
     * \brief Latency that has to be ckecked.
     */
    int candidateLatency;
    /*!
     * \brief Determines the earliest possible start-times for each vertex by creating an ASAP-Schedule without
     * ressource-constraints. Also used to calculate minimal latency.
     */
    void find_earliest_start_times();
    /*!
     * \brief Determines the latest possible start-times for each vertex by creating an ALAP-Schedule without
     * ressource-constraints.
     */
    void find_latest_start_times();
    /*!
     * \brief Calculates the max latency by counting the vertices and multiplying with the max-vertex-latency.
     */
    void calcMaxLatency();
    void calc_max_latency_with_sdc(Graph &g, ResourceModel &resM);

    int getScheduleLatency(unordered_map<Vertex*, int> &vertex_latency, unordered_map<Vertex*, Vertex*> &newtoold);
    /*!
     * \brief Creates the latency space vector.
     */
    void calcLatencySpace();
    /*!
     * Binary search for the optimal latency.
     * @param result from the previous z3 check. (SAT, UNSAT, UNKOWN)
     * @return next candidate latency, or -1 if the search is completed.
     */
    int lat_binary_search(z3::check_result result);
    /*!
     * Containts the information, if the binary search is called the first time.
     */
    bool binary_search_init;
    int left_index;
    int right_index;
    /*!
     * Linear (incremental) search for the optimal latency.
     * @param result from the previous z3 check. (SAT, UNSAT, UNKOWN)
     * @return next candidate latency, or -1 if the search is completed.
     */
    int lat_linear_search(z3::check_result result);

    int lat_reverse_linear_search(z3::check_result result);

    bool reverse_search_init;

    /*!------------------
     * II related stuff
     *------------------*/
    /*!
     * \brief Vector which contains all possible IIs, created by the constructor.
     */
    vector<int>II_space;
    /*!
     * \brief Index of an element in the II-Space. Used by II-linear-search.
     */
    int II_space_index;
    /*!
     * Linear (incremental) search for the optimal II.
     * @param result from the previous z3 check. (SAT, UNSAT, UNKOWN)
     * @return next candidate II, or -1 if the search is completed or search space is exhausted.
     * ToDo: Add something if search space is exhausted.
     */
    int ii_linear_search(z3::check_result result);
    /*!
     * Containts the information, if the II-Search is called the first time.
     */
    bool ii_search_init;

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
    map<Vertex*, int> earliest_start_times;
    /*!
     * \brief Latest possible start times for each operation, determined by an ALAP-Schedule without ressource constraints +
     * candidate latency.
     */
    map<Vertex*, int> latest_start_times;
    /*!
     * \brief Maps each pair of Vertex and start time to a bool. The bool is true if the pair of Vertex and start time is
     * possible and false if it is impossible. The map is used to simplify the cnf for z3 and is created by
     * prohibit_to_early_starts_and_add() and prohibit_to_late_starts_and_add()
     */
    map<pair<Vertex*, int>, bool> start_times_simplification;
    /*!
     * \brief Function prohibits start times which violate the ASAP-Schedule without ressource constraints and adds them
     * to the solver s.
     * @param reference to solver s
     */
    void prohibit_to_early_starts_and_add(z3::solver &s);
    /*!
     * \brief Function prohibits start times which violate the ALAP-Schedule without ressource constraints and adds them
     * to the solver s.
     * @param reference to solver s
     */
    void prohibit_to_late_starts_and_add(z3::solver &s);
    /*!
     * \brief z3 boolean expression for each pair of vertex* and start time. Will be true if z3 schedules a vertex to the start
     * time, and false if it does not.
     */
    map<std::pair<Vertex*, int>, z3::expr> b_variables;
    /*!
     * \brief Creates boolean z3 expressions for each vertex and start time and inserts them to b_variables.
     */
    void generate_b_variables();
    /*!
     * \brief Getter for b_variables
     * @param v - Vertex* v, first part of the key.
     * @param i - Start time i, second part of the key.
     * @return corresponding b_variable to the pair of Vertex* v and start time i.
     */
    z3::expr* get_b_variable(Vertex* v, int i);
    /*!
     * \brief This function inserts the dependency constraints to solver s. It checks if a combination of a vertex* and start
     * time violates dependency constraints. If it does it prohibits this combination.
     * @param Reference to solver s
     * @param candidateII
     */
    void set_b_variables(z3::solver& s, const int &candidateII);
    /*!
     * \brief Checks an obvious unsat condition without starting z3. (prohibit_to_early_starts_and_add() and
     * prohibit_to_late_starts_and_add() prohibiting all time slots for one Vertex)
     * @return True if unsat and false if possibly sat.
     */
    bool unsat_check_shortcut();
    /*!
     * \brief Adds a constraint to solver s, that a vertex is scheduled in exactly one timeslot.
     * @param Reference to solver s
     */
    void add_one_slot_constraints_to_solver(z3::solver &s);
    /*!
     * \brief Adds a constraint to solver s, that no ressource limits are violated.
     * @param Reference to solver s
     */
    void add_resource_limit_constraint_to_solver(z3::solver &s, int candidateII);
    /*!-------------------------------
     *  Print and Debugging Methods
     *-------------------------------*/
    void print_solution(z3::model &m);
    void print_ASAP_ALAP_restictions();
    void print_latency_space(int l_index, int r_index);
    void print_b_variables();
    static z3::check_result test_binary_search(int value_to_check, int target_value);

    string design_name;
    void write_solving_times_to_file(deque<double> &times);
    /*!
     * Gets model m if z3 returns sat and extracts the schedule from this model.
     * @param z3::model &m
     */
    void parse_schedule(z3::model &m);

  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTBINARYSCHEDULER_H
