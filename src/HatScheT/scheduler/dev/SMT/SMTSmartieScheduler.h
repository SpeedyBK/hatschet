#ifndef HATSCHET_SMTSMARTIESCHEDULER_H
#define HATSCHET_SMTSMARTIESCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <utility>

#include <z3++.h>

namespace HatScheT {

  class SMTSmartieScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    enum class iiSearchMethod {linear, binary};

    SMTSmartieScheduler(Graph &g, ResourceModel &resourceModel);

    void schedule() override;

    void setIISearchMethod(iiSearchMethod iism){ this->iiSM = iism; }

  protected:

    int latencyUpperBound;
    int min_Latency;
    int max_Latency;

    iiSearchMethod iiSM;
    vector<int>II_space;
    int II_space_index;

    z3::context c;

    map<Vertex*, int> earliest_start_times;
    map<Vertex*, int> latest_start_times;

    map<std::pair<Vertex*, int>, z3::expr> b_variables;

    void find_earliest_start_times();
    void find_latest_start_times();

    void calcMaxLatency();

    int ii_binary_search(z3::check_result result);
    int ii_linear_search(z3::check_result result);

    void generate_b_variables();
    void set_b_variables(z3::solver& s, const int &candidateII);
    z3::expr* get_b_variable(Vertex* v, int i);
    void print_b_variables();

    void prohibit_to_early_starts_and_add(z3::solver &s);
    void prohibit_to_late_starts_and_add(z3::solver &s);

    void add_one_slot_constraints_to_solver(z3::solver &s);
    void add_resource_limit_constraint_to_solver(z3::solver &s, int candidateII);

    void print_solution(z3::model &m);
    void parse_schedule(z3::model &m);

  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTSMARTIESCHEDULER_H
