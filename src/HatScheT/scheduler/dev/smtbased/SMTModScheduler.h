//
// Created by bkessler on 5/7/22.
//

#ifndef HATSCHET_SMTMODSCHEDULER_H
#define HATSCHET_SMTMODSCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <utility>
#include <vector>
#include <deque>

#include <z3++.h>

namespace HatScheT {

  enum class encodingMode {ite, pbeq, somethingElse};

  class SMTModScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    SMTModScheduler(Graph& g, ResourceModel &resourceModel);
    /*!
     * \brief Attempts to schedule the given instances. The candidate II is incremented until a feasible schedule is found.
     */
    ~SMTModScheduler() override = default;

    void schedule() override;

    void set_encoding_mode(encodingMode e) { this->encode = e; }

    void set_mode(int i) { this->mode = i; }

  protected:

    void build_Data_Structure();

    /*!
     * Problem Context, needed for Z3-Solver;
     */
    z3::context c;

    /* ----------------------------------------- *
     *  t_variables and dependency Constraints   *
     * ----------------------------------------- */
    /*!
     * t_variables are the starttimes of operations represented by a vertex in our dataflow graph.
     */
    deque<z3::expr> t_Variables;
    /*!
     * Mapping Vertices to the index of the corresponding expression in the t_vairables Vector.
     */
    map<Vertex*, int> vertex_t_Variables_Map;
    /*!
     * This Function creates a vector of expressions. Each expression is corresponding to a t_variable of vertex.
     */
    void create_t_Variables();
    /*!
     * Data Dependency Constraints correspond to the edges in the data flow graph. tj - ti >= Li - II * dij
     * t_. are the t_variables for each vertex, Li is the latency for vertex i. II is obvious. dij is the distance of
     * the edge from vertex i to vertex j.
     */
    pair <deque<z3::expr>, deque<z3::expr>> data_Dependency_Constraints;
    /*!
     * This Function uses the t_variables to create the data dependency constraints.
     * @return It returns two vectors of z3::expr. The first vector corresponds with the edges without distance,
     * since they do not depend on the given II. The second vector corresponds with the with distance which depend on II.
     * This is done to create fallback points for the smtbased-Solver, to remove constraints depending on II, then increment
     * II and push those constraints back in the Solver.
     */
    pair <deque<z3::expr>, deque<z3::expr>> build_Dependency_Constraints();

    /* ----------------------------------------- *
     *  b_variables and MRT                      *
     * ----------------------------------------- */
    /* Data Structure:
     *
     *     |  0  |  1  | 2.. | II-1 |
     * ------------------------------
     * V0  |  x  |     |     |      | sum = 1
     * ------------------------------
     * V1  |     |  x  |     |      | sum = 1
     * ------------------------------
     * V2  |     |     |  x  |      | sum = 1
     * ------------------------------
     * V3  |     |     |     |  x   | sum = 1
     * ------------------------------
     * V4  |  x  |     |     |      | sum = 1
     * ------------------------------
     *       Sum   Sum   Sum   Sum
     *       <=FU  <=FU  <=FU  <=FU
     *
     * For each constrained resource there will be
     * one layer like the one above.
     */

    /*!
     * B_Variables are used to model Resourceconstraints. For now im using a struct to be able to crosscheck the
     * corresponding Vertex, Resource and Moduloslot.
     */
    struct b_variable{
      b_variable(int i, z3::expr bVar) : iislot(i),  b_var(std::move(bVar)) {}
      string resourcename = "unused";
      string vertexname;
      int iislot;
      z3::expr b_var;
    };
    /*!
     * 3-Dimentional Matrix to store all B_Variables in a way like shown above.
     */
    deque<deque<deque<b_variable>>> b_variables;
    /*!
     * Maps Vertices to the index in B_Variable Matrix
     */
    map<Vertex*, int> vertex_to_b_vairable_index;
    /*!
     * Maps Resources to the index in B_Variable Matrix
     */
    map<Resource*, int> resource_to_b_vairable_index;
    /*!
     * Function to create B_Variables.
     * It itterates over Resources, Moduloslots and Vertices to store the Variables in the 3D Matrix above.
     */
    void create_b_variables();
    /*!
     * Getter for B_Variables, by Vertex, Resource and Moduloslot.
     */
    b_variable* get_b_var(Vertex* v, const Resource *r, int slot);

    static void add_Constraints_to_solver(z3::solver &s, deque<z3::expr, allocator<z3::expr>> &eVec);

    void add_one_slot_constraints_to_solver(z3::solver &s);

    void add_resource_limit_constraint_to_solver(z3::solver &s);

    void add_linking_constraints_to_solver(z3::solver &s);

    void create_and_add_latency_constraints(z3::solver &s);

    void set_max_latency(z3::solver &s, int maxLat);
    /*!
     * Print Methode.
     */
    void print_b_variables();

    void print_data_dependency_Constraints();

    /* ----------------------------------------- *
     *  I don't know                             *
     * ----------------------------------------- */
    /*!
     * Mapping the index of an Expression in expr_vector to the corresponsing Edge.
     */
    map<int, Edge*> exprToEdgeMap;

    int maxLatency;

    encodingMode encode;

    int mode;

  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTMODSCHEDULER_H
