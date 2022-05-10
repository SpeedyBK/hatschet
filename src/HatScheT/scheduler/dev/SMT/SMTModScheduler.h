//
// Created by bkessler on 5/7/22.
//

#ifndef HATSCHET_SMTMODSCHEDULER_H
#define HATSCHET_SMTMODSCHEDULER_H

#pragma once

#ifdef USE_Z3
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <vector>

#include <z3++.h>

namespace HatScheT {

  class SMTModScheduler : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    SMTModScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    /*!
     * \brief Attempts to schedule the given instances. The candidate II is incremented until a feasible schedule is found.
     */
    virtual void schedule();


  protected:
    /*!
     * not needed
     */
    virtual void setObjective(){}
    virtual void resetContainer(){}
    virtual void constructProblem() {/* unused */}

    void buildDataStructure();


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
    vector<z3::expr> t_Variables;
    /*!
     * Mapping Vertices to the index of the corresponding expression in the t_vairables Vector.
     */
    map<Vertex*, int> vertex_t_Variables_Map;
    /*!
     * This Function creates a vector of expressions. Each expression is corresponding to a t_variable of vertex.
     */
    vector<z3::expr> creating_t_Variables();

    /*!
     * Data Dependency Constraints correspond to the edges in the data flow graph. tj - ti >= Li - II * dij
     * t_. are the t_variables for each vertex, Li is the latency for vertex i. II is obvious. dij is the distance of
     * the edge from vertex i to vertex j.
     */
    pair <vector<z3::expr>, vector<z3::expr>> data_Dependency_Constraints;
    /*!
     * This Function uses the t_variables to create the data dependency constraints.
     * @return It returns two vectors of z3::expr. The first vector corresponds with the edges without distance,
     * since they do not depend on the given II. The second vector corresponds with the with distance which depend on II.
     * This is done to create fallback points for the SMT-Solver, to remove constraints depending on II, then increment
     * II and push those constraints back in the Solver.
     */
    pair <vector<z3::expr>, vector<z3::expr>> build_Dependency_Constraints();

    /* ----------------------------------------- *
     *  b_variables and MRM                      *
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
     *       Sum   Sum   Sum    Sum
     *       <=FU  <=FU  <=FU   <=FU
     *
     * For each constrained resource there will be
     * one layer like the one above.
     */

    /* ----------------------------------------- *
     *  I don't know                             *
     * ----------------------------------------- */
    /*!
     * Mapping the index of an Expression in expr_vector to the corresponsing Edge.
     */
    map<int, Edge*> exprToEdgeMap;

  };

}

#endif //USE_Z3
#endif //HATSCHET_SMTMODSCHEDULER_H
