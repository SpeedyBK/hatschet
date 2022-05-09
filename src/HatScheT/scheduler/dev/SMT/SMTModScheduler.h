//
// Created by bkessler on 5/7/22.
//

#ifndef HATSCHET_SMTMODSCHEDULER_H
#define HATSCHET_SMTMODSCHEDULER_H

#pragma once

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

    /*!
     * Handmade Schedule for development.
     */
    void handmadeSchedule();

  protected:
    /*!
     * not needed
     */
    virtual void setObjective(){}
    virtual void resetContainer(){}
    virtual void constructProblem() {/* unused */}

    /*!
     * This Function creates a vector of expressions. Each expression is corresponding to a vertex.
     */
    z3::expr_vector creatingStartTimesVec();

    pair <z3::expr_vector,z3::expr_vector> createDependencyConstraints(z3::expr_vector &expressions);

    /*!
     * Problem Context, needed for Z3-Solver;
     */
    z3::context c;

    /*!
     * Mapping Vertices to the index of the corresponding expression in the expression Vector.
     */
    map<Vertex*, int> vertexToExprMap;

    /*!
     * Mapping the index of an Expression in expr_vector to the corresponsing Edge.
     */
    map<int, Edge*> exprToEdgeMap;

  };

}

#endif //HATSCHET_SMTMODSCHEDULER_H
