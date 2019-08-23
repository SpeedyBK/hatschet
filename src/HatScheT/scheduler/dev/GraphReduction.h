//
// Created by bkessler on 23/07/19.
//

//UNDER DEVELOPEMENT

#ifndef HATSCHET_GRAPHREDUCTION_H
#define HATSCHET_GRAPHREDUCTION_H

#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/scheduler/dev/SCC.h>
#include <HatScheT/Graph.h>
#include <vector>

namespace HatScheT {

  class GraphReduction : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
  public:
    GraphReduction(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

    /*!
     * main method to do the scheduling
     */
    virtual void schedule();


    /*!
     * not needed
     */
    virtual void setObjective() {/* unused */}
    virtual void resetContainer(){/* unused */}
    virtual void constructProblem() {/* unused */}

    /*!
    * determineType(SCC* scc) figures out, which type an SCC is, at first the number of verticies will be checkt,
    * if it is 1, the SCC is cassified as trivial. The next check is if there are resource constraints. In case there
    * are constraints, the SCC is classified as complex, and in case there are no contraints the SCC is classified as
    * basic.
    */
    scctype determineType(SCC* scc);

    /*!
    * getSccIdbyVertex(Vertex* v) finds the SCC a vertex of the original graph is in and returns the ID of this SCC.
    */
    int getSccIdbyVertex(Vertex* v);

    /*!
    * sortSCCs(SCC* scc) looks up the type of the SCC, and puts the SCC on a vector depending on its type. This will be
    * used to build the basic and complex supergraphs needed for the scheduling.
    */
    void sortSCCs(SCC* scc);

    /*!
    *
    */
    Graph* buildSupergraphs();

  private:

    vector <SCC*> sccs;

    vector <SCC*> basicSCCs;
    vector <SCC*> complexSCCs;
    vector <SCC*> trivialSCCs;

  };

}
#endif //HATSCHET_GRAPHREDUCTION_H
