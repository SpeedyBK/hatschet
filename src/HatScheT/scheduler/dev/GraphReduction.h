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
#include "HatScheT/utility/subgraphs/SCC.h"
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
    SCC* findSupergraphs(vector <SCC*> SCCvec, scctype sT);

  private:


    /*!
     * Vector of all SCCs in a graph, found by Kosarajus Algorithm
     */
    vector <SCC*> sccs;

    /*!
     * A basic supergraph is a graph, which contains all only unconnected basic SCCs. If there is a connection between
     * two basic SCCs, then they can not be in the same supergraph.
     */
    vector <vector<SCC*>> basicSupergraphs;

    /*!
     * A complex supergraph is a graph, which contains all only unconnected complex SCCs. If there is a connection between
     * two complex SCCs, then they can not be in the same supergraph.
     */
    vector <vector<SCC*>> complexSupergraphs;

    /*!
     * Vector off SCCs which are classified as basic.
     * - basic: SCCs which contain multiple verticies, but none of them has a ressource constraint.
     */
    vector <SCC*> basicSCCs;

    /*!
     * Vector off SCCs which are classified as complex.
     * - complex: SCCs which contain multiple verticies, and at least one vertex has a ressource constraint.
     */
    vector <SCC*> complexSCCs;

    /*!
     * Vector off SCCs which are classified as trivial
     * - trivial: SCCs which contain just 1 vertex.
     */
    vector <SCC*> trivialSCCs;

  };

}
#endif //HATSCHET_GRAPHREDUCTION_H
