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

  class DaiZhang19Scheduler : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
  public:
    DaiZhang19Scheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

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
     * getSccIdbyVertex(Vertex* v) finds the SCC a vertex of the original graph is in and returns the ID of this SCC.
     */
    int getSccIdbyVertex(Vertex* v);

    /*!
     * sortSCCs(SCC* scc) looks up the type of the SCC, and puts the SCC on a vector depending on its type. This will be
     * used to build the basic and complex supergraphs needed for the scheduling.
     */
    void sortSCCs(SCC* scc);

    /*!
     * findMaximalIndependentSet finds the maximum independet set for a type of SCCs in a Graph.
     */
    void findMaximalIndependentSet(vector <SCC*> SCCvec, scctype sT);

    /*!
     * findConncetedSCCs finds the NeighborSCCs of a SCC, and writes it to as a list to the individual SCC.
     * @param scc, we are looking for scc's Neighbors.
     */
    void findConnectedSCCs(SCC* scc);

    /*!
     * Takes the SCCs which are associated to a SuperGraph Builds a Supergraph and calls the computeRelativeSchedule
     * Method. The relative Schedule is than saved in a pair with the correspoding Supergraph.
     * @param SCCvec
     * @param sT
     */
    void buildSuperGraph(vector<SCC*> SCCvec, scctype sT);

    /*!
     * Computes the relative schedule for a Supergraph
     * @param g Graph to be scheduled, build by buildSuperGraph
     * @param rm RessourceModel corresponding to the SuperGraph
     * @param sT Type of SCCs (Basic or Complex)
     * @return A map with Vertices and their starttimes.
     */
    std::map<Vertex*, int> computeRelativeSchedule(Graph& g, ResourceModel &rm, scctype sT);


  private:

    /*!
     * Copy of the solverWishlist to pass it to the scheduler.
     */
    std::list<std::string> solverWishlist;

    /*!
     * Vector of all SCCs in a graph, found by Kosarajus Algorithm
     */
    vector <SCC*> sccs;

    /*!
     * A basic supergraph is a graph, which contains all only unconnected basic SCCs. If there is a connection between
     * two basic SCCs, then they can not be in the same supergraph.
     */
    vector <vector<SCC*>> basicSupergraphSCCs;

    /*!
     * A complex supergraph is a graph, which contains all only unconnected complex SCCs. If there is a connection between
     * two complex SCCs, then they can not be in the same supergraph.
     */
    vector <vector<SCC*>> complexSupergraphSCCs;

    /*!
     * List of Basic Supergraphs with their relativ schedule mapped to.
     */
    list <std::pair <vector<SCC*>, std::map <Vertex*, int>>> scheduledBasicSupergraphSCCs;

   /*!
    * List of Complex Supergraphs with their relativ schedule mapped to.
    */
    list <std::pair <vector<SCC*>, std::map <Vertex*, int>>> scheduledComplexSupergraphSCCs;

    /*!
     * Vector off SCCs which are classified as basic.
     * - basic: SCCs which contain multiple Vertices, but none of them has a ressource constraint.
     */
    vector <SCC*> basicSCCs;

    /*!
     * Vector off SCCs which are classified as complex.
     * - complex: SCCs which contain multiple Vertices, and at least one vertex has a ressource constraint.
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
