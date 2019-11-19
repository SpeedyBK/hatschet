/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)
            Benjamin Lagershausen-Kessler (benjaminkessler@student.uni-kassel.de)

    Copyright (C) 2019

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
 * General Idea:
 * Steve Dai, Gai Liu, Zhiru Zhang; A Scalable Approach to Exact Ressource-Constraint Scheduling Based on a Joint
 * SDC and SAT Formulation; FPGA 2018
 *
 * CaDiCaL SAT-Solver:
 * Armin Biere; CADICAL, LINGELING, PLINGELING, TREENGELING and YALSAT Entering the SAT Competition; 2018
 *
 * SDC Incremental Solver:
 * G.Ramalingam, J. Song, L. Joskowicz, R.E. Miller; Solving Systems of Difference Constraints Incrementally;
 * Algorithmica 1999
 *
 * Fibonacci Heap:
 * Michael L. Fredman, Robert E. Tarjan; Fibonacci Heaps and Their Uses in Improved Network Optimization Algorithms
 * Journal of the Association for Computing Machinery 1987
 */

#ifndef HATSCHET_SDSSCHEDULER_H
#define HATSCHET_SDSSCHEDULER_H
#ifdef USE_CADICAL

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>
#include <cstdlib>

#include "cadical.hpp"
#include "HatScheT/utility/Verifier.h"

namespace HatScheT {

  struct bindingVariable {
    int index;
    Resource *resource;
    int resourceID;
    const Vertex *vertex;
    int resourceInstance;
    bool binding;
  };


  class SDSScheduler : public SchedulerBase, public ModuloSchedulerBase {

  public:

    /*!
		 * The SDSScheduler is a SDC and SAT based scheduler, it is based on the paper : "A Scalable Approach to Exact
     * Resource Constraint Scheduling Based on a Joint SDC and SAT Formulation" by Steve Dai, Gai Liu and Zhiru Zhang.
		 * @param g
		 * @param resourceModel
		 */
    SDSScheduler(Graph &g, ResourceModel &resourceModel);

    /*!
     * main method to do the scheduling
     */
    void schedule() override;

    /*!
     * not needed
     */
    virtual void setObjective() {/* unused */}

    virtual void resetContainer() {/* unused */}

    virtual void constructProblem() {/* unused */}

    //Setter Functions
    /*!
     * All cout statements can be supressed by setting "quiet" to true.
     * @param quiet
     */
    void setSilent(bool quiet = true) { this->silent = quiet; }

    /*!
     * Switch for the Binding Type
     * @param bType see below.
     */
    void setBindingType(char bType) { this->bindingType = bType; }

  private:

    /////////////////////////
    //      Functions      //
    /////////////////////////

    /*!
     * This function creates Binding Variables which map each operation to an instance of the limited ressources.
     */
    void createBindingVariables();

    /*!
     * This function sets the binding variables, created by createBindingVariables, to the correct values. Such that
     * each operation is mapped to exactly one resource. It tries to map the operation equaly to the resourceinstances.
     */
    void setBindingVariables();

    /*!
     * Creates and sets a set of boolean Sharing Variables. If the a R(ij) is true, it means that the operations i and j
     * are sharing the same resourceinstance. Tries to distribute the operation equally to the resource instances.
     * @return Sharing Variable <Vi, Vj>, true/false
     */
    map<pair<const Vertex *, const Vertex *>, bool> createShVarsMaxSpeed();

    /*!
     * Creates and sets a set of boolean Sharing Variables. If the a R(ij) is true, it means that the operations i and j
     * are sharing the same resourceinstance. Tries to use the minimum of resource instances.
     * @return Sharing Variable <Vi, Vj>, true/false
     */
    map<pair<const Vertex *, const Vertex *>, bool> createShVarsMinRes();

    /*!
     * This function passes the Resource Constraints and Conflict Clauses from SDC to the SAT-Solver and gets the Solution
     * from the SAT-Solver. The SAT Solution is than tranformed to a SDC formulation:
     * (O(i->j) = true = S(i) - S(j) <= -1
     *  O(i->j) = false = no SDC-constraints)
     * @param shareVars Sharing Variables.
     * @param confClauses conflict clauses determined by the SDC-Solver.
     * @return Map of SDC-Formulations based on the SAT Solution.
     */
    map<pair<const Vertex *, const Vertex *>, int> passToSATSolver(map<pair<const Vertex *, const Vertex *>, bool> &shareVars,
                                                                   vector<vector<int>> &confClauses);

    /*!
     * Swaps a pair.
     * @param inPair Inputpair
     * @return Swapped Version of InPair.
     */
    static pair<const Vertex *, const Vertex *> swapPair(pair<const Vertex *, const Vertex *> inPair);

    /*!
     * Creates Data Dependency and Tming Constraints from the Original Graph.
     * @return Data Dependency and Timing Constraints in an SDC Formulation.
     */
    map<pair<const Vertex *, const Vertex *>, int> createDependencyConstraints();

    /*!
     * @param gr Pointer to Graph which should be checked.
     * @param vertexID Id of the Vertex to search for.
     * @return true, if Vertex exists : false if Vertex does not exist;
     */
    static bool doesVertexExist(Graph *gr, int vertexID);

    void getSATClausesFromSDCConflicts(map<pair<Vertex*, Vertex*>, int> &conflicts);

    /////////////////////////
    //      Variables      //
    /////////////////////////

    bool firstTime;

    /*!
     * If true, cout statements are supressed.
     */
    bool silent;

    /*!
     * BindingType is used as a switch for the resourcebinding:
     * R -> Use minimun of resources: Starts with mapping all operations to 1 resource instance.
     * S -> Get maximum Speed : Distributes the operations as equal as possible to the resource instances.
     */
    char bindingType;

    /*!
     * Number of limited resources.
     */
    int numOfLimitedResources;

    /*!
     * Maps each operation which need a limited Resource to an instance of this ressource.
     */
    list<bindingVariable> bindingVariables;

    /*!
     * Marks if operations share the same resource instance
     */
    map<pair<const Vertex *, const Vertex *>, bool> sharingVariables;

    /*!
     * Solution which the SAT-Solver return for the resource constraints given by the Sharing Variables.
     * in an SDC Formulation.
     */
    map<pair<const Vertex *, const Vertex *>, int> resourceConstraintsSDC;

    /*!
     * Data Dependency Constraints and Timing Constraints from the original Graph.
     */
    map<pair<const Vertex *, const Vertex *>, int> dependencyConstraintsSDC;

    /*!
     * Mapping from SDC to SAT
     */
    map<pair<const Vertex*, const Vertex*>, int> sdcToSATMapping;

    /*!
     * Conflict Clauses detected by SDC
     */
    vector<vector<int>> conflictClauses;

    /*!
     * Flag from SAT-Solver, if problem is not satisfiable
     */
    bool unsatisiable;


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*!
     * This Sheduler needs to remove Edges from the Constraint Graph. The original Graph Class does not support this,
     * so we need to implement it here.
     */
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    class ConstraintGraph : public Graph{

    public:
      void removeEdge(Vertex* srcVertex, Vertex* dstVertex);

      Edge &createEdgeSDS(Vertex &Vsrc, Vertex &Vdst, int distance=0, Edge::DependencyType dependencyType=Edge::Data);

      bool doesEdgeExistID(Vertex* src, Vertex* dst);

      Edge& getEdge(const Vertex *srcV, const Vertex *dstV);

      void setSilent(bool quiet = true) { this->silent = quiet; }

    private:

      bool silent = true;

    };

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*!
     * Bellman Ford Algorithm so solve SDC-Problems.
     */
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    class BellmanFordSDC {

    public:

      BellmanFordSDC(ConstraintGraph cg, Graph* originalGraph);

      /*!
       * The Solver can also be constructed by passing in a map of constraints like "V1 - V2 <= C" in this case it will
       * build it's constraint Graph by itself.
       * @param constraints like "V1 - V2 <= C"
       */
      BellmanFordSDC(map<pair<const Vertex*, const Vertex*>, int> &constraints, Graph* originalGraph);

      /*!
       * If Bellman-Ford finds a feasable solution, it returns the solution and a boolean which says, that a solution is
       * found. If there is no solution, it returns an empty map and a boolean which says, that no solution is found.
       */
      pair<map<Vertex*, int>, bool> getfirstPath();

      /*!
       * If Bellman-Ford finds a feasable solution, it returns the solution and a boolean which says, that a solution is
       * found. If there is no solution, it returns an empty map and a boolean which says, that no solution is found.
       */
      pair<map<Vertex*, int>, bool> getPathIncremental();

      void addConstraints(map<pair<const Vertex*, const Vertex*>, int> &constraints);

      map<pair<Vertex*, Vertex*>, int> getConflicts();

      /*!
       * Sets the Silent variable;
       */
      void setSilent(bool quiet) {this -> silent = quiet;}

      void increaseMinLatency();

      void printConstraintGraph();

    private:

      /*!
       * Sets the cost for the start Vertex to 0 and the cost for all other Vertices to INT_MAX;
       */
      void bellmanFordAlgorithm();

      /*!
       * Method to find the startvertex for the single source shortes path problem.
       * @return ID of the startvertex
       */
      int determineStartVertex();

      /*!
       * Constraint Graph for which shortest Paths will be searched
       */
      ConstraintGraph cg;

      /*!
       * Pointer to the original Graph.
       */
      Graph* origGraph;

      /*!
       * Map with Vertices of the Constraint Graph and the Cost to reach each Vertex
       */
      map<Vertex *, int> vertexCosts;

      /*!
       * Tells if Graph cg has negative cycles. If true, the SDC-System is unsolvable.
       */
      bool hasNegativeCycle;

      /*!
       * Tells the class to shut up.
       */
      bool silent;

      /*!
       * Id of the Vertex which is Startpoint for Bellman Ford Algorithm
       */
      int startID;

      /*!
       * Min latency edges.
       */
      list<Edge*> minLatEdges;

      /*!
       * Ressource Constraints
       */
      map<pair<Vertex*, Vertex*>, int> additionalConstraints;

      /*!
       * Conflict clauses found by SDC
       */
      map<pair<Vertex*, Vertex*>, int> conflicts;

    };

  };

}

#endif //USE_CADICAL

#endif //HATSCHET_SDSSCHEDULER_H
