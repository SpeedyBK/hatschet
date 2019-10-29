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

#ifndef HATSCHET_SDSSCHEDULER_H
#define HATSCHET_SDSSCHEDULER_H
#ifdef USE_CADICAL

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>

#include "cadical.hpp"

namespace HatScheT {

  /*
   * General Idea:
   * Steve Dai, Gai Liu, Zhiru Zhang; A Scalable Approach to Exact Ressource-Constraint Scheduling Based on a Joint
   * SDC and SAT Formulation; FPGA 2018
   *
   * CaDiCaL Paper:
   * http://fmv.jku.at/papers/Biere-SAT-Competition-2018-solvers.pdf
   */

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
                                                                   vector<vector<int>> confClauses);

    /*!
     * Creates a SDC-Constraint-Graph based on Dependency and Timing-Constraints.
     */
    void createBasicConstraintGraph();

    /*!
     * Addes Vertices and Edges to the SDC-Constraint Graph.
     * @param constraintsSDCVer: The first element is the pointer to the Sourcevertex of and Edge,
     *                           The second element is the pointer to the Destination of an Edge.
     * @param weight: Distance of the Edge.
     */
    void addToConstraintGraph(pair<const Vertex *, const Vertex *> constraintsSDCVer, int weight);

    /*!
     * Detects if the SDC-Inequality-System is feasible, and if it is feasible,
     * hopefully solves the SDC-Inequality-System.
     */
    pair<map<Vertex*, int>, bool> solveSDC();

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

    /*!
     * Debugging Shit
     */
    void displayGraph();

    /*!
     * More Debugging Shit
     */
    void displaySDCSolution();

    /////////////////////////
    //      Variables      //
    /////////////////////////

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
     * This Constraint Graph represents the SDC inequality System
     */
    Graph constraintGraph;

    /*!
     * Iterator to resource constraint edges
     */
    map<pair<const Vertex *, const Vertex *>, int>::iterator rceIt;

    /*!
     * Container for the return Values from Bellman-Ford.
     */
    pair<map <Vertex*, int>, bool> isUnsolvableSolution;

    //////////////////////////////////////////////////////////////////////////////
    /*!
     * All the Stuff for the Bellman-Ford Algorithm to solve SDC Constraint Graphs
     */
    //////////////////////////////////////////////////////////////////////////////
    class BellmanFord {

    public:
      explicit BellmanFord(Graph* golfRomeo);

      /*!
       * If Bellman-Ford finds a feasable solution, it returns the solution and a boolean which says, that a solution is
       * found. If there is no solution, it returns an empty map and a boolean which says, that no solution is found.
       */
      pair<map<Vertex*, int>, bool> getPath(int idofStartVertex);

    private:

      /*!
       * Sets the cost for the start Vertex to 0 and the cost for all other Vertices to INT_MAX;
       */
      void bellmanFordAlgorithm(int startID);

      /*!
       * Pointer to Graph for which shortest Paths will be searched
       */
      Graph* g;

      /*!
       * Map with Vertices of the Constraint Graph and the Cost to reach each Vertex
       */
      map<Vertex *, int> vertexCosts;

      /*!
       * Tells if Graph g has negative cycles. If true, the SDC-System is unsolvable.
       */
      bool hasNegativeCycle;
    };

  };

}

#endif //USE_CADICAL

#endif //HATSCHET_SDSSCHEDULER_H
