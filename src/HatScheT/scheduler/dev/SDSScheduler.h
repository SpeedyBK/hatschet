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
#include <HatScheT/base/ILPSchedulerBase.h>
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

  struct orderingVariabletoSDCMapping{

    int satVariable;
    pair<const Vertex*, const Vertex*> constraintOneVertices;
    int constraintOne;
    pair<const Vertex*, const Vertex*> constraintTwoVertices;
    int constraintTwo;

    void createConstraintTwo(int II){
      constraintTwoVertices = make_pair(constraintOneVertices.second, constraintOneVertices.first);
      constraintTwo = II-1;
    }
  };

  enum sdcSolver {ScaLP, BellmanFord};

  enum sdcStatus {unfeasible, feasible};

  class SDSScheduler : public SchedulerBase, public ModuloSchedulerBase, public ILPSchedulerBase {

  public:

    /*!
		 * The SDSScheduler is a SDC and SAT based scheduler, it is based on the paper : "A Scalable Approach to Exact
     * Resource Constraint Scheduling Based on a Joint SDC and SAT Formulation" by Steve Dai, Gai Liu and Zhiru Zhang.
		 * @param g
		 * @param resourceModel
		 */
    SDSScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> &sw);

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

    //Setter Functions:

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
    map<pair<const Vertex *, const Vertex *>, int>
    passToSATSolver(map<pair<const Vertex *, const Vertex *>, bool> &shareVars,
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
    void createDependencyConstraints();

    /*!
     * @param gr Pointer to Graph which should be checked.
     * @param vertexID Id of the Vertex to search for.
     * @return true, if Vertex exists : false if Vertex does not exist;
     */
    static bool doesVertexExist(Graph *gr, int vertexID);

    void createScaLPVariables();

    pair<ScaLP::Constraint, ScaLP::Constraint> createadditionalScaLPConstraits(orderingVariabletoSDCMapping order);

    bool checkfeasibilityScaLP(vector<pair<ScaLP::Constraint, ScaLP::Constraint>> scstr, int maxLatConstr);

    map<Vertex *, int> findstarttimes(ScaLP::Result &r);

    const Vertex *getVertexFromVariable(const ScaLP::Variable &sv);

    vector<int> getSATClausesfromScaLP(list<orderingVariabletoSDCMapping> &conflicts);

    map<Vertex *, int> getFirstSDCSolution();

    /////////////////////////
    //      Variables      //
    /////////////////////////

    sdcSolver sdcS = BellmanFord;

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
    list<orderingVariabletoSDCMapping> resourceConstraints;

    /*!
     * Data Dependency Constraints and Timing Constraints from the original Graph.
     */
    map<pair<const Vertex *, const Vertex *>, int> dependencyConstraintsSDC;

    /*!
     * Mapping from SDC to SAT
     */
    map<pair<const Vertex *, const Vertex *>, int> sdcToSATMapping;

    /*!
     * Conflict Clauses detected by SDC
     */
    vector<vector<int>> conflictClauses;

    /*!
     * Flag from SAT-Solver, if problem is not satisfiable
     */
    bool unsatisiable;

    int initScheduleLength;

    /*!
     * Solver Wishlist for Scalp.
     */
    std::list<std::string> swishlist;

    /*!
     * @brief this map stores the scalp variables associated to each vertex inside the graph
     */
    std::map<const Vertex *, ScaLP::Variable> scalpVariables;

    ScaLP::Variable newVar = ScaLP::newIntegerVariable("SuperINGO", 0, ScaLP::INF());


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*!
     * This Sheduler needs to remove Edges from the Constraint Graph. The original Graph Class does not support this,
     * so we need to implement it here.
     */
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    class ConstraintGraph : public Graph {

    public:
      void removeEdge(Vertex *srcVertex, Vertex *dstVertex);

      Edge &
      createEdgeSDS(Vertex &Vsrc, Vertex &Vdst, int distance = 0, Edge::DependencyType dependencyType = Edge::Data);

      bool doesEdgeExistID(Vertex *src, Vertex *dst);

      Edge &getEdge(const Vertex *srcV, const Vertex *dstV);

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

      BellmanFordSDC(ConstraintGraph cg, Graph *g, map<Vertex *, int> &initsolution);

      /*!
       * The Solver can also be constructed by passing in a map of constraints like "V1 - V2 <= C" in this case it will
       * build it's constraint Graph by itself.
       * @param constraints like "V1 - V2 <= C"
       */
      BellmanFordSDC(map<pair<const Vertex *, const Vertex *>, int> &constraints, Graph *g,
                     map<Vertex *, int> &initsolution);

      void printConstraintGraph();

      void setquiet(bool silence);

      sdcStatus solveSDC();

      map<Vertex *, int> getVertexCosts();

      void setadditionlaConstraints(list<orderingVariabletoSDCMapping> &constraints);

      vector<vector<int>> getConflicts();

      void increaseLatency();

      void ajustConstraintGraph(int asaplength);

    private:

      void bellmanFordAlgorithm();

      /*!
       * Constraint Graph for which shortest Paths will be searched
       */
      ConstraintGraph cg;

      Graph *g;

      /*!
       * Map with Vertices of the Constraint Graph and the Cost to reach each Vertex
       */
      map<Vertex *, int> vertexCosts;

      /*!
       * Tells if Graph cg has negative cycles. If true, the SDC-System is unsolvable.
       */
      bool hasNegativeCycle;

      bool quiet = true;

      vector<vector<int>> conflictClauses;

      list<orderingVariabletoSDCMapping> additionalConstraints;

      int startID;

      map<Vertex *, int> initsol;

      set<Edge *> latencyEdges;
    };

   /*!
    * Runs the schedule loop in a
    */
    void findBestSchedule(BellmanFordSDC &sdcsol);

   /*!
    * Does trys to find a schedule.
    * @param sdcsol Reference to the SDC-Solver
    * @param latency constraint for the schedule;
    * @return
    */
    bool checkSchedule(BellmanFordSDC &sdcsol, int latency);
  };
}

#endif //USE_CADICAL

#endif //HATSCHET_SDSSCHEDULER_H
