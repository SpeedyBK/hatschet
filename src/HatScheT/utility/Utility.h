/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

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

#pragma once

#include <string>
#include "HatScheT/Graph.h"
#include "HatScheT/utility/Binding.h"
#include "HatScheT/ResourceModel.h"
#include "HatScheT/TargetModel.h"
#include "HatScheT/base/SchedulerBase.h"
#include "HatScheT/utility/subgraphs/OccurrenceSet.h"
#include <random>

namespace HatScheT {
  //class ResourceModel; // forward declaration for cyclic include
  //class Resource; // forward declaration for cyclic include
/*!
 * \brief The Utility class use this class for utility functions
 */
  class Utility {
  public:

    static void unroll(Graph *gUnroll, ResourceModel *rmUnroll, const int &numSamples, const int &cycleLength, Graph *g,
                       ResourceModel *rm, std::map<Vertex *, std::vector<Vertex *>> *vertexMappings,
                       const bool &quiet = true);

    static int calcMinNumRegs(Graph *g, ResourceModel *rm, const std::vector<std::map<Vertex *, int>> &schedule,
                              const int &modulo);

    static int calcMinNumRegs(Graph *g, ResourceModel *rm, const std::map<Vertex *, int> &schedule, const int &II);

    static int calcMinNumChainRegs(Graph *g, ResourceModel *rm, const std::vector<std::map<Vertex *, int>> &schedule,
                                   const std::vector<std::map<const Vertex *, int>> &binding, const int &modulo);

    static int calcMinNumChainRegs(Graph *g, ResourceModel *rm, const std::map<Vertex *, int> &schedule,
                                   const std::map<const Vertex *, int> &binding, const int &II);

    /*!
     * converts the register chain binding container into the general one
     * @param g graph
     * @param rm resource model
     * @param II initiation interval
     * @param bChain binding container to convert
     * @param sched the schedule associated to this binding
     * @return converted binding container
     */
    static Binding::BindingContainer
    convertBindingContainer(Graph *g, ResourceModel *rm, const int &II, const Binding::RegChainBindingContainer &bChain,
                            std::map<Vertex *, int> sched);

    /*!
     * CASE INSENSITIVE comparisons of two strings for equality
     * @param s1 first string
     * @param s2 second string
     * @return s1 == s2
     */
    static bool iequals(const std::string &s1, const std::string &s2);

    /*!
     * worst case calculation for register and multiplexer costs for a given DFG, allocation, schedule and II
     * @param g data flow graph
     * @param rm resource model with resource limitations
     * @param times schedule times container
     * @param II initiation interval
     * @return {maxRegs, maxMuxs}
     */
    static std::pair<int, int> getMaxRegsAndMuxs(Graph *g, ResourceModel *rm, std::map<Vertex *, int> times, int II);

    /*!
     * this function computes the number of equivalent 2x1 multiplexers for a binding
     * that needs numFUConnections connections between functional units
     * this function assumes that all operations bound to a resource have the same number of inputs
     * @param numFUConnections the number of connections between FUs that will be implemented in hardware
     * @param g graph
     * @param rm resource model
     * @param numRegsWithPossibleMuxInputs the registers that can possibly have MUXs at their inputs
     *   -> for registers as register chains after FUs, this number is 0
     *   -> otherwise this number is equal to the number of allocated registers
     * @return number of equivalent 2x1 multiplexers
     */
    static double getNumberOfEquivalent2x1Muxs(int numFUConnections, Graph *g, ResourceModel *rm,
                                               int numRegsWithPossibleMuxInputs = 0);

    /*!
     * this is the counterpart to getNumberOfEquivalent2x1Muxs
     * @param num2x1Muxs the number of equivalent 2x1Muxs
     * @param g graph
     * @param rm resource model
     * @return number of interconnect lines between FUs
     */
    static double getNumberOfFUConnections(int num2x1Muxs, Graph *g, ResourceModel *rm);

    /*!
     * selects a random element from a container
     * function definition put into header to prevent linker problems in an easy way
     * feel free to change it if you don't like this...
     * @tparam iter iterator type
     * @param start start iterator
     * @param end end iterator
     * @return an iterator to a randomly selected element between start and end
     */
    template<typename iter>
    static iter selectRandomElement(iter start, iter end) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
        std::advance(start, dis(gen));
        return start;
    }

    /*!
     * \brief getNoOfResConstrVertices
     * \param rm
     * \param g
     * \return
     */
    static int getNoOfResConstrVertices(ResourceModel *rm, Graph *g);

    /*!
     * \brief getNoOfInputs determine the number of inputs a vertex v has in graph g
     * \param g
     * \param v
     * \return
     */
    static int getNoOfInputs(Graph *g, const Vertex *v);

    /*!
     * \brief getNoOfInputsWithoutRegs
     * \param g
     * \param v
     * \return
     */
    static int getNoOfInputsWithoutRegs(Graph *g, const Vertex *v);

    /*!
     * \brief getNoOfOutputs determine the number of outputs a vertex v has in graph g
     * \param g
     * \param v
     * \return
     */
    static int getNoOfOutputs(Graph *g, const Vertex *v);

    /*!
     * \brief getNoOfOutputsWithoutDistance
     * \param g
     * \param v
     * \return
     */
    static int getNoOfOutputsWithoutDistance(Graph *g, const Vertex *v);

    /*!
     * \brief calcMinII precalculate the minimum possible II before modulo scheduling.
     * minII min(ResMII,RecMII)
     * \param rm
     * \param g
     * \return
     */
    static double calcMinII(double minResII, double minRecII);

    /*!
     *
     * @param II
     * @return
     */
    static bool IIisRational(double II);

    /*!
     * @brief use a graph search to calculate the longest path
     * this methode is safer than asap scheduling
     * asap scheduling failes when there is no way to the output without algorithmic delays
     * @param g
     * @param rm
     * @return
     */
    static int getCyclesOfLongestPath(Graph *g, ResourceModel *rm, double II);

#ifdef USE_SCALP

    /*!
     * \brief calcResMII with or witout a specific target
     * \param rm
     * \return
     */
    static double calcResMII(ResourceModel *rm, Target *t = nullptr);

    /*!
     * \brief calcRecMII maximum distance of all edges
     * \param rm
     * \param g
     * \return
     */
    static double calcRecMII(Graph *g, ResourceModel *rm);

    /*!
     * \brief calcMaxII
     * \param rm
     * \param g
     * \return
     */
    static int calcMaxII(Graph *g, ResourceModel *rm);

    /*!
    * \brief calcAbsolutMaxII calculate the absolute max II when all resources are limited to 1 FU
    * \param rm
    * \param g
    * \return
    */
    static int calcAbsolutMaxII(Graph *g, ResourceModel *rm);

    /*!
     * calculate and return the critical path of a graph
     * @param g
     * @param rm
     * @return
     */
    static int getCriticalPath(Graph *g, ResourceModel *rm);

    /*!
     * @brief determine ILP based asap schedule and return schedule length
     * @param g
     * @param rm
     * @return
     */
    static int getILPASAPScheduleLength(Graph *g, ResourceModel *rm);

#endif

    /*!
     * same as above but using a depth first search (DFS) instead of ILP
     * Todo: fix bugs before usage - this does not work, yet!
     * @param g
     * @param rm
     * @return
     */
    static double calcRecMIIDFS(Graph *g, ResourceModel *rm);

    /*!
     * @brief determine asap schedule and return schedule length
     * @param g
     * @param rm
     * @return
     */
    static int getASAPScheduleLength(Graph *g, ResourceModel *rm);

    /*!
     * @brief determine asap schedule for only unlimited resources
     * @param g
     * @param rm
     * @return
     */
    static int getASAPNoHCScheduleLength(Graph *g, ResourceModel *rm);

    /*!
     * \brief sumOfStarttimes
     * \param startTimes
     * \return
     */
    static int sumOfStarttimes(std::map<Vertex *, int> &startTimes);

    /*!
     * \brief resourceAvailable
     * \param startTimes map of starttimes
     * \param ResourceModel the used resource model
     * \param r the resource that is looked for
     * \param checkV avoid self counting
     * \param timeStep the time step that is checked
     * \return
     */
    static bool
    resourceAvailable(std::map<Vertex *, int> &startTimes, ResourceModel *rm, const Resource *r, Vertex *checkV,
                      int timeStep);

    /*!
     * \brief edgeIsInGraph
     * \param g
     * \param e
     * \return
     */
    static bool edgeIsInGraph(Graph *g, const Edge *e);

    /*!
     * \brief isInput returns true if this vertex is no inputs or only inputs of edges with a distance
     * \param g
     * \param v
     * \return
     */
    static bool isInput(Graph *g, Vertex *v);

    /*!
     * \brief occurrencesAreConflictFree
     * \param occ1
     * \param occ2
     * \return
     */
    static bool existEdgeBetweenVertices(Graph *g, Vertex *Vsrc, Vertex *Vdst);

    /*!
     *
     * @param occ1
     * @param occ2
     * @return
     */
    static bool occurrencesAreConflictFree(Occurrence *occ1, Occurrence *occ2);

    /*!
     * \brief occurenceSetsAreConflictFree
     * \param occs1
     * \param occs2
     * \return
     */
    static bool occurenceSetsAreConflictFree(OccurrenceSet *occs1, OccurrenceSet *occs2);

    /*!
     * \brief vertexInOccurrence
     * \param occ
     * \param v
     * \return
     */
    static bool vertexInOccurrence(Occurrence *occ, Vertex *v);

    /*!
     * \brief vertexInOccurrenceSet
     * \param occS
     * \param v
     * \return
     */
    static bool vertexInOccurrenceSet(OccurrenceSet *occS, Vertex *v);

    /*!
     * \brief allInputsAreRegisters
     * \param g
     * \param v
     * \return
     */
    static bool allInputsAreRegisters(Graph *g, Vertex *v);

    /*!
     * print a schedule to console
     * @param schedule
     */
    static void printSchedule(map<Vertex *, int> &schedule);

    /*!
     * \brief printBinding
     * \param binding
     * \param rm
     */
    static void printBinding(map<const Vertex *, int> &binding, ResourceModel &rm);

    /*!
     * \brief calcUsedOperationsOfBinding
     * \param binding
     * \param rm
     * \param r
     * \return
     */
    static int calcUsedOperationsOfBinding(map<const Vertex *, int> &binding, ResourceModel &rm, Resource *r);

    /*!
     *
     * @param rm
     * @param t
     * @return
     */
    static bool resourceModelAndTargetValid(ResourceModel &rm, Target &t);

    /*!
     *
     * @param g
     * @param rm
     * @return
     */
    static bool everyVertexisRegistered(Graph &g, ResourceModel &rm);

    /*!
     * return numerator and denominator of a rational number stored in double
     * If the number is not rational, or a specific budget is hit, it returns the pair <0,0>
     * As no rational number has a 0 as a denominator, this can be used to test the result
     * @param x
     * @return
     */
    static pair<int, int> splitRational(double x);

    /*! Safely rounds a double down to the next integer, taking precision issues into account
    * Inputing 3.9999999999999968 will return 4
    * @param x
    * @return
    */
    static int safeRoundDown(double x);

    /*!
     * @brief print the modulo reservation tables of rational II schedule and binding
     * @param sched
     * @param rm
     * @param modulo
     * @param initIntervalls
     */
    static void printRationalIIMRT(map<Vertex *, int> sched, vector<std::map<const Vertex *, int> > ratIIbindings,
                                   ResourceModel *rm, int modulo, vector<int> initIntervalls);

    /*!
     * for a rational II schedule this returns the sample index and the offset between two samples depending on the edge distance
     * this is needed for data dependency constraints
     * @param distance
     * @param sample
     * @return
     */
    static std::pair<int, int>
    getSampleIndexAndOffset(int distance, const int &sample, const int &samples, const int &modulo);

    /*!
     * same as above but this returns the distance of the edge in the unrolled graph instead of the offset for scheduling
     */
    static std::pair<int, int> getSampleIndexAndDistance(int distance, const int &sample, const int &samples);

    /*!
     * Returns the tranposed graph of g and a map which maps the vertices of the transposed graph to the vertices of g.
     * @param g is the graph to transpose
     * @param gT is the transposed graph
     * @return a mapping from originalVertex -> transposedVertex
     */
    static std::map<Vertex *, Vertex *> transposeGraph(Graph *g, Graph *h);

    /*!
     * Checks if graph g has cycles. https://www.geeksforgeeks.org/detect-cycle-in-a-graph/
     * @param Graph g
     * @return true if graph is cyclic, false if graph is acyclic.
     */
    static bool iscyclic(Graph *g);

    /*!
     * defines non-rectangular MRT height
     * @param n number of vertices of that resource
     * @param M cycle length
     * @param tau modulo slot
     * @return mrt height in modulo slot tau
     */
    static int hFunction(double n, double M, int tau);

    /*!
   * min latency estimation for a given graph/resource model/II (uses ILP and is slower than calcMinLatencyEstimation())
   * @param g: graph
   * @param rm: resource model
   * @param tMin: SDC-ASAP times for all vertices in g
   * @param tMax: SDC-ALAP times for all vertices in g
   * @param II: initiation interval
   * @param sw: ILP solver wishlist
   * @param timeout: timeout in seconds for ILP solver
   * @param threads: amount of threads the ILP solver can use. 'threads <= 0' means no limitation.
   * @return min latency estimation or -1 if something went terribly wrong (e.g. you requested II < minResII)
   */
    static int getMinLatency(Graph *g,
                             ResourceModel *rm,
                             const std::map<Vertex *, int> &tMin,
                             const std::map<Vertex *, int> &tMax,
                             const int &II,
                             const std::list<std::string> &sw = {"Gurobi", "CPLEX", "LPSolve", "SCIP"},
                             const unsigned int &timeout = 300,
                             const int &maxScheduleLength = -1,
                             const int8_t &threads = -1,
                             bool quiet = true);

    /*!
     * max latency estimation for a given graph/resource/II
     * look at getLatencyEstimation() first please.
     * @param g: graph
     * @param rm: resource model
     * @param tMin: SDC-ASAP times for all vertices in g
     * @param tMax: SDC-ALAP times for all vertices in g
     * @param II: initiation interval
     * @param quiet: true surpresses print statements
     * @return max latency estimation
     */
    static int calcMaxLatencyEstimation(Graph *g, ResourceModel *rm,
                                        const map<Vertex *, int> &tMin,
                                        const map<Vertex *, int> &tMax,
                                        const int &II, bool quiet = true);

    /*!
     * Creates ALAP and ASAP Starttimes for all vertices of a given graph/II, ignores resource limitations
     * look at getLatencyEstimation() first please.
     * @param g: graph
     * @param rm: resource model
     * @param II: initiation interval
     * @param quiet: true surpresses print statements
     * @return two maps "<vertex*, int>". First map contains ASAP starttimes, second map contains ALAP starttimes.
     */
    static pair<map<Vertex *, int>, map<Vertex *, int>> getSDCAsapAndAlapTimes(Graph *g, ResourceModel *rm,
                                                                               const double &II, bool quiet = true);

    /*!
     * Calcutates the latency of SDC-Schedules
     * look at getLatencyEstimation() first please.
     * @param vertexStarttimes Starttimes of each operation.
     * @param newToOld mapping from SDC-Graphs to original Graph
     * @param rm: resource model
     * @param quiet: true surpresses print statements
     * @return latency of the given schedule
     */
    static int getSDCScheduleLength(const unordered_map<Vertex *, int> &vertexStarttimes,
                                    const unordered_map<Vertex *, Vertex *> &newToOld,
                                    ResourceModel *rm, bool quiet = true);

    /*!
     * calculates a minimum latency wich is requiered to satisfy all resource constraints (faster than ILP).
     * look at getLatencyEstimation() first please.
     * @param g: graph
     * @param rm: resource model
     * @param tMin: SDC-ASAP times for all vertices in g
     * @param tMax: SDC-ALAP times for all vertices in g
     * @param II: initiation interval
     * @param increment: Amount of timeslots by that the latency has to be increased to resolve a resource conflict in a modslot.
     *                   Function updates this value by itself.
     * @param minLatency: A candidate Latency.
     * @param modAsapLength: Length of the SDC-Schedules
     * @param quiet: true surpresses print statements
     * @return minimum latency to satisfy resource constraints.
     */
    static bool calcMinLatencyEstimation(Graph *g, ResourceModel *rm,
                                         const map<Vertex *, int> &tMin,
                                         const map<Vertex *, int> &tMax,
                                         const int &II,
                                         int &increment,
                                         int &minLatency,
                                         int &modAsapLength,
                                         bool quiet = true);

    enum class latencyBounds {
      minLatency, maxLatency, both
    };
    struct LatencyEstimation {
      int minLat = -1;
      int maxLat = -1;
      map<Vertex*, int> asapStartTimes;
      map<Vertex*, int> alapStartTimes;
    };
    /*!
     * Wrapperfunction for latency estimation. Manages calls for calcMaxLatencyEstimation(), getSDCAsapAndAlapTimes(),
     * getSDCScheduleLength() and calcMinLatencyEstimation(). It calculates a min latency estimation, a max latency
     * estimation, or both for a given Graph, ResourceModel and II.
     * @param g: graph
     * @param rm: resource model
     * @param II: initiation interval
     * @param lb: Switch wich bound (min, max or both) should be calculated.
     * @param quiet: true surpresses print statements
     * @return pair<int, int>: first element = min latency estimation, second element = max latency estimation.
     */
    static LatencyEstimation getLatencyEstimation(Graph *g, ResourceModel *rm,
                                               double II,
                                               latencyBounds lb = latencyBounds::both,
                                               bool quiet = true);

  private:

    /*!
     * Helperfunction for iscyclic(), basicly a variation of a DFS.
     */
    static bool iscyclicHelper(Graph *g, Vertex *V, map<Vertex *, bool> &visited, map<Vertex *, bool> &recStack);

    static void
    cycle(const Edge *e, vector<Vertex *> &visited, int &currLength, Graph *g, ResourceModel *rm, double II);

  };
}
