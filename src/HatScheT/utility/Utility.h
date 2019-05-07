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
#include "HatScheT/ResourceModel.h"
#include "HatScheT/TargetModel.h"
#include "HatScheT/base/SchedulerBase.h"
#include "HatScheT/utility/subgraphs/OccurrenceSet.h"

namespace HatScheT
{
/*!
 * \brief The Utility class use this class for utility functions
 */
class Utility
{
public:
  /*!
   * \brief getNoOfResConstrVertices
   * \param rm
   * \param g
   * \return
   */
  static int getNoOfResConstrVertices(ResourceModel* rm, Graph *g);
 /*!
  * \brief getNoOfInputs determine the number of inputs a vertex v has in graph g
  * \param g
  * \param v
  * \return
  */
 static int getNoOfInputs(Graph* g, const Vertex* v);
 /*!
  * \brief getNoOfInputsWithoutRegs
  * \param g
  * \param v
  * \return
  */
 static int getNoOfInputsWithoutRegs(Graph* g, const Vertex* v);
 /*!
  * \brief getNoOfOutputs determine the number of outputs a vertex v has in graph g
  * \param g
  * \param v
  * \return
  */
 static int getNoOfOutputs(Graph* g, const Vertex* v);
 /*!
  * \brief getNoOfOutputsWithoutDistance
  * \param g
  * \param v
  * \return
  */
 static int getNoOfOutputsWithoutDistance(Graph* g, const Vertex* v);
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
  * asap scheduling failes when there is no way to the ouput without algorithmic delays
  * @param g
  * @param rm
  * @return
  */
 static int getCyclesOfLongestPath(Graph* g, ResourceModel* rm, double II);
#ifdef USE_SCALP
 /*!
  * \brief calcResMII with or witout a specific target
  * \param rm
  * \return
  */
 static double calcResMII(ResourceModel* rm, Target* t=nullptr);
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
#endif
 /*!
  * \brief sumOfStarttimes
  * \param startTimes
  * \return
  */
 static int sumOfStarttimes(std::map<Vertex*,int>& startTimes);
 /*!
  * \brief resourceAvailable
  * \param startTimes map of starttimes
  * \param ResourceModel the used resource model
  * \param r the resource that is looked for
  * \param checkV avoid self counting
  * \param timeStep the time step that is checked
  * \return
  */
 static  bool resourceAvailable(std::map<Vertex*,int>& startTimes, ResourceModel *rm, const Resource *r, Vertex *checkV, int timeStep);
 /*!
  * \brief edgeIsInGraph
  * \param g
  * \param e
  * \return
  */
 static bool edgeIsInGraph(Graph* g, const Edge* e);
 /*!
  * \brief isInput returns true if this vertex is no inputs or only inputs of edges with a distance
  * \param g
  * \param v
  * \return
  */
 static bool isInput(Graph* g, Vertex* v);
 /*!
  * \brief occurrencesAreConflictFree
  * \param occ1
  * \param occ2
  * \return
  */
 static bool existEdgeBetweenVertices(Graph* g, Vertex *Vsrc, Vertex *Vdst);
 /*!
  *
  * @param occ1
  * @param occ2
  * @return
  */
 static bool occurrencesAreConflictFree(Occurrence* occ1, Occurrence* occ2);
 /*!
  * \brief occurenceSetsAreConflictFree
  * \param occs1
  * \param occs2
  * \return
  */
 static bool occurenceSetsAreConflictFree(OccurrenceSet* occs1, OccurrenceSet* occs2);
 /*!
  * \brief vertexInOccurrence
  * \param occ
  * \param v
  * \return
  */
 static bool vertexInOccurrence(Occurrence* occ, Vertex* v);
 /*!
  * \brief vertexInOccurrenceSet
  * \param occS
  * \param v
  * \return
  */
 static bool vertexInOccurrenceSet(OccurrenceSet* occS, Vertex* v);
 /*!
  * \brief allInputsAreRegisters
  * \param g
  * \param v
  * \return
  */
 static bool allInputsAreRegisters(Graph* g, Vertex *v);
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
 static bool resourceModelAndTargetValid(ResourceModel &rm, Target& t);
 /*!
  *
  * @param g
  * @param rm
  * @return
  */
 static bool everyVertexisRegistered(Graph& g, ResourceModel &rm);
 /*!
  * return numerator and denominator of a rational number stored in double
  * If the number is not rational, or a specific budget is hit, it returns the pair <0,0>
  * As no rational number has a 0 as a denominator, this can be used to test the result
  * @param x
  * @return
  */
 static pair<int,int> splitRational(double x);

 /*! Safely rounds a double down to the next integer, taking precision issues into account
 * Inputing 3.9999999999999968 will return 4
 * @param x
 * @return
 */
 static int safeRoundDown(double x);
/*!
 * @brief create simple binding that just assignes resources as vertices appear in the schedule
 * @param sched
 * @param rm
 * @param II
 * @return
 */
 static std::map<const Vertex*,int> getSimpleBinding(map<Vertex*, int> sched, ResourceModel* rm, int II);
/*!
 * UNDER CONSTRUCTION: this function is meant to determine MUX optimal FU bindings after modulo scheduling
 * @param sched
 * @param rm
 * @param II
 * @return
 */
 static std::map<const Vertex*,int> getMUXOptimalBinding(map<Vertex*, int> sched, ResourceModel* rm, int II);
 /*!
  * create a simple binding for a rational II schedule
  * NOTE the binding might result in a bad register and MUX allocation for lifetime variables as it is a very simple utility function
  * @param sched
  * @param rm
  * @param II
  * @param initIntervalls
  * @return
  */
 static vector<std::map<const Vertex*,int> > getSimpleRatIIBinding(map<Vertex*, int> sched, ResourceModel* rm, int modulo, vector<int> initIntervalls);
 /*!
  * @brief print the modulo reservation tables of rational II schedule and binding
  * @param sched
  * @param rm
  * @param modulo
  * @param initIntervalls
  */
 static void printRationalIIMRT(map<Vertex*, int> sched, vector<std::map<const Vertex*,int> > ratIIbindings,
   ResourceModel* rm, int modulo, vector<int> initIntervalls);
#ifdef USE_SCALP
    static vector<std::map<const Vertex*,int> > getBruteForceRatIIBinding(map<Vertex*, int> sched, Graph* g, ResourceModel* rm,
      int modulo, vector<int> initIntervalls, map<Edge*, pair<int, int> > edgePortMappings);
   /*!
   * @brief create an ilp-based binding for a rational II schedule
   * the goal is to minimize MUX and register allocation
   * if you try to understand this: good luck, may the force be with you :-) for questions ask sittel@uni-kassel.de
   * rational II enhancement of 'Simultaneous FU and Register Binding Based on Network Flow Method'
   * Jason Cong and Junjuan Xu
   * DATE 2008
   * @param sched
   * @param rm
   * @param modulo
   * @param initIntervalls
   * @return
   */
   static vector<std::map<const Vertex*,int> > getILPBasedRatIIBinding(map<Vertex*, int> sched, Graph* g, ResourceModel* rm,
     int modulo, vector<int> initIntervalls, std::list<std::string> sw = {}, int timeout=300);
 /*!
  * @brief getILPMinRegBinding create a binding with minimal number of lifetime registers (assuming register sharing!)
  * @param sched schedule times
  * @param g graph
  * @param rm resource model
  * @param II
  * @param sw solver wishlist
  * @param timeout timeout for ilp solver
  * @return binding
  */
 static std::map<const Vertex*,int> getILPMinRegBinding(map<Vertex*, int> sched, Graph *g, ResourceModel* rm, int II, std::list<std::string> sw = {}, int timeout=300);
#endif

  private:

  static void cycle(const Edge* e, vector<Vertex*>& visited, int& currLength, Graph* g, ResourceModel* rm, double II);
};
}
