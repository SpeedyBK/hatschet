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
 static bool edgeIsInGraph(Graph* g, Edge* e);
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

 static bool resourceModelAndTargetValid(ResourceModel &rm, Target& t);

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
};
}
