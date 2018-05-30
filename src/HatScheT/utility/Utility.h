/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once
#include <string>
#include "HatScheT/Graph.h"
#include "HatScheT/ResourceModel.h"
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
  * \brief examplUtilityFunction
  * \param rm
  * \param g
  * \return
  */
 static bool examplUtilityFunction(ResourceModel* rm, Graph *g);
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
  * \brief calcMinII precalculate the minimum possible II before modulo scheduling. minII is based on graph and resource model
  * minII min(ResMII,RecMII)
  * \param rm
  * \param g
  * \return
  */
 static int calcMinII(ResourceModel* rm, Graph *g);
 /*!
  * \brief calcResMII
  * \param rm
  * \param g
  * \return
  */
 static int calcResMII(ResourceModel* rm, Graph *g);
 /*!
  * \brief calcRecMII maximum distance of all edges
  * \param rm
  * \param g
  * \return
  */
 static int calcRecMII(ResourceModel *rm, Graph *g);
 /*!
  * \brief calcMaxII
  * \param sb
  * \return
  */
 static int calcMaxII(SchedulerBase* sb);
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
  * \brief evaluateSchedulers experiment interface for hatschet paper
  * \param g
  * \param resourceModel
  * \param solverWishlist
  */
 static void evaluateSchedulers(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, string logFileName);
 /*!
  * \brief adaptiveScheduling start a scheduling experiment for adaptive scheduling
  * ToDo pass parameters instead of calculating them
  * \param g
  * \param resourceModel
  * \param solverWishlist
  * \param logFileName
  */
 static void adaptiveScheduling(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, string logFileName);
 /*!
  * \brief compareRegisterUsage this function is used to evaluate the experiments published at fpl
  * \param g
  * \param resourceModel
  * \param solverWishlist
  * \param logFileName
  */
 static void compareRegisterUsage(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, string logFileName);
};
}
