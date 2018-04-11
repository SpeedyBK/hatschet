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

namespace HatScheT
{
/*!
 * \brief The Utility class use this class for utility functions
 */
class Utility
{
public:
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
 static int calcRecMII(Graph *g);

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
};
}
