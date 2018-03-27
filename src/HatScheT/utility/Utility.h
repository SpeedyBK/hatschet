#pragma once
#include <string>
#include "HatScheT/Graph.h"
#include "HatScheT/ResourceModel.h"

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

};
}
