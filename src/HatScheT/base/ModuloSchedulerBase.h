#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <map>

namespace HatScheT
{

/*!
 * \brief ModuloSchedulerBase is the base class of all schedulers that solve the modulo scheduling problem.
 *
 * It contains helper functions to determine bounds on the II
 */
class ModuloSchedulerBase
{
public:
  /*!
   * \brief getII
   * \return
   */
  int getII() { return II; }
  /*!
   * \brief getMinII
   * \return
   */
  int getMinII(){return this->minII;}
  /*!
   * \brief getMaxII
   * \return
   */
  int getMaxII(){return this->maxII;}
protected:
  /*!
   * \brief computeMinII
   * \param g
   * \param rm
   */
  int computeMinII(Graph* g, ResourceModel* rm);
  /*!
   * \brief computeMaxSL compute an upper bound for the sample latency
   * \return
   */
  int computeMaxSL();
  /*!
   * \brief minII lower bound for II
   */
  unsigned int minII;
  /*!
   * \brief maxII upper bound for II
   */
  unsigned int maxII;
  /*!
   * \brief IIthe initian interval for a modulo schedule
   */
  unsigned int II;

};

}
