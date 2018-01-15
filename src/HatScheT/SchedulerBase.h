#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>

namespace HatScheT
{


/*!
 * \brief ILPSchedulerBase is the abstract base class of all schedulers.
 *
 * Derived classes should implement schedule() which has to fill the startTimes data structure.
 */
class SchedulerBase
{
public:

  SchedulerBase(Graph& g);

  virtual void schedule() = 0;

  /*!
   * \brief Returns the start times of all nodes
   * \return The start times of all nodes
   */
  std::map<Vertex,int>& getStartTimes(){ return startTimes; }

  /*!
   * \brief Gets the start time of vertex v
   * \param v The vertex for which the start time is requested
   * \return The start time of v or -1 if vertex does not exist
   */
  int getStartTime(Vertex &v);

  /*!
   * \brief Returns the length of the schedule (i.e., the maximum start time)
   * \return The length of the schedule (i.e., the maximum start time)
   */
  int getScheduleLenght();

protected:

  /*!
   * \brief Container for the start times
   */
  std::map<Vertex,int> startTimes;

  /*!
   * \brief A reference to the data dependency graph
   */
  Graph& g;
};
}
