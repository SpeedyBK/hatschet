#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>
#include <HatScheT/ResourceModel.h>

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

  SchedulerBase(Graph& g,ResourceModel &resourceModel);
  /*!
   * \brief schedule main method for all schedulers, not implemented in base class
   */
  virtual void schedule() = 0;
  /*!
   * \brief Returns the start times of all nodes
   * \return The start times of all nodes
   */
  std::map<Vertex*,int>& getStartTimes(){ return startTimes; }

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
  int getScheduleLength();
  /*!
   * \brief setMaxLatencyConstraint manage the allowed maximum latency of the schedule
   * \param l
   */
  void setMaxLatencyConstraint(int l){this->maxLatencyConStraint =l;}
  int getMaxLatencyConstraint(){return this->maxLatencyConStraint;}
  /*!
   * \brief getBindings calculate a naive binding in base class
   * should be overloaded by scheduler that determine specific bindings
   * \return
   */
  virtual std::map<const Vertex*,int> getBindings();
  /*!
   * \brief getLifeTimes II = scheduleLength
   * \return
   */
  virtual std::map<Edge*,int> getLifeTimes();
protected:
  /*!
   * \brief resourceModel
   */
  ResourceModel &resourceModel;
  /*!
   * \brief maxLatencyConStraint default is -1 (unlimited)
   */
  int maxLatencyConStraint;
  /*!
   * \brief Container for the start times
   */
  std::map<Vertex*,int> startTimes;
  /*!
   * \brief A reference to the data dependency graph
   */
  Graph& g;
};
}
