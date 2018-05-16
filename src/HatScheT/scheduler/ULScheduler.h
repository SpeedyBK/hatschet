/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>

namespace HatScheT
{
/*!
 * \brief The ULScheduler class
 */
class ULScheduler : public SchedulerBase
{
public:
  /*!
   * \brief ASAPScheduler
   * \param g
   * \param resourceModel
   */
  ULScheduler(Graph& g,ResourceModel &resourceModel);
  /*!
   * \brief The sort_criterion enum chose which criterion should be chosen during list scheduling
   */
  enum sort_criterion
      { /*URGENCY   urgency not implemented yet
      ,*/ MOBILITY
      } sort_by;
  /*!
   * \brief schedule
   */
  virtual void schedule();
  /*!
   * \brief getII
   * \return
   */
  virtual int getII(){return this->getScheduleLength();}
  /*!
   * \brief setCriterion
   * \param sc
   */
  void setCriterion(sort_criterion &sc){this->sort_by=sc;}
  sort_criterion getCriterion(){return this->sort_by;}

protected:
private:
  /*!
   * \brief mobility
   * \param asap
   * \param alap
   * \return
   */
  std::map<Vertex*,int> *mobility(std::map<Vertex*,int> *asap, std::map<Vertex*,int> *alap);
  /*!
   * \brief zipWith zip both maps with the given function both maps need to have the same keys.
   * Used to combine the results of the ASAP- and ALAP-Schedulers.
   * \param f
   * \param m1
   * \param m2
   * \return
   */
  std::map<Vertex*,int> *zipWith(std::function<int(int,int)> f, std::map<Vertex*,int> m1, std::map<Vertex*,int> m2);
  /*!
   * \brief inputs_not_in
   * \param v
   * \param vList
   * \return
   */
  bool inputs_not_in(Vertex* v, list<Vertex*> vList);

};
}
