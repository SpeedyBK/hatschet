#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>

namespace HatScheT
{

class GraphBasedMs : public SchedulerBase, ModuloSchedulerBase
{
public:
    /*!
   * \brief GraphBasedMs
   * \param g
   * \param resourceModel
   */
  GraphBasedMs(Graph& g,ResourceModel &resourceModel);
  /*!
   * \brief schedule determine a II using the graph based modulo scheduler
   */
  virtual void schedule();

protected:
private:
    //implent methods

};
}
