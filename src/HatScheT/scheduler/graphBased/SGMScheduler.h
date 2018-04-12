#pragma once

#include <HatScheT/MoovacScheduler.h>
#include <vector>

namespace HatScheT
{

class SGMScheduler : public MoovacScheduler
{
public:
    /*!
   * \brief GraphBasedMs
   * \param g
   * \param resourceModel
   */
  SGMScheduler(Graph& g,ResourceModel &resourceModel, std::list<std::string> solverWishlist);

  /*!
   * \brief schedule determine a II using the graph based modulo scheduler
   */
  virtual void schedule();

protected:
private:

};
}
