#pragma once

#include <HatScheT/SchedulerBase.h>
#include <HatScheT/ILPSchedulerBase.h>
#include <HatScheT/ModuloSchedulerBase.h>
#include <HatScheT/ResourceConstrainedSchedulerBase.h>

namespace HatScheT
{

/*!
 * \brief Implementation of the Moovac scheduler
 *
 * For the details of the Moovac scheduler, see Oppermann, Koch, Reuter-Oppermann  & Sinnen, ILP-based modulo scheduling for high-level synthesis,
 * Presented at the Proceedings of the International Conference on Compilers, Architectures and Synthesis for Embedded Systems (CASES) 2016
 *
 */
class MoovacScheduler :  public SchedulerBase, ILPSchedulerBase, ModuloSchedulerBase, ResourceConstrainedSchedulerBase
{
public:
  MoovacScheduler(Graph& g, std::list<std::string> solverWishlist, ResourceModel &resourceModel);

  virtual void schedule();

  virtual void constructProblem();

protected:
};
}
