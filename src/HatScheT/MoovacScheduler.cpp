#include <HatScheT/MoovacScheduler.h>

namespace HatScheT
{

MoovacScheduler::MoovacScheduler(Graph &g, std::list<std::string>  solverWishlist, ResourceModel &resourceModel) : SchedulerBase(g), ILPSchedulerBase(solverWishlist), ResourceConstrainedSchedulerBase(resourceModel)
{
}

void MoovacScheduler::constructProblem()
{
  //solver->addConstraint(...);
}

void MoovacScheduler::schedule()
{
  //construct the problem
  constructProblem();

  //solve it
  solver->solve();

  //fill solution structure
  //startTimes[...] = ...
}

}
