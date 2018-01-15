#include <HatScheT/ILPSchedulerBase.h>
#include <HatScheT/Exception.h>

namespace HatScheT
{

ILPSchedulerBase::ILPSchedulerBase(std::list<std::string> solverWishlist) : solver(new ScaLP::Solver(solverWishlist))
{
}

ILPSchedulerBase::~ILPSchedulerBase()
{
  delete solver;
}

std::string ILPSchedulerBase::getSolverName()
{
  return solver->getBackendName();
}

void ILPSchedulerBase::setSolverTimeout(long timeoutInSeconds)
{
  solver->timeout = timeoutInSeconds;
}

}
