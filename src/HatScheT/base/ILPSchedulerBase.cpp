#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/Exception.h>

namespace HatScheT
{

ILPSchedulerBase::ILPSchedulerBase(std::list<std::string> solverWishlist) : solver(new ScaLP::Solver(solverWishlist))
{ 
  this->threads = 0;
  this->timeoutCounter = 0;
  this->solverQuiet = false;
  this->optimalResult = false;
  this->solverTimeout = 3600;
  this->solvingTime = 0;
  this->totalTime = 0;
  this->scheduleFound = false;
  this->writeLPFile = false;
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
