/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Martin Kumm, Patrick Sittel (kumm, sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>

#include <ScaLP/Solver.h>

namespace HatScheT
{

/*!
 * \brief ILPSchedulerBase is the abstract base class of all schedulers that use ILP solvers.
 *
 * It heavily relies on the ScaLP library, so a ScaLP::Solver instance is created in the constructor.
 * Derived classes should implement constructProblem() (@see constructProblem()) in which the problem is constructed by defining the objective and constraints.
 */
class ILPSchedulerBase
{
public:
  /*!
   * \brief Constructor of ILPSchedulerBase
   * \param solverWishlist A list of strings that specifies the ILP solver(s) that should be used when available, e.g., {"CPLEX","Gurobi","SCIP"}
   */
  ILPSchedulerBase(std::list<std::string> solverWishlist);

  ~ILPSchedulerBase();

  /*!
   * \brief Returns the name of the selected solver.
   * \return the name of the selected solver
   */
  std::string getSolverName();

  /*!
   * \brief sets the timeout of the solver
   * \param timeoutInSeconds the timeout in seconds
   */
  void setSolverTimeout(long timeoutInSeconds);

  /*!
   * \brief setSolverQuiet manage solver output cout
   * \param b
   */
  void setSolverQuiet(bool b){this->solverQuiet = b;}
  bool getSolverQuiet(){return this->solverQuiet;}
  /*!
   * \brief printResult
   */
  void printResult(){std::cout<< this->solver->getResult() << std::endl;}
  /*!
   * \brief getScheduleFound
   * \return
   */
  bool getScheduleFound(){return this->scheduleFound;}
  /*!
   * \brief setWriteLPFile
   * \param b
   */
  void setWriteLPFile(bool b){this->writeLPFile =b;}
  /*!
   * \brief setThreads
   * \param i
   */
  void setThreads(unsigned int i){this->threads=i;}
  unsigned int getThreads(){return this->threads;}
protected:
  /*!
   * \brief This pure virtual function has to be implemented in the derived classes to construct the problem
   */
  virtual void constructProblem() = 0;
  /*!
   * \brief setObjective This pure virtual function has to be implemented in the derived classes to set the objective
   */
  virtual void setObjective() = 0;
  /*!
   * A pointer to the ScaLP solver
   */
  ScaLP::Solver *solver;
  /*!
   * \brief optimalResult
   */
  bool optimalResult;
  /*!
   * \brief scheduleFound
   */
  bool scheduleFound;
  /*!
   * \brief solverQuiet
   */
  bool solverQuiet;
  /*!
   * \brief solverTimeout default is 1 hour
   */
  long solverTimeout;
  /*!
   * \brief timeoutCounter
   */
  unsigned int timeoutCounter;
  /*!
   * \brief solvingTime
   */
  double solvingTime;
  /*!
   * \brief totalTime
   */
  double totalTime;
  /*!
   * \brief threads
   */
  unsigned int threads;
  ///
  /// \brief writeResultFile
  ///
  bool writeLPFile;
};
}
