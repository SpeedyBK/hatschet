/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>
#include <ctime>

#include <ScaLP/Solver.h>

namespace HatScheT
{

/*!
 * \brief ILPSchedulerBase is the abstract base class of all schedulers that use ILP solvers.
 *
 * It heavily relies on the ScaLP library, so a ScaLP::Solver instance is created in the constructor. (see www.uni-kassle.de/go/scalp)
 *
 * Further Information about ScaLP can be found at : Sittel, Schönwälder, Kumm, and Zipf: ScaLP: A Light-Weighted (MI)LP-Library, Presented at the 21th MBMV 2018.
 *
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
  long getSolverTimeout(){return this->solverTimeout;}
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
   * \brief setWriteLPFile
   * \param b
   */
  void setWriteLPFile(bool b){this->writeLPFile =b;}
  /*!
   * \brief setThreads
   * \param i
   */
  void setThreads(unsigned int i){
    this->threads=i;
    this->solver->threads=i;
  }
  unsigned int getThreads(){return this->threads;}
  /*!
   * access ilp solver status (OPTIMAL, TIMEOUT, ...) see ScaLP paper for more information
   * @return
   */
  virtual ScaLP::status getScaLPStatus(){return this->stat;}
  /*!
   * @brief this functions retuns the solving time in seconds
   * (if set by the schedulers, -1.0 otherwise)
   * @return
   */
  double getSolvingTime(){ return this->solvingTime; }
  /*!
   * @brief default false to avoid errors using SCIP and LPSolve
   * @param p
   */
  void setPresolve(bool p) { this->solver->presolve = p; }
  /*!
   * Getter for this->solver->presolve.
   * @return this->solver->presolve
   */
  bool getPresolve(){return this->solver->presolve;}
  /*!
   * use this container to specify an initial solution from which the solver should start its solving process
   * for rational II schedulers only
   */
  std::vector<std::map<Vertex *, int>> initialSolutionRatII;
  /*!
   * use this container to specify an initial solution from which the solver should start its solving process
   * for integer II schedulers only
   */
  std::map<Vertex *, int> initialSolutionIntII;

protected:
  /*!
   * store the status of ilp-based scheduling
   */
  ScaLP::status stat;
  /*!
   * \brief This pure virtual function has to be implemented in the derived classes to construct the problem
   */
  virtual void constructProblem() = 0;
  /*!
   * \brief setObjective This pure virtual function has to be implemented in the derived classes to set the objective
   */
  virtual void setObjective() = 0;
  /*!
   * setObjective This pure virtual function has to be implemented in the derived classes
   * to set the reset/clear the respective containers
   * if needed
   */
  virtual void resetContainer() = 0;
  /*!
   * A pointer to the ScaLP solver
   */
  ScaLP::Solver *solver;
  /*!
   * store the results of ilp-based scheduling
   */
  ScaLP::Result r;
  /*!
   * \brief optimalResult
   */
  bool optimalResult;
  /*!
   * \brief solverQuiet
   */
  bool solverQuiet;
  /*!
   * \brief solverTimeout in seconds, default is 3600sec=1 hour
   */
  long solverTimeout;
  /*!
   * \brief solvingTime
   */
  double solvingTime;
  /*!
   * \brief threads
   */
  unsigned int threads;
  /*!
   * flag to chose whether to write the lp file
   */
  bool writeLPFile;
  /*!
   * @brief timestamp that can be used for time measurements
   */
  clock_t begin;
  /*!
   * @brief timestamp that can be used for time measurements
   */
  clock_t end;
};
}
