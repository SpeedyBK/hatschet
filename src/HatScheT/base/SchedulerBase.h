/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Benjamin Lagershausen-Keßler (benjaminkessler@student.uni-kassel.de)
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

#include <map>
#include <chrono>
#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
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
  virtual ~SchedulerBase();
  /*!
   * Mainly for debugging.
   * @return Name of the scheduler
   */
  virtual string getName() { return "Unnamed Scheduler"; }
  /*!
   * \brief schedule main method for all schedulers, not implemented in base class
   */
  virtual void schedule() = 0;
  /*!
   * \brief Returns the start times of all nodes
   * \return The start times of all nodes
   */
  std::map<Vertex*,int>& getSchedule();
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
  virtual int getScheduleLength();
  /*!
   * \brief setMaxLatencyConstraint manage the allowed maximum latency of the schedule
   * \param l
   */
  void setMaxLatencyConstraint(int l) { this->maxLatencyConstraint =l; }
  /*!
   * Getter for maxLatencyConstraint
   * @return maxLatencyConstraint
   */
  int getMaxLatencyConstraint() const { return this->maxLatencyConstraint; }
  /*!
   * \brief getBindings calculate a naive binding in base class
   * should be overloaded by scheduler that determine specific bindings
   * \return
   */
  virtual std::map<const Vertex*,int> getBindings();
  /*!
   * \brief getLifeTimes using the determined II
   * \return
   */
  virtual std::map<Edge*,int> getLifeTimes();
  /*!
   * \brief Generates an HTML file which contains the schedule chart
	 * \param filename is the file name where to store the chart
   */
  void writeScheduleChart(string filename);
  /*!
   * \brief getII getter for the II. The II has to be set by each scheduler when schedule is done (also non modulo schedulers!)
   * \return initiation interval (II)
   */
  virtual double getII() { return this->II;}
  /*!
   *
   * @return the used graph description of the scheduling problem
   */
  Graph* getGraph(){return &this->g;}
  /*!
   *
   * @return the used resource Model
   */
  ResourceModel* getResourceModel(){return &this->resourceModel;}
  /*!
 * \brief getScheduleFound
 * \return
 */
  bool getScheduleFound() const { return this->scheduleFound; }
  /*!
   * use this flag to control the mux optimal binding algorithm
   * NOTE: EXPERIMENTAL
   * @param b
   */
  void setUseOptBinding(bool b){ this->useOptimalBinding = b; }
  /*!
   * Getter for useOptimalBinding
   * @return useOptimalBinding
   */
  bool getUseOptBinding() const { return this->useOptimalBinding; }
  /*!
   * enable/disable debug couts
   * @param q
   */
  void setQuiet(bool q) { this->quiet = q; }

  /*!-----------------------------------------------------------
   *  Timetracking in HatScheT:
   *  Based on C++ Chrono Library.
   *
   *  It is designed to easily get time measurements.
   *
   *  There are 4 Variables for timetracking:
   *  - solverTimeout: The time a scheduler has to solve one
   *  specific II.
   *  - solvingTimePerIteration: The time between calling startTimeTracking()
   *  and calling endTimeTracking().
   *  - solvingTimeTotal: Inteded for iterative use. Sums up
   *  solvingTimePerIteration after each iteration.
   *  - timeRemaining: Basically timeBugdet - solvingTimePerIteration. The time
   *  which is left after a timetracked function. Within one II-Iteration.
   *
   *  Functions public:
   *  - setSolverTimeout(): Takes a double value in seconds and
   *  sets solverTimeout accordingly. If not called, solverTimeout
   *  will be set by the constructor to INT32_MAX (about 68 Years ^^ ).
   *  !!!
   *  If you are using ILP-, SAT-, SMT- or other solvers, this function
   *  has to be overwritten.
   *  !!!
   *  - getSolvingTimePerIteration() and getTimeRemaining():
   *  Just getters for the variables, from outside a Scheduler
   *  only the last II-Iteration is accessible.
   *  - getSolverTimeout() getter for the solverTimeout-Variable.
   *
   *  Functions protected:
   *  - startTimeTracking(): Starts timetracking.
   *  - endTimeTracking(): Ends timetracking and calculates
   *  solvingTimePerIteration and timeRemaining.
   *  - updateSolvingTimeTotal(): For iterative use, it sums up
   *  the solving times measured in each iteration. And is
   *  used in the LayerClasses.
   *
   *  Benjamin Lagershausen-Keßler
   ----------------------------------------------------------- */
  /*!
   * Sets the amount of time which is available for each iteration.
   *  !!!
   *  If you are using ILP-, SAT-, SMT- or other solvers, this function
   *  has to be overwritten by your Scheduler.
   *  !!!
   * Default is INT32_MAX.
   * @param seconds
   */
  virtual void setSolverTimeout (double seconds);
  /*!
   * Getter for the solverTimeout Variable.
   * @return
   */
  double getSolverTimeout() const { return this->solverTimeout; }
  /*!
   * Getter for solvingTimePerIteration, look above.
   * From outside of the Scheduler only accessible for the last Iteration.
   * @return solvingTimePerIteration if set by the scheduler, -1 otherwise.
   */
  double getSolvingTimePerIteration() const { return this->solvingTimePerIteration; }
	/*!
	 * Getter for solvingTimeTotal, look above.
	 * @return solvingTimeTotal if set by the scheduler, 0 otherwise.
	 */
	double getSolvingTimeTotal() const { return this->solvingTimeTotal; }
	/*!
	 * Alias for getSolvingTimeTotal, look above.
	 * @return solvingTimeTotal if set by the scheduler, 0 otherwise.
	 */
	double getSolvingTime() const { return this->getSolvingTimeTotal(); }
  /*!
   * Getter for timeRemaining, look above
   * From outside of the Scheduler only accessable for the last Iteration.
   * @return timeRemaining if set by the scheduler, -1 otherwise.
   */
  double getTimeRemaining () const { return this->timeRemaining; }

protected:
  /*!--------------
   * Timetracking:
   *--------------*/
  /*!
   * Starts the time messurement, should be called directly before solver->solve() function.
   */
  virtual void startTimeTracking();
  /*!
   * End of time messurement, should be called directly after solver->solve() function.
   * It calculates the results and stores them in "timeRemaining" and "solvingTimePerIteration".
   */
  virtual void endTimeTracking();
  /*!
   * For iterative use, it sums up the solving times measured in each iteration.
   */
  virtual void updateSolvingTimeTotal();
  /*!
   * Starttime of Timemessurement
   */
  std::chrono::high_resolution_clock::time_point start_t;
  /*!
   * Endtime of Timemessurement
   */
  std::chrono::high_resolution_clock::time_point end_t;
  /*!
   * \brief Time available for an iteration in seconds
   */
  double solverTimeout;
  /*!
   * Time spent in solvers
   */
  double solvingTimePerIteration;
  /*!
   * Total Time spent in solvers over multiple Iterations
   */
  double solvingTimeTotal;
  /*!
   * solverTimeout - solvingTimePerIteration
   */
  double timeRemaining;
  /*!-----------------
   * End Timetracking
   *-----------------*/

  /*!
   * no couts if this is true
   */
  bool quiet;
  /*!
   * \brief resourceModel
   */
  ResourceModel &resourceModel;
  /*!
   * \brief maxLatencyConStraint default is -1 (unlimited)
   */
  int maxLatencyConstraint = -1;
  /*!
   * \brief Container for the start times
   */
  std::map<Vertex*,int> startTimes;
  /*!
   * \brief A reference to the data dependency graph
   */
  Graph& g;
  /*!
   * \brief II the initiation interval of a schedule
   */
  double II = -1.0f;
  /*!
   * the binding after scheduling
   */
  std::map<const Vertex*,int> binding;
  /*!
   * \brief scheduleFound
   */
  bool scheduleFound;
  /*!
   * \brief use this flag to determine optimal binding using ILP
   */
  bool useOptimalBinding;

};
}
