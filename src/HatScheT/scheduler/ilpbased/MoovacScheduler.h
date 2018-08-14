/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>

namespace HatScheT
{
/*!
 * \brief Implementation of the Moovac scheduler
 *
 * For the details of the Moovac scheduler, see Oppermann, Koch, Reuter-Oppermann  & Sinnen, ILP-based modulo scheduling for high-level synthesis,
 * Presented at the Proceedings of the International Conference on Compilers, Architectures and Synthesis for Embedded Systems (CASES) 2016
 *
 */
class MoovacScheduler :  public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase
{
public:
  MoovacScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
  /*!
   * \brief schedule the schedule method of moocav will try to find the smallest possible II respecting the minII/maxII bounds iteratively.
   * as long as no feasible solution is found, the problem will be constructed again with an increased II until maxII is reached.
   */
  virtual void schedule();
  /*!
   * \brief getII
   * \return
   */
  virtual int getII() { return this->II;}
  /*!
   * \brief getNoOfImplementedRegisters return -1 if no schedule was determined
   * \return
   */
  virtual int getNoOfImplementedRegisters();
  /*!
   * \brief getNoOfMuxInputs
   * \return
   */
  virtual int getNoOfMuxInputs();
  /*!
   * \brief getBindings getBdings based on solution of the ilp solver
   * non limited resources are assumed to be bound on pairwise different hardware units
   * \return
   */
  virtual std::map<const Vertex*,int> getBindings();
  /*!
   * \brief getLifeTimes considering II
   * \return
   */
  virtual std::map<Edge*,int> getLifeTimes();

protected:
  /*!
   * \brief constructProblem Using the graph, resource model, an II and solver settings, the problem is constructed
   */
  virtual void constructProblem();
  /*!
   * \brief setObjective currently asap
   */
  virtual void setObjective();
  /*!
   * \brief setGeneralConstraints read the paper for further information
   */
  virtual void setGeneralConstraints();
  /*!
   * \brief setSourceVerticesToZero pin all source vertices(inputs, constants,...) to starttime 0
   */
  void setSourceVerticesToZero();
  /*!
   * \brief resetContainer
   */
  void resetContainer();
  /*!
   * \brief setUpSolverSettings
   */
  void setUpSolverSettings();
  /*!
   * \brief setTVectorVariables fill the container for ILP variables of vertex starting times
   */
  void setVectorVariables();
  /*!
   * \brief fillRegVector not used in moovac
   */
  virtual void fillRegVector(){ }
  /*!
   * \brief setModuloAndResourceConstraints read the paper for further information
   */
  void setModuloAndResourceConstraints();
  /*!
   * \brief fillSolutionStructure pass the solution of the modulo schedule
   */
  void fillSolutionStructure();
  /*!
   * \brief mi contains all mi for i in L. (20) in paper
   */
  vector< vector<ScaLP::Variable> > mi;
  /*!
   * \brief regVector
   */
  vector<ScaLP::Variable> registers;
  /*!
   * \brief ri contains all ri for i in L. (18) in paper
   */
  vector<ScaLP::Variable> ri;
  /*!
   * \brief yi contains all yr for i in L. (19) in paper
   */
  vector<vector<ScaLP::Variable> > yi;
  /*!
   * \brief eps_container (21) in paper
   */
  vector<vector<vector<ScaLP::Variable> > > epsij;
  /*!
   * \brief mu_container (22) in paper
   */
  vector<vector<vector<ScaLP::Variable> > > muij;
  /*!
   * \brief tIndices
   */
  map<const Vertex*, unsigned int> tIndices;
  /*!
   * \brief rIndices
   */
  map<const Vertex*, unsigned int> rIndices;
  /*!
   * \brief registerIndices
   */
  map<const Edge*, unsigned int> registerIndices;
  /*!
   * \brief r
   */
  ScaLP::Result r;
  /*!
   * \brief ti contains the time variables of all operations in O
   */
  vector<ScaLP::Variable> ti;
  /*!
   * \brief minII
   */
  unsigned int minII;
  /*!
   * \brief maxII
   */
  unsigned int maxII;
  /*!
   * \brief SLMax
   */
  unsigned int SLMax;
};
}
