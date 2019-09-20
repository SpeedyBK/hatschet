/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Thomas Schönwälder, Patrick Sittel (thomas.schoenwaelder@student.uni-kassel.de, sittel@uni-kassel.de)

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
#include <HatScheT/base/IterativeSchedulerBase.h>

#include <queue>
#include <functional>
#include <vector>

namespace HatScheT
{
  using Queue = std::priority_queue<Vertex*,std::vector<Vertex*>,std::function<bool(Vertex*,Vertex*)>>;
  /*!
   * \brief The MRT class is used to determine a schedule without resource conflict.
   * implementation according to ModuloSDC scheduler paper
   */
  class MRT
  {
    public:
      /*!
       * \brief MRT the modulo reservation table can only be constructed using a resource model of hatschet
       * and for a specific II
       * \param r
       * \param II
       */
      MRT(HatScheT::ResourceModel& r, int II);
      /*!
       * \brief resourceConflict determine whether there is a resource conflict of vertex i at time within the current mrt
       * \param i
       * \param time
       * \return
       */
      bool resourceConflict(HatScheT::Vertex* i,int time);
      /*!
       * \brief update try to add vertex i at a specific time to the mrt
       * \param i
       * \param time
       * \return
       */
      bool update(HatScheT::Vertex* i, int time);
      /*!
       * \brief remove try to remove vertex i from the mrt
       * \param i
       * \return
       */
      bool remove(HatScheT::Vertex* i);
      /*!
       * \brief getInstructionAt get an instruction of resource r at timestep r as vertex*
       * \param i
       * \param r
       * \return
       */
      HatScheT::Vertex* getInstructionAt(unsigned int i, const Resource *r);
      /*!
       * \brief vertexIsIn return true if v is currently in the mrt
       * \param v
       * \return
       */
      bool vertexIsIn(Vertex* v);
      /*!
       * \brief data the main container of the mrt
       * resource
       * timestep
       * vertex (this is a nullptr when empty)
       */
      std::map<const HatScheT::Resource*,std::vector<std::vector<HatScheT::Vertex*>>> data;
  private:
      /*!
       * \brief rm the used resource model
       */
      HatScheT::ResourceModel* rm;
      /*!
       * \brief II the used II (0 invalid)
       */
      int II=0;
  };
  /*!
   * \brief The ModuloSDCScheduler class this scheduler is an implementation of the modulo scheduler
   * presented in the paper: Modulo SDC scheduling with recurrence minimization in high-level synthesis
   * Authors: Andrew Canis, Stephen D. Brown, Jason H. Anderson
   */
  class ModuloSDCScheduler :public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase
  {
  public:
    /*!
     * \brief ModuloSDCScheduler for scheduling in hatschet a graph, a resource model is needed
     * for solving the SDC problems, a solver wishlist to enable the ScaLP ILP backend is needed
     * (for example "Gurobi", "SCIP", "CPLEX")
     * \param g
     * \param resourceModel
     * \param solverWishlist
     */
    ModuloSDCScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    /*!
     * \brief schedule main method that runs the algorithm and determines a schedule
     */
    virtual void schedule();

    /*!
     * \brief setVerbose manage a level of verbosity (default off)
     * \param b
     */
    void setVerbose(bool b){
      this->verbose=b;
    }
    /*!
     * @brief define the budget (see paper)
     * if not user defined, the proposed value is set on default
     * @param b
     */
    void setUserDefBudget(unsigned int b) {
      this->userdef_budget = b;
    }
    /*!
     * the status of the ilp solver does not provide any information
     * about the quality of the solution, because many ilp problems are
     * solved for MRTs
     * @return the unknown status
     */
    virtual ScaLP::status getScaLPStatus(){return ScaLP::status::UNKNOWN;}
  private:
    /*!
     * \brief constructProblem
     */
    virtual void constructProblem();
    /*!
     * \brief setObjective
     */
    virtual void setObjective();
    /*!
    * not needed
    */
      virtual void resetContainer(){}
    /*!
     * \brief baseConstraints contains the base constraints that have to reset for very new II
     */
    std::vector<ScaLP::Constraint> baseConstraints;
    /*!
     * \brief constraints contains that constraints that are determined using the ModuloSDC algorithm
     */
    std::vector<ScaLP::Constraint> constraints;
    /*!
     * \brief variables contains the used variables for SDC
     */
    std::map<Vertex*,ScaLP::Variable> variables;
    /*!
     * \brief neverScheduled contains information whether a vertex has been tried to schedule before
     * to avoid endless backtracking
     */
    std::map<Vertex*,bool> neverScheduled;
    /*!
     * \brief mrt the current used mrt
     */
    MRT mrt;
    /*!
     * \brief sched the main method for each run (II)
     * \param budget time budget for each run (according to the original paper)
     * \param priority
     * \param asap
     * \return
     */
    bool sched(int budget, const std::map<HatScheT::Vertex*,unsigned int>& priority,  const std::map<HatScheT::Vertex*,int>& asap);
    /*!
     * \brief backtracking the backtracking method according to the original paper
     * \param schedQueue
     * \param prevsched
     * \param I
     * \param asapI
     * \param time
     * \param II
     */
    void backtracking(Queue& schedQueue, std::map<Vertex*,int>& prevsched, HatScheT::Vertex* I, int asapI, int time, int II);
    /*!
     * \brief dependencyConflict check whether there is a dependency conflict if I gets inserted at slot time
     * \param prevsched
     * \param I
     * \param time
     * \return
     */
    bool dependencyConflict(std::map<Vertex*,int>& prevsched, HatScheT::Vertex* I, int time);
    /*!
     * \brief createBaseConstraints
     * \param II
     */
    void createBaseConstraints(int II);
    /*!
     * \brief solveBasicWithConstraint
     * \param c
     * \return
     */
    bool solveBasicWithConstraint(ScaLP::Constraint&& c);
    /*!
     * \brief verbose default false
     */
    bool verbose;

    int userdef_budget;
    int timesOutOfBudget;
  };

}
