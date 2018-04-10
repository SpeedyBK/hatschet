/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
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
  MoovacScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, unsigned int maxII);
  /*!
   * \brief schedule the schedule method of moocav will try to find the smallest possible II respecting the minII/maxII bounds iteratively.
   * as long as no feasible solution is found, the problem will be constructed again with an increased II until maxII is reached.
   */
  virtual void schedule();
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
  void setTVectorVariables();
  /*!
   * \brief fillRegVector not in moovac
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
   * \brief m_container
   */
  vector< vector<ScaLP::Variable> > m_container;
  /*!
   * \brief regVector
   */
  vector<ScaLP::Variable> regVector;
  /*!
   * \brief r_vector from moovac paper
   */
  vector<ScaLP::Variable> r_vector;
  /*!
   * \brief y_container
   */
  vector<vector<ScaLP::Variable> > y_container;
  /*!
   * \brief eps_container
   */
  vector<vector<vector<ScaLP::Variable> > > eps_container;
  /*!
   * \brief mu_container
   */
  vector<vector<vector<ScaLP::Variable> > > mu_container;
  /*!
   * \brief t_vectorIndices
   */
  map<const Vertex*, unsigned int> t_vectorIndices;
  /*!
   * \brief r_vectorIndices
   */
  map<const Vertex*, unsigned int> r_vectorIndices;
  /*!
   * \brief reg_vectorIndices
   */
  map<const Edge*, unsigned int> reg_vectorIndices;
  /*!
   * \brief sampleLatency
   */
  unsigned int sampleLatency;
  /*!
   * \brief r
   */
  ScaLP::Result r;
  /*!
   * \brief t_vector
   */
  vector<ScaLP::Variable> t_vector;
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
