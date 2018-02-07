#pragma once

#include <HatScheT/SchedulerBase.h>
#include <HatScheT/ILPSchedulerBase.h>
#include <HatScheT/ModuloSchedulerBase.h>
#include <HatScheT/ResourceConstrainedSchedulerBase.h>
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
class MoovacScheduler :  public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public ResourceConstrainedSchedulerBase
{
public:
  MoovacScheduler(Graph& g, std::list<std::string> solverWishlist, ResourceModel &resourceModel, unsigned int minII, unsigned int maxII);
  /*!
   * \brief schedule the schedule method of moocav will try to find the smallest possible II respecting the minII/maxII bounds iteratively.
   * as long as no feasible solution is found, the problem will be constructed again with an increased II until maxII is reached.
   */
  virtual void schedule();

protected:
  /*!
   * \brief constructProblem Using the graph, resource model, an II and solver settings, the problem is constructed
   */
  virtual void constructProblem();
  /*!
   * \brief setObjective currently asap
   */
  virtual void setObjective();
private:
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
   * \brief fillRegVector fill the container for ILP variables of registers
   */
  void fillRegVector();
  /*!
   * \brief setModuloAndResourceConstraints read the paper for further information
   */
  void setModuloAndResourceConstraints();
  /*!
   * \brief setGeneralConstraints read the paper for further information
   */
  void setGeneralConstraints();
  /*!
   * \brief m_container
   */
  vector< vector<ScaLP::Variable> > m_container;
  /*!
   * \brief regVector
   */
  vector<ScaLP::Variable> regVector;
  /*!
   * \brief r_container
   */
  vector<vector<ScaLP::Variable> > r_container;
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
   * \brief reg_vectorIndices
   */
  map<const Edge*, unsigned int> reg_vectorIndices;
  /*!
   * \brief schedFound
   */
  bool schedFound;
  /*!
   * \brief considerAlgorithmicDelays
   */
  bool considerAlgorithmicDelays;
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
