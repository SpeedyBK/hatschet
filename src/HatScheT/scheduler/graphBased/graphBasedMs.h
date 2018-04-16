#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>

namespace HatScheT
{

class GraphBasedMs : public SchedulerBase, ModuloSchedulerBase
{
public:
    /*!
   * \brief GraphBasedMs
   * \param g
   * \param resourceModel
   */
  GraphBasedMs(Graph& g,ResourceModel &resourceModel);

  /*!
   * \brief schedule determine a II using the graph based modulo scheduler
   */
  virtual void schedule();

protected:
private:
    //implent methods
  /*!
   * \brief getVertexWithLeastNoOfConnections
   * \param g
   * \param vVec
   * \return
   */
  Vertex* getVertexWithLeastNoOfConnections(Graph* g, vector<Vertex*> vVec);

  /*!
   * \brief getNoOfConnections
   * \param g
   * \param v
   * \param vVec
   * \return
   */
  int getNoOfConnections(Graph* g, Vertex* v, vector<Vertex*> vVec);

  /*!
   * \brief getSampleTimesWithHeuristic
   * \param g
   * \param vVec
   * \param sampleTimes
   */
  void getSampleTimesWithHeuristic(Graph* g, vector<Vertex*> vVec,vector<int> &sampleTimes);

  /*!
   * \brief removeUsedVertices
   * \param g
   * \param v
   * \param vVec
   */
void removeUsedVertices(Graph* g, Vertex* v, vector<Vertex *> &vVec);

/*!
 * \brief isVertexInVector
 * \param vVec
 * \param v
 * \return
 */
bool isVertexInVector(vector<Vertex*> vVec, Vertex* v);

/*!
 * \brief isMemberInVector
 * \param vVec
 * \param v
 */
template<typename T> bool isMemberInVector(vector<T> vVec, T v);

};
}
