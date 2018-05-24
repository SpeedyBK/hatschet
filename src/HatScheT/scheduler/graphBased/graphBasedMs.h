/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Lennart Henne
  All rights reserved.
*/
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
  GraphBasedMs(SchedulerBase* sched, Graph& g, ResourceModel &resourceModel, float moduloLowerBound, float moduloUpperBound);

  /*!
   * \brief schedule determine a II using the graph based modulo scheduler
   */
  virtual void schedule();
  //plz think about a method to implement this function, maybe throw error?
  //iz complex
  /*!
   * \brief getII
   * \return
   */
  //virtual int getII() { }

protected:
private:
  float moduloLowerBound;
  float moduloUpperBound;
  SchedulerBase* scheduler;
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
   * \brief getSampleTimesWithHeuristic4All
   * \param g
   * \param vVec
   * \param sampleTimes
   */
  void getSampleTimesWithHeuristic4All(vector<Graph*> g, vector<vector<Vertex *> > vVec, vector<int> &sampleTimes);

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

/*!
 * \brief createConflictMatrix
 * \param conflMatrix
 * \param sampleVec
 * \param n
 * \return
 */
vector<vector<int>> createConflictMatrix(vector<vector<int>> conflMatrix, vector<int> sampleVec, int modulo);

/*!
 * \brief createConflictGraph
 * \param conflGraph
 * \param conflVertices
 * \param conflMatrix
 * \param currConstraint
 * \param n
 * \param minII
 */
void createConflictGraph(Graph &conflGraph, vector<Vertex*> &conflVertices, vector<vector<int>> conflMatrix, int currConstraint, int minII);

/*!
 * \brief getAllVertexConnections
 * \param g
 * \param vVec
 * \return
 */
vector<int> getAllVertexConnections(Graph* g, vector<Vertex *> vVec);

/*!
 * \brief getVectorMatrixColumnWithMinSum
 * \param matrix
 * \param allGraphs
 * \param vVec
 * \return
 */
int getVectorMatrixColumnWithMinSum(vector<vector<int>> matrix, vector<Graph *> allGraphs, vector<vector<Vertex *> > vVec);
};
}
