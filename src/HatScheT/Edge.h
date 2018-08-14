/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Martin Kumm, Patrick Sittel (kumm, sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include "Vertex.h"

namespace HatScheT
{

/*!
 * \brief Implementation of a edge used in a Graph
 *
 * An edge expresses a precedence relation between the operations represented by the source and destination vertices.
 * Each edge carries two attributes:
 *
 * The **delay** is interpreted as an additional number of time steps that need to pass before `dst` can start after `src` finishes.
 * While the main use for this facility is to support (or more precisely, limit) operator chaining, it could be used to model
 * special propagation delays as well.
 *
 * The **distance** is only relevant for modulo scheduling problems. A non-zero distance indicates that a dependence
 * does not hold for the current iteration (or input sample), but `distance`-many iterations later.
 *
 * In the most generic case, an edge represents the following linear constraint between the start times \f$t_{src}\f$ and \f$t_{dst}\f$:
 *
 * \f$ t_{src} + ResModel.getVertexLatency(src) + delay <= t_{dst} + distance \cdot II \f$
 *
 * \sa Graph
 *     ResourceModel
 */
class Edge
{
public:
  enum DependencyType { Data, Precedence };
   /**
   * @brief Edge Constructor
   * @param Vsrc The source vertex
   * @param Vdst The destination vertex
   * @param distance The distance (=no of registers) on an edge
   * @param dependencyType Enum to distinguish dependencies and data edges
   */
  Edge(Vertex &src, Vertex &dst, int distance=0, DependencyType dependencyType=Data);
  /*!
   * copy constructor is forbidden for this class
   */
   Edge(const Edge&) = delete;

  Vertex& getVertexSrc(){ return Vsrc; }
  Vertex& getVertexDst(){ return Vdst; }

  const string getVertexSrcName() const {return this->Vsrc.getName();}
  const string getVertexDstName() const {return this->Vdst.getName();}

  int getDelay() const { return this->delay; }
  void setDelay(int delay) { this->delay = delay; }

  int getDistance() const { return distance; }
  void setDistance(int distance){ this->distance = distance; }

  int getID() const {return this->id;}
  void setID(int id){this->id = id;}

  void setDependencyType(DependencyType dt){this->dependencyType= dt;}
  DependencyType getDependencyType() const {return this->dependencyType;}

  bool isDataEdge();
protected:
  int id = -1;
  DependencyType dependencyType;

  /**
   * @brief delay defines the integer edge delay, i.e. an additional delay between src and dst vertices.
   * The delay is intended to be inferred internally by HatScheT, and thus is not a constructor argument.
   */
  int delay = 0;

  /**
   * @brief distance defines the integer distance in the schedule. Used to model algorithmic/functional delays (like appear in digital filters)
   */
  int distance;

  Vertex &Vsrc;
  Vertex &Vdst;

};
ostream& operator<<(ostream& os, const Edge& e);
}
