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
 * \brief Implementation of a simple edge used in a graph (@see Graph)
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

  Vertex& getVertexSrc(){ return Vsrc; }
  Vertex& getVertexDst(){ return Vdst; }

  const string getVertexSrcName() const {return this->Vsrc.getName();}
  const string getVertexDstName() const {return this->Vdst.getName();}

  const int getDelay() const { return this->delay; }
  void setDelay(int delay) { this->delay = delay; }

  const int getDistance() const { return distance; }
  void setDistance(int distance){ this->distance = distance; }

  const int getID() const {return this->id;}
  void setID(int id){this->id = id;}

  void setDependencyType(DependencyType dt){this->dependencyType= dt;}
  DependencyType getDependencyType() const {return this->dependencyType;}

  bool isDataEdge();
protected:
  int id;
  DependencyType dependencyType;

  /**
   * @brief delay defines the integer edge delay, i.e. an additional delay between src and dst vertices.
   * The delay is intended to be inferred internally by HatScheT, and thus is not a constructor argument.
   */
  int delay;

  /**
   * @brief distance defines the integer distance in the schedule. Used to model algorithmic/functional delays (like appear in digital filters)
   */
  int distance;

  Vertex &Vsrc;
  Vertex &Vdst;

};
ostream& operator<<(ostream& os, const Edge& e);
}
