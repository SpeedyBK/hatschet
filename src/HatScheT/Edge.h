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

  const double getDistance() const { return distance; }
  void setDistance(double distance){ this->distance = distance; }

  const int getID() const {return this->id;}
  void setID(int id){this->id = id;}

  //ToDo: add distance (no idea what it means in the UML)

protected:
  int id;
  DependencyType dependencyType;

  /**
   * @brief distance defines the integer distance in the schedule. Used to model algorithmic/functional delays (like appear in digital filters)
   */
  int distance;

  Vertex &Vsrc;
  Vertex &Vdst;

};
ostream& operator<<(ostream& os, const Edge& e);
}
