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
   * @param delay The delay (=no of registers) on an edge
   * @param dependencyType Enum to distinguish dependencies and data edges
   */
  Edge(Vertex &src, Vertex &dst, int delay=0, DependencyType dependencyType=Data);

  Vertex& getVertexSrc(){ return Vsrc; }
  Vertex& getVertexDst(){ return Vdst; }

  const string getVertexSrcName() const {return this->Vsrc.getName();}
  const string getVertexDstName() const {return this->Vdst.getName();}

  const double getDelay() const { return delay; }
  void setDelay(double delay){ this->delay = delay; }

  const int getID() const {return this->id;}
  void setID(int id){this->id = id;}

  //ToDo: add distance (no idea what it means in the UML)

protected:
  int id;
  DependencyType dependencyType;
  int delay;

  Vertex &Vsrc;
  Vertex &Vdst;

};
ostream& operator<<(ostream& os, const Edge& e);
}
