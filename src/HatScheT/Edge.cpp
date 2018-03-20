#include "Edge.h"

namespace HatScheT
{

Edge::Edge(Vertex &src, Vertex &dst, int delay, DependencyType dependencyType) : Vsrc(src), Vdst(dst), delay(delay), dependencyType(dependencyType)
{
  id = -1;
}

ostream& operator<<(ostream& os, const Edge& e)
{
   os << "Edge Id: " << e.getID() << ". From Node " << e.getVertexSrcName()<< " to " << e.getVertexDstName() << " (Delay " << e.getDelay() << ")";

  return os;
}

}
