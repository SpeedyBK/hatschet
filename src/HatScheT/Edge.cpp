#include "Edge.h"

namespace HatScheT
{

Edge::Edge(Vertex &src, Vertex &dst, int delay, bool backward, DependencyType dependencyType) : Vsrc(src), Vdst(dst), delay(delay), backward(backward), dependencyType(dependencyType)
{
  backward = 0;
  id = -1;
}

ostream& operator<<(ostream& os, const Edge& e)
{
   os << "Edge Id: " << e.getID() << ". From Node " << e.getVertexSrcName()<< " to " << e.getVertexDstName() << " (Delay " << e.getDelay() << ", Backward " << to_string(e.getBackward()) << ")";

  return os;
}

}
