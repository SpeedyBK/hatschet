#include "Edge.h"

namespace HatScheT
{

Edge::Edge(Vertex &src, Vertex &dst, int distance, DependencyType dependencyType) : Vsrc(src), Vdst(dst), distance(distance), dependencyType(dependencyType)
{
  id = -1;
  delay = 0;
}

ostream& operator<<(ostream& os, const Edge& e)
{
  string datatype = "Data";
  if(e.getDependencyType()==Edge::DependencyType::Precedence) datatype = "Precedence";
   os << "Edge Id: " << e.getID() << ". From Node " << e.getVertexSrcName()<< " to " << e.getVertexDstName()
      << " (Delay " << e.getDelay() << " Distance " << e.getDistance() << ")" << " depType " << datatype;

  return os;
}

bool Edge::isDataEdge()
{
  if(this->dependencyType==DependencyType::Data) return true;
  return false;
}

}
