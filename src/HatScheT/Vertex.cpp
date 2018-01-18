#include "Vertex.h"

namespace HatScheT
{

Vertex::Vertex(int id) : id(id)
{
}

ostream& operator<<(ostream& os, const Vertex& v)
{
   os << "Vertex:" << v.getName() << "(ID " << to_string(v.getId()) << ", Latency " << to_string(v.getLatency()) << ")";

  return os;
}

}
