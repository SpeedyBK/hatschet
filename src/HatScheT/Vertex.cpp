#include "Vertex.h"

namespace HatScheT
{

Vertex::Vertex(int id) : id(id)
{
  this->name = "vertex_" + to_string(id);
}

ostream& operator<<(ostream& os, const Vertex& v)
{
   os << "Vertex:" << v.getName() << "(ID " << to_string(v.getId()) << ")";

  return os;
}

}
