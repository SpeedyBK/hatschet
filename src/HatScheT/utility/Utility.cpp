#include "HatScheT/utility/Utility.h"


namespace HatScheT {

bool Utility::examplUtilityFunction(ResourceModel *rm, Graph *g)
{
  return true;
}

int Utility::getNoOfInputs(Graph *g, const Vertex *v)
{
  int no=0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV==v) no++;
  }

  return no;
}

}
