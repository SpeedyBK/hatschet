#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>

namespace HatScheT
{

/*!
 * \brief ResourceModel
 */
class ResourceModel
{
public:
  ResourceModel(Graph &g);



protected:
  Graph &g;
};
}
