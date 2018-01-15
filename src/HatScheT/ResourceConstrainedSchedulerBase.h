#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <HatScheT/ResourceModel.h>

#include <map>

namespace HatScheT
{

/*!
 * \brief ResourceConstrainedSchedulerBase is the base class of all schedulers that involve resource constraints.
 */

class ResourceConstrainedSchedulerBase
{
public:
  ResourceConstrainedSchedulerBase(ResourceModel &resourceModel);

protected:
  ResourceModel &resourceModel;
};
}
