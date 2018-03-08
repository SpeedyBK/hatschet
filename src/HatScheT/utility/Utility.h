#pragma once
#include <string>
#include "HatScheT/Graph.h"
#include "HatScheT/ResourceModel.h"

namespace HatScheT
{
/*!
 * \brief The Utility class use this class for utility functions
 */
class Utility
{
public:
 static bool examplUtilityFunction(ResourceModel* rm, Graph *g);
 static int getNoOfInputs(Graph* g, const Vertex* v);

};
}
