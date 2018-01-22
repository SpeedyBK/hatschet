#pragma once

#include <HatScheT/Graph.h>
#include <map>

namespace HatScheT
{

/*!
 * \brief ModuloSchedulerBase is the base class of all schedulers that solve the modulo scheduling problem.
 *
 * It contains helper functions to determine bounds on the II
 */
class ModuloSchedulerBase
{
public:

  int getII() { return II; }

  int computeMinMaxII();
  int computeMaxSL();

protected:
  unsigned int II;

};

}
