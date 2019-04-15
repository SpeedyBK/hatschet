/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2019

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include <HatScheT/scheduler/ilpbased/MoovacScheduler.h>
#include <vector>
#include <HatScheT/utility/subgraphs/OccurrenceSetCombination.h>

namespace HatScheT
{
/*!
 * \brief The SGMScheduler class An ILP-based modulo scheduler that uses connected subgraphs for scheduling
 */
class SGMScheduler : public MoovacScheduler
{
public:
    /*!
   * \brief GraphBasedMs
   * \param g
   * \param resourceModel
   */
  SGMScheduler(Graph& g,ResourceModel &resourceModel, std::list<std::string> solverWishlist, OccurrenceSetCombination* occSC);

protected:
  /*!
   * \brief setGeneralConstraints read the paper for further information
   */
  virtual void setGeneralConstraints();
private:
  /*!
   * \brief occSC store the subgraph combination informations
   */
  OccurrenceSetCombination* occSC;
  /*!
   * \brief setSubgraphConstraints forcevalid bindings and lifetimes
   */
  void setSubgraphConstraints();

};
}
