/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

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
#include <HatScheT/utility/subgraphs/OccurrenceSet.h>

namespace HatScheT
{
/*!
 * \brief The OccurrenceSetCombination class this class manages a distribution of a graph into conflict free occurrenceSets of occurrences
 */
class OccurrenceSetCombination
{
public:
  /*!
   * \brief OccurrenceSetCombination
   * \param g
   */
  OccurrenceSetCombination(Graph* g);
  /*!
   * \brief addOccurrenceSet
   * \param occs
   * \return
   */
  bool addOccurrenceSet(OccurrenceSet* occs);
  /*!
   * \brief getOccurrenceSets
   * \return
   */
  std::set<OccurrenceSet*> getOccurrenceSets() const {return this->occsComb;}
  /*!
   * \brief getGraph return the graph the occurrences and occurrenceSets are in
   * \return
   */
  Graph* getGraph() const {return this->g;}
protected:

private:
  /*!
   * \brief occsComb
   */
  set<OccurrenceSet*> occsComb;
  /*!
   * \brief g
   */
  Graph* g;
};
}

