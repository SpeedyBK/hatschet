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
#include <HatScheT/utility/subgraphs/Occurrence.h>
#include <vector>

namespace HatScheT
{
/*!
 * \brief The OccurrenceSet class
 */
class OccurrenceSet
{
public:
  /*!
   * \brief OccurrenceSet
   * \param g
   */
  OccurrenceSet(Graph* g);
  /*!
   * \brief addOccurrence only possible for conflict free and in graph g occurrences
   * \param occ
   * \return
   */
  bool addOccurrence(Occurrence* occ);
  /*!
   * \brief getOccurrences
   * \return
   */
  std::set<Occurrence*> getOccurrences() const {return this->occurrences;}
  /*!
   * \brief getGraph return the graph the occurrenceSets are in
   * \return
   */
  Graph* getGraph() const {return this->g;}
  /*!
   * @brief the frequency is the number of times the graph occurrs within the input model
   * @return
   */
  int getFrequency(){
    return this->occurrences.size();
  }
protected:

private:
  /*!
   * \brief occurrences container for conflict free occurrences
   */
  std::set<Occurrence*> occurrences;
  /*!
   * \brief g
   */
  Graph* g;

};
}

