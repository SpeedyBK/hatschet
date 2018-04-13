/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once
#include <HatScheT/Graph.h>
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

