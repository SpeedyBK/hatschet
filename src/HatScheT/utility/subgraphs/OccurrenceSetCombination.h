/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
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

