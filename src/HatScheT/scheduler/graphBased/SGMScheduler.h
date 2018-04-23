/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/MoovacScheduler.h>
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
  /*!
   * \brief getII
   * \return
   */
  virtual int getII() { return this->II;}
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

};
}
