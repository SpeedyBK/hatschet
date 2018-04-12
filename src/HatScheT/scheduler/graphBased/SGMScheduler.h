/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/MoovacScheduler.h>
#include <vector>

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
  SGMScheduler(Graph& g,ResourceModel &resourceModel, std::list<std::string> solverWishlist);

  void addOccurenceSet(vector<vector<Edge* > > occSet);

protected:
  /*!
   * \brief setGeneralConstraints read the paper for further information
   */
  virtual void setGeneralConstraints();
private:

};
}
