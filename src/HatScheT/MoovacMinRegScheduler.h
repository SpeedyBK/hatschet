/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/MoovacScheduler.h>

namespace HatScheT
{
/*!
 * \brief The MoovacMinRegScheduler class
 */
class MoovacMinRegScheduler : public MoovacScheduler
{
public:
    MoovacMinRegScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

protected:
    /*!
     * \brief setGeneralConstraints read the paper for further information
     */
    virtual void setGeneralConstraints();
    /*!
     * \brief setObjective minimum register number
     */
    virtual void setObjective();
    /*!
     * \brief fillRegVector fill the container for ILP variables of registers
     */
    virtual void fillRegVector();
    /*!
     * \brief getOutGoingEdgesOfResource get all edges that start from a vertex of resource r
     * \param r
     * \return
     */
    vector<Edge*> getOutGoingEdgesOfResource(Resource* r);
};
}
