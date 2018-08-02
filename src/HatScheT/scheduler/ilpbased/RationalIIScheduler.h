/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/scheduler/ilpbased/MoovacScheduler.h>

namespace HatScheT
{
/*!
 * experimental: This scheduler determines a modulo schedule with uneven/rational initiation intervals
 * THIS CLASSES CONSTRUCTOR IS CURRENTLY DISABLED
 */
class RationalIIScheduler : public MoovacScheduler
{
public:
    /*!
     * provide a graph, resource and solver wishlist since this scheduler is ilp-based and uses ScaLP for
     * generating ILP and solving
     * @param g
     * @param resourceModel
     * @param solverWishlist
     */
    RationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    /*!
     * To Be Updated: II has to be rational or a vector for this scheduler to work
     * @return
     */
    virtual int getII() { return this->II;}
protected:


};
}
