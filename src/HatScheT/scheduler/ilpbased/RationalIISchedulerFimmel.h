/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Piasecki (?)
  All rights reserved.
*/
#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>

namespace HatScheT
{
/*!
 * experimental: This scheduler determines a modulo schedule with uneven/rational initiation intervals
 * THIS CLASSES CONSTRUCTOR IS CURRENTLY DISABLED
 */
class RationalIISchedulerFimmel :  public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase
{
public:
    /*!
     *
     * @param g
     * @param resourceModel
     * @param solverWishlist
     */
    RationalIISchedulerFimmel(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    /*!
     * To Be Updated: II has to be rational or a vector for this scheduler to work
     * @return
     */
    virtual double getII() { return this->II;}
    /*!
     *
     */
    virtual void schedule();
protected:
    virtual void constructProblem(){};
    virtual void setObjective(){};
private:
    void generateTestSetup();
    void generateTestSetup2();
};
}
