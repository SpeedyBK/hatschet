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
 *
 * For the details see
 * Fimmel, Mueller: Optimal Software Pipelining under resource constraints
 * Journal of Foundations of Computer Science 2001
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

    void setPmax(int pmax){this->pmax = pmax;}
    int getPmax() {return pmax;}
    /*!
     * Function to set the solver Timeout
     * @param seconds
     */
  void setSolverTimeout(double timeoutInSeconds) override;
protected:
    virtual void constructProblem(){};
    virtual void setObjective(){};

    /*!
     * not needed
     */
    virtual void resetContainer(){}
private:
    void generateTestSetup();
    void generateTestSetup2();

    int pmax = -1;
};
}
