//
// Created by sittel on 15/11/19.
//

#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <vector>

namespace HatScheT {

  /*!
   * @brief options for the used standard integer II scheduler
   */
  enum SchedulerType {MOOVAC, MODULOSDC, ED97, SUCHAHANZALEK, PBS, SAT, SATCOMBINED};

  class UnrollRationalIIScheduler : public RationalIISchedulerLayer, public ILPSchedulerBase {
  public:
    UnrollRationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, int M=-1, int S=-1);

    /*!
     * @brief get and set the values for s and m
     * @param s
     * @param m
     */
    void setSM(int s, int m){
      this->s_start = s;
      this->m_start = m;
    };

    void setIntIIScheduler(SchedulerType t) {
      this->scheduler = t;
    }

    ScaLP::status getScaLPStatus() override { return this->stat; }

  protected:
    virtual void constructProblem() override {};
    virtual void setObjective() override {};
    virtual void resetContainer() override {};
    /*!
		 * each scheduler should overload this function for one schedule iteration
		 * M/S are already set automatically by RationalIISchedulerLayer::schedule
		 *
		 */
    void scheduleIteration() override;

  private:
    SchedulerType scheduler;
    std::list<std::string> solverWishlist;

    //void unroll(Graph& g_unrolled, ResourceModel& rm_unrolled, int s);

    /*!
		 * fill interface to pass values to next step in the tool flow after solving
		 */
    void fillSolutionStructure(SchedulerBase * scheduler, Graph* g_unrolled, ResourceModel* rm_unrolled);

  };
}


