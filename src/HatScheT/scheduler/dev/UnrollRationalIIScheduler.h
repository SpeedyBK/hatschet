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
  enum SchedulerType {MOOVAC, MODULOSDC, ED97};

  class UnrollRationalIIScheduler : public RationalIISchedulerLayer, public ILPSchedulerBase, public IterativeSchedulerBase {
  public:
    UnrollRationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

    /*!
    * the main function of the scheduler. The rational II scheduler tries to identify high throughput schedules on
    * the theoretical min II boundary. For this the variables s / m are used
     * Then the graph is unrolled s times and a standard integer II scheduler is chosen
     * default scheduler: EichenbergerDavidson97
    */
    virtual void schedule();

    /*!
     * @brief get and set the values for s and m
     * @param s
     * @param m
     */
    void setSM(int s, int m){
      this->s_start = s;
      this->m_start = m;
    };
    int getS_Start(){
      return this->s_start;
    }
    int getM_Start() {
      return this->m_start;
    }

    void setIntIIScheduler(SchedulerType t) {
      this->scheduler = t;
    }

    //TODO finish those functions
    ScaLP::status getScaLPStatus() override { return this->stat; }
    virtual std::map<Edge*,int> getLifeTimes() override {
      throw Exception("UnrollRationalIIScheduler.getLifeTimes: not implemented yet!");
    };
    virtual std::map<const Vertex*,int> getBindings() override {
      throw Exception("UnrollRationalIIScheduler.getBindings: not implemented yet!");
    };
    virtual vector<std::map<const Vertex*,int> > getRationalIIBindings() override {
      throw Exception("UnrollRationalIIScheduler.getRationalIIBindings: not implemented yet!");
    };

  private:
    SchedulerType scheduler;
    std::list<std::string> solverWishlist;

    void unroll(Graph& g_unrolled, ResourceModel& rm_unrolled, int s);
    int s_start;
    int m_start;


    virtual void constructProblem() override {};
    virtual void setObjective() override {};
    virtual void resetContainer() override {};
    /*!
		 * fill interface to pass values to next step in the tool flow after solving
		 */
    void fillSolutionStructure(SchedulerBase * scheduler, Graph* g_unrolled, ResourceModel* rm_unrolled);

  };
}


