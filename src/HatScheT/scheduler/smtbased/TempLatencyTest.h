//
// Created by bkessler on 8/9/22.
//

#ifndef HATSCHET_TEMPLATENCYTEST_H
#define HATSCHET_TEMPLATENCYTEST_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>

namespace HatScheT {

  class TempLatencyTest : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase{

  public:

    struct result{
      string algo;
      int minLat;
      int maxLat;
      double II;
      double solvingTime;
    };

    TempLatencyTest(Graph &g, ResourceModel &resourceModel);

    void schedule() override;

    result minLatencyWithILP(double InitI);

    result minLatencyWithCustom(double  InitI);

    result maxLatencyWithCustom(double InitI);

    result maxLatencyOppermann();

    result maxLatencyEichenberger(double InitI);

    result minLatencyWithASAP();

    result minLatencyWithSDC(double InitI);

    void setnames(string graphstr, string resstr) { this->_graphstr = graphstr; this->_resstr = resstr; };

    void openFileAndCollectData(list<deque<result>*>& data);

    static void writeData(deque<result>& data, ofstream &of);

    string _graphstr;

    string _resstr;

  };

}
#endif //HATSCHET_TEMPLATENCYTEST_H
