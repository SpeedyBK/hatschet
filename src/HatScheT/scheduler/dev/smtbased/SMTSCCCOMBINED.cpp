//
// Created by bkessler on 9/1/22.
//

#include "SMTSCCCOMBINED.h"
#include <chrono>

namespace HatScheT{


  SMTSCCCOMBINED::SMTSCCCOMBINED(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      this->timeLimit = INT32_MAX/2;
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
      II = minII;
  }

  void SMTSCCCOMBINED::schedule() {

      int timeBudget;

      SMTSCCScheduler smtSCC(this->g, this->resourceModel);
      smtSCC.setQuiet(quiet);
      smtSCC.setMode(SMTSCCScheduler::schedule_t::automatic);
      smtSCC.setSolverTimeout(timeLimit);
      smtSCC.schedule();
      scheduleFound = smtSCC.getScheduleFound();
      timeBudget = smtSCC.getTimeBudget();
      if (scheduleFound) {
          firstObjectiveOptimal = smtSCC.getObjectivesOptimal().first;
          II = smtSCC.getII();
          startTimes = smtSCC.getSchedule();
      }

      if (scheduleFound) {
          cout << endl;
          cout << "*******************************************************************" << endl;
          cout << " Heuristic Schedule for II = " << II << " and Latency = " << smtSCC.getScheduleLength()
               << " found..." << endl;
          cout << "*******************************************************************" << endl;
          cout << endl << "SMTCOMBINED::Searching for better Latency... Time Left: " << timeBudget << endl << endl;
          SMTBinaryScheduler smtBIN(this->g, this->resourceModel, II);
          smtBIN.setQuiet(quiet);
          smtBIN.setLatencySearchMethod(SMTBinaryScheduler::latSearchMethod::BINARY);
          smtBIN.setSchedulePreference(SMTBinaryScheduler::schedulePreference::MOD_ASAP);
          smtBIN.setSolverTimeout(timeBudget);
          smtBIN.schedule();
          if (smtBIN.getScheduleFound()) {
              cout << "SMTCOMBINED::Better Schedule found..." << endl;
              startTimes = smtBIN.getSchedule();
              secondObjectiveOptimal = smtBIN.getObjectivesOptimal().second;
          }
      } else {
          II = -1;
      }
  }

  void SMTSCCCOMBINED::setSolverTimeout(int seconds) {
      timeLimit = seconds;
  }
}