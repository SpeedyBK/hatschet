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

          SMTSCCScheduler smtSCC(this->g, this->resourceModel);
          smtSCC.setQuiet(quiet);
          smtSCC.setMode(schedule_t::fast);
          smtSCC.setSolverTimeout(timeLimit);
          smtSCC.schedule();
          scheduleFound = smtSCC.getScheduleFound();
          if (scheduleFound) {
              firstObjectiveOptimal = smtSCC.getObjectivesOptimal().first;
              scheduleFound = smtSCC.getScheduleFound();
              II = smtSCC.getII();
              startTimes = smtSCC.getSchedule();
          }

          if (scheduleFound) {
              cout << endl;
              cout << "*******************************************************************" << endl;
              cout << " Heuristic Schedule for II = " << II << " and Latency = " << smtSCC.getScheduleLength() << " found..." << endl;
              cout << "*******************************************************************" << endl;
              cout << endl << "Searching for better Latency..." << endl << endl;
              SMTBinaryScheduler smtBIN(this->g, this->resourceModel, II);
              smtBIN.setQuiet(quiet);
              smtBIN.setLatencySearchMethod(SMTBinaryScheduler::latSearchMethod::LINEAR);
              smtBIN.setSchedulePreference(SMTBinaryScheduler::schedulePreference::MOD_ASAP);
              smtBIN.setSolverTimeout(timeLimit);
              smtBIN.schedule();
              if (smtBIN.getScheduleFound()){
                  startTimes = smtBIN.getSchedule();
                  secondObjectiveOptimal = smtBIN.getObjectivesOptimal().second;
              }
          }else{
              II = -1;
          }

  }

  void SMTSCCCOMBINED::setSolverTimeout(int seconds) {
      timeLimit = seconds;
  }
}