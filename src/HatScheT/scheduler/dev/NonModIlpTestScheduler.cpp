//
// Created by Benjamin on 12.04.24.
//

#include "NonModIlpTestScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ilpbased/ASAPILPScheduler.h"
#include "HatScheT/utility/Verifier.h"

namespace HatScheT {
    NonModIlpTestScheduler::NonModIlpTestScheduler(Graph &g, ResourceModel &resourceModel, const std::list<std::string>& solverWishlist)
            : SchedulerBase(g, resourceModel){

        sW = solverWishlist;
    }

    void NonModIlpTestScheduler::schedule() {

        ASAPScheduler asap(this->g, this->resourceModel);
        asap.setQuiet(true);
        asap.schedule();

        if (asap.getScheduleFound()){
            this->startTimes = asap.getSchedule();
            this->II = asap.getScheduleLength();
        }else{
            scheduleFound = false;
            throw (HatScheT::Exception("ASAPHC: No Schedule found ... terminating."));
        }

        int initialScheduleLength = asap.getScheduleLength();

        ASAPILPScheduler asapILP(this->g, this->resourceModel, this->sW);
        cout << "Using " << asapILP.getSolverName() << " Solver" << endl;
        asapILP.setMaxLatencyConstraint(initialScheduleLength);
        cout << "Setting Max Latency Constraint to " << initialScheduleLength << " cycles." << endl;
        asapILP.setQuiet(this->quiet);
        cout << "Setting Scheduler quiet to " << this->quiet << endl;
        asapILP.setSolverTimeout(this->solverTimeout);
        cout << "Setting Solver Timeout to " << this->solverTimeout << " seconds." << endl;
        asapILP.schedule();

        if (asapILP.getScheduleFound()) {
            this->startTimes = asapILP.getSchedule();
            this->II = asapILP.getScheduleLength();
        }

        if (verifyResourceConstrainedSchedule(this->g, this->resourceModel, this->startTimes, this->getScheduleLength(), this->quiet)){
            if (!this->quiet){
                cout << "NonModIlpTestScheduler: Schedule Valid" << endl;
                cout << "Schedule minimized by " << initialScheduleLength - this->getScheduleLength() << " Timesteps." << endl;
            }
            isOptimal = asapILP.getScaLPStatus() == ScaLP::status::OPTIMAL;
            this->scheduleFound = true;
        }else{
            cout << "NonModIlpTestScheduler: Schedule NOT Valid" << endl;
            this->scheduleFound = false;
        }

    }
} // HatScheT