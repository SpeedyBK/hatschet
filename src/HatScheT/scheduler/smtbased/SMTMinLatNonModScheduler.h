//
// Created by Benjamin on 19.03.24.
//

#ifndef HATSCHET_SMTMINLATNONMODSCHEDULER_H
#define HATSCHET_SMTMINLATNONMODSCHEDULER_H

#if USE_Z3

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/Z3SchedulerBase.h>

namespace HatScheT {

    class SMTMinLatNonModScheduler : public SchedulerBase, public Z3SchedulerBase {

    public:

        SMTMinLatNonModScheduler(Graph& g, ResourceModel& rm);

        /*!
         * use this methode to solve the scheduling problem
         */
        void schedule() override;

        void setSolverTimeout(double seconds) override;

        void setSearchMode(string &sreachMode) { mode = sreachMode; };

        bool isLatencyOptimal() const { return latencyOptimal; };

    private:

        void scheduleAutoSearch();

        void scheduleASAPHCSearch();

        void generateTVariables();

        void generateBVariables();

        z3::check_result findMaxLatencyConstraints();

        int getLatestStarttime();

        z3::check_result minimizeLatency();

        z3::check_result addNonNegativeConstraints();

        z3::check_result addDependencyConstraints();

        z3::check_result addVariableConnections();

        z3::check_result addResourceConstraints();

        z3::expr* getTVariable(Vertex* vPtr);

        z3::expr *getBvariable(Vertex *v, int i);

        void z3CheckWithTimeTracking();

        map<Vertex*, z3::expr> tVariables;

        map<std::pair<Vertex*, int>, z3::expr> bVariables;

        int acturalRow;

        int lastRow;

        bool latencyOptimal;

        string mode;

    };

} // HatScheT

#endif

#endif //HATSCHET_SMTMINLATNONMODSCHEDULER_H
