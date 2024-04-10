//
// Created by Benjamin on 19.03.24.
//

#include "SMTMinLatNonModScheduler.h"
#include <sstream>
#include <cmath>

#include "HatScheT/utility/Verifier.h"
#include "HatScheT/scheduler/ASAPScheduler.h"

#if USE_Z3

namespace HatScheT {

    SMTMinLatNonModScheduler::SMTMinLatNonModScheduler(Graph &g, ResourceModel &rm) : SchedulerBase(g, rm) {

        acturalRow = 0;
        lastRow = 0;
        latencyOptimal = false;
        mode = "ASAPHC";

    }

    void SMTMinLatNonModScheduler::schedule() {

        if (mode == "Auto"){
            scheduleAutoSearch();
        }else if (mode == "ASAPHC"){
            scheduleASAPHCSearch();
        }

    }

    void SMTMinLatNonModScheduler::generateTVariables() {
        tVariables.clear();
        for (auto &vIt: g.Vertices()) {
            std::stringstream name;
            name << vIt->getName();
            z3::expr e(c.int_const(name.str().c_str()));
            tVariables.insert(std::make_pair(vIt, e));
        }
    }

    z3::check_result SMTMinLatNonModScheduler::findMaxLatencyConstraints() {

        int tMax = 0;
        for (auto &It : startTimes){
            if (It.second > tMax){
                tMax = It.second;
            }
        }

        for (auto &vIt : g.Vertices()){
            z3::expr e = *getTVariable(vIt) <= tMax;
            s.add(e);
        }

        z3CheckWithTimeTracking();

        return getZ3Result();

    }

    z3::check_result SMTMinLatNonModScheduler::addDependencyConstraints() {

        for (auto &eIt: g.Edges()) {

            if (eIt->getDistance() > 0) {
                continue;
            }

            auto vSrc = &(eIt->getVertexSrc());
            auto vDst = &(eIt->getVertexDst());

            s.add(tVariables.at(vSrc) - tVariables.at(vDst) + this->resourceModel.getVertexLatency(vSrc) +
                  eIt->getDelay() <= 0);

        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMTMinLatNonModScheduler::addNonNegativeConstraints() {

        for (auto &vIt: g.Vertices()) {
            s.add(tVariables.at(vIt) >= 0);
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMTMinLatNonModScheduler::addResourceConstraints() {

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }
            for (int i = lastRow; i <= acturalRow; ++i) {
                z3::expr_vector ev(c);
                for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {
                    ev.push_back(*getBvariable((Vertex *) vIt, i));
                }
                this->s.add(z3::atmost(ev, rIt->getLimit()));
            }
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::expr *SMTMinLatNonModScheduler::getTVariable(Vertex *vPtr) {
        try {
            return &tVariables.at(vPtr);
        } catch (std::out_of_range &) {
            //cout << "Out_of_Range: " << vPtr->getName() << endl;
            throw (HatScheT::Exception("SMTMinLatNonModScheduler: getTVariable std::out_of_range"));
        }
    }

    z3::expr *SMTMinLatNonModScheduler::getBvariable(Vertex *v, int i) {
        auto key = std::make_pair(v, i);
        try {
            return &bVariables.at(key);
        } catch (std::out_of_range &) {
            //cout << "Out_of_Range: " << v->getName() << "_" << i << endl;
            throw (HatScheT::Exception("smtbased Scheduler: getBvariable std::out_of_range"));
        }
    }

    void SMTMinLatNonModScheduler::generateBVariables() {

        int tMax = 0;

        if (mode == "Auto"){
            tMax = getLatestStarttime();
        }else if (mode == "ASAPHC"){
            lastRow = 0;
            for (auto &it : startTimes){
                if (it.second > tMax){
                    tMax = it.second;
                }
            }
            acturalRow = tMax;
        }

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }
            for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {

                for (int i = lastRow; i <= tMax; ++i) {

                    try {
                        getBvariable((Vertex *) vIt, i);
                    } catch (HatScheT::Exception &) {

                        std::stringstream name;
                        name << "B_" << vIt->getName() << "_" << i;
                        z3::expr e(c.bool_const(name.str().c_str()));
                        auto key = std::make_pair((Vertex *) vIt, i);
                        bVariables.insert(std::make_pair(key, e));
                    }
                }
            }
        }
    }

    int SMTMinLatNonModScheduler::getLatestStarttime() {
        int tMax = 0;
        for (auto &vIt: g.Vertices()) {
            if (resourceModel.getResource(vIt)->isUnlimited()) {
                continue;
            }
            if (tMax < m.eval(*getTVariable(vIt)).get_numeral_int()) {
                tMax = m.eval(*getTVariable(vIt)).get_numeral_int();
            }
        }
        return tMax;
    }

    z3::check_result SMTMinLatNonModScheduler::addVariableConnections() {

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }

            for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {
                for (int i = lastRow; i <= acturalRow; ++i) {
                    z3::expr constraint(c);
                    constraint = (*getTVariable((Vertex *) vIt) == i) == *getBvariable((Vertex *) vIt, i);
                    s.add(constraint);
                }
            }
        }

        z3CheckWithTimeTracking();

        return getZ3Result();

    }

    z3::check_result SMTMinLatNonModScheduler::minimizeLatency() {

        z3::expr ex(c);
        int maxValue = 0;

        for (auto &vIt: g.Vertices()) {
            if (m.eval(*getTVariable(vIt)).get_numeral_int() > maxValue) {
                maxValue = m.eval(*getTVariable(vIt)).get_numeral_int();
            }
        }

        if (!this->quiet) { cout << "Latest Starttime: " << maxValue << endl; }

        for (auto &vIt : g.Vertices()){
            ex = *getTVariable(vIt);
            s.add(ex < maxValue);
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    void SMTMinLatNonModScheduler::z3CheckWithTimeTracking() {

        setZ3Timeout((uint32_t) (round(timeRemaining)));
        //printZ3Params();
        startTimeTracking();
        z3Check();
        endTimeTracking();

    }

    void SMTMinLatNonModScheduler::setSolverTimeout(double seconds) {
        setZ3Timeout((int)seconds);
        this->solverTimeout = (int)seconds;
    }

    void SMTMinLatNonModScheduler::scheduleAutoSearch() {

        cout << "Generating T-Variables ..." << endl;
        generateTVariables();

        cout << "Non Negative Constraints ..." << endl;
        addNonNegativeConstraints();

        cout << "Adding Dependency Constraints ..." << endl;
        addDependencyConstraints();

        int i = 0;
        bool breakCondition = false;
        do {
            i++;
            cout << acturalRow << ": " << getLatestStarttime() << endl;
            acturalRow = getLatestStarttime();
            generateBVariables();

            cout << i << ": Variable Connections ..." << endl;
            addVariableConnections();

            cout << i << ": Resource Constraints ..." << endl;
            addResourceConstraints();
            lastRow = acturalRow;
            breakCondition = (acturalRow >= getLatestStarttime()) or i > 1000;
        } while (!breakCondition);

        cout << getZ3Result() << endl;

        if (getZ3Result() == z3::sat) {
            for (auto &vIt: g.Vertices()) {
                startTimes.insert(std::make_pair(vIt, m.eval(*getTVariable(vIt)).get_numeral_int()));
            }
        }

        cout << "Schedule found... minimizing Starttimes" << endl;
        while (minimizeLatency() == z3::sat) {
            cout << getZ3Result() << endl;
        }
        cout << getZ3Result() << endl;

        latencyOptimal = (getZ3Result() == z3::unsat);

        for (auto &vIt : g.Vertices()){
            startTimes.at(vIt) = m.eval(*getTVariable(vIt)).get_numeral_int();
        }

        auto L = getScheduleLength();

        if (verifyResourceConstrainedSchedule(this->g, this->resourceModel, this->startTimes, L)) {
            cout << endl << "Schedule Valid" << endl;
            this->scheduleFound = true;
            this->II = getScheduleLength();
            cout << "Length of schedule: " << II << " is optimal: " << latencyOptimal << endl << endl;

            cout << "******************" << endl;
            cout << "* Final Schedule *" << endl;
            cout << "******************" << endl << endl;

            for (auto &vIt : g.Vertices()){
                cout << vIt->getName() << ": " << startTimes.at(vIt) << endl;
            }

        } else {
            this->scheduleFound = false;
            cout << "Schedule not Valid" << endl;
        }

    }

    void SMTMinLatNonModScheduler::scheduleASAPHCSearch() {

        if (!this->quiet) { cout << endl << "Generating ASAPHC Schedule ..." << endl << endl; }
        ASAPScheduler asapSched(this->g, this->resourceModel);
        asapSched.setQuiet(true);
        asapSched.schedule();
        startTimes = asapSched.getSchedule();

        if (!this->quiet) { cout << "****************************************************" << endl; }
        if (!this->quiet) { cout << "* Searching optimal Latency ... Building Model ... *" << endl; }
        if (!this->quiet) { cout << "****************************************************" << endl; }

        if (!this->quiet) { cout << "Generating T-Variables ... " << endl; }
        generateTVariables();

        if (!this->quiet) { cout << "Adding Non Negative Constraints ... " << endl; }
        addNonNegativeConstraints();

        if (!this->quiet) { cout << "Finding / Adding Max Latency Constaints ... " << endl; }
        findMaxLatencyConstraints();

        if (!this->quiet) { cout << "Adding Dependency Constraints ... " << endl; }
        addDependencyConstraints();

        if (!this->quiet) { cout << "Generating B-Variables ... " << endl; }
        generateBVariables();

        if (!this->quiet) { cout << "Connecting Variables ... " << endl;}
        addVariableConnections();

        if (!this->quiet) { cout << "Adding Resource Constraints ... Solving" << endl; }
        addResourceConstraints();

        if (!this->quiet) { cout << getZ3Result() << endl; }

        while (minimizeLatency() == z3::sat) {
            if (!this->quiet) { cout << getZ3Result() << endl;}
        }

        latencyOptimal = (getZ3Result() == z3::unsat);

        for (auto &vIt : g.Vertices()){
            startTimes.at(vIt) = m.eval(*getTVariable(vIt)).get_numeral_int();
        }

        auto L = getScheduleLength();

        if (verifyResourceConstrainedSchedule(this->g, this->resourceModel, this->startTimes, L)) {
            if (!this->quiet) { cout << endl << "Schedule Valid" << endl; }
            this->scheduleFound = true;
            this->II = getScheduleLength();
            if (!this->quiet) { cout << "Length of schedule: " << II << " is optimal: " << latencyOptimal << endl << endl; }

            if (!this->quiet) { cout << "******************" << endl; }
            if (!this->quiet) { cout << "* Final Schedule *" << endl; }
            if (!this->quiet) { cout << "******************" << endl << endl; }

            for (auto &vIt : g.Vertices()){
                cout << vIt->getName() << ": " << startTimes.at(vIt) << endl;
            }

        } else {
            this->scheduleFound = false;
            if (!this->quiet) { cout << "Schedule not Valid" << endl; }
        }

    }

} // HatScheT

#endif