//
// Created by Benjamin on 19.03.24.
//

#include "SMTMinLatNonModScheduler.h"
#include <sstream>
#include <cmath>

#include "HatScheT/utility/Verifier.h"

#if USE_Z3

namespace HatScheT {

    SMTMinLatNonModScheduler::SMTMinLatNonModScheduler(Graph &g, ResourceModel &rm) : SchedulerBase(g, rm) {

        acturalRow = 0;
        lastRow = 0;
        latencyOptimal = false;

    }

    void SMTMinLatNonModScheduler::schedule() {

        generateTVariables();

        cout << "Non Negative Constraints ..." << endl;
        addNonNegativeConstraints();

        cout << "Dependency Constraints ..." << endl;
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

    void SMTMinLatNonModScheduler::generateTVariables() {
        tVariables.clear();
        for (auto &vIt: g.Vertices()) {
            std::stringstream name;
            name << vIt->getName();
            z3::expr e(c.int_const(name.str().c_str()));
            tVariables.insert(std::make_pair(vIt, e));
        }
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

        auto tMax = getLatestStarttime();

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

        cout << "Latest Starttime: " << maxValue << endl;

        for (auto &vIt : g.Vertices()){
            ex = *getTVariable(vIt);
            s.add(ex < maxValue);
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    void SMTMinLatNonModScheduler::z3CheckWithTimeTracking() {

        setZ3Timeout((uint32_t) (round(timeRemaining)));
        if (!this->quiet) {
            printZ3Params();
        }
        startTimeTracking();
        z3Check();
        endTimeTracking();

    }

    void SMTMinLatNonModScheduler::setSolverTimeout(double seconds) {
        setZ3Timeout((int)seconds);
        this->solverTimeout = (int)seconds;
    }

} // HatScheT

#endif