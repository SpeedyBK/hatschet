//
// Created by bkessler on 4/1/24.
//

#include <sstream>
#include "SMASHScheduler.h"

#include "HatScheT/utility/Verifier.h"

namespace HatScheT {

    SMASHScheduler::SMASHScheduler(Graph &g, ResourceModel &resourceModel, int II)
            : IterativeModuloSchedulerLayer(g, resourceModel, II) {

        actualLength = 0; // Max Latency estimation...
        lastLength = 0;
        this->latencyEstimation = Utility::LatencyEstimation();

    }

    void SMASHScheduler::scheduleInit() {

        if(!quiet) {
            cout << "Scheduling with " << this->getName() <<":" << endl;
            cout << "Rec-Min-II: " << recMinII << endl;
            cout << "Res-Min-II: " << resMinII << endl;
            cout << "Max. II: " << maxII << endl;
            cout << "Vertices of G: " << g.getNumberOfVertices() << endl;
            cout << "Edges of G: " << g.getNumberOfEdges() << endl;
        }

        if (this->solverTimeout > 0) {
            setZ3Timeout((int) solverTimeout);
        }

    }

    void SMASHScheduler::scheduleIteration() {

        this->latencyEstimation = Utility::getLatencyEstimation(&g, &resourceModel, II, Utility::latencyBounds::both, true);

        actualLength = latencyEstimation.maxLat;

        generateTVariables();

        addNonNegativeConstraints();

        addMaxLatencyConstraint();

        addDependencyConstraints();

        if(getZ3Result() != z3::sat){
            return;
        }

        bool breakCondition = false;
        do {
            breakCondition = actualLength >= getLatestStarttime() or getZ3Result() == z3::unsat or getZ3Result()==z3::unknown;
            cout << "Actual Length: " << actualLength << " II: " << II << endl;
            generateBVariables();

            cout << "Variable Connections ... ";
            addVariableConnections();
            cout << getZ3Result() << endl;

            cout << "Resource Constraints ... ";
            addResourceConstraints((int)II);
            cout << getZ3Result() << endl;
        } while (!breakCondition);

        cout << getZ3Result() << endl;

        if (getZ3Result() == z3::sat)
        {
            for (auto &vIt : g.Vertices())
            {
                startTimes.insert(std::make_pair(vIt, m.eval(*getTVariable(vIt)).get_numeral_int()));
            }
        }
        else if (getZ3Result() == z3::unknown)
        {
            firstObjectiveOptimal = false;
            bVariables.clear();
            tVariables.clear();
            z3Reset();
            return;
        }
        else
        {
            bVariables.clear();
            tVariables.clear();
            z3Reset();
            return;
        }

        cout << "Schedule found... minimizing Starttimes" << endl;
        if (!disableSecObj) {
            while (minimizeLatency() == z3::sat) {
                cout << getZ3Result() << endl;
            }
            cout << getZ3Result() << endl;

            for (auto &vIt: g.Vertices()) {
                startTimes.at(vIt) = m.eval(*getTVariable(vIt)).get_numeral_int();
            }
        }

        secondObjectiveOptimal = (getZ3Result() == z3::unsat and !disableSecObj);

        scheduleFound = verifyModuloSchedule(g, resourceModel, startTimes, (int)II);
        if ( scheduleFound ){
            cout << "Schedule Valid for II = " << (int)II << endl;
            cout << maxII << endl;
        }else {
            cout << "Schedule NOT Valid" << endl;
            bVariables.clear();
            tVariables.clear();
            z3Reset();
            return;
        }

    }

    void SMASHScheduler::generateTVariables() {
        tVariables.clear();
        for (auto &vIt: g.Vertices()) {
            std::stringstream name;
            name << vIt->getName();
            z3::expr e(c.int_const(name.str().c_str()));
            tVariables.insert(std::make_pair(vIt, e));
        }
    }

    z3::expr *SMASHScheduler::getTVariable(Vertex *vPtr) {
        try {
            return &tVariables.at(vPtr);
        } catch (std::out_of_range &) {
            throw (HatScheT::Exception("SMASHScheduler: getTVariable() std::out_of_range"));
        }
    }

    z3::check_result SMASHScheduler::addNonNegativeConstraints() {

        for (auto &vIt: g.Vertices()) {
            s.add(tVariables.at(vIt) >= 0);
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMASHScheduler::addDependencyConstraints() {

        for (auto &eIt: g.Edges()) {
            Vertex *src = &(eIt->getVertexSrc());
            Vertex *dst = &(eIt->getVertexDst());

            auto ti = *getTVariable(src);
            auto tj = *getTVariable(dst);

            z3::expr e = ti - tj + this->resourceModel.getVertexLatency(src) + eIt->getDelay() - (int) this->II * (eIt->getDistance()) <= 0;
            e = e.simplify();
            this->s.add(e);
        }

        //z3CheckWithTimeTracking();

        return getZ3Result();

    }

    int SMASHScheduler::getLatestStarttime() {
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

    void SMASHScheduler::generateBVariables() {

        auto l = getScheduleLengthOfGivenSchedule(latencyEstimation.alapStartTimes, &resourceModel);

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }
            for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {

                for (int i = 0; i <= actualLength; ++i) {
                    if (i < latencyEstimation.asapStartTimes.at((Vertex*)vIt)){
                        continue;
                    }
                    if (i > actualLength - (l - latencyEstimation.alapStartTimes.at((Vertex*)vIt))){
                        continue;
                    }
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

    z3::expr *SMASHScheduler::getBvariable(Vertex *v, int i) {
        auto key = std::make_pair(v, i);
        try {
            return &bVariables.at(key);
        } catch (std::out_of_range &) {
            //cout << "Out_of_Range: " << v->getName() << "_" << i << endl;
            throw (HatScheT::Exception("smtbased Scheduler: getBvariable std::out_of_range"));
        }
    }

    z3::check_result SMASHScheduler::addVariableConnections() {

        auto l = getScheduleLengthOfGivenSchedule(latencyEstimation.alapStartTimes, &resourceModel);

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }

            for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {
                for (int i = lastLength; i <= actualLength; ++i) {
                    if (i < latencyEstimation.asapStartTimes.at((Vertex*)vIt)){
                        continue;
                    }
                    if (i > actualLength - (l - latencyEstimation.alapStartTimes.at((Vertex*)vIt))){
                        continue;
                    }
                    z3::expr constraint(c);
                    constraint = (*getTVariable((Vertex *) vIt) == i) == *getBvariable((Vertex *) vIt, i);
                    constraint = constraint.simplify();
                    s.add(constraint);
                }
            }
        }

        //z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMASHScheduler::addResourceConstraints(int candidateII) {

        auto l = getScheduleLengthOfGivenSchedule(latencyEstimation.alapStartTimes, &resourceModel);

        for (auto &it: resourceModel.Resources()) {
            if (it->isUnlimited()) {
                continue;
            }
            for (int i = 0; i < candidateII; i++) {
                set<const Vertex *> vSet = resourceModel.getVerticesOfResource(it);
                z3::expr_vector b_expressions(c);
                for (auto &vIt: vSet) {
                    for (int j = 0; j <= actualLength; j++) {
                        if ((j % candidateII) != i) {
                            continue;
                        }
                        if (j < latencyEstimation.asapStartTimes.at((Vertex*)vIt)){
                            continue;
                        }
                        if (j > actualLength - (l - latencyEstimation.alapStartTimes.at((Vertex*)vIt))){
                            continue;
                        }
                        b_expressions.push_back(*getBvariable((Vertex *) vIt, j));
                    }
                }
                if (!b_expressions.empty()) {
                    //z3::expr debug = z3::atmost(b_expressions, it->getLimit());
                    //cout << debug << endl;
                    z3::expr e = z3::atmost(b_expressions, it->getLimit());
                    e = e.simplify();
                    this->s.add(e);
                }
            }
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMASHScheduler::addMaxLatencyConstraint() {
        if (this->maxLatencyConstraint > 0) {
            std:array<int, 2> len = {actualLength, maxLatencyConstraint };
            actualLength = *std::min_element(len.begin(), len.end());
            auto l = getScheduleLengthOfGivenSchedule(latencyEstimation.alapStartTimes, &resourceModel);
            for (auto &vIt : g.Vertices()){
                z3::expr e = (*getTVariable(vIt) >= latencyEstimation.asapStartTimes.at(vIt)) && (*getTVariable(vIt) <= actualLength - (l - latencyEstimation.alapStartTimes.at(vIt)));
                e = e.simplify();
                s.add(e);
            }
        }else{
            auto l = getScheduleLengthOfGivenSchedule(latencyEstimation.alapStartTimes, &resourceModel);
            for (auto &vIt : g.Vertices()){
                z3::expr e = (*getTVariable(vIt) >= latencyEstimation.asapStartTimes.at(vIt)) && (*getTVariable(vIt) <= actualLength - (l - latencyEstimation.alapStartTimes.at(vIt)));
                e = e.simplify();
                s.add(e);
            }
        }

        //z3CheckWithTimeTracking();

        return getZ3Result();

    }

    void SMASHScheduler::setSolverTimeout(double seconds) {
        setZ3Timeout((int)seconds);
        this->solverTimeout = (int)seconds;
    }

  void SMASHScheduler::z3CheckWithTimeTracking() {

      setZ3Timeout((uint32_t)(ceil(timeRemaining)));
      if (!this->quiet){
          printZ3Params();
      }
      startTimeTracking();
      z3Check();
      endTimeTracking();

  }

    z3::check_result SMASHScheduler::minimizeLatency() {

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

    int SMASHScheduler::getScheduleLengthOfGivenSchedule(map<Vertex *, int> &vertexStarttimes, ResourceModel *rm) {
        int maxTime = -1;
        for (std::pair<Vertex *, int> vtPair: vertexStarttimes) {
            try {
                Vertex *v = vtPair.first;
                if ((vtPair.second + rm->getVertexLatency(v)) > maxTime) {
                    maxTime = (vtPair.second + rm->getVertexLatency(v));
                    //Debugging:
                    if (!quiet) {
                        //cout << vtPair.first->getName() << ": " << vtPair.second << " + "
                        //     << rm->getVertexLatency(v) << " = " << maxTime << endl;
                    }
                }
            } catch (std::out_of_range &) {
                cout << vtPair.first->getName() << ": " << vtPair.second << endl;
                throw (HatScheT::Exception("Utility::getSDCScheduleLength(): OUT_OF_RANGE"));
            }
        }
        return maxTime;
    }

}
