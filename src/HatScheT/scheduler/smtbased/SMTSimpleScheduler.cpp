//
// Created by bkessler on 4/1/24.
//

#include <sstream>
#include "SMTSimpleScheduler.h"

#include "HatScheT/utility/Verifier.h"
#include "HatScheT/utility/Utility.h"

namespace HatScheT {

    SMTSimpleScheduler::SMTSimpleScheduler(Graph &g, ResourceModel &resourceModel, int II)
            : IterativeModuloSchedulerLayer(g, resourceModel, II) {

        actualLength = 0; // Max Latency estimation...
        lastLength = 0;

    }

    void SMTSimpleScheduler::scheduleInit() {
        if(!quiet) {
            cout << "Scheduling with " << this->getName() <<":" << endl;
            cout << "Rec-Min-II: " << recMinII << endl;
            cout << "Res-Min-II: " << resMinII << endl;
            cout << "Max. II: " << maxII << endl;
            cout << "Vertices of G: " << g.getNumberOfVertices() << endl;
            cout << "Edges of G: " << g.getNumberOfEdges() << endl;
        }
    }

    void SMTSimpleScheduler::scheduleIteration() {

        if (this->solverTimeout > 0) {
            setZ3Timeout((int) solverTimeout);
        }

        actualLength = Utility::getLatencyEstimation(&g, &resourceModel, II, Utility::latencyBounds::maxLatency, true).maxLat;

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

        if (getZ3Result() == z3::unknown){
            firstObjectiveOptimal = false;
        }

        if (getZ3Result() == z3::sat){
            for (auto &vIt : g.Vertices()){
                startTimes.insert(std::make_pair(vIt, m.eval(*getTVariable(vIt)).get_numeral_int()));
            }
        }

        scheduleFound = verifyModuloSchedule(g, resourceModel, startTimes, (int)II);
        if ( scheduleFound ){
            cout << "Schedule Valid for II = " << (int)II << endl;
        }else {
            cout << "Schedule NOT Valid" << endl;
            bVariables.clear();
            tVariables.clear();
            z3Reset();
            return;
        }

    }

    void SMTSimpleScheduler::generateTVariables() {
        tVariables.clear();
        for (auto &vIt: g.Vertices()) {
            std::stringstream name;
            name << vIt->getName();
            z3::expr e(c.int_const(name.str().c_str()));
            tVariables.insert(std::make_pair(vIt, e));
        }
    }

    z3::expr *SMTSimpleScheduler::getTVariable(Vertex *vPtr) {
        try {
            return &tVariables.at(vPtr);
        } catch (std::out_of_range &) {
            throw (HatScheT::Exception("SMTSimpleScheduler: getTVariable() std::out_of_range"));
        }
    }

    z3::check_result SMTSimpleScheduler::addNonNegativeConstraints() {

        for (auto &vIt: g.Vertices()) {
            s.add(tVariables.at(vIt) >= 0);
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMTSimpleScheduler::addDependencyConstraints() {

        for (auto &eIt: g.Edges()) {
            Vertex *src = &(eIt->getVertexSrc());
            Vertex *dst = &(eIt->getVertexDst());

            auto ti = *getTVariable(src);
            auto tj = *getTVariable(dst);

            this->s.add(ti - tj + this->resourceModel.getVertexLatency(src) + eIt->getDelay() -
                        (int) this->II * (eIt->getDistance()) <= 0);
        }

        z3CheckWithTimeTracking();

        return getZ3Result();

    }

    int SMTSimpleScheduler::getLatestStarttime() {
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

    void SMTSimpleScheduler::generateBVariables() {

        auto tMax = actualLength;

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }
            for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {

                for (int i = 0; i <= tMax; ++i) {

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

    z3::expr *SMTSimpleScheduler::getBvariable(Vertex *v, int i) {
        auto key = std::make_pair(v, i);
        try {
            return &bVariables.at(key);
        } catch (std::out_of_range &) {
            //cout << "Out_of_Range: " << v->getName() << "_" << i << endl;
            throw (HatScheT::Exception("smtbased Scheduler: getBvariable std::out_of_range"));
        }
    }

    z3::check_result SMTSimpleScheduler::addVariableConnections() {

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }

            for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {
                for (int i = lastLength; i <= actualLength; ++i) {
                    z3::expr constraint(c);
                    constraint = (*getTVariable((Vertex *) vIt) == i) == *getBvariable((Vertex *) vIt, i);
                    s.add(constraint);
                }
            }
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMTSimpleScheduler::addResourceConstraints(int candidateII) {

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
                        b_expressions.push_back(*getBvariable((Vertex *) vIt, j));
                    }
                }
                if (!b_expressions.empty()) {
                    //z3::expr debug = z3::atmost(b_expressions, it->getLimit());
                    //cout << debug << endl;
                    this->s.add(z3::atmost(b_expressions, it->getLimit()));
                }
            }
        }

        z3CheckWithTimeTracking();

        return getZ3Result();
    }

    z3::check_result SMTSimpleScheduler::addMaxLatencyConstraint() {
        if (this->maxLatencyConstraint > 0) {
            for (auto &vIt : g.Vertices()){
                s.add(*getTVariable(vIt) <= maxLatencyConstraint);
            }
        }else{
            for (auto &vIt : g.Vertices()){
                s.add(*getTVariable(vIt) <= actualLength);
            }
        }

        z3CheckWithTimeTracking();

        return getZ3Result();

    }

    void SMTSimpleScheduler::setSolverTimeout(double seconds) {
        setZ3Timeout((int)seconds);
        this->solverTimeout = (int)seconds;
    }

  void SMTSimpleScheduler::z3CheckWithTimeTracking() {

      setZ3Timeout((uint32_t)(ceil(timeRemaining)));
      if (!this->quiet){
          printZ3Params();
      }
      startTimeTracking();
      z3Check();
      endTimeTracking();

  }

}
