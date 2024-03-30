//
// Created by Benjamin on 19.03.24.
//

#include "ASAPSMTScheduler.h"
#include <sstream>

#include "HatScheT/utility/Verifier.h"

#if USE_Z3

namespace HatScheT {

    ASAPSMTScheduler::ASAPSMTScheduler(Graph &g, ResourceModel &rm) : SchedulerBase(g, rm) {

    }

    void ASAPSMTScheduler::schedule() {

        generateTVariables();

        cout << "Non Negative Constraints ..." << endl;
        addNonNegativeConstraints();

        cout << "Dependency Constraints ..." << endl;
        addDependencyConstraints();

        generateBVariables();

        cout << "Variable Connections ..." << endl;
        addVariableConnections();

        cout << "Resource Constraints ..." << endl;
        addResourceConstraints();

        cout << getZ3Result() << endl;

        if (getZ3Result() == z3::sat){
            for (auto &vIt : g.Vertices()){
                cout << vIt->getName() << ": " << m.eval(*getTVariable(vIt)).get_numeral_int() << endl;
                startTimes.insert(std::make_pair(vIt, m.eval(*getTVariable(vIt)).get_numeral_int()));
            }
        }

        auto L = getScheduleLength();

        if (verifyResourceConstrainedSchedule(this->g, this->resourceModel, this->startTimes, L)){
            cout << "Schedule Valid" << endl;
        }else {
            cout << "Schedule not Valid" << endl;
        }

        exit(-1);

    }

    void ASAPSMTScheduler::generateTVariables() {
        tVariables.clear();
        for (auto &vIt : g.Vertices()){
            std::stringstream name;
            name << vIt->getName();
            z3::expr e(c.int_const(name.str().c_str()));
            tVariables.insert(std::make_pair(vIt, e));
        }
    }

    z3::check_result ASAPSMTScheduler::addDependencyConstraints() {

        for (auto &eIt : g.Edges()){

            if (eIt->getDistance() > 0){
                continue;
            }

            auto vSrc = &(eIt->getVertexSrc());
            auto vDst = &(eIt->getVertexDst());

            s.add(tVariables.at(vSrc) - tVariables.at(vDst) + this->resourceModel.getVertexLatency(vSrc) + eIt->getDelay() <= 0);

        }

        return z3Check();

    }

    z3::check_result ASAPSMTScheduler::addNonNegativeConstraints() {

        for (auto &vIt : g.Vertices()){
            s.add(tVariables.at(vIt) >= 0);
        }

        return z3Check();
    }

    z3::check_result ASAPSMTScheduler::addResourceConstraints() {

        for (auto &rIt : resourceModel.Resources()){
            if (rIt->isUnlimited()){
                continue;
            }

            int i = 0;
            int end = 0;
            do{
                end = getLatestStarttime();
                cout << i << ": " << end << endl;
                ++i;
                generateBVariables();
                addVariableConnections();
                z3::expr_vector ev(c);
                for (auto &vIt : resourceModel.getVerticesOfResource(rIt)){
                    ev.push_back(*getBvariable((Vertex*)vIt, i));
                }
                this->s.add(z3::atmost(ev, rIt->getLimit()));
                z3Check();
            } while ( i < end );
        }

        return getZ3Result();

    }

    z3::expr *ASAPSMTScheduler::getTVariable(Vertex* vPtr) {
        try {
            return &tVariables.at(vPtr);
        }catch(std::out_of_range&){
            //cout << "Out_of_Range: " << vPtr->getName() << endl;
            throw (HatScheT::Exception("ASAPSMTScheduler: getTVariable std::out_of_range"));
        }
    }

    z3::expr *ASAPSMTScheduler::getBvariable(Vertex *v, int i) {
        auto key = std::make_pair(v, i);
        try {
            return &bVariables.at(key);
        }catch(std::out_of_range&){
            //cout << "Out_of_Range: " << v->getName() << "_" << i << endl;
            throw (HatScheT::Exception("smtbased Scheduler: getBvariable std::out_of_range"));
        }
    }

    void ASAPSMTScheduler::generateBVariables() {

        auto tMax = getLatestStarttime();

        for (auto &rIt: resourceModel.Resources()) {
            if (rIt->isUnlimited()) {
                continue;
            }
            for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {

                for (int i = 0; i <= tMax; ++i) {

                    try {
                        getBvariable((Vertex*)vIt, i);
                    }catch (HatScheT::Exception&){

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

    int ASAPSMTScheduler::getLatestStarttime() {
        int tMax = 0;
        for (auto &vIt: g.Vertices()) {
            if (resourceModel.getResource(vIt)->isUnlimited()){
                continue;
            }
            if (tMax < m.eval(*getTVariable(vIt)).get_numeral_int()){
                tMax = m.eval(*getTVariable(vIt)).get_numeral_int();
            }
        }
        return tMax;
    }

    z3::check_result ASAPSMTScheduler::addVariableConnections() {

        for (auto &rIt : resourceModel.Resources()){
            if (rIt->isUnlimited()){
                continue;
            }

            for (auto &vIt : resourceModel.getVerticesOfResource(rIt)){
                for (int i = 0; i < getLatestStarttime(); ++i) {
                    z3::expr constraint(c);
                    constraint = z3::ite(*getTVariable((Vertex *) vIt) == i,
                                         *getBvariable((Vertex *) vIt, i),
                                         !*getBvariable((Vertex *) vIt, i));
                    s.add(constraint);
                }
            }

        }

        return z3Check();
    }

} // HatScheT

#endif