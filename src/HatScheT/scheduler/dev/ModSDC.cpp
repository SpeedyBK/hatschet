//
// Created by nfiege on 19/11/18.
//

#include "ModSDC.h"
#include <cmath>

#include "HatScheT/scheduler/ASAPScheduler.h"

namespace HatScheT
{

    ModSDC::ModSDC(Graph &g, ResourceModel &rm, std::list<std::string> &sw) : SchedulerBase(g,rm), ILPSchedulerBase(sw), priorityForSchedQueue(), schedQueue(), scalpVariables() {
        this->computeMinII(&this->g,&resourceModel);
        this->minII = ceil(this->minII);
        this->computeMaxII(&g, &resourceModel);
        if (minII >= maxII) maxII = (int)minII+1;
        this->budget = -1;
        this->uniqueVariableName = "";
        this->solverQuiet = false;
    }

    void ModSDC::schedule() {
        this->mrt = std::map<Vertex*, int>(); // create empty mrt
        if(this->budget<0) this->setDefaultBudget(); // set default budget according to paper if no budget was given by the user

        cout << "#q# ModSDC::schedule START for minII=" << this->minII << " and maxII=" << this->maxII << endl;
        for(this->II=this->minII; this->II<=this->maxII; this->II++)
        {
            //this->setUpScalp();
            bool foundSolution = this->modSDCIteration((int)this->II,this->budget);
            if(foundSolution)
            {
                this->startTimes = this->mrt;
                this->fillStartTimesWithUnconstrainedInstructions();
                break;
            }
        }
    }

    void ModSDC::constructProblem() {
        // clear old solver settings and create new variables
        this->createScalpVariables();
        this->createDataDependencyConstraints();
        this->createAdditionalConstraints();
    }

    void ModSDC::setObjective() {
        ScaLP::Term o;
        // minimize the latest end time
        // do that by creating a new variable with the constraint t_new >= t_start(i) + latency(i)
        this->setUniqueVariableName();
        ScaLP::Variable newVar = ScaLP::newIntegerVariable(this->uniqueVariableName,0,ScaLP::INF());

        for(auto it : this->scalpVariables)
        {
            ScaLP::Constraint c = (newVar-it.second >= this->resourceModel.getResource(it.first)->getLatency());
            *this->solver << c;
        }
        o = 1*newVar;
        this->solver->setObjective(ScaLP::minimize(o));
    }

    void ModSDC::createSchedulingQueue(const std::map<Vertex*, int> &scheduleMe) {
        for(auto it : scheduleMe)
        {
            Vertex* v = it.first;
            if(this->resourceModel.getResource(v)->getLimit()>0)
            {
                this->schedQueue.emplace_back(v);
            }
        }
    }

    bool ModSDC::modSDCIteration(const int &II, int budget) {
        cout << "#q# ModSDC::modSDCIteration BEGIN FOR II=" << II << endl;
        //////////////////////
        // ALGORITHM LINE 1 //
        //////////////////////
        // asap scheduling without resource constraints
        this->createInitialSchedule();
        cout << "#q# modSDCIteration line 1 finished" << endl;
        //////////////////////
        // ALGORITHM LINE 2 //
        //////////////////////
        // create scheduling queue from all resource constrained instructions
        this->createSchedulingQueue(this->asapTimes);
        this->sdcTimes = this->asapTimes;
        // delete scheduling constraints from previous iteration
        this->clearAllAdditionalConstraints();
        cout << "#q# modSDCIteration line 2 finished" << endl;
        while((!this->schedQueue.empty()) && (budget>=0))
        {
            cout << "#####################" << endl;
            cout << "#q# budget: " << budget << endl;
            cout << "#q# schedQueue:" << endl;
            for(auto it : this->schedQueue) cout << "#q#    " << it->getName() << endl;
            //////////////////////
            // ALGORITHM LINE 4 //
            //////////////////////
            // pop first element from scheduling queue
            Vertex* I = this->schedQueue.front();
            this->schedQueue.pop_front();
            cout << "#q# Instruction: " << I->getName() << endl;
            cout << "#q# modSDCIteration line 4 finished" << endl;
            //////////////////////
            // ALGORITHM LINE 5 //
            //////////////////////
            int time = this->sdcTimes[I];
            cout << "#q# schedule time: " << time << endl;
            cout << "#q# modSDCIteration line 5 finished" << endl;
            if(!this->hasResourceConflict(I,time))
            {
                cout << "#q# NO RESOURCE CONFLICT scheduling instruction '" << I->getName() << "' at time slot " << time << endl;
                //////////////////////
                // ALGORITHM LINE 7 //
                //////////////////////
                // add constraint t_I = time to ilp formulation
                ScaLP::Constraint c(this->scalpVariables[I] == time);
                this->createAdditionalConstraint(I,c);
                cout << "#q# modSDCIteration line 7 finished" << endl;
                //////////////////////
                // ALGORITHM LINE 8 //
                //////////////////////
                // add schedule time to mrt
                this->mrt[I] = time;
                // add schedule time to prevSched
                this->prevSched[I] = time;
                cout << "#q# set prevSched[" << I->getName() << "] = " << time << endl;
                cout << "#q# modSDCIteration line 8 finished" << endl;
            }
            else
            {
                cout << "#q# RESOURCE CONFLICT scheduling instruction '" << I->getName() << "' at time slot " << time << endl;
                ///////////////////////
                // ALGORITHM LINE 10 //
                ///////////////////////
                // add constraint t_I >= time+1 to ilp formulation
                ScaLP::Constraint c(this->scalpVariables[I] >= (time+1));
                this->clearConstraintForVertex(I);
                this->createAdditionalConstraint(I,c);
                cout << "#q# modSDCIteration line 10 finished" << endl;
                ///////////////////////
                // ALGORITHM LINE 11 //
                ///////////////////////
                bool foundSolution = this->solveSDCProblem();
                cout << "#q# modSDCIteration line 11 finished" << endl;
                if(foundSolution)
                {
                    ///////////////////////
                    // ALGORITHM LINE 13 //
                    ///////////////////////
                    this->putIntoSchedQueue(I);
                    cout << "#q# modSDCIteration line 13 finished" << endl;
                }
                else
                {
                    ///////////////////////
                    // ALGORITHM LINE 15 //
                    ///////////////////////
                    this->clearConstraintForVertex(I);
                    cout << "#q# modSDCIteration line 15 finished" << endl;
                    ///////////////////////
                    // ALGORITHM LINE 16 //
                    ///////////////////////
                    this->backtracking(I,time);
                    cout << "#q# modSDCIteration line 16 finished" << endl;
                    ///////////////////////
                    // ALGORITHM LINE 17 //
                    ///////////////////////
                    foundSolution = this->solveSDCProblem();
                    if(!foundSolution){

                        throw HatScheT::Exception("ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution");
                    }

                    cout << "#q# modSDCIteration line 17 finished" << endl;
                }
            }
            ///////////////////////
            // ALGORITHM LINE 20 //
            ///////////////////////
            budget--;
            cout << "#q# modSDCIteration line 20 finished" << endl;
            cout << "#q# MRT after this iteration:" << endl;
            for(auto it : this->mrt)
            {
                cout << "#q#    " << it.first->getName() << ": " << it.second << endl;
            }
            cout << "#####################" << endl << endl << endl;
        }
        ///////////////////////
        // ALGORITHM LINE 22 //
        ///////////////////////
        if(!this->schedQueue.empty())
        {
            cout << "#q# ModSDC::modSDCIteration END FOR II=" << II << endl;
            return false;
        }
        cout << "#q# ModSDC::modSDCIteration END FOR II=" << II << endl;
        return true;
    }

    bool ModSDC::hasResourceConflict(const Vertex *I, const int &t) const {
        cout << "#q# ModSDC::hasResourceConflict for instruction " << I->getName() << " at time " << t << endl;
        int limit = this->resourceModel.getResource(I)->getLimit();
        cout << "#q#     limit: " << limit << endl;
        int neededResources(0);
        for(auto it : this->mrt)
        {
            auto v = it.first;
            cout << "#q# other vertex: " << v->getName() << endl;
            auto t2 = it.second;
            // resource can only happen if both vertices use the same resource type
            if(this->resourceModel.getResource(I) == this->resourceModel.getResource(v))
            {
                // they need the same resource type in the same time slot
                if((t%((int)this->II)) == (t2%((int)this->II))) neededResources++;
                cout << "#q#    t: " << t << "; t2: " << t2 << " at II=" << this->II << endl;
                cout << "#q#    needed resources: " << neededResources << endl;
                // resource conflict if more resources are needed than available
                if(neededResources >= limit) return true;
            }
        }
        return false;
    }

    void ModSDC::setDefaultBudget() {
        this->budget = 6 * ModSDC::getNumberOfConstrainedVertices(this->g,this->resourceModel);
    }

    void ModSDC::createScalpVariables() {
        if(this->scalpVariables.empty())
        {
            for(auto it=this->g.verticesBegin(); it!=this->g.verticesEnd(); it++)
            {
                auto v = (*it);
                this->scalpVariables[v] = ScaLP::newIntegerVariable(v->getName(),0,ScaLP::INF());
            }
        }
    }

    void ModSDC::setUpScalp() {
        this->initSolver();
        this->constructProblem();
        this->setObjective();
    }

    int ModSDC::getNumberOfConstrainedVertices(Graph &g, ResourceModel &rm) {
        int counter = 0;
        for(auto it=g.verticesBegin(); it!=g.verticesEnd(); it++)
        {
            if(rm.getResource(*it)->getLimit()>0) counter++;
        }
        return counter;
    }

    int ModSDC::getPrevSched(Vertex *v) {
        cout << "#q# ModSDC::getPrevSched START" << endl;
        for(auto it : this->prevSched)
        {
            cout << "#q#    " << it.first->getName() << ": " << it.second << endl;
        }
        try
        {
            return this->prevSched.at(v);
        }
        catch(std::out_of_range&)
        {
            return -1;
        }
    }

    void ModSDC::createDataDependencyConstraints() {
        for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); it++)
        {
            Edge* edge = (*it);
            Vertex& src = edge->getVertexSrc();
            Vertex& dst = edge->getVertexDst();
            ScaLP::Constraint c = (this->scalpVariables.at(&src) + this->resourceModel.getResource(&src)->getLatency() + edge->getDelay() - this->scalpVariables.at(&dst) <= ((int)this->II) * edge->getDistance());
            *this->solver << c;
            cout << "#q# CONSTRAINT: " << c << endl;
        }
    }

    ScaLP::Constraint* ModSDC::getAdditionalConstraint(Vertex *v) {
        try
        {
            return this->additionalConstraints.at(v);
        }
        catch(std::out_of_range&)
        {
            return nullptr;
        }
    }

    void ModSDC::clearConstraintForVertex(Vertex *v) {
        if(this->getAdditionalConstraint(v)!=nullptr)
        {
            delete this->additionalConstraints.at(v);
            this->additionalConstraints.erase(v);
        }
    }

    void ModSDC::createAdditionalConstraint(Vertex *v, ScaLP::Constraint &c) {
        this->additionalConstraints[v] = new ScaLP::Constraint(c);
    }

    void ModSDC::createAdditionalConstraints() {
        for(auto it : this->additionalConstraints)
        {
            *this->solver << *it.second;
            cout << "#q# ADDITIONAL CONSTRAINT: " << *it.second << endl;
        }
    }

    void ModSDC::clearAllAdditionalConstraints() {
        for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); it++)
        {
            this->clearConstraintForVertex(*it);
        }
    }

    bool ModSDC::solveSDCProblem() {
        this->setUpScalp();
        ScaLP::status s = this->solver->solve();
        if((s!=ScaLP::status::FEASIBLE) && (s!=ScaLP::status::OPTIMAL)) {
            cout << "#q# ModSDC::solveSDCProblem: failed to find feasible solution; solution status: " << s << endl;
            cout << "#q#    Attempted solution:" << endl;
            cout << this->solver->getResult();
            return false;
        }
        ScaLP::Result r = this->solver->getResult();
        for(auto it : r.values)
        {
            Vertex* v = this->getVertexFromVariable(it.first);
            if(v != nullptr) this->sdcTimes[v] = (int)it.second;
        }
        cout << "#q# SOLUTION: " << endl;
        cout << r << endl;
        return true;
    }

    Vertex *ModSDC::getVertexFromVariable(const ScaLP::Variable &sv) {
        for(auto it : this->scalpVariables)
        {
            if(it.second == sv) return it.first;
        }
        return nullptr;
    }

    void ModSDC::backtracking(Vertex *I, const int &time) {
        cout << "#q# ModSDC::backtracking BEGIN" << endl;
        int minTime;
        for(minTime = this->asapTimes.at(I); minTime<=time; minTime++)
        {
            //////////////////////
            // ALGORITHM LINE 2 //
            //////////////////////
            this->clearConstraintForVertex(I);
            auto tempConstraint = ScaLP::Constraint(this->scalpVariables.at(I) == minTime);
            this->createAdditionalConstraint(I,tempConstraint);
            bool foundSolution = this->solveSDCProblem();
            cout << "#q# backtracking line 2 finished" << endl;
            //////////////////////
            // ALGORITHM LINE 3 //
            //////////////////////
            if(foundSolution) break;
            if(minTime == time)
                throw HatScheT::Exception("backtracking algorithm can't find time slot for instruction "+I->getName());
            cout << "#q# backtracking line 3 finished (didn't find solution)" << endl;
        }
        //////////////////////
        // ALGORITHM LINE 5 //
        //////////////////////
        int prevSchedTime = this->getPrevSched(I);
        int evictTime;
        cout << "#q# backtracking line 5 finished" << endl;
        if(prevSchedTime<0 || minTime>=prevSchedTime)
        {
            //////////////////////
            // ALGORITHM LINE 7 //
            //////////////////////
            evictTime = minTime;
            cout << "#q# backtracking line 7 finished" << endl;
        }
        else
        {
            //////////////////////
            // ALGORITHM LINE 9 //
            //////////////////////
            evictTime = prevSchedTime+1;
            cout << "#q# backtracking line 9 finished" << endl;
        }
        std::list<Vertex*> evictInst = this->getResourceConflicts(I,evictTime);
        ///////////////////////////
        // ALGORITHM LINES 11-15 //
        ///////////////////////////
        for(auto it : evictInst)
        {
            this->unscheduleInstruction(it);
            // PUTTING OPERATION BACK INTO QUEUE EVEN THO IT IS NOT SPECIFIED IN PSEUDOCODE!
            this->putIntoSchedQueue(it);
        }
        cout << "#q# backtracking lines 11-15 finished" << endl;
        ///////////////////////
        // ALGORITHM LINE 16 //
        ///////////////////////
        cout << "#q# backtracking line 16 finished" << endl;
        if(this->dependencyConflict(I,evictTime))
        {
            cout << "#q# FOUND DEPENDENCY CONFLICT; UNSCHEDULING ALL OTHER OPERATIONS" << endl;
            ///////////////////////
            // ALGORITHM LINE 17 //
            ///////////////////////
            for(auto it : this->prevSched)
            {
                ///////////////////////////
                // ALGORITHM LINES 18-19 //
                ///////////////////////////
                this->unscheduleInstruction(it.first);
                cout << "#q# backtracking lines 18-19 finished" << endl;
                ///////////////////////
                // ALGORITHM LINE 20 //
                ///////////////////////
                this->putIntoSchedQueue(it.first);
                cout << "#q# backtracking line 20 finished" << endl;
            }
        }
        ///////////////////////
        // ALGORITHM LINE 23 //
        ///////////////////////
        auto tempConstraint(this->scalpVariables.at(I) == evictTime);
        this->createAdditionalConstraint(I,tempConstraint);
        cout << "#q# backtracking line 23 finished" << endl;
        ///////////////////////
        // ALGORITHM LINE 24 //
        ///////////////////////
        // add schedule time to mrt
        this->mrt[I] = time;
        // add schedule time to prevSched
        this->prevSched[I] = time;
        cout << "#q# set prevSched[" << I->getName() << "] = " << time << endl;
        cout << "#q# backtracking line 24 finished" << endl;
        cout << "#q# ModSDC::backtracking END" << endl;
    }

    bool ModSDC::resourceConflict(Vertex *I, const int &evictTime) {
        unsigned int usedResources = 0;
        auto maxResources = (unsigned int)this->resourceModel.getResource(I)->getLimit(); // limit=-1 => unsigned int maximum
        std::list<Vertex*> verticesAtTimeSlot = this->getVerticesAtTimeSlot(evictTime);
        for(auto v : verticesAtTimeSlot)
        {
            if(this->resourceModel.getResource(v) == this->resourceModel.getResource(I)) usedResources++;
            if(usedResources>maxResources) return true;
        }
        return false;
    }

    void ModSDC::createInitialSchedule() {
        this->setUpScalp();
        ScaLP::status s = this->solver->solve();
        if((s!=ScaLP::status::FEASIBLE) && (s!=ScaLP::status::OPTIMAL))
        {
            cout << "ScaLP Backend: " << this->solver->getBackendName() << endl;
            cout << "ScaLP Status: " << s << endl;
            cout << "Attempted solution: " << endl;
            cout << this->solver->getResult() << endl;
            throw HatScheT::Exception("ModSDC::createInitialSchedule: failed to find schedule without resource constraints");
        }

        ScaLP::Result r = this->solver->getResult();
        for(auto it : r.values)
        {
            Vertex* v = this->getVertexFromVariable(it.first);
            if(v != nullptr) this->asapTimes[v] = (int)it.second;
        }

        cout << "#q# found ASAP values:" << endl;
        cout << r << endl;
        this->calculatePriorities(); // calculate priorities for scheduling queue
    }

    std::list<Vertex *> ModSDC::getVerticesAtTimeSlot(const int &t) {
        std::list<Vertex*> l;
        for(auto it : this->sdcTimes)
        {
            if(it.second == t) l.emplace_back(it.first);
        }
        return l;
    }

    std::list<Vertex*> ModSDC::getResourceConflicts(Vertex* I, const int &evictTime) {
        std::list<Vertex*> l;
        const Resource* resourceType = this->resourceModel.getResource(I);
        for(auto it : this->mrt)
        {
            auto v = it.first;
            auto t = it.second;
            if(this->resourceModel.getResource(v) == resourceType && t==evictTime && v!=I) l.emplace_back(v);
        }
        return l;
    }

    void ModSDC::unscheduleInstruction(Vertex *evictInst) {
        this->clearConstraintForVertex(evictInst);
        this->mrt.erase(evictInst);
    }

    bool ModSDC::dependencyConflict(Vertex *I, const int &evictTime) {
        for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); it++)
        {
            auto e = (*it);
            // edge destination is in mrt and I == e.start
            if((this->mrt.find(&e->getVertexDst()) != this->mrt.end()) && (&e->getVertexSrc() == I))
            {
                if(this->dependencyConflictForTwoInstructions(I,evictTime,this->sdcTimes.at(&e->getVertexDst()),e->getDelay(),e->getDistance()))
                {
                    return true;
                }
            }

            // edge source is in mrt and I == e.end
            if((this->mrt.find(&e->getVertexSrc()) != this->mrt.end()) && (&e->getVertexDst() == I))
            {
                if(this->dependencyConflictForTwoInstructions(&e->getVertexSrc(),this->sdcTimes.at(&e->getVertexSrc()),evictTime,e->getDelay(),e->getDistance()))
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool ModSDC::dependencyConflictForTwoInstructions(Vertex* i, const int &newStartTime_i, const int &newStartTime_j, const int& edgeDelay, const int& distance) {
        return ((newStartTime_i + this->resourceModel.getResource(i)->getLatency() + edgeDelay - newStartTime_j) <= (this->II * distance));
    }

    void ModSDC::setUniqueVariableName() {
        if(!this->uniqueVariableName.empty()) return;
        this->uniqueVariableName = "you_sexy_creature";
        bool unique = false;
        while(!unique)
        {
            unique = true;
            for(auto it : this->scalpVariables)
            {
                if(this->uniqueVariableName == it.second->getName())
                {
                    unique = false;
                    this->uniqueVariableName += "_I_like_your_choice_of_vertex_names";
                    break;
                }
            }
        }
    }

    void ModSDC::initSolver() {
        this->solver->reset();
        if(this->threads>0) this->solver->threads = this->threads;
        this->solver->quiet=this->solverQuiet;
        this->solver->timeout=this->solverTimeout;
    }

    void ModSDC::fillStartTimesWithUnconstrainedInstructions() {
        for(auto it : this->sdcTimes)
        {
            try
            {
                this->startTimes.at(it.first);
            }
            catch(std::out_of_range&)
            {
                this->startTimes[it.first] = it.second;
            }
        }
    }

    void ModSDC::putIntoSchedQueue(Vertex *I) {
        int prio = this->priorityForSchedQueue.at(I);
        for(auto it = this->schedQueue.begin(); it != this->schedQueue.end(); it++)
        {
            if(prio >= this->priorityForSchedQueue.at(*it))
            {
                this->schedQueue.emplace(it,I);
                return;
            }
        }
        this->schedQueue.emplace_back(I);
    }

    void ModSDC::calculatePriorities() {
        // priority of an instruction depends on how many operations depend on the result of this one
        // use asap times to estimate that metric :)
        int scheduleLength = 0;
        for(auto it : this->solver->getResult().values)
        {
            bool found = false;
            for(auto it2 : this->scalpVariables)
            {
                if(it.first == it2.second)
                {
                    found = true;
                    break;
                }
            }
            // schedule length is coded into the additional variable (created in this->setObjective())
            if(!found) {
                scheduleLength = (int)it.second;
                break;
            }
        }
        for(auto it : this->asapTimes)
        {
            this->priorityForSchedQueue[it.first] = scheduleLength - it.second;
        }
    }

    ModSDC::~ModSDC() {
        for(auto it : this->additionalConstraints) delete it.second;
    }


}
