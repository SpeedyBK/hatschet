//
// Created by nfiege on 19/11/18.
//

#include "ModSDC.h"
#include <cmath>
#include <ctime>

namespace HatScheT
{
    const char* TimeoutException::what() const noexcept
    {
        return msg.c_str();
    }

    std::ostream& operator<<( std::ostream& oss, HatScheT::TimeoutException &e)
    {
        return oss << e.msg;
    }




    ModSDC::ModSDC(Graph &g, ResourceModel &rm, std::list<std::string> &sw) : SchedulerBase(g,rm), ILPSchedulerBase(sw), priorityForSchedQueue(), schedQueue(), scalpVariables() {
        cout << "#q# MODSDC CONSTRUCTOR START" << endl;
        this->computeMinII(&this->g,&resourceModel);
        cout << "#q# 1" << endl;
        this->minII = ceil(this->minII);
        cout << "#q# 2" << endl;
        this->computeMaxII(&g, &resourceModel);
        cout << "#q# 3" << endl;
        if (minII >= maxII) maxII = (int)minII+1;
        cout << "#q# 4" << endl;
        this->budget = -1;
        cout << "#q# 5" << endl;
        this->uniqueVariableName = "";
        cout << "#q# 6" << endl;
        this->solverQuiet = false;
        cout << "#q# MODSDC CONSTRUCTOR END" << endl;
    }

    void ModSDC::schedule() {
        cout << "ModSDC::schedule: start scheduling graph '" << this->g.getName() << "'" << endl;
        this->mrt = std::map<Vertex*, int>(); // create empty mrt
        if(this->budget<0) this->setDefaultBudget(); // set default budget according to paper if no budget was given by the user
        this->timeBudget = (double)this->solverTimeout;
        //this->timeTracker = time(nullptr);
        this->timeTracker = std::chrono::high_resolution_clock::now();

        for(this->II=this->minII; this->II<=this->maxII; this->II++)
        {
            cout << "ModSDC::schedule: Trying to find solution for II=" << this->II << endl;
            bool foundSolution = this->modSDCIteration((int)this->II,this->budget);
            if(foundSolution)
            {
                this->manageTimeBudget(); // this should never throw an error at this point
                /*
                this->startTimes = this->mrt;
                 */
                this->startTimes = this->sdcTimes; // last result from ILP solver is a valid schedule!
                this->fillStartTimesWithUnconstrainedInstructions();
                this->printSchedule();
                break;
            }
            else
            {
                cout << "ModSDC::schedule: Didn't find solution for II=" << this->II << endl;
                if(this->II == this->maxII)
                {
                    cout << "ERROR: ModuloSDC heuristic didn't find solution for graph '" << this->g.getName() << "', consider a higher budget" << endl;
                    throw HatScheT::Exception("ModuloSDC heuristic didn't find solution for graph '" + this->g.getName() + "', consider a higher budget");
                }
            }
            this->clearMaps();
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
        //////////////////////
        // ALGORITHM LINE 1 //
        //////////////////////
        // asap scheduling without resource constraints
        this->createInitialSchedule();
        //////////////////////
        // ALGORITHM LINE 2 //
        //////////////////////
        // create scheduling queue from all resource constrained instructions
        this->createSchedulingQueue(this->asapTimes);
        this->sdcTimes = this->asapTimes;
        // delete scheduling constraints from previous iteration
        this->clearAllAdditionalConstraints();
        while((!this->schedQueue.empty()) && (budget>=0))
        {
            try {
                this->manageTimeBudget();
            }
            catch(HatScheT::TimeoutException &e){
                this->timeBudget = this->solverTimeout;
                //this->timeTracker = time(nullptr);
                this->timeTracker = std::chrono::high_resolution_clock::now();
                return false;
            }
            //////////////////////
            // ALGORITHM LINE 4 //
            //////////////////////
            // pop first element from scheduling queue
            Vertex* I = this->schedQueue.front();
            this->schedQueue.pop_front();
            //////////////////////
            // ALGORITHM LINE 5 //
            //////////////////////
            int time = this->sdcTimes.at(I);
            if(time<0)
                throw HatScheT::Exception("Error: ModSDC::modSDCIteration: invalid time ("+to_string(time)+") found by ILP solver for instruction '"+I->getName()+"'");
            if(!this->hasResourceConflict(I,time))
            {
                ////////////////////////
                // ALGORITHM LINE 7-8 //
                ////////////////////////
                scheduleInstruction(I,time);
            }
            else
            {
                ///////////////////////
                // ALGORITHM LINE 10 //
                ///////////////////////
                // add constraint t_I >= time+1 to ilp formulation
                ScaLP::Constraint c(this->scalpVariables.at(I) >= (time+1));
                this->clearConstraintForVertex(I);
                this->createAdditionalConstraint(I,c);
                ///////////////////////
                // ALGORITHM LINE 11 //
                ///////////////////////
                bool foundSolution = this->solveSDCProblem();
                if(foundSolution)
                {
                    ///////////////////////
                    // ALGORITHM LINE 13 //
                    ///////////////////////
                    this->putIntoSchedQueue(I);
                }
                else
                {
                    ///////////////////////
                    // ALGORITHM LINE 15 //
                    ///////////////////////
                    this->clearConstraintForVertex(I);
                    ///////////////////////
                    // ALGORITHM LINE 16 //
                    ///////////////////////
                    this->backtracking(I,time);
                    ///////////////////////
                    // ALGORITHM LINE 17 //
                    ///////////////////////
                    foundSolution = this->solveSDCProblem(true);
                    if(!foundSolution){
                        cout << "ERROR: ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution" << endl;
                        throw HatScheT::Exception("ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution");
                    }
                }
            }
            ///////////////////////
            // ALGORITHM LINE 20 //
            ///////////////////////
            budget--;
        }
        // reset timer for next II calculation
        this->timeBudget = this->solverTimeout;
        //this->timeTracker = time(nullptr);
        this->timeTracker = std::chrono::high_resolution_clock::now();
        ///////////////////////
        // ALGORITHM LINE 22 //
        ///////////////////////
        if(!this->schedQueue.empty())
        {
            return false;
        }
        return true;
    }

    bool ModSDC::hasResourceConflict(const Vertex *I, const int &t) const {
        int limit = this->resourceModel.getResource(I)->getLimit();
        int neededResources(0);
        for(auto it : this->mrt)
        {
            auto v = it.first;
            auto t2 = it.second;
            // resource can only happen if both vertices use the same resource type
            if(this->resourceModel.getResource(I) == this->resourceModel.getResource(v))
            {
                // they need the same resource type in the same time slot
                if((t%((int)this->II)) == (t2%((int)this->II))) neededResources++;
                // resource conflict if more resources are needed than available
                if(neededResources >= limit) return true;
            }
        }
        return false;
    }

    void ModSDC::setDefaultBudget() {
        this->budget = 6 * this->g.getNumberOfVertices();
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
        }
    }

    void ModSDC::clearAllAdditionalConstraints() {
        for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); it++)
        {
            this->clearConstraintForVertex(*it);
        }
    }

    bool ModSDC::solveSDCProblem(bool printDetailsOnFailure) {
        this->setUpScalp();
        ScaLP::status s = this->solver->solve();
        if((s!=ScaLP::status::FEASIBLE) && (s!=ScaLP::status::OPTIMAL) && (s!=ScaLP::status::TIMEOUT_FEASIBLE)) {
            if(printDetailsOnFailure)
            {
                cout << "ModSDC::solveSDCProblem: didnt find solution. Status: " << s << endl;
                cout << "    constraints: " << endl;
                for(auto it : this->additionalConstraints)
                {
                    cout << "        " << it.first->getName() << ": " << (*it.second) << endl;
                }
            }
            return false;
        }
        ScaLP::Result r = this->solver->getResult();
        for(auto it : r.values)
        {
            Vertex* v = this->getVertexFromVariable(it.first);
            if(v != nullptr) this->sdcTimes[v] = (int)it.second;
            else this->scheduleLength = (int)it.second;
        }
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
        int minTime;
        for(minTime = this->asapTimes.at(I); minTime<=time; minTime++)
        {
            //////////////////////
            // ALGORITHM LINE 2 //
            //////////////////////
            auto tempConstraint = ScaLP::Constraint(this->scalpVariables.at(I) == minTime);
            this->createAdditionalConstraint(I,tempConstraint);
            bool foundSolution = this->solveSDCProblem();
            //////////////////////
            // ALGORITHM LINE 3 //
            //////////////////////
            this->clearConstraintForVertex(I);
            if(foundSolution) break;
            if(minTime == time){
                cout << "ERROR: backtracking algorithm can't find time slot for instruction " << I->getName() << endl;
                throw HatScheT::Exception("backtracking algorithm can't find time slot for instruction "+I->getName());
            }
        }
        //////////////////////
        // ALGORITHM LINE 5 //
        //////////////////////
        int prevSchedTime = this->getPrevSched(I);
        int evictTime;
        if(prevSchedTime<0 || minTime>=prevSchedTime)
        {
            //////////////////////
            // ALGORITHM LINE 7 //
            //////////////////////
            evictTime = minTime;
        }
        else
        {
            //////////////////////
            // ALGORITHM LINE 9 //
            //////////////////////
            evictTime = prevSchedTime+1;
        }
        if(evictTime<0)
            throw HatScheT::Exception("Error: ModSDC::backtracking: Invalid evict time ("+to_string(evictTime)+") for instruction '"+I->getName()+"'");
        std::list<Vertex*> evictInst = this->getResourceConflicts(I,evictTime);
        ///////////////////////////
        // ALGORITHM LINES 11-15 //
        ///////////////////////////
        for(auto it : evictInst)
        {
            this->unscheduleInstruction(it);
            // PUT OPERATION BACK INTO QUEUE EVEN THO IT IS NOT SPECIFIED IN PSEUDOCODE!
            this->putIntoSchedQueue(it);
        }
        ///////////////////////
        // ALGORITHM LINE 16 //
        ///////////////////////
        if(this->dependencyConflict(I,evictTime))
        {
            ///////////////////////
            // ALGORITHM LINE 17 //
            ///////////////////////
            for(auto it : this->additionalConstraints)
            {
                ///////////////////////////
                // ALGORITHM LINES 18-19 //
                ///////////////////////////
                this->unscheduleInstruction(it.first);
                ///////////////////////
                // ALGORITHM LINE 20 //
                ///////////////////////
                this->putIntoSchedQueue(it.first);
            }
        }
        //////////////////////////
        // ALGORITHM LINE 23-24 //
        //////////////////////////
        scheduleInstruction(I,evictTime);
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
        if((s!=ScaLP::status::FEASIBLE) && (s!=ScaLP::status::OPTIMAL) && (s!=ScaLP::status::TIMEOUT_FEASIBLE))
        {
            cout << "ScaLP Backend: " << this->solver->getBackendName() << endl;
            cout << "ScaLP Status: " << s << endl;
            cout << "ERROR: ModSDC::createInitialSchedule: failed to find schedule without resource constraints" << endl;
            throw HatScheT::Exception("ModSDC::createInitialSchedule: failed to find schedule without resource constraints");
        }

        ScaLP::Result r = this->solver->getResult();
        for(auto it : r.values)
        {
            Vertex* v = this->getVertexFromVariable(it.first);
            if(v != nullptr) this->asapTimes[v] = (int)it.second;
            else this->scheduleLength = (int)it.second;
        }

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

    void ModSDC::unscheduleInstruction(Vertex *evictInst, bool verbose) {
        if(verbose) cout << "ModSDC::unscheduleInstruction: unscheduling instruction " << evictInst->getName() << endl;
        this->clearConstraintForVertex(evictInst);
        this->mrt.erase(evictInst);
    }

    bool ModSDC::dependencyConflict(Vertex *I, const int &evictTime) {
        for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); it++)
        {
            auto e = (*it);
            // I == e.start
            if((&e->getVertexSrc() == I))
            {
                if(this->dependencyConflictForTwoInstructions(I,evictTime,this->sdcTimes.at(&e->getVertexDst()),e->getDelay(),e->getDistance()))
                {
                    return true;
                }
            }
            // I == e.dst
            if((&e->getVertexDst() == I))
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
        return ((newStartTime_i + this->resourceModel.getResource(i)->getLatency() + edgeDelay - newStartTime_j) > (this->II * distance));
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
        this->solver->timeout=(long)this->timeBudget;
    }

    void ModSDC::fillStartTimesWithUnconstrainedInstructions() {
        /*
        for(auto it : this->sdcTimes)
        {
            if(it.second<0)
                throw HatScheT::Exception("Error: ModSDC::fillStartTimesWithUnconstrainedInstructions: invalid start time found by ILP solver for Instruction '"+it.first->getName()+"': "+to_string(it.second)+"");
            try
            {
                this->startTimes.at(it.first);
            }
            catch(std::out_of_range&)
            {
                this->startTimes[it.first] = it.second;
            }
        }
         */
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

    void ModSDC::manageTimeBudget() {
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds timeSpan = std::chrono::duration_cast<std::chrono::milliseconds>(tp-this->timeTracker);
        double elapsedTime = ((double)timeSpan.count())/1000.0;
        this->timeTracker = tp;
        this->timeBudget -= elapsedTime;
        if(this->timeBudget<0) {
            cout << "timeout :)" << endl;
            throw HatScheT::TimeoutException("Timeout for II="+to_string(this->II));
        }
    }

    void ModSDC::printSchedule() {
        cout << "Found schedule with ModSDC!" << endl;
        cout << "    elapsed time: " << (double)this->solverTimeout - this->timeBudget << " sec; II=" << this->II << endl;
        for(auto it : this->startTimes)
        {
            cout << "    " << it.first->getName() << ": " << it.second << endl;
        }
    }

    void ModSDC::clearMaps() {
        this->mrt.clear();
        this->asapTimes.clear();
        this->sdcTimes.clear();
        this->prevSched.clear();
        this->schedQueue.clear();
    }

    void ModSDC::scheduleInstruction(Vertex *I, int t, bool verbose) {
        if(verbose) cout << "ModSDC::scheduleInstruction: scheduling instruction " << I->getName() << endl;
        if(t<0)
            throw HatScheT::Exception("Error: ModSDC::scheduleInstruction: Can't schedule instruction '"+I->getName()+"' to a cycle <0");
        ScaLP::Constraint c(this->scalpVariables[I] == t);
        this->createAdditionalConstraint(I,c);
        this->mrt[I] = t;
        this->prevSched[I] = t;
    }


}
