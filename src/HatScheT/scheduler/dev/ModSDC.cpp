//
// Created by nfiege on 19/11/18.
//

#include "ModSDC.h"
#include <cmath>
#include <ctime>
#include <stdlib.h>
#include <sstream>
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/utility/Utility.h"

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

    ModSDC::ModSDC(Graph &g, ResourceModel &rm, std::list<std::string> &sw) :
        SchedulerBase(g,rm), ILPSchedulerBase(sw),
        priorityForSchedQueue(), schedQueue(), scalpVariables(), timeInILPSolvers(0.0), fastObjective(true),
        pType(PriorityHandler::priorityType::ALAP), budget(-1), uniqueVariableName(""),
        budgetMultiplier(6) {

        this->solverQuiet = true;
        this->computeMinII(&this->g,&resourceModel);
        this->minII = ceil(this->minII);
        this->computeMaxII(&g, &resourceModel);
        if (minII >= maxII) maxII = (int)minII+1;

    }

    void ModSDC::schedule() {
        time_t t = time(nullptr);
        cout << "ModSDC::schedule: start scheduling graph '" << this->g.getName() << "' " << ctime(&t) << endl;
		this->calculatePriorities(); // calculate priorities for scheduling queue
		t = time(nullptr);
        this->mrt = std::map<Vertex*, int>(); // create empty mrt
        if(this->budget<0) this->setDefaultBudget(); // set default budget according to paper if no budget was given by the user
        this->timeBudget = (double)this->solverTimeout;
        this->timeTracker = std::chrono::high_resolution_clock::now();

        for(this->II=this->minII; this->II<=this->maxII; this->II++)
        {
            cout << "ModSDC::schedule: Trying to find solution for II=" << this->II << endl;
            bool foundSolution = this->modSDCIteration((int)this->II,this->budget);
            if(foundSolution)
            {
				cout << "Spent " << this->timeInILPSolvers << " sec in ILP solvers" << endl;
                if(!this->manageTimeBudgetSuccess())
                {
                    throw HatScheT::Exception("Time budget used up after scheduling success - that should never happen!");
                }
                this->startTimes = this->sdcTimes; // last result from ILP solver is a valid schedule!
				this->scheduleFound = true;
                break;
            }
            else
            {
                cout << "ModSDC::schedule: Didn't find solution for II=" << this->II << endl;
                if(this->II == this->maxII)
                {
                	cout << "Spent " << this->timeInILPSolvers << " sec in ILP solvers" << endl;
                    cout << "ERROR: ModuloSDC heuristic didn't find solution for graph '" << this->g.getName() << "', consider a higher budget or another priority function" << endl;
                    throw HatScheT::Exception("ModuloSDC heuristic didn't find solution for graph '" + this->g.getName() + "', consider a higher budget or another priority function");
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
        if(this->fastObjective) {
            // minimize sum of all start times (significantly less constraints, but slightly more complex objective)
            ScaLP::Term o;
            for(auto it : this->scalpVariables) {
                o += it.second;
            }
            this->solver->setObjective(ScaLP::minimize(o));
        }
        else {
            // minimize the latest end time
            // do that by creating a new variable with the constraint t_new >= t_start(i) + latency(i)
            this->setUniqueVariableName();
            ScaLP::Variable newVar = ScaLP::newIntegerVariable(this->uniqueVariableName,0,ScaLP::INF());
            for(auto it : this->scalpVariables)
            {
                ScaLP::Constraint c = (newVar-it.second >= this->resourceModel.getResource(it.first)->getLatency());
                *this->solver << c;
            }
            ScaLP::Term o = 1*newVar;
            this->solver->setObjective(ScaLP::minimize(o));
        }
    }

    void ModSDC::createSchedulingQueue(const std::list<Vertex*> scheduleMe) {
        for(auto it : scheduleMe)
        {
            if(this->resourceModel.getResource(it)->getLimit()>0)
            {
                PriorityHandler::putIntoSchedQueue(it,this->pType,&this->priorityForSchedQueue,&this->schedQueue);
            }
        }
    }

    bool ModSDC::modSDCIteration(const int &II, int budget) {

        // delete scheduling constraints from previous iteration
        this->clearAllAdditionalConstraints();
        //////////////////////
        // ALGORITHM LINE 1 //
        //////////////////////
        // asap scheduling without resource constraints
        this->createInitialSchedule();
        //////////////////////
        // ALGORITHM LINE 2 //
        //////////////////////
        // create scheduling queue from all resource constrained instructions
        this->createSchedulingQueue(this->getResourceConstrainedVertices());
        this->sdcTimes = this->asapTimes;

        while((!this->schedQueue.empty()) && (budget>=0))
        {
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
                bool foundSolution;
                try
                {
                    foundSolution = this->solveSDCProblem();
                }
                catch(HatScheT::TimeoutException &e)
                {
                    this->handleTimeout();
                    return false;
                }
                if(foundSolution)
                {
                    ///////////////////////
                    // ALGORITHM LINE 13 //
                    ///////////////////////
                    PriorityHandler::putIntoSchedQueue(I,this->pType,&this->priorityForSchedQueue,&this->schedQueue);
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
                    try{
                        this->backtracking(I,time);
                    }
                    catch(HatScheT::TimeoutException &e)
                    {
                        this->handleTimeout();
                        return false;
                    }
                    ///////////////////////
                    // ALGORITHM LINE 17 //
                    ///////////////////////
                    try {
                        foundSolution = this->solveSDCProblem(true);
                        if(!foundSolution){
                            cout << "ERROR: ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution" << endl;
                            throw HatScheT::Exception("ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution");
                        }
                    }
                    catch(HatScheT::TimeoutException &e)
                    {
                        this->handleTimeout();
                        return false;
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
        this->timeTracker = std::chrono::high_resolution_clock::now();
        ///////////////////////
        // ALGORITHM LINE 22 //
        ///////////////////////
        return this->schedQueue.empty();
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
                if((t%((int)this->II)) == (t2%((int)this->II))) {
                	neededResources++;
                }
                // resource conflict if more resources are needed than available
                if(neededResources >= limit) {
                	return true;
                }
            }
        }
        return false;
    }

    void ModSDC::setDefaultBudget() {
		this->budget = (int)(this->budgetMultiplier * this->g.getNumberOfVertices());
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
        for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); it++) {
            Edge* edge = (*it);
            Vertex& src = edge->getVertexSrc();
            Vertex& dst = edge->getVertexDst();
            ScaLP::Constraint c = (this->scalpVariables.at(&src) + this->resourceModel.getResource(&src)->getLatency() + edge->getDelay() - this->scalpVariables.at(&dst) <= ((int)this->II) * edge->getDistance());
            *this->solver << c;
        }
    }

    ScaLP::Constraint* ModSDC::getAdditionalConstraint(Vertex *v) {
        try {
            return this->additionalConstraints.at(v);
        }
        catch(std::out_of_range&) {
            return nullptr;
        }
    }

    void ModSDC::clearConstraintForVertex(Vertex *v) {
        if(this->getAdditionalConstraint(v)!=nullptr) {
            delete this->additionalConstraints.at(v);
            this->additionalConstraints.erase(v);
        }
    }

    void ModSDC::createAdditionalConstraint(Vertex *v, ScaLP::Constraint &c) {
        this->additionalConstraints[v] = new ScaLP::Constraint(c);
    }

    void ModSDC::createAdditionalConstraints() {
        for(auto it : this->additionalConstraints) {
            *this->solver << *it.second;
        }
    }

    void ModSDC::clearAllAdditionalConstraints() {
        for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); it++) {
            this->clearConstraintForVertex(*it);
        }
    }

    bool ModSDC::solveSDCProblem(bool printDetailsOnFailure) {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        this->setUpScalp();
        if(!this->manageTimeBudgetSuccess()) {
            cout << "Timeout!" << endl;
            throw HatScheT::TimeoutException("Time budget empty when trying to find solution for II="+to_string(this->II));
        }

        // solve ilp and track time
        ScaLP::status s = this->solver->solve();
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1);
        this->timeInILPSolvers += (((double)timeSpan.count())/1000000000.0);

        if(s==ScaLP::status::TIMEOUT_INFEASIBLE)
        {
            cout << "Timeout!" << endl;
            throw HatScheT::TimeoutException("Solver timeout when trying to find solution for II="+to_string(this->II));
        }
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
        if(minTime>=prevSchedTime || prevSchedTime<0)
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
            PriorityHandler::putIntoSchedQueue(it,this->pType,&this->priorityForSchedQueue,&this->schedQueue);
        }
        ///////////////////////
        // ALGORITHM LINE 16 //
        ///////////////////////
        if(this->dependencyConflict(I,evictTime))
        {
            ///////////////////////
            // ALGORITHM LINE 17 //
            ///////////////////////
            auto copy = this->additionalConstraints;
            for(auto it : copy)
            {
                ///////////////////////////
                // ALGORITHM LINES 18-19 //
                ///////////////////////////
                this->unscheduleInstruction(it.first);
                ///////////////////////
                // ALGORITHM LINE 20 //
                ///////////////////////
                PriorityHandler::putIntoSchedQueue(it.first,this->pType,&this->priorityForSchedQueue,&this->schedQueue);
            }


        }
        //////////////////////////
        // ALGORITHM LINE 23-24 //
        //////////////////////////
        scheduleInstruction(I,evictTime);
    }

    void ModSDC::createInitialSchedule() {
        this->setUpScalp();
        ScaLP::status s = this->solver->solve(); // solver should never timeout here...
        if((s!=ScaLP::status::FEASIBLE) && (s!=ScaLP::status::OPTIMAL) && (s!=ScaLP::status::TIMEOUT_FEASIBLE))
        {
            cout << "ScaLP Backend: " << this->solver->getBackendName() << endl;
            cout << "ScaLP Status: " << s << endl;
            cout << "Additional constraints: " << endl;
            this->printAdditionalSolverConstraints();
            cout << "ERROR: ModSDC::createInitialSchedule: failed to find schedule without resource constraints" << endl;
            throw HatScheT::Exception("ModSDC::createInitialSchedule: failed to find schedule without resource constraints");
        }

        ScaLP::Result r = this->solver->getResult();
        for(auto it : r.values)
        {
            Vertex* v = this->getVertexFromVariable(it.first);
            if(v != nullptr) this->asapTimes[v] = (int)it.second;
        }

    }

    std::list<Vertex*> ModSDC::getResourceConflicts(Vertex* I, const int &evictTime) {
        std::list<Vertex*> l;
        const Resource* resourceType = this->resourceModel.getResource(I);
        for(auto it : this->mrt)
        {
            auto v = it.first;
            auto t = it.second;
            if(this->resourceModel.getResource(v) == resourceType && (t%((int)this->II))==(evictTime%((int)this->II)) && v!=I) l.emplace_back(v);
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

    void ModSDC::calculatePriorities() {
        // Paper: priority of an instruction depends on how many operations depend on the result of this one
        // => different metrics possible; only god knows whats best
        switch (this->pType) {
            case PriorityHandler::priorityType::ALAP: {
                auto p = this->getALAPScheduleWithoutResourceConstraints();
                for(auto it : p) {
                	if(this->resourceModel.getResource(it.first)->getLimit()>=0){
						this->priorityForSchedQueue[it.first] = new PriorityHandler(this->pType,it.second);
                	}
                }
                break;
            }
            case PriorityHandler::priorityType::ASAP: {
                auto p = this->getASAPScheduleWithoutResourceConstraints();
                for(auto it : p) {
					if(this->resourceModel.getResource(it.first)->getLimit()>=0){
						this->priorityForSchedQueue[it.first] = new PriorityHandler(this->pType,it.second);
					}
                }
                break;
            }
            case PriorityHandler::priorityType::MOBILITY_LOW: {
                auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
                auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
                for(auto it : pALAP) {
                    try {
						if(this->resourceModel.getResource(it.first)->getLimit()>=0){
							this->priorityForSchedQueue[it.first] = new PriorityHandler(this->pType,it.second-pASAP.at(it.first));
						}
                    }
                    catch(std::out_of_range&) {
                        throw HatScheT::Exception("Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
                    }
                }
                break;
            }
            case PriorityHandler::priorityType::MOBILITY_HIGH: {
                auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
                auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
                for(auto it : pALAP) {
                    try {
						if(this->resourceModel.getResource(it.first)->getLimit()>=0){
							this->priorityForSchedQueue[it.first] = new PriorityHandler(this->pType,it.second-pASAP.at(it.first));
						}
                    }
                    catch(std::out_of_range&) {
                        throw HatScheT::Exception("Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
                    }
                }
                break;
            }
            case PriorityHandler::priorityType::MOBLAP: {
                auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
                auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
                for(auto it : pALAP) {
                    try {
						if(this->resourceModel.getResource(it.first)->getLimit()>=0){
							this->priorityForSchedQueue[it.first] = new PriorityHandler(this->pType,it.second-pASAP.at(it.first),it.second);
						}
                    }
                    catch(std::out_of_range&) {
                        throw HatScheT::Exception("Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
                    }
                }
                break;
            }
            case PriorityHandler::priorityType::ALABILITY: {
                auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
                auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
                for(auto it : pALAP) {
                    try {
						if(this->resourceModel.getResource(it.first)->getLimit()>=0){
							this->priorityForSchedQueue[it.first] = new PriorityHandler(this->pType,it.second,it.second-pASAP.at(it.first));
						}
                    }
                    catch(std::out_of_range&) {
                        throw HatScheT::Exception("Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
                    }
                }
                break;
            }
            case PriorityHandler::priorityType::SUBSEQUALAP: {
                auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
                for(auto it : pALAP) {
                    auto noOfSubseq = this->getNoOfSubsequentVertices(it.first);
                    this->priorityForSchedQueue[it.first] = new PriorityHandler(this->pType,noOfSubseq,it.second);
                }
                break;
            }
            case PriorityHandler::priorityType::RANDOM: {
                srand((unsigned int)time(nullptr));
                for(auto it : this->g.Vertices()) {
					if(this->resourceModel.getResource(it)->getLimit()>=0){
						this->priorityForSchedQueue[it] = new PriorityHandler(this->pType,rand());
					}
                }
                break;
            }
            case PriorityHandler::priorityType::CUSTOM: {
				// just check if all priorities are set
				for(auto it : this->g.Vertices()) {
					if(this->resourceModel.getResource(it)->getLimit()>=0) {
						try {
							this->priorityForSchedQueue.at(it);
						}
						catch(std::out_of_range&) {
							throw HatScheT::Exception("Priority for vertex '"+it->getName()+"' is not set");
						}
					}
				}
				break;
			}
            default:
                throw HatScheT::Exception("No priority type for scheduling queue order specified");
        }
    }

    ModSDC::~ModSDC() {
		for(auto it : this->additionalConstraints) delete it.second;
		for(auto it : this->priorityForSchedQueue) delete it.second;
    }

    bool ModSDC::manageTimeBudgetSuccess() {
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
        std::chrono::milliseconds timeSpan = std::chrono::duration_cast<std::chrono::milliseconds>(tp-this->timeTracker);
        double elapsedTime = ((double)timeSpan.count())/1000.0;
        this->timeTracker = tp;
        this->timeBudget -= elapsedTime;
        return this->timeBudget>=0.0;
    }

    void ModSDC::clearMaps() {
        this->mrt.clear();
        this->asapTimes.clear();
        this->sdcTimes.clear();
        this->prevSched.clear();
        this->schedQueue.clear();
    }

    void ModSDC::scheduleInstruction(Vertex *I, int t) {
        ScaLP::Constraint c(this->scalpVariables.at(I) == t);
        this->createAdditionalConstraint(I,c);
        this->mrt[I] = t;
        this->prevSched[I] = t;
    }

    void ModSDC::handleTimeout() {
        this->timeBudget = this->solverTimeout;
        this->timeTracker = std::chrono::high_resolution_clock::now();
    }

    void ModSDC::printAdditionalSolverConstraints() {
        for(auto it : this->additionalConstraints) {
            cout << (*it.second) << endl;
        }
    }

	int ModSDC::getNoOfSubsequentVertices(Vertex *v) {
        int noOfSubseq = 0;
        std::map<Vertex*,bool> visited;
        for(auto it : this->g.Vertices()) {
            visited[it] = false;
        }

        std::list<Vertex*> queue = {v};

        while(!queue.empty()) {
            Vertex* pop = queue.front();
            queue.pop_front();

            for(auto it : this->g.Edges()) {
                if(&it->getVertexSrc() == pop) {
                    Vertex* dst = &it->getVertexDst();
                    if(!visited[dst]) {
                        visited[dst] = true;
                        queue.emplace_back(dst);
                        noOfSubseq++;
                    }
                }
            }
        }

        return noOfSubseq;
    }

    map<Vertex *, int> ModSDC::getASAPScheduleWithoutResourceConstraints() {
        auto resM = this->getUnlimitedResourceModel();
        auto asap = ASAPScheduler(this->g,*resM);
        asap.schedule();
        delete resM;
        return asap.getSchedule();
    }

    map<Vertex *, int> ModSDC::getALAPScheduleWithoutResourceConstraints() {
        auto resM = this->getUnlimitedResourceModel();
        auto alap = ALAPScheduler(this->g,*resM);
        alap.schedule();
        delete resM;
        return alap.getSchedule();
    }

    ResourceModel* ModSDC::getUnlimitedResourceModel() {
        auto resM = new ResourceModel();
        for(auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); it++) {
            // copy resource but make it unlimited
            auto originalRes = (*it);
            int limit = -1; // unlimited resource
            // special_loop always throws an error if it's not set to 1
            if(originalRes->getName() == "special_loop") limit = 1;
            Resource &res = resM->makeResource(originalRes->getName(),limit,originalRes->getLatency(),originalRes->getBlockingTime());
            auto vertices = this->resourceModel.getVerticesOfResource(originalRes);
            for(auto it2 : vertices) {
                resM->registerVertex(it2,&res);
            }
        }
        return resM;
    }

    list<Vertex *> ModSDC::getResourceConstrainedVertices() {
        list<Vertex*> returnMe;
        for(auto it : this->g.Vertices()) {
            if(this->resourceModel.getResource(it)->getLimit() >= 0)
                returnMe.emplace_back(it);
        }
        return returnMe;
    }

	void ModSDC::setPriority(Vertex *v, PriorityHandler p) {
        if(this->pType!=PriorityHandler::priorityType::CUSTOM)
            throw HatScheT::Exception("ModSDC::setPriority: priority type must be CUSTOM but is "+PriorityHandler::getPriorityTypeAsString(this->pType));
		try{
			auto a = this->priorityForSchedQueue.at(v);
			delete a;
		}
		catch(std::out_of_range&){
			// chill for a second and enjoy the view
		}
		this->priorityForSchedQueue[v] = new PriorityHandler(p);
	}

    std::map<const Vertex *, int> ModSDC::getBindings() {
        return Utility::Utility::getSimpleBinding(this->getSchedule(),&this->resourceModel,(int)this->II);
        //return Utility::Utility::getILPMinRegBinding(this->getSchedule(),&this->g,&this->resourceModel,(int)this->II,{"CPLEX"});
    }

    std::map<Edge *, int> ModSDC::getLifeTimes() {
        if(this->startTimes.size()==0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: cant return lifetimes! no startTimes determined!");

        std::map<Edge*,int> lifetimes;

        for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
            Edge* e = *it;
            Vertex* vSrc = &e->getVertexSrc();
            Vertex* vDst = &e->getVertexDst();
            int lifetime = this->startTimes[vDst] - this->startTimes[vSrc] - this->resourceModel.getVertexLatency(vSrc) + e->getDistance()*(int)this->II;

            if(lifetime < 0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: negative lifetime detected!");
            else lifetimes.insert(make_pair(e, lifetime));
        }
        return lifetimes;
    }

    PriorityHandler::PriorityHandler(PriorityHandler::priorityType p, int prio1, int prio2) :
        pType(p), firstPriority(prio1), secondPriority(prio2){
    }

    void PriorityHandler::putIntoSchedQueue(Vertex *v, const priorityType &p, const map<Vertex*,PriorityHandler*> *pHandlers, std::list<Vertex *> *schedQ) {
		PriorityHandler* pV;
    	try{
			pV = pHandlers->at(v);
    	}
        catch(std::out_of_range&){
    		stringstream err;
			err << "PriorityHandler::putIntoSchedQueue: can't find vertex '";
			if(v!=nullptr) err << v->getName();
			err << "' (" << v << ") in priority map";
    		throw HatScheT::Exception(err.str());
    	}
        for(auto it = (*schedQ).begin(); it != (*schedQ).end(); it++) {
			PriorityHandler* pI;
			try{
				pI = pHandlers->at(*it);
			}
			catch(std::out_of_range&){
				stringstream err;
				err << "PriorityHandler::putIntoSchedQueue: can't find vertex '";
				if((*it)!=nullptr) err << (*it)->getName();
				err << "' (" << (*it) << ") in priority map";
				throw HatScheT::Exception(err.str());
			}
            switch(p) {
                case priorityType::ASAP:{
                    if(pV->getFirstPriority() < pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    break;
                }
                case priorityType::ALAP:{
                    if(pV->getFirstPriority() < pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    break;
                }
                case priorityType::MOBLAP:{
                    if(pV->getFirstPriority() < pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    else if(pV->getFirstPriority() == pI->getFirstPriority() && pV->getSecondPriority() < pI->getSecondPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    break;
                }
                case priorityType::ALABILITY:{
                    if(pV->getFirstPriority() < pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    else if(pV->getFirstPriority() == pI->getFirstPriority() && pV->getSecondPriority() < pI->getSecondPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    break;
                }
                case priorityType::MOBILITY_HIGH:{
                    if(pV->getFirstPriority() > pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    break;
                }
                case priorityType::MOBILITY_LOW:{
                    if(pV->getFirstPriority() < pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    break;
                }
				case priorityType::SUBSEQUALAP:{
					if(pV->getFirstPriority() > pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
					}
					else if(pV->getFirstPriority() == pI->getFirstPriority() && pV->getSecondPriority() < pI->getSecondPriority()){
						schedQ->emplace(it,v);
						return;
					}
					break;
				}
                case priorityType::RANDOM:{
                    if(pV->getFirstPriority() > pI->getFirstPriority()){
						schedQ->emplace(it,v);
						return;
                    }
                    break;
                }
                case priorityType::CUSTOM:{
                    if(pV->getFirstPriority() > pI->getFirstPriority()){
                        schedQ->emplace(it,v);
                        return;
                    }
                    break;
                }
                default:{
                    throw HatScheT::Exception("PriorityHandler::putIntoSchedQueue: unknown priority type ("+PriorityHandler::getPriorityTypeAsString(p)+")");
                }
            }
        }
        schedQ->emplace_back(v);
    }

    std::string PriorityHandler::getPriorityTypeAsString(const priorityType &p) {
        switch(p){
            case priorityType::ASAP:{
                return "ASAP";
            }
            case priorityType::ALAP:{
                return "ALAP";
            }
            case priorityType::MOBLAP:{
                return "MOBLAP";
            }
            case priorityType::ALABILITY:{
                return "ALABILITY";
            }
            case priorityType::MOBILITY_HIGH:{
                return "MOBILITY_HIGH";
            }
            case priorityType::MOBILITY_LOW:{
                return "MOBILITY_LOW";
            }
            case priorityType::RANDOM:{
                return "RANDOM";
            }
            case priorityType::CUSTOM:{
                return "CUSTOM";
            }
            case priorityType::SUBSEQUALAP:{
                return "SUBSEQUALAP";
            }
			case priorityType::NONE:{
				return "NONE";
			}
        }
    }

    PriorityHandler::priorityType PriorityHandler::getPriorityTypeFromString(std::string priorityTypeStr) {
        if(priorityTypeStr=="ASAP") return priorityType::ASAP;
        if(priorityTypeStr=="ALAP") return priorityType::ALAP;
        if(priorityTypeStr=="MOBLAP") return priorityType::MOBLAP;
        if(priorityTypeStr=="ALABILITY") return priorityType::ALABILITY;
        if(priorityTypeStr=="MOBILITY_HIGH") return priorityType::MOBILITY_HIGH;
        if(priorityTypeStr=="MOBILITY_LOW") return priorityType::MOBILITY_LOW;
        if(priorityTypeStr=="RANDOM") return priorityType::RANDOM;
        if(priorityTypeStr=="CUSTOM") return priorityType::CUSTOM;
        if(priorityTypeStr=="SUBSEQUALAP") return priorityType::SUBSEQUALAP;
        return priorityType::NONE;
    }
}
