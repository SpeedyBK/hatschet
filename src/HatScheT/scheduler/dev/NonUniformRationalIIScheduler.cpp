//
// Created by nfiege on 13/11/19.
//

#include "NonUniformRationalIIScheduler.h"
#include <iomanip>
#include <math.h>
#include <HatScheT/scheduler/ilpbased/ASAPILPScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>


namespace HatScheT
{
	NonUniformRationalIIScheduler::NonUniformRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
		: SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
	{
		this->integerMinII = -1;
		this->tpBuffer = 0.0f;
		this->minRatIIFound = false;
		this->maxLatencyConstraint = -1;
		this->maxRuns = 1;
		this->s_found = -1;
		this->m_found = -1;
		this->latencySequence.clear();

		this->computeMinII(&this->g, &this->resourceModel);
	}

	void NonUniformRationalIIScheduler::resetContainer() {
		this->latencySequence.clear();
		this->tVariables.clear();
		this->bVariables.clear();
	}

	void NonUniformRationalIIScheduler::setObjective()
	{
		//supersink latency objective
		ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink");

		// it is enough to only consider the last insertion
		// Todo: ask patrick if this is true
		for(auto &v : this->g.Vertices())
			this->solver->addConstraint(supersink - this->tVariables[v].back() >= 0);

		this->solver->addConstraint(supersink>=0);
		if(this->maxLatencyConstraint>0)
			this->solver->addConstraint(supersink<=this->maxLatencyConstraint);

		this->solver->setObjective(ScaLP::minimize(supersink));
	}

	void NonUniformRationalIIScheduler::constructProblem()
	{
		if(this->maxLatencyConstraint == 0) {
			throw HatScheT::Exception("NonUniformRationalIIScheduler::constructProblem: irregular maxLatencyConstraint " + to_string(this->maxLatencyConstraint));
		}

		this->setGeneralConstraints();
		this->setModuloConstraints();
		this->setResourceConstraints();
	}

	void NonUniformRationalIIScheduler::printBindingToConsole() {
		Utility::printRationalIIMRT(this->startTimes, this->ratIIbindings, &this->resourceModel, this->modulo, this->latencySequence);
	}

	void NonUniformRationalIIScheduler::printScheduleToConsole()
	{
		cout << "----" << "Samples: " << this->samples << " mod: "
				 << this->modulo << " maxLat: " << this->maxLatencyConstraint << endl;

		std::setprecision(6);
		cout << "----" << "Throughput: " << ((double)this->samples)/((double)this->modulo) << endl;

		cout << "Printing absolute start times" << endl;
		for(auto i=0; i<this->samples; ++i) {
			auto it = this->startTimesVector[i];
			std::cout << "  Sample number " << i << std::endl;
			for(auto &it2 : it) {
				std::cout << "    " << it2.first->getName() << " - " << it2.second << std::endl;
			}
		}
		cout << "-------" << endl;

		cout << "Printing modulo " << this->modulo << " start times" << endl;
		for(auto i=0; i<this->samples; ++i) {
			auto it = this->startTimesVector[i];
			std::cout << "  Sample number " << i << std::endl;
			for(auto &it2 : it) {
				std::cout << "    " << it2.first->getName() << " - " << it2.second%this->modulo << std::endl;
			}
		}
		cout << "-------" << endl;
	}

	void NonUniformRationalIIScheduler::autoSetMAndS() {
		double minII = this->getMinII();
		//ceiling
		this->integerMinII = (int)ceil(minII);
		pair<int,int> frac =  Utility::splitRational(minII);

		if(!this->quiet) {
			cout << "------------------------" << endl;
			cout << "NonUniformRationalIIScheduler.autoSetMAndS: auto setting samples to " << frac.second << endl;
			cout << "NonUniformRationalIIScheduler.autoSetMAndS:auto setting modulo to " << frac.first << endl;
			cout << "------------------------" << endl;
		}

		this->samples = frac.second;
		this->modulo = frac.first;
	}

	std::map<Edge*,int> NonUniformRationalIIScheduler::getLifeTimes(){
		throw HatScheT::Exception("NonUniformRationalIIScheduler.getLifeTimes: Rational II Lifetimes are more complicated! Don't use this function! Use getRatIILifeTimes() instead!");
	}

	std::map<Edge*,vector<int> > NonUniformRationalIIScheduler::getRatIILifeTimes(){
		if(this->startTimesVector.empty()) throw HatScheT::Exception("NonUniformRationalIIScheduler.getRatIILifeTimes: cant return lifetimes! no startTimes determined!");
		if(this->latencySequence.empty()) throw HatScheT::Exception("NonUniformRationalIIScheduler.getRatIILifeTimes: No initIntervalls determined by the scheduler yet!");
		if(this->II <= 0) throw HatScheT::Exception("NonUniformRationalIIScheduler.getRatIILifeTimes: cant return lifetimes! no II determined!");

		std::map<Edge*,vector<int> > allLifetimes;

		for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
			Edge* e = *it;
			Vertex* vSrc = &e->getVertexSrc();
			Vertex* vDst = &e->getVertexDst();

			vector<int > lifetimes;

			for(int i = 0; i < this->samples; i++){
				auto so = Utility::getSampleIndexAndOffset(e->getDistance(),i,this->samples,this->modulo);
				auto sampleIndex = so.first;
				auto offset = so.second;
				int lifetime = this->startTimesVector[i][vDst] - this->startTimesVector[sampleIndex][vSrc]
											 - this->resourceModel.getVertexLatency(vSrc) + offset;

				if(lifetime < 0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: negative lifetime detected!");
				else lifetimes.push_back(lifetime);
			}
			allLifetimes.insert(make_pair(e, lifetimes));
		}
		return allLifetimes;
	}

	void NonUniformRationalIIScheduler::autoSetNextMAndS(){
		int currS = this->samples;
		int currM = this->modulo;

		//check whether it is useful to reduce the samples by 1 on this modulo
		if(currS > 2){
			double t = (double)(currS-1) / (double)currM ;
			if(t >= this->tpBuffer and t >= ((double)1.0 / this->integerMinII)){
				//in this case, it is still usefull to reduce s
				//reduce s and schedule again
				this->samples--;
				return;
			}
		}

		//when its not useful to reduce s anymore
		//increase m and set s on the maximum possible value for this problem
		this->modulo++;
		this->samples = this->modulo-1;
		double t = (double)this->samples / (double)(this->modulo);
		while(t > (double)1/this->getMinII()){
			this->samples--;
			t = (double)this->samples / (double)(this->modulo);
		}
	}

	vector<std::map<const Vertex *, int> > NonUniformRationalIIScheduler::getRationalIIBindings(){
		//generate new binding when no binding is available
		if(this->ratIIbindings.empty())
			this->ratIIbindings = Utility::getSimpleRatIIBinding(this->getSchedule(),&this->resourceModel,this->modulo, this->latencySequence);

		//throw exception when no binding was generated
		if(this->ratIIbindings.empty())
			throw Exception("NonUniformRationalIIScheduler.getBindings: Error no binding could be generated! No schedule available?");

		//return the stored binding
		return this->ratIIbindings;
	}

	std::map<const Vertex *, int> NonUniformRationalIIScheduler::getBindings() {
		throw Exception("NonUniformRationalIIScheduler.getBindings: Dont use this function for rational II schedules! Use getRationalIIBinding!");
	}

	void NonUniformRationalIIScheduler::schedule() {
		this->scheduleFound = false;

		//experimental auto set function for the start values of modulo and sample
		if(this->samples <= 0 or this->modulo <= 0) this->autoSetMAndS();
		this->s_start = this->samples;
		this->m_start = this->modulo;

		if(!this->quiet) {
			std::cout << "maxLatencyConstraint: " << maxLatencyConstraint << std::endl;
			std::cout << "modulo: " << modulo << std::endl;
		}

		if(this->samples <= 0) {
			throw HatScheT::Exception("RationalIIScheduler.schedule : moduloClasses <= 0! Scheduling not possible!");
		}

		if(this->modulo <= 0) {
			throw HatScheT::Exception("RationalIIScheduler.schedule : consideredModuloCycle <= 0! Scheduling not possible!");
		}

		if(!this->quiet) {
			cout << "------------------------" << endl;
			cout << "NonUniformRationalIIScheduler.schedule: start for " << this->g.getName() << endl;
			cout << "NonUniformRationalIIScheduler.schedule: solver timeout (s): " << this->getSolverTimeout() << endl;
			cout << "NonUniformRationalIIScheduler.schedule: ILP solver: " << this->solver->getBackendName() << endl;
			cout << "NonUniformRationalIIScheduler.schedule: max runs for rat ii scheduling " << this->getMaxRuns() << endl;
			cout << "NonUniformRationalIIScheduler.schedule: maxLatency " << this->maxLatencyConstraint << endl;
			cout << "NonUniformRationalIIScheduler::schedule: recMinII is " << this->getRecMinII() << endl;
			cout << "NonUniformRationalIIScheduler::schedule: resMinII is " << this->getResMinII() << endl;
			cout << "------------------------" << endl;
		}

		//count runs, set maxRuns
		int runs = 0;
		int maxRuns = this->maxRuns;
		if(maxRuns == -1) maxRuns = 1000000; // 'infinity'

		while(runs < maxRuns){
			if(!this->quiet) cout << "NonUniformRationalIIScheduler.schedule: building ilp problem for s / m : " << this->samples << " / " << this->modulo << endl;
			//clear up and reset
			this->solver->reset();
			this->resetContainer();

			//set up new variables and constraints
			this->fillTContainer();
			this->fillBContainer();
			this->constructProblem();

			//set up objective, currently asap using supersink
			this->setObjective();

			if(!this->quiet) cout << "NonUniformRationalIIScheduler.schedule: try to solve for s / m : " << this->samples << " / " << this->modulo << endl;
			//solve the current problem
			if(this->writeLPFile) this->solver->writeLP(to_string(this->samples) + to_string(this->modulo) + ".lp");

			//timestamp
			this->begin = clock();
			//solve
			stat = this->solver->solve();
			//timestamp
			this->end = clock();

			//log time
			if(this->solvingTime == -1.0) this->solvingTime = 0.0;
			this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;

			if(!this->quiet) cout << "Finished solving: " << stat << endl;

			//check result and act accordingly
			if(stat==ScaLP::status::FEASIBLE || stat==ScaLP::status::OPTIMAL || stat==ScaLP::status::TIMEOUT_FEASIBLE) {
				this->r = this->solver->getResult();
				this->tpBuffer = (double)(this->samples) / (double)(this->modulo);

				this->s_found = this->samples;
				this->m_found = this->modulo;
				this->II = (double)(this->modulo) / (double)(this->samples);

				this->scheduleFound = true;
				this->fillSolutionStructure();

				// verification
				bool ver = verifyRationalIIModuloSchedule(this->g, this->resourceModel, this->startTimesVector, this->samples,
																									this->modulo);

				//determine whether rational minimum II was identified
				if(((double)this->modulo / (double)this->samples) == this->getMinII()) this->minRatIIFound = true;
				//this->getRationalIIBindings();

				if(!this->quiet) {
					cout << "------------------------" << endl;
					if (ver) cout << "NonUniformRationalIIScheduler.schedule: Result is verified! " << endl;
					else cout << "NonUniformRationalIIScheduler.schedule: Result verification FAILED! " << endl;
					cout << "NonUniformRationalIIScheduler.schedule: Found result is " << stat << endl;
					cout << "NonUniformRationalIIScheduler.schedule: this solution is s / m : " << this->samples << " / " << this->modulo
							 << endl;
					cout << "NonUniformRationalIIScheduler.schedule: II: " << (double) (this->modulo) / (double) (this->samples)
							 << " (integer minII " << ceil((double) (this->modulo) / (double) (this->samples)) << ")" << endl;
					cout << "NonUniformRationalIIScheduler.schedule: throughput: " << this->tpBuffer << endl;
					cout << "------------------------" << endl;
					this->printScheduleToConsole();
				}
			}
			else {
				if(!this->quiet) cout << "NonUniformRationalIIScheduler.schedule: no schedule found for s / m : " << this->samples << " / " << this->modulo << " ( " << stat << " )" << endl;
				this->scheduleFound = false;
			}

			//break while loop when a schedule was found
			if(this->scheduleFound) break;
			else {
				this->timeouts++;
				this->tpBuffer = (double)this->modulo / (double)this->samples;
				this->autoSetNextMAndS();
				runs++;
			}
		}
	}

	void NonUniformRationalIIScheduler::fillTContainer() {
		// create one time variable for each vertex in the graph
		for(auto &v : this->g.Vertices()) {
			this->tVariables[v] = std::vector<ScaLP::Variable>();
			for(auto i=0; i<this->samples; ++i) {
				auto var = ScaLP::newIntegerVariable(v->getName()+"_"+to_string(i));
				this->tVariables[v].emplace_back(var);
				this->solver->addConstraint(var>=0);
			}
		}
	}

	void NonUniformRationalIIScheduler::fillBContainer() {
		for(auto &v : this->g.Vertices()) {
			for(auto s=0; s<this->samples; ++s) {
				this->bVariables[v].emplace_back(std::vector<ScaLP::Variable>());
				for(auto m=0; m<this->modulo; ++m) {
					auto var = ScaLP::newBinaryVariable(v->getName()+"_"+to_string(s)+"_"+to_string(m));
					this->bVariables[v][s].emplace_back(var);
				}
			}
		}
	}

	void NonUniformRationalIIScheduler::setGeneralConstraints() {
		for(auto &e : this->g.Edges()) {
			auto *src = &e->getVertexSrc();
			auto *dst = &e->getVertexDst();
			for(auto i=0; i<this->samples; ++i) {
				auto sampleIndexOffset = Utility::getSampleIndexAndOffset(e->getDistance(),i,this->samples,this->modulo);
				auto index = sampleIndexOffset.first;
				auto offset = sampleIndexOffset.second;
				this->solver->addConstraint(this->tVariables[src][index]-this->tVariables[dst][i]<=offset-this->resourceModel.getVertexLatency(src)-e->getDelay());
			}
		}
	}

	void NonUniformRationalIIScheduler::setModuloConstraints() {
		for(auto &v : this->g.Vertices()) {
			for(auto s=0; s<this->samples; ++s) {
				// create remainder variable k_v_s
				ScaLP::Variable k_v_s = ScaLP::newIntegerVariable("k_"+v->getName()+"_"+to_string(s));
				this->solver->addConstraint(k_v_s >= 0);

				// create constraint
				ScaLP::Term bSum;
				for(auto m=0; m<this->modulo; ++m) {
					bSum += (m * this->bVariables[v][s][m]);
				}
				ScaLP::Term leftHandSide = this->tVariables[v][s];
				ScaLP::Term rightHandSide = bSum + (this->modulo * k_v_s);
				this->solver->addConstraint(leftHandSide-rightHandSide == 0);
			}
		}
	}

	void NonUniformRationalIIScheduler::setResourceConstraints() {
		// each vertex is assigned exactly one modulo slot
		for(auto &v : this->g.Vertices()) {
			for(auto s=0; s<this->samples; ++s) {
				ScaLP::Term sum;
				for(auto m=0; m<this->modulo; ++m) {
					sum += this->bVariables[v][s][m];
				}
				this->solver->addConstraint(sum==1);
			}
		}

		// resource limits are met
		for(auto &res : this->resourceModel.Resources()) {
			auto vertices = this->resourceModel.getVerticesOfResource(res);
			auto limit = res->getLimit();
			for(auto m=0; m<this->modulo; ++m) {
				ScaLP::Term bSum;
				for(auto &v : vertices) {
					for(auto s=0; s<this->samples; ++s) {
						bSum += this->bVariables[v][s][m];
					}
				}
				this->solver->addConstraint(bSum <= limit);
			}
		}
	}

	void NonUniformRationalIIScheduler::fillSolutionStructure() {
		// store result
		this->r = this->solver->getResult();

		// set start times to the start times of the first sample to not leave it empty
		for(auto &v : this->g.Vertices()) {
			this->startTimes[v] = (int)this->r.values[this->tVariables[v][0]];
		}

		// set start times vector
		for(int i=0; i<this->samples; ++i) {
			std::map<Vertex*,int> startTimesTemp;
			for(auto &v : this->g.Vertices()) {
				startTimesTemp[v] = (int)this->r.values[this->tVariables[v][i]];
			}
			this->startTimesVector.emplace_back(startTimesTemp);
		}

		// check if minII was found
		this->minRatIIFound = (this->II == this->minII);
	}


}
