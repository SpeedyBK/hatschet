//
// Created by nfiege on 12/11/19.
//

#include "UniformRationalIIScheduler.h"
#include <iomanip>
#include <math.h>
#include <HatScheT/scheduler/ilpbased/ASAPILPScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>


namespace HatScheT
{
	UniformRationalIIScheduler::UniformRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
		: RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist)
	{
		this->integerMinII = -1;
		this->tpBuffer = 0.0f;
		this->minRatIIFound = false;
		this->maxLatencyConstraint = -1;
		this->maxRuns = 1;
		this->s_found = -1;
		this->m_found = -1;
		this->latencySequence.clear();
		this->initiationIntervals.clear();

		this->computeMinII(&this->g, &this->resourceModel);
	}

	void UniformRationalIIScheduler::resetContainer() {
		this->latencySequence.clear();
		this->initiationIntervals.clear();
		this->tVariables.clear();
		this->bVariables.clear();
	}

	void UniformRationalIIScheduler::setObjective()
	{
		//supersink latency objective
		ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink");

		for(auto &v : this->g.Vertices())
			this->solver->addConstraint(supersink - this->tVariables[v] >= 0);

		this->solver->addConstraint(supersink>=0);
		if(this->maxLatencyConstraint>0)
			this->solver->addConstraint(supersink<=this->maxLatencyConstraint);

		this->solver->setObjective(ScaLP::minimize(supersink));
	}

	void UniformRationalIIScheduler::constructProblem()
	{
		if(this->maxLatencyConstraint == 0) {
			throw HatScheT::Exception("UniformRationalIIScheduler::constructProblem: irregular maxLatencyConstraint " + to_string(this->maxLatencyConstraint));
		}

		this->setGeneralConstraints();
		this->setModuloConstraints();
		this->setResourceConstraints();
	}

	void UniformRationalIIScheduler::printBindingToConsole() {
		Utility::printRationalIIMRT(this->startTimes, this->ratIIbindings, &this->resourceModel, this->modulo, this->latencySequence);
	}

	void UniformRationalIIScheduler::printScheduleToConsole()
	{
		cout << "----" << "Samples: " << this->samples << " mod: "
				 << this->modulo << " maxLat: " << this->maxLatencyConstraint << endl;

		cout << "----" << "Found IIs: ";
		for(auto &II : this->initiationIntervals) {
			cout << II << "  ";
		}
		cout << endl;

		cout << "----" << "Resulting Insertion latency: ";
		for(auto &l : this->latencySequence) {
			cout << l << "  ";
		}
		cout << endl;

		std::setprecision(6);
		cout << "----" << "Throughput: " << ((double)this->samples)/((double)this->modulo) << endl;

		cout << "Printing absolute start times" << endl;
		for(auto i=0; i<this->initiationIntervals.size(); ++i) {
			auto it = this->startTimesVector[i];
			auto insertionTime = this->initiationIntervals[i];
			std::cout << "  Insertion time " << insertionTime << std::endl;
			for(auto &it2 : it) {
				std::cout << "    " << it2.first->getName() << " - " << it2.second << std::endl;
			}
		}
		cout << "-------" << endl;

		cout << "Printing modulo " << this->modulo << " start times" << endl;
		for(auto i=0; i<this->initiationIntervals.size(); ++i) {
			auto it = this->startTimesVector[i];
			auto insertionTime = this->initiationIntervals[i];
			std::cout << "  Insertion time " << insertionTime << std::endl;
			for(auto &it2 : it) {
				std::cout << "    " << it2.first->getName() << " - " << it2.second%this->modulo << std::endl;
			}
		}
		cout << "-------" << endl;
	}

	void UniformRationalIIScheduler::autoSetMAndS() {
		double minII = this->getMinII();
		//ceiling
		this->integerMinII = (int)ceil(minII);
		pair<int,int> frac =  Utility::splitRational(minII);

		if(!this->quiet) {
			cout << "------------------------" << endl;
			cout << "UniformRationalIIScheduler.autoSetMAndS: auto setting samples to " << frac.second << endl;
			cout << "UniformRationalIIScheduler.autoSetMAndS:auto setting modulo to " << frac.first << endl;
			cout << "------------------------" << endl;
		}

		this->samples = frac.second;
		this->modulo = frac.first;
	}

	std::map<Edge*,int> UniformRationalIIScheduler::getLifeTimes(){
		throw HatScheT::Exception("UniformRationalIIScheduler.getLifeTimes: Rational II Lifetimes are more complicated! Don't use this function! Use getRatIILifeTimes() instead!");
	}

	std::map<Edge*,vector<int> > UniformRationalIIScheduler::getRatIILifeTimes(){
		if(this->startTimesVector.empty()) throw HatScheT::Exception("UniformRationalIIScheduler.getRatIILifeTimes: cant return lifetimes! no startTimes determined!");
		if(this->latencySequence.empty()) throw HatScheT::Exception("UniformRationalIIScheduler.getRatIILifeTimes: No initIntervalls determined by the scheduler yet!");
		if(this->II <= 0) throw HatScheT::Exception("UniformRationalIIScheduler.getRatIILifeTimes: cant return lifetimes! no II determined!");

		std::map<Edge*,vector<int> > allLifetimes;

		for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
			Edge* e = *it;
			Vertex* vSrc = &e->getVertexSrc();
			Vertex* vDst = &e->getVertexDst();

			vector<int > lifetimes;

			for(int i = 0; i < (int)(this->latencySequence.size()); i++){
				int lifetime = this->startTimes[vDst] - this->startTimes[vSrc]
											 - this->resourceModel.getVertexLatency(vSrc) + this->getSampleDistanceAsInt(e->getDistance(), i);

				if(lifetime < 0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: negative lifetime detected!");
				else lifetimes.push_back(lifetime);
			}
			allLifetimes.insert(make_pair(e, lifetimes));
		}
		return allLifetimes;
	}

	void UniformRationalIIScheduler::autoSetNextMAndS(){
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

	vector<std::map<const Vertex *, int> > UniformRationalIIScheduler::getRationalIIBindings(){
		//generate new binding when no binding is available
		if(this->ratIIbindings.empty())
			this->ratIIbindings = Utility::getSimpleRatIIBinding(this->getSchedule(),&this->resourceModel,this->modulo, this->latencySequence);

		//throw exception when no binding was generated
		if(this->ratIIbindings.empty())
			throw Exception("SchedulerBase.getBindings: Error no binding could be generated! No schedule available?");

		//return the stored binding
		return this->ratIIbindings;
	}

	std::map<const Vertex *, int> UniformRationalIIScheduler::getBindings() {
		throw Exception("UniformRationalIIScheduler.getBindings: Dont use this function for rational II schedules! Use getRationalIIBinding!");
	}

	void UniformRationalIIScheduler::schedule() {
		this->scheduleFound = false;

		//experimental auto set function for the start values of modulo and sample
		if(this->samples <= 0 or this->modulo <= 0) this->autoSetMAndS();
		this->s_start = this->samples;
		this->m_start = this->modulo;

		//experimental
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
			cout << "UniformRationalIIScheduler.schedule: start for " << this->g.getName() << endl;
			cout << "UniformRationalIIScheduler.schedule: solver timeout (s): " << this->getSolverTimeout() << endl;
			cout << "UniformRationalIIScheduler.schedule: ILP solver: " << this->solver->getBackendName() << endl;
			cout << "UniformRationalIIScheduler.schedule: max runs for rat ii scheduling " << this->getMaxRuns() << endl;
			cout << "UniformRationalIIScheduler.schedule: maxLatency " << this->maxLatencyConstraint << endl;
			cout << "UniformRationalIIScheduler::schedule: recMinII is " << this->getRecMinII() << endl;
			cout << "UniformRationalIIScheduler::schedule: resMinII is " << this->getResMinII() << endl;
			cout << "------------------------" << endl;
		}

		//count runs, set maxRuns
		int runs = 0;
		int maxRuns = this->maxRuns;
		if(maxRuns == -1) maxRuns = 1000000; // 'infinity'

		while(runs < maxRuns){
			if(!this->quiet) cout << "UniformRationalIIScheduler.schedule: building ilp problem for s / m : " << this->samples << " / " << this->modulo << endl;
			//clear up and reset
			this->solver->reset();
			this->resetContainer();

			//init latency sequence, init intervals, deltaMin containers
			this->initiationIntervals = ModuloQScheduler::getOptimalInitiationIntervalSequence(this->samples,this->modulo,this->quiet);
			this->latencySequence = ModuloQScheduler::getLatencySequenceFromInitiationIntervals(this->initiationIntervals,this->modulo);
			this->calcDeltaMins();

			//set up new variables and constraints
			this->fillTContainer();
			this->fillBContainer();
			this->constructProblem();

			//set up objective, currently asap using supersink
			this->setObjective();

			if(!this->quiet) cout << "UniformRationalIIScheduler.schedule: try to solve for s / m : " << this->samples << " / " << this->modulo << endl;
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

				bool ver = HatScheT::verifyRationalIIModuloSchedule2(this->g, this->resourceModel, this->startTimesVector,
																														 this->latencySequence, this->getScheduleLength());
				bool ver2 = verifyRationalIIModuloSchedule(this->g, this->resourceModel, this->startTimesVector, this->samples,
																									 this->modulo);
				if(ver!=ver2) {
					std::cout << "ATTENTION!!!! Rational II verifiers do not lead to the same result! One of them is buggy!!!" << std::endl;
				}

				//determine whether rational minimum II was identified
				if(((double)this->modulo / (double)this->samples) == this->getMinII()) this->minRatIIFound = true;
				this->getRationalIIBindings();

				if(!this->quiet) {
					cout << "------------------------" << endl;
					if (ver) cout << "UniformRationalIIScheduler.schedule: Result is verified! " << endl;
					else cout << "UniformRationalIIScheduler.schedule: Result verification FAILED! " << endl;
					cout << "UniformRationalIIScheduler.schedule: Found result is " << stat << endl;
					cout << "UniformRationalIIScheduler.schedule: this solution is s / m : " << this->samples << " / " << this->modulo
							 << endl;
					cout << "UniformRationalIIScheduler.schedule: II: " << (double) (this->modulo) / (double) (this->samples)
							 << " (integer minII " << ceil((double) (this->modulo) / (double) (this->samples)) << ")" << endl;
					cout << "UniformRationalIIScheduler.schedule: throughput: " << this->tpBuffer << endl;
					cout << "------------------------" << endl;
					this->printScheduleToConsole();
				}
			}
			else {
				if(!this->quiet) cout << "UniformRationalIIScheduler.schedule: no schedule found for s / m : " << this->samples << " / " << this->modulo << " ( " << stat << " )" << endl;
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

	void UniformRationalIIScheduler::calcDeltaMins() {
		if(this->initiationIntervals.empty() or this->latencySequence.empty())
			throw HatScheT::Exception("UniformRationalIIScheduler::calcDeltaMins: need to specify initiation intervals/latency sequence");
		if(this->samples<=0 or this->modulo<=0)
			throw HatScheT::Exception("UniformRationalIIScheduler::calcDeltaMins: need to specify samples and modulo");
		// distance 0 is trivial
		this->deltaMins[0] = 0;
		if(!this->quiet)
			std::cout << "set min delta (0) = " << 0 << std::endl;
		for(auto &e : this->g.Edges()) {
			auto edgeDistance = e->getDistance();
			// check if delta for this distance was already calculated
			if(edgeDistance==0) continue;
			if(this->deltaMins.find(edgeDistance) != this->deltaMins.end()) continue;
			// calc minimum delta
			unsigned int minDelta = 10000000; // 'infinity'
			for(auto offset=0; offset<this->samples; ++offset) {
				unsigned int delta = 0;
				for(auto d=0; d<edgeDistance; ++d) {
					delta += this->latencySequence[(offset+d)%this->samples];
				}
				if(delta<minDelta) minDelta = delta;
			}
			this->deltaMins[edgeDistance] = minDelta;
			if(!this->quiet)
				std::cout << "set min delta (" << edgeDistance << ") = " << this->deltaMins[edgeDistance] << std::endl;
		}
	}

	void UniformRationalIIScheduler::fillTContainer() {
		// create one time variable for each vertex in the graph
		for(auto &v : this->g.Vertices()) {
			auto var = ScaLP::newIntegerVariable(v->getName());
			this->tVariables[v] = var;
			this->solver->addConstraint(var>=0);
		}
	}

	void UniformRationalIIScheduler::fillBContainer() {
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

	void UniformRationalIIScheduler::setGeneralConstraints() {
		for(auto &e : this->g.Edges()) {
			auto *src = &e->getVertexSrc();
			auto *dst = &e->getVertexDst();
			this->solver->addConstraint(this->tVariables[src]-this->tVariables[dst]<=int(this->deltaMins[e->getDistance()])-this->resourceModel.getVertexLatency(src)-e->getDelay());
		}
	}

	void UniformRationalIIScheduler::setModuloConstraints() {
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
				ScaLP::Term leftHandSide = this->tVariables[v] + this->initiationIntervals[s];
				ScaLP::Term rightHandSide = bSum + (this->modulo * k_v_s);
				this->solver->addConstraint(leftHandSide-rightHandSide == 0);
			}
		}
	}

	void UniformRationalIIScheduler::setResourceConstraints() {
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

	void UniformRationalIIScheduler::fillSolutionStructure() {
		// store result
		this->r = this->solver->getResult();

		// set start times
		for(auto &v : this->g.Vertices()) {
			this->startTimes[v] = (int)this->r.values[this->tVariables[v]];
		}

		// set start times vector
		for(auto &I : this->initiationIntervals) {
			std::map<Vertex*,int> startTimesTemp;
			for(auto &it : this->startTimes) {
				startTimesTemp[it.first] = it.second + I;
			}
			this->startTimesVector.emplace_back(startTimesTemp);
		}

		// check if minII was found
		this->minRatIIFound = (this->II == this->minII);
	}

	int UniformRationalIIScheduler::getSampleDistanceAsInt(int d, int startIndex) {
		if((startIndex > this->latencySequence.size()-1) or (startIndex < 0))
			throw Exception("RationalIIScheduler.getSampleDistanceAsTerm: out of range II_vector entry requested: " + to_string(startIndex));

		int distance = 0;
		while(d>0){
			// calc index
			if(startIndex>0)
				startIndex-=1;
			else
				startIndex=(int)this->latencySequence.size()-1;

			// accumulate distance
			distance += this->latencySequence[startIndex];
			--d;
		}

		return distance;
	}
}
