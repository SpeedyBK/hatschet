//
// Created by nfiege on 08/04/20.
//

#include "UniformRationalIISchedulerNew.h"
#include <iomanip>
#include <math.h>
#include <HatScheT/scheduler/ilpbased/ASAPILPScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>


namespace HatScheT
{
	UniformRationalIISchedulerNew::UniformRationalIISchedulerNew(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
		: RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist), initiationIntervals()
	{
	}

	void UniformRationalIISchedulerNew::resetContainer() {
		this->latencySequence.clear();
		this->initiationIntervals.clear();
		this->tVariables.clear();
		this->bVariables.clear();
		this->kVariables.clear();
	}

	void UniformRationalIISchedulerNew::setObjective()
	{
		//supersink latency objective
		ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink");

		for(auto &v : this->g.Vertices())
			this->solver->addConstraint(supersink - this->tVariables[v] - this->resourceModel.getVertexLatency(v) >= 0);

		if(this->maxLatencyConstraint>0)
			this->solver->addConstraint(supersink<=this->maxLatencyConstraint);

		this->solver->setObjective(ScaLP::minimize(supersink));
	}

	void UniformRationalIISchedulerNew::constructProblem()
	{
		this->solver->quiet = this->solverQuiet;

		if(this->maxLatencyConstraint == 0) {
			throw HatScheT::Exception("UniformRationalIISchedulerNew::constructProblem: irregular maxLatencyConstraint " + to_string(this->maxLatencyConstraint));
		}

		this->setGeneralConstraints();
		this->setModuloConstraints();
		this->setResourceConstraints();
	}

	void UniformRationalIISchedulerNew::printBindingToConsole() {
		Utility::printRationalIIMRT(this->startTimes, this->ratIIbindings, &this->resourceModel, this->modulo, this->latencySequence);
	}

	void UniformRationalIISchedulerNew::printScheduleToConsole()
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

	void UniformRationalIISchedulerNew::calcDeltaMins() {
		if(this->initiationIntervals.empty() or this->latencySequence.empty())
			throw HatScheT::Exception("UniformRationalIISchedulerNew::calcDeltaMins: need to specify initiation intervals/latency sequence");
		if(this->samples<=0 or this->modulo<=0)
			throw HatScheT::Exception("UniformRationalIISchedulerNew::calcDeltaMins: need to specify samples and modulo");
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

	void UniformRationalIISchedulerNew::fillTContainer() {
		// create one time variable for each vertex in the graph
		for(auto &v : this->g.Vertices()) {
			if (!this->quiet) {
				std::cout << "UniformRationalIISchedulerNew::fillTContainer: creating variable '" << v->getName() << "'" << std::endl;
			}
			// check if initial solution was given and act accordingly
			if(this->initialSolutionRatII.empty()) {
				auto var = ScaLP::newIntegerVariable(v->getName());
				this->tVariables[v] = var;
				this->solver->addConstraint(var>=0);
			}
			else {
				auto initVal = this->initialSolutionRatII[0][v];
				auto var = ScaLP::newIntegerVariable(v->getName(),0,ScaLP::INF(),initVal);
				this->tVariables[v] = var;
			}
		}
	}

	void UniformRationalIISchedulerNew::fillBContainer() {
		for(auto &v : this->g.Vertices()) {
			for(auto m=0; m<this->modulo; ++m) {
				if (!this->quiet) {
					std::cout << "UniformRationalIISchedulerNew::fillTContainer: creating variable '" << "b_"+v->getName()+"_"+to_string(m) << "'" << std::endl;
				}
				// check if initial solution was given and act accordingly
				if(this->initialSolutionRatII.empty()) {
					auto var = ScaLP::newBinaryVariable("b_"+v->getName()+"_"+to_string(m));
					this->bVariables[v].emplace_back(var);
				}
				else {
					bool initVal = (this->initialSolutionRatII[0][v] % this->modulo == 0);
					auto var = ScaLP::newBinaryVariable("b_"+v->getName()+"_"+to_string(m),0,1,initVal);
					this->bVariables[v].emplace_back(var);
				}
			}
		}
	}

	void UniformRationalIISchedulerNew::setGeneralConstraints() {
		for(auto &e : this->g.Edges()) {
			auto *src = &e->getVertexSrc();
			auto *dst = &e->getVertexDst();
			this->solver->addConstraint(this->tVariables[src]-this->tVariables[dst]<=int(this->deltaMins[e->getDistance()])-this->resourceModel.getVertexLatency(src)-e->getDelay());
		}
	}

	void UniformRationalIISchedulerNew::setModuloConstraints() {
		for(auto &v : this->g.Vertices()) {
			ScaLP::Variable k_v = nullptr;
			if (!this->quiet) {
				std::cout << "UniformRationalIISchedulerNew::fillTContainer: creating variable '" << "k_"+v->getName() << "'" << std::endl;
			}
			// create remainder variable k_v and check if initial solution was given and act accordingly
			if(this->initialSolutionRatII.empty()) {
				k_v = ScaLP::newIntegerVariable("k_"+v->getName());
				this->solver->addConstraint(k_v >= 0);
			}
			else {
				auto initVal = floor((double)this->initialSolutionRatII[0][v]/(double)this->modulo);
				k_v = ScaLP::newIntegerVariable("k_"+v->getName(),0,ScaLP::INF(),initVal);
			}
			this->kVariables[v] = k_v;

			// create constraint
			ScaLP::Term bSum;
			for(auto m=0; m<this->modulo; ++m) {
				bSum += (m * this->bVariables[v][m]);
			}
			ScaLP::Term leftHandSide = this->tVariables[v];
			ScaLP::Term rightHandSide = bSum + (this->modulo * k_v);
			this->solver->addConstraint(leftHandSide-rightHandSide == 0);
		}
	}

	void UniformRationalIISchedulerNew::setResourceConstraints() {
		// each vertex is assigned exactly one modulo slot
		for(auto &v : this->g.Vertices()) {
			ScaLP::Term sum;
			for(auto m=0; m<this->modulo; ++m) {
				sum += this->bVariables[v][m];
			}
			this->solver->addConstraint(sum==1);
		}

		// resource limits are met
		for(auto &res : this->resourceModel.Resources()) {
			auto vertices = this->resourceModel.getVerticesOfResource(res);
			auto limit = res->getLimit();
			if(limit==UNLIMITED) continue;
			for(auto m=0; m<this->modulo; ++m) {
				ScaLP::Term bSum;
				for(auto &v : vertices) {
					for(auto s=0; s<this->samples; ++s) {
						auto mHat = this->getNewModuloslot(s,m);
						bSum += this->bVariables[v][mHat];
					}
				}
				this->solver->addConstraint(bSum <= limit);
			}
		}
	}

	void UniformRationalIISchedulerNew::fillSolutionStructure() {
		// store result
		this->r = this->solver->getResult();

		// set start times
		for(auto &v : this->g.Vertices()) {
			this->startTimes[v] = (int)round(this->r.values[this->tVariables[v]]);
		}

		// set start times vector
		for(auto &I : this->initiationIntervals) {
			std::map<Vertex*,int> startTimesTemp;
			for(auto &it : this->startTimes) {
				startTimesTemp[it.first] = it.second + I;
			}
			this->startTimesVector.emplace_back(startTimesTemp);
		}
	}

	int UniformRationalIISchedulerNew::getSampleDistanceAsInt(int d, int startIndex) {
		if((startIndex > this->latencySequence.size()-1) or (startIndex < 0))
			throw Exception("UniformRationalIISchedulerNew.getSampleDistanceAsTerm: out of range II_vector entry requested: " + to_string(startIndex));

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

	void UniformRationalIISchedulerNew::scheduleIteration() {
		//clear up and reset
		this->solver->reset();
		this->resetContainer();

		//init latency sequence, init intervals, deltaMin containers
		this->initiationIntervals = RationalIISchedulerLayer::getOptimalInitiationIntervalSequence(this->samples,this->modulo,this->quiet);
		this->latencySequence = RationalIISchedulerLayer::getLatencySequenceFromInitiationIntervals(this->initiationIntervals,this->modulo);
		this->calcDeltaMins();

		//set up new variables and constraints
		this->fillTContainer();
		this->fillBContainer();
		this->constructProblem();

		//set up objective, currently asap using supersink
		this->setObjective();

		if(!this->quiet) cout << "UniformRationalIISchedulerNew.schedule: try to solve for s / m : " << this->samples << " / " << this->modulo << endl;
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

			this->scheduleFound = true;
			this->fillSolutionStructure();

			//this->getRationalIIBindings();

			if(!this->quiet) {
				cout << "------------------------" << endl;
				cout << "UniformRationalIISchedulerNew.schedule: Found result is " << stat << endl;
				cout << "UniformRationalIISchedulerNew.schedule: this solution is s / m : " << this->samples << " / " << this->modulo
						 << endl;
				cout << "UniformRationalIISchedulerNew.schedule: II: " << (double) (this->modulo) / (double) (this->samples)
						 << " (integer minII " << ceil((double) (this->modulo) / (double) (this->samples)) << ")" << endl;
				cout << "UniformRationalIISchedulerNew.schedule: throughput: " << (double) (this->samples) / (double) (this->modulo) << endl;
				cout << "------------------------" << endl;
				this->printScheduleToConsole();
			}
		}
		else {
			if(!this->quiet) {
				cout << "UniformRationalIISchedulerNew.schedule: no schedule found for s / m : " << this->samples << " / " << this->modulo << " ( " << this->stat << " )" << endl;
			}
			this->scheduleFound = false;
		}
	}

	int UniformRationalIISchedulerNew::getNewModuloslot(int s, int oldModuloslot) {
		if(s > this->samples)
			throw HatScheT::Exception("UniformRationalIISchedulerNew::getNewModuloslot: s>S ("+to_string(s)+">"+to_string(this->samples)+")");
		return ((oldModuloslot + this->initiationIntervals[s]) % this->modulo);
	}
}
