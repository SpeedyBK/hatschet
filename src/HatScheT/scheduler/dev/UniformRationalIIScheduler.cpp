//
// Created by nfiege on 12/11/19.
//

#include "UniformRationalIIScheduler.h"
#include <iomanip>
#include <math.h>
#include <HatScheT/scheduler/ilpbased/ASAPILPScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>


namespace HatScheT
{
	UniformRationalIIScheduler::UniformRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
		: RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist), initiationIntervals()
	{
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
			this->solver->addConstraint(supersink - this->tVariables[v] - this->resourceModel.getVertexLatency(v) >= 0);

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

	void UniformRationalIIScheduler::scheduleIteration() {
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

			this->scheduleFound = true;
			this->fillSolutionStructure();

			//this->getRationalIIBindings();

			if(!this->quiet) {
				cout << "------------------------" << endl;
				cout << "UniformRationalIIScheduler.schedule: Found result is " << stat << endl;
				cout << "UniformRationalIIScheduler.schedule: this solution is s / m : " << this->samples << " / " << this->modulo
						 << endl;
				cout << "UniformRationalIIScheduler.schedule: II: " << (double) (this->modulo) / (double) (this->samples)
						 << " (integer minII " << ceil((double) (this->modulo) / (double) (this->samples)) << ")" << endl;
				cout << "UniformRationalIIScheduler.schedule: throughput: " << (double) (this->samples) / (double) (this->modulo) << endl;
				cout << "------------------------" << endl;
				this->printScheduleToConsole();
			}
		}
		else {
			if(!this->quiet) cout << "UniformRationalIIScheduler.schedule: no schedule found for s / m : " << this->samples << " / " << this->modulo << " ( " << this->stat << " )" << endl;
			this->scheduleFound = false;
		}
	}
}
