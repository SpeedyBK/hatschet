//
// Created by nfiege on 13/11/19.
//

#include "NonUniformRationalIIScheduler.h"
#include <iomanip>
#include <cmath>
#include <HatScheT/scheduler/ilpbased/ASAPILPScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>


namespace HatScheT
{
	NonUniformRationalIIScheduler::NonUniformRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
		: RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist)
	{
	}

	void NonUniformRationalIIScheduler::resetContainer() {
		this->tVariables.clear();
		this->bVariables.clear();
	}

	void NonUniformRationalIIScheduler::setObjective() {
		//supersink latency objective
		ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink");

		for(auto &v : this->g.Vertices()) {
			for (int s = 0; s < this->samples; ++s) {
				this->solver->addConstraint(supersink - this->tVariables[v][s] - this->resourceModel.getVertexLatency(v) >= 0);
			}
		}

		if(this->maxLatencyConstraint>0)
			this->solver->addConstraint(supersink<=this->maxLatencyConstraint);

		this->solver->setObjective(ScaLP::minimize(supersink));
	}

	void NonUniformRationalIIScheduler::constructProblem() {
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

	void NonUniformRationalIIScheduler::printScheduleToConsole() {
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
					auto var = ScaLP::newBinaryVariable("b_"+v->getName()+"_"+to_string(s)+"_"+to_string(m));
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
			if(limit==UNLIMITED) continue;
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
			this->startTimes[v] = (int)std::round(this->r.values[this->tVariables[v][0]]);
		}

		// set start times vector
		for(int i=0; i<this->samples; ++i) {
			std::map<Vertex*,int> startTimesTemp;
			for(auto &v : this->g.Vertices()) {
				startTimesTemp[v] = (int)std::round(this->r.values[this->tVariables[v][i]]);
			}
			this->startTimesVector.emplace_back(startTimesTemp);
		}

		// check if minII was found
		this->minRatIIFound = (this->II == this->minII);
	}

	void NonUniformRationalIIScheduler::scheduleIteration() {
		//clear up and reset
		this->solver->reset();
		this->resetContainer();

		//set up new variables and constraints
		this->fillTContainer();
		this->fillBContainer();
		this->constructProblem();

		//set up objective, currently asap using supersink
		this->setObjective();

		if(!this->quiet) cout << "NonUniformRationalIIScheduler.scheduleIteration: try to solve for s / m : " << this->samples << " / " << this->modulo << endl;
		//solve the current problem
		if(this->writeLPFile) this->solver->writeLP("NonUniformRationalIIScheduler_" + to_string(this->samples) + "_" + to_string(this->modulo) + ".lp");

//		//timestamp
//		this->begin = clock();
//		//solve
//		stat = this->solver->solve();
//		//timestamp
//		this->end = clock();
//
//		//log time
//		if(this->solvingTime == -1.0) this->solvingTime = 0.0;
//		this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;

		//timestamp
		startTimeTracking();
		//solve
		stat = this->solver->solve();
		//timestamp
		endTimeTracking();

		if(!this->quiet) {
			cout << "Finished solving: " << stat << endl;
			cout << "ScaLP results: " << this->solver->getResult() << endl;
			cout << "Time Used: " << getTimeUsed() << endl;
		}

		// track optimality of first objective (i.e., II)
		if (this->stat == ScaLP::status::TIMEOUT_INFEASIBLE) {
			this->firstObjectiveOptimal = false;
		}
		// track optimality of second objective (i.e., schedule length)
		if (this->stat == ScaLP::status::OPTIMAL) {
			this->secondObjectiveOptimal = true;
		}
		else {
			this->secondObjectiveOptimal = false;
		}

		//check result and act accordingly
		if(stat==ScaLP::status::FEASIBLE || stat==ScaLP::status::OPTIMAL || stat==ScaLP::status::TIMEOUT_FEASIBLE) {
			this->r = this->solver->getResult();

			this->scheduleFound = true;

			this->fillSolutionStructure();

			if(!this->quiet) {
				cout << "------------------------" << endl;
				cout << "NonUniformRationalIIScheduler.scheduleIteration: Found result is " << this->stat << endl;
				cout << "NonUniformRationalIIScheduler.scheduleIteration: this solution is s / m : " << this->samples << " / " << this->modulo
						 << endl;
				cout << "NonUniformRationalIIScheduler.scheduleIteration: II: " << (double) (this->modulo) / (double) (this->samples)
						 << " (integer minII " << ceil((double) (this->modulo) / (double) (this->samples)) << ")" << endl;
				cout << "NonUniformRationalIIScheduler.scheduleIteration: throughput: " << (double) (this->samples) / (double) (this->modulo) << endl;
				cout << "------------------------" << endl;
				this->printScheduleToConsole();
			}
		}
		else {
			if(!this->quiet) cout << "NonUniformRationalIIScheduler.scheduleIteration: no schedule found for s / m : " << this->samples << " / " << this->modulo << " ( " << this->stat << " )" << endl;
			this->scheduleFound = false;
		}
	}

}
