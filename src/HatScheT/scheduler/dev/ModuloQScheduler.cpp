//
// Created by nfiege on 17/10/19.
//

#include "ModuloQScheduler.h"
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <HatScheT/utility/subgraphs/KosarajuSCC.h>
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <cmath>



namespace HatScheT {
	ModuloQMRT::ModuloQMRT() : quiet(true), rm(nullptr), II(0), mrt() {}

	ModuloQMRT::ModuloQMRT(ResourceModel &rm, unsigned int II) : quiet(true), rm(&rm), II(II) {
		for(auto resIt = this->rm->resourcesBegin(); resIt != this->rm->resourcesEnd(); ++resIt) {
			auto res = *resIt;
			// insert matrix for each resource
			std::vector<std::vector<Vertex*>> matrix;
			for(int i=0; i<res->getLimit(); ++i) {
				std::vector<Vertex*> row(this->II,nullptr);
				matrix.emplace_back(row);
			}
			if(res->getLimit()>0) this->mrt[const_cast<const Resource*>(res)] = matrix;
		}
	}

	bool ModuloQMRT::insertVertex(Vertex *v, unsigned int moduloSlot) {
		if(moduloSlot>=this->II)
			throw HatScheT::Exception("Invalid modulo slot requested: "+to_string(moduloSlot)+", II="+to_string(this->II));
		auto *res = this->rm->getResource(v);
		if(this->mrt.find(res) == this->mrt.end())
			throw HatScheT::Exception("Invalid vertex provided, its resource type doesn't exist in MRT");
		auto &column = this->mrt[res][moduloSlot];
		for(auto &it : column) {
			if(it == nullptr) {
				it = v;
				return true;
			}
		}
		return false;
	}

	bool ModuloQMRT::removeVertex(Vertex *v) {
		bool removed = false;
		try {
			for(auto &column : this->mrt.at(this->rm->getResource(v))) {
				for(auto &it : column) {
					if(it == v) {
						removed = true;
						it = nullptr;
					}
				}
			}
		}
		catch(std::out_of_range&) {
			// resource does not exist in MRT
			return false;
		}

		return removed;
	}

	void ModuloQMRT::print() {
		std::cout << "MRT" << std::endl;
		for(auto &it : this->mrt) {
			std::cout << "  Resource " << it.first->getName() << std::endl;
			unsigned int maxHeight = 0;
			for(auto &it2 : it.second) {
				if(it2.size() > maxHeight) maxHeight = (unsigned int)it2.size();
			}
			for(unsigned int row = 0; row<maxHeight; ++row) {
				for(unsigned int column = 0; column < this->II; ++column) {
					if(it.second[column].size()<=row) {
						std::cout << "---------- ";
						continue;
					}
					if(it.second[column][row]==nullptr) {
						std::cout << "0000000000 ";
						continue;
					}
					auto name = it.second[column][row]->getName();
					std::cout << name << " ";
					for(auto i=0; i<(10-name.size()); ++i) {
						std::cout << " ";
					}
				}
				std::cout << std::endl;
			}
		}
	}

	std::vector<int> ModuloQMRT::getModuloSlots(Vertex *v) {
		vector<int> slots;
		if(v==nullptr)
			throw HatScheT::Exception("ModuloQMRT::getModuloSlots: can't request nullptr");
		for(auto row : this->mrt[this->rm->getResource(v)]) {
			int slot = 0;
			for(auto vertex : row) {
				if(vertex == v) slots.emplace_back(slot);
				++slot;
			}
		}
		return slots;
	}

	int ModuloQMRT::getHeight(const Resource* res, int column) {
		try {
			return (int)this->mrt.at(res).at(column).size();
		}
		catch(...) {
			throw HatScheT::Exception("Invalid resource or MRT column requested");
		}
	}

	void ModuloQMRT::specifyColumnHeight(const Resource *res, unsigned int column, unsigned int height) {
		if(this->mrt.find(res) == this->mrt.end())
			throw HatScheT::Exception("Invalid resource provided - it does not exist in MRT");
		if(column>=this->II)
			throw HatScheT::Exception("Invalid column requested: "+to_string(column)+", II="+to_string(this->II));
		this->mrt.at(res).at(column).resize(height);
	}

	void ModuloQMRT::setResourceModelAndII(ResourceModel &rm, unsigned int II) {
		this->II = II;
		this->rm = &rm;
		this->mrt.clear();
		for(auto resIt = this->rm->resourcesBegin(); resIt != this->rm->resourcesEnd(); ++resIt) {
			auto res = *resIt;
			auto limit = res->getLimit();
			if(limit<0) continue;
			this->rotations[res] = 0;
			for(unsigned int i=0; i<II; ++i) {
				this->mrt[res].emplace_back(std::vector<Vertex*>());
			}
		}
	}

	void ModuloQMRT::rotateLeft() {
		bool newRotation = true;
		for(auto &innerMrt : this->mrt) {
			if(newRotation) {
				// some resources in this MRT might have rectangular matrices that do not have to be rotated
				auto rotatable = false;
				auto height = innerMrt.second.front().size();
				for(auto &column : innerMrt.second) {
					if(column.size() != height) {
						rotatable = true;
						break;
					}
				}
				if(!rotatable) {
					if(!this->quiet) std::cout << "resource '" << innerMrt.first->getName() << "' is not rotatable - skip it" << std::endl;
					continue;
				}

				// THE MATRIX OF THIS RESOURCE CAN BE ROTATED!
				++this->rotations[innerMrt.first];
				if(this->rotations[innerMrt.first] == this->II) {
					this->rotations[innerMrt.first] = 0;
				}
				else {
					newRotation = false;
				}
				if(!this->quiet) std::cout << "rotate resource '" << innerMrt.first->getName() << "'; rotation counter: " << this->rotations[innerMrt.first] << std::endl;
				std::vector<Vertex*> backup = innerMrt.second[0];
				for(unsigned int i=0; i<innerMrt.second.size()-1; ++i) {
					innerMrt.second[i] = innerMrt.second[i+1];
				}
				innerMrt.second[innerMrt.second.size()-1] = backup;
			}
		}
	}

	unsigned long ModuloQMRT::getMaxNumberOfRotations() {
		unsigned long rot = 1;
		for(auto &it : this->mrt) {
			auto matrix = it.second;
			auto height = matrix.front().size();
			for(auto &column : matrix) {
				if(column.size() != height) {
					rot *= this->II;
					break;
				}
			}
		}
		return rot;
	}


	ModuloQScheduler::ModuloQScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																		 std::list<std::string> solverWishlist) :
		SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist), quiet(true), maxSequenceIterations(1)
	{

		this->computeMinII(&this->g, &this->resourceModel);
		double minII = this->getMinII();
		//ceiling
		this->integerMinII = (int)ceil(minII);
		pair<int, int> frac = Utility::splitRational(minII);

		if(!this->quiet) {
			cout << "rational min II is " << minII << endl;
			cout << "integer min II is " << this->integerMinII << endl;
			cout << "auto setting samples to " << frac.second << endl;
			cout << "auto setting modulo to " << frac.first << endl;
		}

		this->S = frac.second;
		this->M = frac.first;
	}

	std::pair<int, int> ModuloQScheduler::getSM() {
		std::make_pair(this->S,this->M);
	}

	void ModuloQScheduler::schedule() {
		this->scheduleFound = false;
		if(!this->quiet) {
			std::cout << "graph: " << std::endl;
			std::cout << this->g << std::endl;
			std::cout << "resource model: " << std::endl;
			std::cout << this->resourceModel << std::endl;
			std::cout << "M: " << this->M << std::endl;
			std::cout << "S: " << this->S << std::endl;
		}

		// clear containers
		this->allInitiationIntervals.clear();
		this->latencySequence.clear();
		this->initiationIntervals.clear();
		this->discardedInitiationIntervals.clear();

		// compute all latency sequences
		auto lat = getAllInitiationIntervals(this->M, this->S);
		// sort latency sequences (latency sequences with low variance are scheduled first)
		std::map<double, std::vector<std::vector<int>>> sortedInitIntervals;
		for(auto &sequence : lat) {
			auto initIntervals = getLatencySequenceFromInitiationIntervals(sequence, this->M);
			int min = initIntervals.front();
			for(auto i : initIntervals) {
				if(i<min) min = i;
			}
			sortedInitIntervals[this->M-min].emplace_back(sequence);
		}
		for(auto &it : sortedInitIntervals) {
			for(auto &it2 : it.second) {
				this->allInitiationIntervals.emplace_back(it2);
			}
		}

		// iterate through latency sequences and try to find a schedule for one of them
		this->scheduleFound = false;
		for(int i=0; i<this->allInitiationIntervals.size() and i<this->maxSequenceIterations; ++i) {
			this->initiationIntervals = this->allInitiationIntervals[i];
			// determine initiation intervals from latency sequence
			this->latencySequence = getLatencySequenceFromInitiationIntervals(this->latencySequence, this->M);
			if(!this->quiet) {
				std::cout << "Start scheduling Attempt!" << std::endl;
				std::cout << "Latency Sequence: " << std::endl;
				for (auto l : this->initiationIntervals) std::cout << l << " ";
				std::cout << std::endl;
				std::cout << "Initiation Intervals: " << std::endl;
				for (auto i : this->latencySequence) std::cout << i << " ";
				std::cout << std::endl;
			}
			// set a valid non-rectangular MRT for the given latency sequence
			this->setMRT();
			if(!this->quiet) this->mrt.print();
			// start scheduling
			this->scheduleFound = this->scheduleAttempt();
			if(this->scheduleFound) {
				if(!this->quiet) std::cout << "Found feasible solution!" << std::endl;
				// set start times
				auto solution = this->solver->getResult().values;
				for (auto *v : this->g.Vertices())
					this->startTimes[v] = (int) std::lround(solution.find(this->time[v])->second);
				for(auto &late : this->initiationIntervals) {
					std::map<Vertex*,int> additionalStartTimes;
					for(auto startTime : this->startTimes) {
						additionalStartTimes[startTime.first] = startTime.second + late;
					}
					this->startTimesVector.emplace_back(additionalStartTimes);
				}
				this->II = this->minII;
				bool valid = verifyRationalIIModuloSchedule(this->g,this->resourceModel,this->startTimesVector,this->latencySequence,this->getScheduleLength());
				if(!this->quiet) {
					if (valid) {
						std::cout << "Valid rational II modulo schedule found with:" << std::endl;
						std::cout << "  II=" << this->II << std::endl;
						std::cout << "  S=" << this->S << std::endl;
						std::cout << "  M=" << this->M << std::endl;
						std::cout << "  IIs=";
						for (auto i : this->latencySequence) {
							std::cout << i << " ";
						}
						std::cout << std::endl;
						std::cout << "  Latency=" << this->getScheduleLength() << std::endl;
					} else {
						std::cout << "Invalid rational II modulo schedule found - this should never happen" << std::endl;
					}
				}
				break;
			}
			else {
				if(!this->quiet) std::cout << "Did not find feasible solution :(" << std::endl;
				this->discardedInitiationIntervals.emplace_back(this->initiationIntervals);
			}
		}
	}

	std::vector<std::vector<int>> ModuloQScheduler::getAllInitiationIntervals(int M, int S) {
		if(M<1 or S<1)
			throw HatScheT::Exception("Invalid values for M and S given: "+to_string(M)+" and "+to_string(S));
		vector<vector<int>> initIntervals;

		vector<int> nextSequence = {0};
		for(unsigned int i=0; i<S-1; ++i) {
			nextSequence.emplace_back(nextSequence[i]+1);
		}

		bool finished = false;
		while(!finished) {
			// push latency sequence into list
			initIntervals.emplace_back(nextSequence);

			// calculate next sequence
			for(unsigned int i=0; i<=S-1; ++i) {
				unsigned int index = S - 1 - i;
				++nextSequence[index];
				auto diff = nextSequence.size()-index;
				if(nextSequence[index]<M-diff+1) break;
			}
			for(unsigned int i=0; i<=S-1; ++i) {
				unsigned int index = S - 1 - i;
				auto diff = nextSequence.size()-index;
				if(nextSequence[index]==M-diff+1) nextSequence[index] = index;
			}
			for(unsigned int i=0; i<S-1; ++i) {
				unsigned int index = i;
				while(nextSequence[index+1]<=nextSequence[index]) ++nextSequence[index+1];
			}

			// check if finished
			if(nextSequence[0] != 0) finished = true;
		}

		return initIntervals;
	}

	void ModuloQScheduler::setMRT() {
		if(!this->quiet) {
			std::cout << "setting MRT for latency sequence '";
			for (auto l : this->initiationIntervals) {
				std::cout << l << " ";
			}
			std::cout << "'" << std::endl;
		}
		this->mrt.setResourceModelAndII(this->resourceModel,this->M);
		if(this->S==1) {
			// construct trivial MRT
			for(auto resIt = this->resourceModel.resourcesBegin(); resIt != this->resourceModel.resourcesEnd(); ++resIt) {
				auto res = *resIt;
				auto limit = res->getLimit();
				if(limit<0) continue;
				for(int i=0; i<this->M; ++i) {
					this->mrt.specifyColumnHeight(res,i,limit);
				}
			}
			return;
		}
		// "copy" resource model (only relevant info)
		ResourceModel rm;
		std::map<const Resource*, const Resource*> resourceMap;
		for(auto resIt = this->resourceModel.resourcesBegin(); resIt != this->resourceModel.resourcesEnd(); ++resIt) {
			auto res = *resIt;
			resourceMap[&rm.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime())] = res;
		}
		// construct rectangular dummy MRT
		ModuloQMRT dummyMRT;
		dummyMRT.setResourceModelAndII(rm,this->M);
		for(auto resIt = rm.resourcesBegin(); resIt != rm.resourcesEnd(); ++resIt) {
			auto res = *resIt;
			auto limit = res->getLimit();
			auto numberOfVertices = (unsigned int) this->resourceModel.getVerticesOfResource(resourceMap[res]).size();
			if(limit<0) continue;
			// handle case - vertices for the resource is leq the limit => build trivial MRT with height=limit
			if(numberOfVertices<=res->getLimit()) {
				if(!this->quiet) std::cout << "Found limited resource with only one vertex registered to it - build trivial MRT for this resource" << std::endl;
				for(unsigned int i=0; i<this->M; ++i) {
					this->mrt.specifyColumnHeight(resourceMap[res],i,resourceMap[res]->getLimit());
					if(!this->quiet) std::cout << "specified MRT column height = " << this->mrt.getHeight(resourceMap[res],i) << " for resource " << resourceMap[res]->getName() << " at column " << i << std::endl;
				}
				continue;
			}
			// handle case - resource limit modulo #samples == 0 => build trivial MRT with height=limit/#samples
			if(resourceMap[res]->getLimit() % this->S == 0) {
				if(!this->quiet) std::cout << "Found limited resource with limit modulo #samples == 0 - build trivial MRT for this resource" << std::endl;
				for(unsigned int i=0; i<this->M; ++i) {
					this->mrt.specifyColumnHeight(resourceMap[res],i,resourceMap[res]->getLimit()/this->S);
					if(!this->quiet) std::cout << "specified MRT column height = " << this->mrt.getHeight(resourceMap[res],i) << " for resource " << resourceMap[res]->getName() << " at column " << i << std::endl;
				}
				continue;
			}
			for(unsigned int i=0; i<this->M; ++i) {
				if(!this->quiet) std::cout << "i=" << i << ", limit = " << limit << std::endl;
				dummyMRT.specifyColumnHeight(res,i,(unsigned int)limit);
				if(!this->quiet) std::cout << "specified dummy MRT column height = " << dummyMRT.getHeight(res,i) << " for resource " << res->getName() << " at column " << i << std::endl;
			}
			if(!this->quiet) std::cout << "created dummy MRT" << std::endl;
			// create container with dummy vertices
			std::vector<Vertex*> dummyVertices;
			for(unsigned int i=0; i<this->M * limit; ++i) {
				dummyVertices.emplace_back(new Vertex(i));
				rm.registerVertex(dummyVertices[i],res);
			}
			if(!this->quiet) std::cout << "created dummy vertices" << std::endl;
			// fill dummy MRT with dummy vertices and track MRT height
			unsigned int counter=0;
			unsigned int vertexCounter=0;
			unsigned int failedAttempts=0;
			std::vector<unsigned int> mrtHeights(this->M,0);
			while(failedAttempts<this->M) {
				if(!this->quiet) {
					std::cout << "  counter=" << counter << std::endl;
					std::cout << "  vertexCounter=" << vertexCounter << std::endl;
				}
				bool valid = true;
				unsigned int firstModuloSlot = 0;
				for(auto &latency : this->initiationIntervals) {
					unsigned int moduloSlot = (counter+latency) % this->M;
					if(latency==0) firstModuloSlot = moduloSlot;
					if(!this->quiet) {
						std::cout << "    latency = " << latency << std::endl;
						std::cout << "    moduloSlot = " << latency << std::endl;
					}
					if(!dummyMRT.insertVertex(dummyVertices[vertexCounter],moduloSlot)) {
						if(!this->quiet) std::cout << "    MRT at column " << moduloSlot << " is already full" << std::endl;
						valid = false;
						break;
					}
					else {
						if(!this->quiet) std::cout << "    inserted vertex " << vertexCounter << " to modulo slot " << moduloSlot << std::endl;
					}
				}
				if(valid) {
					++mrtHeights[firstModuloSlot];
					++vertexCounter;
					failedAttempts = 0;
					if(!this->quiet) {
						std::cout << "    found MRT slot for vertex" << std::endl;
						std::cout << "    MRT height: " << mrtHeights[counter % this->M] << std::endl;
					}
				}
				else {
					if(!this->quiet) std::cout << "    removing vertex from MRT" << std::endl;
					dummyMRT.removeVertex(dummyVertices[vertexCounter]);
					++failedAttempts;
				}
				++counter;
			}
			// free memory
			for(auto it : dummyVertices) {
				delete it;
			}
			// construct MRT based on mrtHeights
			for(unsigned int i=0; i<mrtHeights.size(); ++i) {
				this->mrt.specifyColumnHeight(resourceMap[res],i,mrtHeights[i]);
				if(!this->quiet) std::cout << "specified MRT column height = " << this->mrt.getHeight(resourceMap[res],i) << " for resource " << resourceMap[res]->getName() << " at column " << i << std::endl;
			}
		}
	}

	void ModuloQScheduler::setObjective() {
		// minimize sum of start times
		ScaLP::Term sum;
		for(auto v : this->g.Vertices()) {
			sum.add(this->time[v],1);
		}
		this->solver->setObjective(ScaLP::minimize(sum));
	}

	void ModuloQScheduler::constructDecisionVariables() {
		time.clear();
		//row.clear();
		a.clear();
		a.resize(this->M);
		k.clear();

		for (auto *i : g.Vertices()) {
			auto id = "_" + std::to_string(i->getId());
			// (1)
			for (int r = 0; r < this->M; ++r) a[r][i] = ScaLP::newBinaryVariable("a_" + std::to_string(r) + id);

			// (2)
			k[i]    = ScaLP::newIntegerVariable("k" + id);
			//row[i]  = ScaLP::newIntegerVariable("row" + id, 0, this->M - 1);
			time[i] = ScaLP::newIntegerVariable("time" + id);

			solver->addConstraint(k[i] >= 0);
			solver->addConstraint(time[i] >= 0);
			if (maxLatencyConstraint >= 0) {
				solver->addConstraint(k[i]    <= (int)floor(maxLatencyConstraint / (double)this->M));
				solver->addConstraint(time[i] <= maxLatencyConstraint);
			}
		}
	}

	void ModuloQScheduler::constructConstraints() {
		for (auto *i : g.Vertices()) {
			// anchor source vertices, but only if they are not resource-limited
			if (g.isSourceVertex(i) && resourceModel.getResource(i)->getLimit() == UNLIMITED) {
				solver->addConstraint(k[i] == 0);
				solver->addConstraint(a[0][i] == 1);
			}

			// bind result variables (2)
			ScaLP::Term sumBind;
			for (int r = 0; r < this->M; ++r) sumBind.add(a[r][i], r);
			//solver->addConstraint(row[i] - sumBind == 0);
			solver->addConstraint(time[i] - (this->M * k[i]) - sumBind == 0);

			// assignment constraints (1)
			ScaLP::Term sumAssign;
			for (int r = 0; r < this->M; ++r) sumAssign.add(a[r][i], 1);
			solver->addConstraint(sumAssign == 1);
		}

		// resource constraints (5)
		// this could be extended to general reservation tables
		for (auto qIt = resourceModel.resourcesBegin(), qEnd = resourceModel.resourcesEnd(); qIt != qEnd; ++qIt) {
			auto *q = *qIt;
			if (q->getLimit() == UNLIMITED)
				continue;
			if (q->isReservationTable())
				throw HatScheT::Exception("ModuloQ currently handles only simple resources");

			auto using_q = resourceModel.getVerticesOfResource(q);

			for (int r = 0; r < this->M; ++r) {
				ScaLP::Term sumRes;
				for (auto *i : using_q)
					for (int c = 0; c < q->getBlockingTime(); ++c)
						sumRes.add(a[mod(r - c, this->M)][i], 1);
				///////solver->addConstraint(sumRes <= q->getLimit());
				solver->addConstraint(sumRes <= this->mrt.getHeight(q,r));
			}
		}
		// "normal" dependence constraints (4)
		for (auto *e : g.Edges()) {
			auto *i = &e->getVertexSrc();
			auto *j = &e->getVertexDst();
			auto l_ij = resourceModel.getVertexLatency(i) + e->getDelay();
			vector<int> omega_ij;
			int omega_ij_min = 1000000; // "inifinity"
			if(!this->quiet) std::cout << "Edge: " << i->getName() << " -> " << j->getName() << ", distance: " << e->getDistance() << std::endl;
			if(e->getDistance() == 0) {
				omega_ij_min = 0;
			}
			else {
				for(unsigned int i=0; i<this->S; ++i) {
					int omegaTemp = 0;
					for(unsigned int j=0; j<e->getDistance(); ++j) {
						int ind = (i+j)%this->S;
						omegaTemp += this->latencySequence[ind];
					}
					omega_ij.emplace_back(omegaTemp);
				}
				for(auto omega : omega_ij) {
					if(!this->quiet) {
						std::cout << "  omega: " << omega << std::endl;
						std::cout << "  omega_ij_min: " << omega_ij_min << std::endl;
					}
					if(omega < omega_ij_min) omega_ij_min = omega;
				}
			}
			if(!this->quiet) std::cout << "  resulting omega_ij_min: " << omega_ij_min << std::endl;
			ScaLP::Term weightedSum;
			for(int r=1; r<this->M; ++r) {
				weightedSum.add(this->a[r][j],r);
				weightedSum.add(this->a[r][i],-r);
			}
			this->solver->addConstraint(weightedSum + this->M * this->k[j] - this->M * this->k[i] >= l_ij - omega_ij_min);
		}
	}

	void ModuloQScheduler::setUpSolverSettings() {
		this->solver->reset();
		this->solver->quiet = this->solverQuiet;
		this->solver->threads = this->threads;
		this->solver->timeout = this->solverTimeout;
	}

	bool ModuloQScheduler::scheduleAttempt() {
		this->setUpSolverSettings();
		this->constructDecisionVariables();
		this->constructConstraints();
		this->setObjective();

		//debug
		if(!this->quiet) {
			std::cout << "SOLVER INTERNAL" << std::endl;
			std::cout << this->solver->showLP() << std::endl;
		}

		//timestamp
		this->begin = clock();
		//solve
		stat = this->solver->solve();
		//timestamp
		this->end = clock();

		//log time
		if(this->solvingTime == -1.0) this->solvingTime = 0.0;
		this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;

		if(!this->quiet) {
			std::cout << "Time to solve: " << (double)(this->end - this->begin) / CLOCKS_PER_SEC << " sec" << std::endl;
			std::cout << "ScaLP status:" << stat << std::endl;
		}
		return (stat == ScaLP::status::OPTIMAL) or (stat == ScaLP::status::FEASIBLE) or (stat == ScaLP::status::TIMEOUT_FEASIBLE);
	}

	std::vector<int> ModuloQScheduler::getLatencySequenceFromInitiationIntervals(std::vector<int> &initIntervals, int M) {
		std::vector<int> latSeq;
		for(unsigned int i=0; i<initIntervals.size()-1; ++i) {
			latSeq.emplace_back(initIntervals[i + 1] - initIntervals[i]);
		}
		latSeq.emplace_back(M - initIntervals.back());
		return latSeq;
	}
}






#if 0
namespace HatScheT {


	ModuloQScheduler::ModuloQScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																							 std::list<std::string> solverWishlist) :
		SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist), latencySequence(),
		solverWishlist(solverWishlist)
	{

		this->computeMinII(&this->g, &this->resourceModel);
		double minII = this->getMinII();
		//ceiling
		this->integerMinII = (int)ceil(minII);
		pair<int, int> frac = Utility::splitRational(minII);

		cout << "rational min II is " << minII << endl;
		cout << "integer min II is " << this->integerMinII << endl;
		cout << "auto setting samples to " << frac.second << endl;
		cout << "auto setting modulo to " << frac.first << endl;

		this->S = frac.second;
		this->M = frac.first;

		// count number of operations for each operation type
		for(auto resIt=resourceModel.resourcesBegin(); resIt!=resourceModel.resourcesEnd(); ++resIt) {
			auto res = *resIt;
			this->numberOperations[const_cast<const Resource*>(res)] = 0;
		}
		for(auto it : g.Vertices()) {
			++this->numberOperations[this->resourceModel.getResource(it)];
		}

		// init MRT
		this->completeMrt = ModuloQMRT(this->resourceModel,this->M);
		this->firstSampleMrt = ModuloQMRT(this->resourceModel,this->M);
	}

	void ModuloQScheduler::schedule() {
		this->scheduleFound = false;
		std::cout << "graph: " << std::endl;
		std::cout << this->g << std::endl;
		std::cout << "resource model: " << std::endl;
		std::cout << this->resourceModel << std::endl;
		std::cout << "M: " << this->M << std::endl;
		std::cout << "S: " << this->S << std::endl;

		// find SCCs
		KosarajuSCC k(this->g);
		k.setQuiet();
		auto sccs = k.getSCCs();

		// schedule SCCs
		auto sccSchedule = this->getSCCSchedule(sccs);

		if(sccSchedule.empty())
			throw HatScheT::Exception("SCC scheduling unsuccessful for S/M = "+to_string(this->S)+"/"+to_string(this->M));

		// print schedule for debugging reasons
		/*
		std::cout << "scc schedule:" << std::endl;
		for(auto it : sccSchedule) {
			auto v = it.first;
			auto t = it.second.first;
			auto id = it.second.second;
			std::cout << "    " << v->getName() << ": " << t << ", " << id << std::endl;
		}
		 */

		// determine MRT based on SCCs
		determineMRT(sccs, sccSchedule);

		// determine start times based on MRT and DFG
		determineStartTimes(sccs, sccSchedule);

		// print start times
		/*
		std::cout << "Start times:" << std::endl;
		for(auto it : this->startTimes) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}
		 */

		// fill ratII start times vector
		for(auto II : this->latencySequence) {
			std::map<Vertex*,int> tempStartTimes;
			for(auto it : this->startTimes) {
				auto v = it.first;
				auto t = it.second;
				tempStartTimes[v] = t+II;
			}
			this->startTimesVector.emplace_back(tempStartTimes);
		}

		// print ratII start times
		/*
		std::cout << "Rat II Start times:" << std::endl;
		for(unsigned int i=0; i<this->startTimesVector.size(); ++i) {
			auto startT = this->startTimesVector[i];
			std::cout << "  Start time vector " << i << ":" << std::endl;
			for(auto it : startT) {
				std::cout << "    " << it.first->getName() << " - " << it.second << std::endl;
			}
		}
		 */

		this->scheduleFound = true;
		this->II = (double)(this->M) / (double)(this->S);
	}

	std::vector<std::vector<unsigned int>> ModuloQScheduler::getAllLatencySequences(int M, int S) {
		if(M<1 or S<1)
			throw HatScheT::Exception("Invalid values for M and S given: "+to_string(M)+" and "+to_string(S));
		vector<vector<unsigned int>> latencySequences;

		vector<unsigned int> nextSequence = {0};
		for(unsigned int i=0; i<S-1; ++i) {
			nextSequence.emplace_back(nextSequence[i]+1);
		}

		bool finished = false;
		while(!finished) {
			// push latency sequence into list
			latencySequences.emplace_back(nextSequence);

			// calculate next sequence
			for(unsigned int i=0; i<=S-1; ++i) {
				unsigned int index = S - 1 - i;
				++nextSequence[index];
				auto diff = nextSequence.size()-index;
				if(nextSequence[index]<M-diff+1) break;
			}
			for(unsigned int i=0; i<=S-1; ++i) {
				unsigned int index = S - 1 - i;
				auto diff = nextSequence.size()-index;
				if(nextSequence[index]==M-diff+1) nextSequence[index] = index;
			}
			for(unsigned int i=0; i<S-1; ++i) {
				unsigned int index = i;
				while(nextSequence[index+1]<=nextSequence[index]) ++nextSequence[index+1];
			}

			// check if finished
			if(nextSequence[0] != 0) finished = true;
		}

		return latencySequences;
	}

	std::map<Vertex *, pair<int, int>> ModuloQScheduler::getSCCSchedule(std::vector<SCC *> &sccs) {
		map<Vertex *, pair<int, int>> sccSchedule;
		// generate new graph containing all non-trivial SCCs
		Graph tempG;
		std::map<Vertex*,Vertex*> originalVertexMap; // original vertex -> tempG vertex
		std::map<Vertex*,Vertex*> tempGVertexMap; // tempG vertex -> original vertex
		std::map<Vertex*,int> sccMap; // tempG vertex -> id of the scc it belongs to
		for(auto scc : sccs) {
			// insert scc into tempG
			// insert vertices into tempG
			auto vertices = scc->getVerticesOfSCC();
			if(vertices.size()<2) continue;
			for(auto v : vertices) {
				auto &newV = tempG.createVertex(v->getId());
				originalVertexMap[v] = &newV;
				tempGVertexMap[&newV] = v;
				sccMap[&newV] = scc->getId();
			}
			// insert edges into tempG
			for(auto e : scc->getSCCEdges()) {
				auto src = originalVertexMap[&e->getVertexSrc()];
				auto dst = originalVertexMap[&e->getVertexDst()];
				auto &newE = tempG.createEdge(*src,*dst,e->getDistance(),e->getDependencyType());
				newE.setDelay(e->getDelay());
			}
		}

		// generate resource model
		ResourceModel rm;
		for(auto v : this->g.Vertices()) {
			// skip vertices that are only in trivial SCCs
			if(originalVertexMap.find(v) == originalVertexMap.end()) continue;
			auto res = this->resourceModel.getResource(v);
			Resource* newRes;
			// only create new resource if it does not already exist
			try {
				newRes = rm.getResource(res->getName());
			}
			catch(HatScheT::Exception&) {
				newRes = &rm.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
			}
			// register vertex of tempG to new resource
			rm.registerVertex(originalVertexMap[v],newRes);
		}

		// schedule graph with ILP based rational II scheduler
		auto* ratII = new RationalIIScheduler(tempG,rm,this->solverWishlist);
		auto timeout = this->solver->timeout;
		if(timeout>0) ratII->setSolverTimeout(timeout);
		ratII->setModulo(this->M);
		ratII->setSamples(this->S);
		ratII->setMaxLatencyConstraint(this->getMaxLatencyConstraint());
		ratII->setMaxRuns(1);
		ratII->schedule();

		if(!ratII->getScheduleFound()) {
			sccSchedule.clear(); // clear to be safe that it's empty
			return sccSchedule; // return empty schedule
		}

		// verify result from ratII
		if(!verifyRationalIIModuloSchedule(tempG,rm,ratII->getStartTimeVector(),ratII->getInitIntervalls(),ratII->getScheduleLength())) {
			sccSchedule.clear(); // clear to be safe that it's empty
			return sccSchedule; // return empty schedule
		}

		this->initiationIntervals = ratII->getInitIntervalls();
		if(this->initiationIntervals.empty())
			throw HatScheT::Exception("Initiation intervals vector is empty after ratII schedule - this should never happen!");
		this->latencySequence = {0};
		for(unsigned int i=0; i<this->initiationIntervals.size()-1; ++i) {
			this->latencySequence.emplace_back(this->latencySequence[this->latencySequence.size()-1]+this->initiationIntervals[i]);
		}

		// return solution
		for(auto p : ratII->getSchedule()) {
			auto v = p.first;
			auto t = p.second;
			sccSchedule[tempGVertexMap[v]] = std::make_pair(t,sccMap[v]);
		}
		return sccSchedule;
	}

	void ModuloQScheduler::determineMRT(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {
		// check which SCCs are trivial
		vector<int> trivialSCCs;
		for(auto &it : sccs) {
			if(it->getVerticesOfSCC().size()==1) trivialSCCs.emplace_back(it->getId());
		}
		// put scheduled SCCs into MRT
		std::cout << std::endl;
		for(auto &it : sccSchedule) {
			auto v = it.first;
			auto res = this->resourceModel.getResource(v);
			if(res->getLimit()<1) continue;
			// skip trivial SCCs -> they are scheduled seperately
			bool trivial = false;
			for(auto id : trivialSCCs) {
				if(it.second.second == id) {
					trivial = true;
					break;
				}
			}
			if(trivial) continue;
			// put into MRT
			for(auto offset : this->latencySequence) {
				auto moduloSlot = (it.second.first+offset) % this->M;
				auto feasible = this->completeMrt.insertVertex(v,(unsigned int)moduloSlot);
				if(!feasible)
					throw HatScheT::Exception("Failed to insert vertex '"+v->getName()+"' into MRT - this should never happen");
				else if(offset==0) {
					this->firstSampleMrt.insertVertex(v,(unsigned int)moduloSlot);
				}
			}
		}

		// put trivial SCCs into MRT while respecting latency sequence
		for(auto &scc : sccs) {
			auto vertices = scc->getVerticesOfSCC();
			if(vertices.size() != 1) continue; // only consider trivial sccs
			auto v = vertices.front();
			auto res = this->resourceModel.getResource(v);
			if(res->getLimit() < 1) continue; // only consider limited resources

			// check all possibilities : [0 ... M-1] until one that works is encountered
			bool foundSlot = false;
			for(int i=0; i<this->M; ++i) {
				foundSlot = true;
				for(auto offset : this->latencySequence) {
					// put into MRT
					auto moduloSlot = (i+offset) % this->M;
					auto feasible = this->completeMrt.insertVertex(v,(unsigned int)moduloSlot);
					if(offset==0)
						this->firstSampleMrt.insertVertex(v,(unsigned int)moduloSlot);
					if(!feasible) {
						this->completeMrt.removeVertex(v);
						this->firstSampleMrt.removeVertex(v);
						foundSlot = false;
						break;
					}
				}
				if(foundSlot) break;
			}
			if(!foundSlot) {
				this->completeMrt.print();
				throw HatScheT::Exception("Failed to find MRT slot for vertex '"+v->getName()+"' - this should never happen");
			}
		}

		// debugging
		/*
		this->completeMrt.print();
		this->firstSampleMrt.print();
		 */
	}

	void ModuloQScheduler::determineStartTimes(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {
		auto queueG = this->getScheduleQueue(sccs,sccSchedule);
		auto queue = queueG.first;
		auto sccGraph = queueG.second;

		// debugging
			/*
		std::cout << "queue: " << std::endl;
		for(auto it : queue) {
			std::cout << "  " << it << std::endl;
			for(auto scc : sccs) {
				if(scc->getId() == it) {
					for(auto v : scc->getVerticesOfSCC()) {
						std::cout << "    " << v->getName() << std::endl;
					}
				}
			}
		}
			 */

		for(auto sccId : queue) {
			// debugging
			//std::cout << "scheduling SCC with ID '" << sccId << "' now" << std::endl;
			// check which SCCs are inputs to this SCC
			vector<int> inputStartTimes;
			// find the right SCC to this ID
			SCC* scc = nullptr;
			for(auto &sccIt : sccs) {
				if(sccIt->getId() == sccId) {
					scc = sccIt;
					break;
				}
			}
			if(scc == nullptr)
				throw HatScheT::Exception("Did not find SCC with ID '"+to_string(sccId)+"' - this should never happen");
			for(auto &edge : sccGraph->Edges()) {
				// debugging
				//std::cout << "  Edge from '" << edge->getVertexSrc().getName() << "' to '" << edge->getVertexDst().getName() << "'" << std::endl;
				if(edge->getVertexDst().getId() == sccId) {
					//std::cout << "    Dst ID == SCC ID (" << edge->getVertexDst().getId() << " == " << sccId << ")" << std::endl;
					// check start times of all vertices in source SCC
					SCC* sourceSCC = nullptr;
					for(auto sourceSCCIt : sccs) {
						if(sourceSCCIt->getId() == edge->getVertexSrc().getId()) sourceSCC = sourceSCCIt;
					}
					if(sourceSCC == nullptr)
						throw HatScheT::Exception("Did not find source SCC with ID '"+to_string(edge->getVertexSrc().getId())+"' - this should never happen");
					for(auto &vertex : sourceSCC->getVerticesOfSCC()) {
						try {
							inputStartTimes.emplace_back(this->startTimes.at(vertex));
						}
						catch(std::out_of_range&) {
							printStartTimes(this->startTimes);
							throw HatScheT::Exception("Vertex '"+vertex->getName()+"' not scheduled yet - this should never happen!");
						}
					}

				}
			}

			// if no inputs - schedule ASAP
			if(inputStartTimes.empty()) {
				for(auto vertex : scc->getVerticesOfSCC()) {
					auto slots = this->firstSampleMrt.getModuloSlots(vertex);
					if(slots.size() != 1)
						throw HatScheT::Exception("MRT for first sample is buggy");
					this->startTimes[vertex] = slots[0];
					// debugging
					//std::cout << "SCHEDULED VERTEX '" << vertex->getName() << "' IN TIME SLOT '" << slots[0] << "' (NO INPUTS)" << std::endl;
				}
				continue;
			}

			// check earliest start time according to edges
			int minimumOffset = 0;
			for(auto edge : this->g.Edges()) {
				// check if src is inside the SCC or if dst is outside the SCC
				bool srcInside = false;
				bool dstOutside = true;
				auto *src = &edge->getVertexSrc();
				auto *dst = &edge->getVertexDst();
				for(auto vertex : scc->getVerticesOfSCC()) {
					if(vertex == src) {
						srcInside = true;
						break;
					}
					if(vertex == dst) {
						dstOutside = false;
					}
				}
				if(srcInside or dstOutside) continue;
				// check start time of src (it MUST have been scheduled already)
				auto srcStart = this->startTimes[src];
				// S[src] + rm.getVertexLatency(i) + e->getDelay() <= S[dst] + e->getDistance() * II;
				int vertexStartTime = 0;
				try {
					vertexStartTime = sccSchedule.at(dst).first;
				}
				catch(std::out_of_range&) {
					vertexStartTime = this->firstSampleMrt.getModuloSlots(dst)[0];
				}
				unsigned int minDelta = -1;
				for(int i=0; i<this->M; ++i) {
					unsigned int delta = 0;

					for(int counter = 0; counter < edge->getDistance(); ++counter) {
						auto II = this->latencySequence[(counter+i) % M];
						delta += II;
					}
					if(delta < minDelta) minDelta = delta;
				}

				auto ok = srcStart + this->resourceModel.getVertexLatency(src) + edge->getDelay() <= vertexStartTime + minimumOffset + minDelta;
				while(!ok) {
					minimumOffset += this->M;
					ok = srcStart + this->resourceModel.getVertexLatency(src) + edge->getDelay() <= vertexStartTime + minimumOffset + minDelta;
				}
			}

			// schedule SCC according to offset
			for(auto vertex : scc->getVerticesOfSCC()) {
				int vertexStartTime = 0;
				try {
					vertexStartTime = sccSchedule.at(vertex).first;
				}
				catch(std::out_of_range&) {
					vertexStartTime = this->firstSampleMrt.getModuloSlots(vertex)[0];
				}
				this->startTimes[vertex] = vertexStartTime + minimumOffset;
			}
		}

		// free memory
		delete sccGraph;
	}

	std::pair<std::list<int>,Graph*> ModuloQScheduler::getScheduleQueue(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {
		Graph* sccGraph = new Graph();
		// insert SCCs
		for(auto scc : sccs) {
			auto &v = sccGraph->createVertex(scc->getId());
		}
		// insert edges between SCCs
		for(auto &edge : this->g.Edges()) {
			// check if edge is in one of the SCCs
			bool foundEdge = false;
			for(auto &scc : sccs) {
				for(auto &sccEdge : scc->getSCCEdges()) {
					if(edge == sccEdge) {
						foundEdge = true;
						break;
					}
				}
				if(foundEdge) break;
			}
			if(foundEdge) continue;
			// src and dst in original graph
			auto &src = edge->getVertexSrc();
			auto &dst = edge->getVertexDst();
			// src-SCC-vertex and dst-SCC-vertex in SCC graph that were just created
			int srcId;
			int dstId;
			try {
				srcId = sccSchedule.at(&src).second;
				dstId = sccSchedule.at(&dst).second;
			}
			catch(std::out_of_range&) {
				// trivial SCCs do not appear in sccSchedule because they are not scheduled by the ratII ILP scheduler
				bool foundSrcId = false;
				bool foundDstId = false;
				for(auto &scc : sccs) {
					for(auto &v : scc->getVerticesOfSCC()) {
						if(v == &src) {
							foundSrcId = true;
							srcId = scc->getId();
						}
						if(v == &dst) {
							foundDstId = true;
							dstId = scc->getId();
						}
						if(foundSrcId and foundDstId) break;
					}
					if(foundSrcId and foundDstId) break;
				}
				if(!foundSrcId)
					throw HatScheT::Exception("Did not find SCC ID of trivial vertex '"+src.getName()+"' - this should never happen");
				if(!foundDstId)
					throw HatScheT::Exception("Did not find SCC ID of trivial vertex '"+dst.getName()+"' - this should never happen");
			}
			auto &sccSrc = sccGraph->getVertexById(srcId);
			auto &sccDst = sccGraph->getVertexById(dstId);
			// create data edge with distance 0
			sccGraph->createEdge(sccSrc,sccDst);
			std::cout << "created edge " << sccSrc.getName() << " -> " << sccDst.getName() << ")" << std::endl;
		}

		// create resource model with one resource with limit=latency=1 and register every vertex to it
		ResourceModel sccRm;
		auto &resource = sccRm.makeResource("you_are_a_nice_person",1,1);
		for(auto &scc : sccs) {
			sccRm.registerVertex(&sccGraph->getVertexById(scc->getId()),&resource);
			std::cout << "registered vertex " << sccGraph->getVertexById(scc->getId()).getName() << std::endl;
		}

		// schedule SCCs
		ASAPScheduler asap(*sccGraph,sccRm);
		asap.schedule();

		// fill vector to return
		std::list<int> queue;
		auto sched = asap.getSchedule();
		for(auto it : sched) {
			auto sccId = it.first->getId();
			auto t = it.second;
			// check where to insert
			bool inserted = false;
			for(auto q = queue.begin(); q != queue.end(); ++q) {
				auto &v = sccGraph->getVertexById(*q);
				auto time = sched[&v];
				if(t < time) {
					queue.insert(q,sccId);
					inserted = true;
					break;
				}
			}
			if(!inserted) queue.emplace_back(sccId);
		}

		return std::make_pair(queue,sccGraph);
	}

	void ModuloQScheduler::printStartTimes(std::map<Vertex *, int> startTimes) {
		std::cout << "------------------------------------------------------" << std::endl;
		std::cout << "Start times" << std::endl;
		for(auto it : startTimes) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}
		if(startTimes.empty()) std::cout << "  *EMPTY*" << std::endl;
		std::cout << "------------------------------------------------------" << std::endl;
	}

	std::pair<int, int> ModuloQScheduler::getSM() {
		return std::make_pair(this->S,this->M);
	}
}

#endif