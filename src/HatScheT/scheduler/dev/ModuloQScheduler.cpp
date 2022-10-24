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

	void ModuloQMRT::print() const {
		std::cout << "MRT" << std::endl;
		for(auto &it : this->mrt) {
			std::cout << "  Resource " << it.first->getName() << std::endl;
			unsigned int maxHeight = 0;
			for(auto &it2 : it.second) {
				if(it2.size() > maxHeight) maxHeight = (unsigned int)it2.size();
			}
			for(unsigned int row = 0; row<maxHeight; ++row) {
				for(unsigned int column = 0; column < this->II; ++column) {
					if(it.second.at(column).size()<=row) {
						std::cout << "---------- ";
						continue;
					}
					if(it.second.at(column).at(row)==nullptr) {
						std::cout << "0000000000 ";
						continue;
					}
					auto name = it.second.at(column).at(row)->getName();
					std::cout << name << " ";
					for(auto i=0; i<(10-name.size()); ++i) {
						std::cout << " ";
					}
				}
				std::cout << std::endl;
			}
		}
	}

	std::vector<int> ModuloQMRT::getModuloSlots(Vertex *v) const {
		vector<int> slots;
		if(v==nullptr)
			throw HatScheT::Exception("ModuloQMRT::getModuloSlots: can't request nullptr");
		for(auto row : this->mrt.at(this->rm->getResource(v))) {
			int slot = 0;
			for(auto vertex : row) {
				if(vertex == v) slots.emplace_back(slot);
				++slot;
			}
		}
		return slots;
	}

	int ModuloQMRT::getHeight(const Resource* res, int column) const {
		try {
			return (int)this->mrt.at(res).at(column).size();
		}
		catch(...) {
			throw HatScheT::Exception("Invalid resource or MRT column requested");
		}
	}

	void ModuloQMRT::specifyColumnHeight(const Resource *res, unsigned int column, unsigned int height) {
		if(this->mrt.find(res) == this->mrt.end())
			throw HatScheT::Exception("Invalid resource '"+res->getName()+"' provided - it does not exist in MRT");
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
			//if(limit<0) continue;
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

	std::list<Vertex *> ModuloQMRT::getVerticesInModuloSlot(int m, const Resource* res) {
		if(m >= this->mrt[res].size()) {
			throw Exception("ModuloQMRT::getVerticesInModuloSlot: invalid modulo slot requested");
		}
		list<Vertex *> l;
		for(auto v : this->mrt[res][m]) {
			if(v != nullptr) l.emplace_back(v);
		}
		return l;
	}


	ModuloQScheduler::ModuloQScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																		 std::list<std::string> solverWishlist) :
		RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist)
	{
	}

	void ModuloQScheduler::setMRT(ModuloQMRT &mrt, ResourceModel &resourceModel, std::vector<int> &initiationIntervals, int samples, int modulo, bool quiet) {
		if(!quiet) {
			std::cout << "setting MRT for latency sequence '";
			for (auto l : initiationIntervals) {
				std::cout << l << " ";
			}
			std::cout << "'" << std::endl;
		}
		mrt.setResourceModelAndII(resourceModel,modulo);
		if(samples==1) {
			// construct trivial MRT
			for(auto res : resourceModel.Resources()) {
				auto limit = res->getLimit();
				bool limited = limit>0;
				int height = limit;
				if(!limited)
					height = samples*resourceModel.getVerticesOfResource(res).size();
				for(int i=0; i<modulo; ++i) {
					mrt.specifyColumnHeight(res,i,height);
				}
			}
			return;
		}
		// "copy" resource model (only relevant info)
		ResourceModel rm;
		std::map<const Resource*, const Resource*> resourceMap;
		for(auto res : resourceModel.Resources()) {
			auto *copiedResource = &rm.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
			resourceMap[copiedResource] = res;
			if(!quiet) {
				std::cout << "Created resource copy from '" << res->getName() << "' -> '" << copiedResource->getName() << "'" << std::endl;
			}
		}
		// construct rectangular dummy MRT
		if(!quiet) {
			std::cout << "Constructing dummy MRT now" << std::endl;
		}
		ModuloQMRT dummyMRT;
		dummyMRT.setResourceModelAndII(rm,modulo);
		for(auto res : rm.Resources()) {
			auto limit = res->getLimit();
			auto numberOfVertices = (unsigned int) resourceModel.getVerticesOfResource(resourceMap[res]).size();
			if(limit<0) {
				auto noOfVertices = resourceModel.getVerticesOfResource(res).size();
				for(int i=0; i<modulo; ++i) {
					mrt.specifyColumnHeight(resourceMap[res], i, noOfVertices * samples);
				}
				continue;
			}
			if(!quiet) {
				std::cout << "Finished constructing dummy MRT now" << std::endl;
			}
			// handle case - vertices for the resource is 1 => build trivial MRT with height=limit
			if(numberOfVertices==1) {
				if(!quiet) std::cout << "Found limited resource (" << res->getName() << ") with only one vertex registered to it - build trivial MRT for this resource" << std::endl;
				for(unsigned int i=0; i<modulo; ++i) {
					mrt.specifyColumnHeight(resourceMap[res],i,resourceMap[res]->getLimit());
					if(!quiet) std::cout << "specified MRT column height = " << mrt.getHeight(resourceMap[res],i) << " for resource " << resourceMap[res]->getName() << " at column " << i << std::endl;
				}
				continue;
			}
			// handle case - resource limit modulo #samples == 0 => build trivial MRT with height=limit/#samples
			if(limit % samples == 0) {
				if(!quiet) std::cout << "Found limited resource with limit modulo #samples == 0 - build trivial MRT for this resource" << std::endl;
				for(unsigned int i=0; i<modulo; ++i) {
					mrt.specifyColumnHeight(resourceMap[res],i,resourceMap[res]->getLimit()/samples);
					if(!quiet) std::cout << "specified MRT column height = " << mrt.getHeight(resourceMap[res],i) << " for resource " << resourceMap[res]->getName() << " at column " << i << std::endl;
				}
				continue;
			}
			for(unsigned int i=0; i<modulo; ++i) {
				if(!quiet) std::cout << "i=" << i << ", limit = " << limit << std::endl;
				dummyMRT.specifyColumnHeight(res,i,(unsigned int)limit);
				if(!quiet) std::cout << "specified dummy MRT column height = " << dummyMRT.getHeight(res,i) << " for resource " << res->getName() << " at column " << i << std::endl;
			}
			if(!quiet) std::cout << "created dummy MRT" << std::endl;
			// create container with dummy vertices
			std::vector<Vertex*> dummyVertices;
			for(unsigned int i=0; i<modulo * limit; ++i) {
				dummyVertices.emplace_back(new Vertex(i));
				rm.registerVertex(dummyVertices[i],res);
			}
			if(!quiet) std::cout << "created dummy vertices" << std::endl;
			// fill dummy MRT with dummy vertices and track MRT height
			unsigned int counter=0;
			unsigned int vertexCounter=0;
			unsigned int failedAttempts=0;
			std::vector<unsigned int> mrtHeights(modulo,0);
			while(failedAttempts<modulo) {
				if(!quiet) {
					std::cout << "  counter=" << counter << std::endl;
					std::cout << "  vertexCounter=" << vertexCounter << std::endl;
				}
				bool valid = true;
				unsigned int firstModuloSlot = 0;
				for(auto &latency : initiationIntervals) {
					unsigned int moduloSlot = (counter+latency) % modulo;
					if(latency==0) firstModuloSlot = moduloSlot;
					if(!quiet) {
						std::cout << "    latency = " << latency << std::endl;
						std::cout << "    moduloSlot = " << latency << std::endl;
					}
					if(!dummyMRT.insertVertex(dummyVertices[vertexCounter],moduloSlot)) {
						if(!quiet) std::cout << "    MRT at column " << moduloSlot << " is already full" << std::endl;
						valid = false;
						break;
					}
					else {
						if(!quiet) std::cout << "    inserted vertex " << vertexCounter << " to modulo slot " << moduloSlot << std::endl;
					}
				}
				if(valid) {
					++mrtHeights[firstModuloSlot];
					++vertexCounter;
					failedAttempts = 0;
					if(!quiet) {
						std::cout << "    found MRT slot for vertex" << std::endl;
						std::cout << "    MRT height: " << mrtHeights[counter % modulo] << std::endl;
					}
				}
				else {
					if(!quiet) std::cout << "    removing vertex from MRT" << std::endl;
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
				mrt.specifyColumnHeight(resourceMap[res],i,mrtHeights[i]);
				if(!quiet) std::cout << "specified MRT column height = " << mrt.getHeight(resourceMap[res],i) << " for resource " << resourceMap[res]->getName() << " at column " << i << std::endl;
			}
		}
	}

	void ModuloQScheduler::setObjective() {
		// minimize latest start time
		ScaLP::Variable ss = ScaLP::newIntegerVariable("supersink");
		this->solver->addConstraint(ss >= 0);
		if (this->maxLatencyConstraint >= 0)
			this->solver->addConstraint(ss <= this->maxLatencyConstraint);

		for (auto *i : this->g.Vertices()) {
			if (this->g.hasNoZeroDistanceOutgoingEdges(i)) {
				this->solver->addConstraint(ss - this->time[i] >= this->resourceModel.getVertexLatency(i));
			}
		}

		solver->setObjective(ScaLP::minimize(ss));
	}

	void ModuloQScheduler::constructDecisionVariables() {
		time.clear();
		//row.clear();
		a.clear();
		a.resize(this->modulo);
		k.clear();

		for (auto *i : g.Vertices()) {
			auto id = "_" + std::to_string(i->getId());
			// (1)
			for (int r = 0; r < this->modulo; ++r) a[r][i] = ScaLP::newBinaryVariable("a_" + std::to_string(r) + id);

			// (2)
			k[i]    = ScaLP::newIntegerVariable("k" + id);
			//row[i]  = ScaLP::newIntegerVariable("row" + id, 0, this->M - 1);
			time[i] = ScaLP::newIntegerVariable("time" + id);

			solver->addConstraint(k[i] >= 0);
			solver->addConstraint(time[i] >= 0);
			if (maxLatencyConstraint >= 0) {
				solver->addConstraint(k[i]    <= (int)floor(maxLatencyConstraint / (double)this->modulo));
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
			for (int r = 0; r < this->modulo; ++r) sumBind.add(a[r][i], r);
			//solver->addConstraint(row[i] - sumBind == 0);
			solver->addConstraint(time[i] - (this->modulo * k[i]) - sumBind == 0);

			// assignment constraints (1)
			ScaLP::Term sumAssign;
			for (int r = 0; r < this->modulo; ++r) sumAssign.add(a[r][i], 1);
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

			for (int r = 0; r < this->modulo; ++r) {
				ScaLP::Term sumRes;
				for (auto *i : using_q)
					for (int c = 0; c < q->getBlockingTime(); ++c)
						sumRes.add(a[mod(r - c, this->modulo)][i], 1);
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
				for(unsigned int i=0; i<this->samples; ++i) {
					int omegaTemp = 0;
					for(unsigned int j=0; j<e->getDistance(); ++j) {
						int ind = (i+j)%this->samples;
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

			this->solver->addConstraint(this->time[j] - this->time[i] >= l_ij - omega_ij_min);
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

	std::map<Resource*,std::vector<int>> ModuloQScheduler::getMRTShape() const {
		std::map<Resource*,std::vector<int>> shape;
		for(auto res : this->resourceModel.Resources()) {
			// skip unlimited resources
			if(res->getLimit()<0) continue;
			for(int i=0; i<this->modulo; ++i) {
				shape[res].emplace_back(this->mrt.getHeight(res,i));
			}
		}
		return shape;
	}

	void ModuloQScheduler::setInitiationIntervals() {
		if(!this->quiet) std::cout << "Creating 'optimal' initiation interval sequence" << std::endl;
		if(this->samples==1) {
			this->initiationIntervals = {0};
			return;
		}

		this->initiationIntervals = RationalIISchedulerLayer::getOptimalInitiationIntervalSequence(this->samples,this->modulo,this->quiet);
	}

	void ModuloQScheduler::scheduleIteration() {
		if (!this->quiet) {
			std::cout << "MODULO Q SCHEDULER graph: " << std::endl;
			std::cout << this->g << std::endl;
			std::cout << "MODULO Q SCHEDULER resource model: " << std::endl;
			std::cout << this->resourceModel << std::endl;
			std::cout << "M: " << this->modulo << std::endl;
			std::cout << "S: " << this->samples << std::endl;
		}

		// clear containers
		this->latencySequence.clear();
		this->initiationIntervals.clear();

		this->setInitiationIntervals();

		// iterate through latency sequences and try to find a schedule for one of them
		this->scheduleFound = false;
		// determine initiation intervals from latency sequence
		this->latencySequence = getLatencySequenceFromInitiationIntervals(this->initiationIntervals, this->modulo);
		if (!this->quiet) {
			std::cout << "Start scheduling Attempt!" << std::endl;
			std::cout << "Latency Sequence: " << std::endl;
			for (auto l : this->initiationIntervals) std::cout << l << " ";
			std::cout << std::endl;
			std::cout << "Initiation Intervals: " << std::endl;
			for (auto j : this->latencySequence) std::cout << j << " ";
			std::cout << std::endl;
		}
		// set a valid non-rectangular MRT for the given latency sequence
		ModuloQScheduler::setMRT(this->mrt, this->resourceModel, this->initiationIntervals, this->samples, this->modulo,
														 this->quiet);
		if (!this->quiet) this->mrt.print();
		// start scheduling
		this->scheduleFound = this->scheduleAttempt();

		// stop function if no schedule was found
		if(!this->scheduleFound) return;

		// looks like we found a schedule! :)
		if (!this->quiet) std::cout << "Found feasible solution!" << std::endl;
		// set start times
		auto solution = this->solver->getResult().values;
		for (auto *v : this->g.Vertices())
			this->startTimes[v] = (int) std::lround(solution.find(this->time[v])->second);
		for (auto &late : this->initiationIntervals) {
			std::map<Vertex *, int> additionalStartTimes;
			for (auto startTime : this->startTimes) {
				additionalStartTimes[startTime.first] = startTime.second + late;
			}
			this->startTimesVector.emplace_back(additionalStartTimes);
		}
		this->II = this->minII;
		bool emptyGraph = this->g.isEmpty();

		if (!this->quiet) {
			std::cout << "Rational II modulo schedule found with:" << std::endl;
			std::cout << "  S=" << this->samples << std::endl;
			std::cout << "  M=" << this->modulo << std::endl;
			std::cout << "  IIs=";
			for (auto i : this->latencySequence) {
				std::cout << i << " ";
			}
			std::cout << std::endl;
			std::cout << "  Latency=" << this->getScheduleLength() << std::endl;
		}
	}
}