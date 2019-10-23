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
	ModuloQMRT::ModuloQMRT() : rm(nullptr), II(0), mrt() {}

	ModuloQMRT::ModuloQMRT(ResourceModel &rm, unsigned int II) : rm(&rm), II(II) {
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
		if(this->mrt.find(this->rm->getResource(v)) == this->mrt.end())
			throw HatScheT::Exception("Invalid vertex provided, its resource type doesn't exist in MRT");
		for(auto &it : this->mrt[this->rm->getResource(v)]) {
			if(it[moduloSlot] == nullptr) {
				it[moduloSlot] = v;
				return true;
			}
		}
		return false;
	}

	bool ModuloQMRT::removeVertex(Vertex *v) {
		bool removed = false;
		try {
			for(auto &row : this->mrt.at(this->rm->getResource(v))) {
				for(auto &it : row) {
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
			for(auto &it2 : it.second) {
				std::cout << "    ";
				for(auto &it3 : it2) {
					if(it3==nullptr) {
						std::cout << "           ";
						continue;
					}
					auto name = it3->getName();
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
		std::make_pair(this->S,this->M);
	}
}


