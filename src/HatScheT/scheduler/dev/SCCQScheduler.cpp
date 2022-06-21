//
// Created by nfiege on 04/11/19.
//

#include "SCCQScheduler.h"
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <HatScheT/utility/subgraphs/KosarajuSCC.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <cmath>
#include <bits/stdc++.h>

namespace HatScheT {


	SCCQScheduler::SCCQScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist) :
		RationalIISchedulerLayer(g, resourceModel), solverWishlist(solverWishlist), ILPSchedulerBase(solverWishlist)
	{
	}

	std::pair<bool,std::map<Vertex *, pair<int, int>>> SCCQScheduler::getSCCSchedule(std::vector<SCC *> &sccs) {
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
			// skip trivial SCCs
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
		if(!this->quiet) std::cout << "START MODULO Q SCHEDULER" << std::endl;
		auto mq = ModuloQScheduler(tempG,rm,this->solverWishlist);
		mq.setQuiet(this->quiet);
		mq.setSolverTimeout(this->solverTimeout);
		mq.setSamples(this->samples);
		mq.setModulo(this->modulo);
		mq.setMaxLatencyConstraint(this->getMaxLatencyConstraint());
		mq.schedule();
		if(mq.getScaLPStatus() == ScaLP::status::INFEASIBLE_OR_UNBOUND or mq.getScaLPStatus() == ScaLP::status::INFEASIBLE)
			this->stat = mq.getScaLPStatus();

		if(!tempG.isEmpty()) {
			// for a graph without cycles, nothing has to be scheduled
			// we are still interested in the MRT shape and the latency sequence/initiation intervals
			if(!mq.getScheduleFound()) {
				sccSchedule.clear(); // clear to be safe that it's empty
				return std::make_pair(false,sccSchedule); // return empty schedule
			}

			// verify result from modulo q scheduler
			if(!verifyRationalIIModuloSchedule2(tempG, rm, mq.getStartTimeVector(), mq.getLatencySequence(),
																					mq.getScheduleLength())) {
				sccSchedule.clear(); // clear to be safe that it's empty
				return std::make_pair(false,sccSchedule); // return empty schedule
			}
		}

		this->latencySequence = mq.getLatencySequence();
		this->initiationIntervals = mq.getInitiationIntervals();
		if(this->latencySequence.empty())
			throw HatScheT::Exception("Latency sequence vector is empty after ratII schedule - this should never happen!");
		ModuloQScheduler::setMRT(this->mrt,this->resourceModel,this->initiationIntervals,this->samples,this->modulo,this->quiet);

		//debugging
		if(!this->quiet) this->mrt.print();

		// return solution
		if(!tempG.isEmpty()) {
			for(auto p : mq.getSchedule()) {
				auto v = p.first;
				auto t = p.second;
				sccSchedule[tempGVertexMap[v]] = std::make_pair(t,sccMap[v]);
				this->mrt.insertVertex(tempGVertexMap[v],(unsigned int)t%this->modulo);
			}
		}

		return std::make_pair(true,sccSchedule);
	}

	void SCCQScheduler::determineMRT(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {
		// check which SCCs are trivial
		vector<int> trivialSCCs;
		for (auto &it : sccs) {
			if (it->getVerticesOfSCC().size() == 1) trivialSCCs.emplace_back(it->getId());
		}
		// put scheduled SCCs into MRT
		for (auto &it : sccSchedule) {
			auto v = it.first;
			auto res = this->resourceModel.getResource(v);
			if (res->getLimit() < 1) continue;
			// skip trivial SCCs -> they are scheduled seperately
			bool trivial = false;
			for (auto id : trivialSCCs) {
				if (it.second.second == id) {
					trivial = true;
					break;
				}
			}
			if (trivial) continue;
			// put into MRT
			auto moduloSlot = it.second.first % this->modulo;
			auto feasible = this->mrt.insertVertex(v, (unsigned int) moduloSlot);
			if (!feasible) {
				this->mrt.print();
				throw HatScheT::Exception("Failed to insert vertex '" + v->getName() + "' into MRT - this should never happen");
			}
			else {
				if(!this->quiet) std::cout << "Inserted vertex '' into MRT at time slot " << moduloSlot << std::endl;
			}
		}
	}

	bool SCCQScheduler::determineStartTimes(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {

		// track time
		this->end = clock();
		std::cout << "SCC solver needed " << this->solvingTime << " sec - determining start times now" << std::endl;

		auto queueG = this->getScheduleQueue(sccs,sccSchedule);
		auto queue = queueG.first;
		auto sccGraph = queueG.second;

		if(sccGraph == nullptr) {
			// timeout during schedule queue generation is encoded as a nullptr in the graph object
			std::cout << "timeout during schedule queue generation after " << this->solvingTime << " sec" << std::endl;
			return false;
		}

		if(!this->quiet) {
			std::cout << "Determined queue for scheduling SCCs:" << std::endl;
			for(auto &sccID : queue) {
				std::cout << "  SCC ID " << sccID << std::endl;
				SCC* scc = nullptr;
				for(auto &sccIt : sccs) {
					if(sccIt->getId() == sccID) {
						scc = sccIt;
						break;
					}
				}
				for(auto &vertex : scc->getVerticesOfSCC()) {
					std::cout << "    " << vertex->getName() << std::endl;
				}
			}
		}

		for(auto sccId : queue) {
			// track time
			this->end = clock();
			this->solvingTime = (double) (this->end - this->begin) / CLOCKS_PER_SEC;
			if(this->solvingTime > this->solverTimeout) {
				std::cout << "timeout during start time determination after " << this->solvingTime << " sec" << std::endl;
				return false;
			}
			// debugging
			if(!this->quiet) std::cout << "scheduling SCC with ID '" << sccId << "' now" << std::endl;
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
				if(!this->quiet) std::cout << "  Edge from '" << edge->getVertexSrc().getName() << "' to '" << edge->getVertexDst().getName() << "'" << std::endl;
				if(edge->getVertexDst().getId() == sccId) {
					if(!this->quiet) std::cout << "    Dst ID == SCC ID (" << edge->getVertexDst().getId() << " == " << sccId << ")" << std::endl;
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
							if(!this->quiet) {
								std::cout << "    Tried scheduling SCC with ID '" << sccId << "' but vertex '" << vertex->getName() << "' (SCC ID " << sourceSCC->getId() << ") was not scheduled yet" << std::endl;
							}
							throw HatScheT::Exception("Vertex '"+vertex->getName()+"' not scheduled yet - this should never happen!");
						}
					}

				}
			}

			if(!this->quiet) {
				std::cout << "  SCC " << sccId << " has " << inputStartTimes.size() << " inputs with following start times:" << std::endl;
				for(auto &t : inputStartTimes) {
					std::cout << "    " << t << std::endl;
				}
			}

			// if no inputs - schedule ASAP
			auto vertices = scc->getVerticesOfSCC();

			if(!this->quiet) {
				std::cout << "  SCC " << sccId << " consists of the following vertices" << std::endl;
				for(auto &v : vertices) {
					std::cout << "    " << v->getName() << std::endl;
				}
			}
			if(inputStartTimes.empty()) {
				if(vertices.size()==1) {
					// trivial SCC schedule ASAP while respecting MRT
					// they were not scheduled in SCC supergraph so we must determine their starting times manually
					auto vertex = vertices.front();
					int offset = 0;
					bool scheduled = false;
					bool limited = this->resourceModel.getResource(vertex)->getLimit()>0;
					while(!scheduled and limited) {
						// try scheduling vertex
						if(!this->quiet)
							std::cout << "Try scheduling vertex '" << vertex->getName() << "' at time slot " << offset << std::endl;
						scheduled = this->mrt.insertVertex(vertex,offset);
						if(!scheduled) ++offset;
						if(offset>=this->modulo)
							throw HatScheT::Exception("Could not determine time slot for vertex '"+vertex->getName()+"' - this should never happen");
					}
					this->startTimes[vertex] = offset;
					if(!this->quiet) std::cout << "SCHEDULED VERTEX '" << vertex->getName() << "' FROM TRIVIAL SCC IN TIME SLOT '" << this->startTimes[vertex] << "' (NO INPUTS)" << std::endl;
				}
				else {
					// non-trivial SCC
					for(auto vertex : vertices) {
						// just use SCC schedule time since it does not depend on other SCCs
						this->startTimes[vertex] = sccSchedule[vertex].first;
					}
				}
				// debugging
				if(!this->quiet) {
					std::cout << "MRT after scheduling SCC" << sccId << ":" << std::endl;
					this->mrt.print();
				}
				continue;
			}

			// check earliest start time according to edges
			if(!this->quiet) {
				std::cout << "  start checking earliest start time according to edges" << std::endl;
			}
			int minimumOffset = 0;
			for(auto edge : this->g.Edges()) {
				if(!this->quiet) std::cout << "    minimum offset: " << minimumOffset << std::endl;
				// check if src is inside the SCC or if dst is outside the SCC
				bool srcInside = false;
				bool dstOutside = true;
				auto *src = &edge->getVertexSrc();
				auto *dst = &edge->getVertexDst();
				for(auto vertex : vertices) {
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
				if(!this->quiet) std::cout << "    check schedule time of vertex '" << src->getName() << "'" << std::endl;
				auto srcStart = this->startTimes[src];
				if(!this->quiet) std::cout << "    schedule time of vertex '" << src->getName() << "': " << srcStart << std::endl;
				// S[src] + rm.getVertexLatency(i) + e->getDelay() <= S[dst] + DELTA;
				int vertexStartTime = 0;
				try {
					vertexStartTime = sccSchedule.at(dst).first;
				}
				catch(std::out_of_range&) {
					// vertexStartTime is left at 0;
				}
				unsigned int minDelta = -1;
				for(int i=0; i<this->modulo; ++i) {
					unsigned int delta = 0;

					for(int counter = 0; counter < edge->getDistance(); ++counter) {
						auto II = this->initiationIntervals[(counter+i) % samples];
						delta += II;
					}
					if(delta < minDelta) minDelta = delta;
				}
				if(!this->quiet) std::cout << "    determined minimum delta according to latency sequence: " << minDelta << std::endl;

				auto ok = srcStart + this->resourceModel.getVertexLatency(src) + edge->getDelay() <= vertexStartTime + minimumOffset + minDelta;
				if(!this->quiet) std::cout << "    determining minimum offset while respecting MRT" << std::endl;
				// insert trivial SCCs into MRT
				if(vertices.size()==1 and this->resourceModel.getResource(vertices.front())->getLimit()>0) {
					// only insert them if they were not already inserted from a previous iteration when checking dependency of another edge
					if (this->mrt.getModuloSlots(vertices.front()).empty()) {
						if(!this->quiet) std::cout << "    try inserting vertex '" << vertices.front()->getName() << "' from trivial SCC into MRT" << std::endl;
						ok &= this->mrt.insertVertex(vertices.front(), (vertexStartTime + minimumOffset) % this->modulo);
					}
				}

				while(!ok) {
					if(vertices.size()==1) {
						vertexStartTime = minimumOffset;
						// trivial SCC is not yet scheduled - just pick the next free time slot in MRT
						if(!this->quiet) std::cout << "    removing vertex '" << vertices.front()->getName() << "' from MRT again" << std::endl;
						this->mrt.removeVertex(vertices.front());
						if(!this->quiet) std::cout << "    try inserting vertex '" << vertices.front()->getName() << "' into MRT at time slot " << vertexStartTime%this->modulo << std::endl;
						bool freeSlot = true;
						if(this->resourceModel.getResource(vertices.front())->getLimit()>0)
							freeSlot = this->mrt.insertVertex(vertices.front(),vertexStartTime%this->modulo);
						if(!this->quiet) {
							std::cout << "    slot is " << (freeSlot?string(""):string("not ")) << "free" << std::endl;
							if(!freeSlot) {
								std::cout << "    MRT" << std::endl;
								this->mrt.print();
							}
						}
						while(!freeSlot) {
							++vertexStartTime;
							if(!this->quiet) std::cout << "    try inserting vertex '" << vertices.front()->getName() << "' into MRT at time slot " << vertexStartTime%this->modulo << std::endl;
							freeSlot = this->mrt.insertVertex(vertices.front(),vertexStartTime%this->modulo);
							if(!this->quiet) std::cout << "    slot is " << (freeSlot?string(""):string("not ")) << "free" << std::endl;
							minimumOffset = vertexStartTime;
						}
						ok = srcStart + this->resourceModel.getVertexLatency(src) + edge->getDelay() <= minimumOffset + minDelta;
						if(!ok) ++minimumOffset;
					}
					else {
						minimumOffset += this->modulo;
						ok = srcStart + this->resourceModel.getVertexLatency(src) + edge->getDelay() <= vertexStartTime + minimumOffset + minDelta;
					}
				}
			}

			// schedule SCC according to offset
			if(!this->quiet) std::cout << "Scheduling SCC " << sccId << " with offset " << minimumOffset << std::endl;
			if(vertices.size()==1) {
				auto vertex = vertices.front();
				this->startTimes[vertex] = minimumOffset;
				if(!this->quiet) std::cout << "SCHEDULED VERTEX '" << vertex->getName() << "' IN TIME SLOT '" << this->startTimes[vertex] << "'" << std::endl;
			}
			else {
				for(auto vertex : vertices) {
					int vertexStartTime = 0;
					vertexStartTime = sccSchedule[vertex].first;
					this->startTimes[vertex] = vertexStartTime + minimumOffset;
					if(!this->quiet) std::cout << "SCHEDULED VERTEX '" << vertex->getName() << "' IN TIME SLOT '" << this->startTimes[vertex] << "'" << std::endl;
				}
			}

			// debugging
			if(!this->quiet) {
				std::cout << "MRT after scheduling SCC" << sccId << ":" << std::endl;
				this->mrt.print();
			}
		}

		// free memory
		delete sccGraph;
		return true;
	}

	std::pair<std::list<int>,Graph*> SCCQScheduler::getScheduleQueue(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {
		auto* sccGraph = new Graph();
		// insert SCCs
		for(auto scc : sccs) {
			auto &v = sccGraph->createVertex(scc->getId());
		}
		// insert edges between SCCs
		for(auto &edge : this->g.Edges()) {
			this->end = clock();
			this->solvingTime = ((double)(this->end - this->begin) / CLOCKS_PER_SEC);
			if(this->solvingTime > this->solverTimeout) {
				return {std::list<int>(),nullptr};
			}
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
			if(!this->quiet) std::cout << "created edge " << sccSrc.getName() << " -> " << sccDst.getName() << ")" << std::endl;
		}

		// create resource model with one resource with limit=latency=1 and register every vertex to it
		ResourceModel sccRm;
		auto &resource = sccRm.makeResource("you_are_a_nice_person",1,1);
		for(auto &scc : sccs) {
			sccRm.registerVertex(&sccGraph->getVertexById(scc->getId()),&resource);
			if(!this->quiet) std::cout << "registered vertex " << sccGraph->getVertexById(scc->getId()).getName() << std::endl;
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

	void SCCQScheduler::printStartTimes(std::map<Vertex *, int> startTimes) {
		std::cout << "------------------------------------------------------" << std::endl;
		std::cout << "Start times" << std::endl;
		for(auto it : startTimes) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}
		if(startTimes.empty()) std::cout << "  *EMPTY*" << std::endl;
		std::cout << "------------------------------------------------------" << std::endl;
	}

	void SCCQScheduler::optimizeStartTimes() {
		int minimumCS = INT_MAX;
		for(auto &it : this->startTimes) {
			if(it.second < minimumCS) {
				if(!this->quiet) std::cout << "minimum control step so far: " << minimumCS << std::endl;
				minimumCS = it.second;
			}
		}
		if(!this->quiet) std::cout << "minimum control step: " << minimumCS << std::endl;
		if(minimumCS==0) return;
		if(!this->quiet) std::cout << "shifting back all control steps by " << minimumCS << std::endl;
		for(auto &it : this->startTimes) {
			this->startTimes[it.first] -= minimumCS;
		}
	}

	void SCCQScheduler::scheduleIteration() {
		this->secondObjectiveOptimal = false; // we can't guarantee anything about the schedule length with this scheduler
		if (!this->quiet) {
			std::cout << "SCC Q SCHEDULER graph: " << std::endl;
			std::cout << this->g << std::endl;
			std::cout << "SCC Q SCHEDULER resource model: " << std::endl;
			std::cout << this->resourceModel << std::endl;
			std::cout << "M: " << this->modulo << std::endl;
			std::cout << "S: " << this->samples << std::endl;
		}

		//timestamp
		this->begin = clock();
		// reset solving time
		this->solvingTime = 0.0;
		// find SCCs
		KosarajuSCC k(this->g);
		k.setQuiet(this->quiet);
		auto sccs = k.getSCCs();

		// schedule SCCs
		auto p = this->getSCCSchedule(sccs);
		auto sccScheduleValid = p.first;
		auto sccSchedule = p.second;

		if (!sccScheduleValid) {
			if (!this->quiet)
				std::cout << "SCC scheduling unsuccessful for S/M = " << this->samples << "/" << this->modulo << std::endl;
			this->II = -1;
			this->modulo = -1;
			this->samples = -1;
			this->scheduleFound = false;
			this->startTimes.clear();
			this->startTimesVector.clear();
			this->end = clock();
			this->solvingTime += (double) (this->end - this->begin) / CLOCKS_PER_SEC;
			this->firstObjectiveOptimal = false;
			// free memory
			for(auto scc : sccs) {
				delete scc;
			}
			return;
		}

		if (!this->quiet) {
			// print schedule for debugging reasons
			std::cout << "scc schedule:" << std::endl;
			for (auto it2 : sccSchedule) {
				auto v = it2.first;
				auto t = it2.second.first;
				auto id = it2.second.second;
				std::cout << "    " << v->getName() << ": " << t << ", " << id << std::endl;
			}
			// print MRT shape with already inserted SCCs
			this->mrt.print();
		}

		// determine start times based on MRT and DFG
		bool scheduleSuccess = this->determineStartTimes(sccs, sccSchedule);
		if(!scheduleSuccess) {
			this->scheduleFound = false;
			this->II = -1;
			this->modulo = -1;
			this->samples = -1;
			this->startTimes.clear();
			this->startTimesVector.clear();
			this->firstObjectiveOptimal = false;
			this->end = clock();
			this->solvingTime += (double) (this->end - this->begin) / CLOCKS_PER_SEC;
			// free memory
			for(auto scc : sccs) {
				delete scc;
			}
			return;
		}

		// optimize start times if possible
		this->optimizeStartTimes();
		//timestamp
		this->end = clock();
		//log time
		if (this->solvingTime == -1.0) this->solvingTime = 0.0;
		this->solvingTime += (double) (this->end - this->begin) / CLOCKS_PER_SEC;

		// fill ratII start times vector
		for (auto II : this->initiationIntervals) {
			std::map<Vertex *, int> tempStartTimes;
			for (auto it2 : this->startTimes) {
				auto v = it2.first;
				auto t = it2.second;
				tempStartTimes[v] = t + II;
			}
			this->startTimesVector.emplace_back(tempStartTimes);
		}

		// print ratII start times
		if (!this->quiet) {
			std::cout << "Rat II Start times:" << std::endl;
			for (unsigned int i = 0; i < this->startTimesVector.size(); ++i) {
				auto startT = this->startTimesVector[i];
				std::cout << "  Start time vector " << i << ":" << std::endl;
				for (auto it : startT) {
					std::cout << "    " << it.first->getName() << " - " << it.second << std::endl;
				}
			}
		}

		this->scheduleFound = true;

		// free memory
		for(auto scc : sccs) {
			delete scc;
		}
	}
}