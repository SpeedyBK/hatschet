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
		SchedulerBase(g, resourceModel), solverWishlist(solverWishlist), ILPSchedulerBase(solverWishlist)
	{
		this->computeMinII(&this->g, &this->resourceModel);
		double minII = this->getMinII();
		this->integerMinII = (int)ceil(minII);
		pair<int, int> frac = Utility::splitRational(minII);

		if(!this->quiet) {
			std::cout << "rational min II is " << minII << endl;
			std::cout << "integer min II is " << this->integerMinII << endl;
			std::cout << "auto setting samples to " << frac.second << endl;
			std::cout << "auto setting modulo to " << frac.first << endl;
		}

		this->samples = frac.second;
		this->modulo = frac.first;
	}

	void SCCQScheduler::schedule() {
		this->scheduleFound = false;
		if(!this->quiet) {
			std::cout << "SCC Q SCHEDULER graph: " << std::endl;
			std::cout << this->g << std::endl;
			std::cout << "SCC Q SCHEDULER resource model: " << std::endl;
			std::cout << this->resourceModel << std::endl;
			std::cout << "M: " << this->modulo << std::endl;
			std::cout << "S: " << this->samples << std::endl;
		}

		//timestamp
		this->begin = clock();
		// find SCCs
		KosarajuSCC k(this->g);
		k.setQuiet();
		auto sccs = k.getSCCs();

		// schedule SCCs
		auto p = this->getSCCSchedule(sccs);
		auto sccScheduleValid = p.first;
		auto sccSchedule = p.second;

		if(!sccScheduleValid) {
			if(!this->quiet) std::cout << "SCC scheduling unsuccessful for S/M = " << this->samples << "/" << this->modulo << std::endl;
			this->II = -1;
			this->modulo = -1;
			this->samples = -1;
			this->scheduleFound = false;
			this->startTimes.clear();
			this->startTimesVector.clear();
			this->end = clock();
			if(this->solvingTime == -1.0) this->solvingTime = 0.0;
			this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;
			return;
		}

		if(!this->quiet) {
			// print schedule for debugging reasons
			std::cout << "scc schedule:" << std::endl;
			for(auto it : sccSchedule) {
				auto v = it.first;
				auto t = it.second.first;
				auto id = it.second.second;
				std::cout << "    " << v->getName() << ": " << t << ", " << id << std::endl;
			}
			// print MRT shape with already inserted SCCs
			this->mrt.print();
		}

		// determine start times based on MRT and DFG
		determineStartTimes(sccs, sccSchedule);

		// optimize start times if possible
		this->optimizeStartTimes();
		//timestamp
		this->end = clock();
		//log time
		if(this->solvingTime == -1.0) this->solvingTime = 0.0;
		this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;

		// fill ratII start times vector
		for(auto II : this->initiationIntervals) {
			std::map<Vertex*,int> tempStartTimes;
			for(auto it : this->startTimes) {
				auto v = it.first;
				auto t = it.second;
				tempStartTimes[v] = t+II;
			}
			this->startTimesVector.emplace_back(tempStartTimes);
		}

		// print ratII start times
		if(!this->quiet) {
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
		this->II = (double)(this->modulo) / (double)(this->samples);
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
		auto timeout = this->solver->timeout;
		if(timeout>0) mq.setSolverTimeout(timeout);
		mq.setSamples(this->samples);
		mq.setModulo(this->modulo);
		mq.setMaxLatencyConstraint(this->getMaxLatencyConstraint());
		mq.setMaxLatencySequenceIterations(1);
		mq.schedule();

		if(!tempG.isEmpty()) {
			// for a graph without cycles, nothing has to be scheduled
			// we are still interested in the MRT shape and the latency sequence/initiation intervals
			if(!mq.getScheduleFound()) {
				sccSchedule.clear(); // clear to be safe that it's empty
				return std::make_pair(false,sccSchedule); // return empty schedule
			}

			// verify result from modulo q scheduler
			if(!verifyRationalIIModuloSchedule(tempG,rm,mq.getStartTimeVector(),mq.getLatencySequence(),mq.getScheduleLength())) {
				sccSchedule.clear(); // clear to be safe that it's empty
				return std::make_pair(false,sccSchedule); // return empty schedule
			}
		}


		// init MRT
		/*
		this->mrt.setResourceModelAndII(this->resourceModel,this->modulo);
		for(auto it : mq.getMRTShape()) {
			auto res = it.first;
			auto originalRes = this->resourceModel.getResource(res->getName());
			auto table = it.second;
			for(unsigned int i=0; i<this->modulo; ++i) {
				this->mrt.specifyColumnHeight(originalRes,i,(unsigned int)table[i]);
			}
		}
		 */

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

	void SCCQScheduler::determineStartTimes(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {
		auto queueG = this->getScheduleQueue(sccs,sccSchedule);
		auto queue = queueG.first;
		auto sccGraph = queueG.second;

		for(auto sccId : queue) {
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
							throw HatScheT::Exception("Vertex '"+vertex->getName()+"' not scheduled yet - this should never happen!");
						}
					}

				}
			}

			// if no inputs - schedule ASAP
			auto vertices = scc->getVerticesOfSCC();
			if(inputStartTimes.empty()) {
				if(vertices.size()==1) {
					// trivial SCC schedule ASAP while respecting MRT
					auto vertex = vertices.front();
					int offset = 0;
					bool scheduled = false;
					bool limited = this->resourceModel.getResource(vertex)->getLimit()>1;
					while(!scheduled and limited) {
						if(!this->quiet)
							std::cout << "Try scheduling vertex '" << vertex->getName() << "' at time slot " << offset << std::endl;
						scheduled = this->mrt.insertVertex(vertex,offset);
						if(!scheduled) ++offset;
						if(offset>=this->modulo)
							throw HatScheT::Exception("Could not determine time slot for vertex '"+vertex->getName()+"' - this should never happen");
					}
					this->startTimes[vertex] = offset;
					if(!this->quiet) std::cout << "SCHEDULED VERTEX '" << vertex->getName() << "' IN TIME SLOT '" << this->startTimes[vertex] << "' (NO INPUTS)" << std::endl;
				}
				else {
					// non-trivial SCC
					for(auto vertex : vertices) {
						bool limited = this->resourceModel.getResource(vertex)->getLimit()>1;
						if(limited) {
							auto slots = this->mrt.getModuloSlots(vertex);
							if(slots.size() != 1)
								throw HatScheT::Exception("MRT for first sample is buggy");
							this->startTimes[vertex] = slots[0];
							if(!this->quiet) std::cout << "SCHEDULED VERTEX '" << vertex->getName() << "' IN TIME SLOT '" << this->startTimes[vertex] << "' (NO INPUTS)" << std::endl;
						}
						else {

						}
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
			int minimumOffset = 0;
			for(auto edge : this->g.Edges()) {
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
				auto srcStart = this->startTimes[src];
				// S[src] + rm.getVertexLatency(i) + e->getDelay() <= S[dst] + DELTA;
				int vertexStartTime = 0;
				try {
					vertexStartTime = sccSchedule.at(dst).first;
				}
				catch(std::out_of_range&) {
					//vertexStartTime = this->mrt.getModuloSlots(dst)[0];
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

				auto ok = srcStart + this->resourceModel.getVertexLatency(src) + edge->getDelay() <= vertexStartTime + minimumOffset + minDelta;
				if(vertices.size()==1) ok &= this->mrt.insertVertex(vertices.front(),(vertexStartTime+minimumOffset)%this->modulo);
				while(!ok) {
					if(vertices.size()==1) {
						vertexStartTime = minimumOffset;
						// trivial SCC is not yet scheduled - just pick the next free time slot in MRT
						this->mrt.removeVertex(vertices.front());
						bool freeSlot = this->mrt.insertVertex(vertices.front(),vertexStartTime%this->modulo);
						while(!freeSlot) {
							++vertexStartTime;
							freeSlot = this->mrt.insertVertex(vertices.front(),vertexStartTime%this->modulo);
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
	}

	std::pair<std::list<int>,Graph*> SCCQScheduler::getScheduleQueue(vector<SCC *> &sccs, std::map<Vertex *, std::pair<int, int>> &sccSchedule) {
		auto* sccGraph = new Graph();
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
}