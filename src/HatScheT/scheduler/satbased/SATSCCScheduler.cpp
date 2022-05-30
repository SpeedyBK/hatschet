//
// Created by nfiege on 5/20/22.
//

#include "SATSCCScheduler.h"
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/satbased/SATScheduler.h>
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>
#ifdef USE_CADICAL
namespace HatScheT {

	SATSCCScheduler::SATSCCScheduler(Graph &g, ResourceModel &resourceModel, int II) :
		SchedulerBase(g, resourceModel), solverTimeout(300), IIFeasible(true), solvingTime(-1.0) {
		this->II = II;
	}

	void SATSCCScheduler::schedule() {
		this->solvingTime = 0.0;
		auto timerStart = std::chrono::steady_clock::now();
		this->initMRT();
		this->computeSCCs();
		this->computeEarliestAndLatestStartTimes();
		this->computeSCCSchedule();
		if (!this->scheduleFound) {
			this->solvingTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timerStart).count() / 1000.0;
			return;
		}
		this->orderSCCs();
		this->computeFinalSchedule();
		this->postProcessSchedule();
		this->solvingTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timerStart).count() / 1000.0;
	}

	void SATSCCScheduler::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}

	void SATSCCScheduler::computeSCCs() {
		// use kosaraju scc finder to find sccs
		KosarajuSCC k(this->g);
		k.setQuiet(this->quiet);
		this->sccs = k.getSCCs();
	}

	void SATSCCScheduler::computeSCCSchedule() {
		// graph and resource model for scc graph
		Graph sccG;
		ResourceModel sccR;
		// insert vertices into scc graph
		for(auto scc : sccs) {
			// insert scc into tempG
			// insert vertices into tempG
			auto vertices = scc->getVerticesOfSCC();
			// skip trivial SCCs
			if (vertices.size() < 2) continue;
			for (auto v : vertices) {
				auto &newV = sccG.createVertex(v->getId());
				this->vertexToSCCVertexMap[v] = &newV;
				this->sccVertexToVertexMap[&newV] = v;
				this->sccVertexToSCCMap[&newV] = scc;
			}
			// insert edges into tempG
			for (auto e : scc->getSCCEdges()) {
				auto src = this->vertexToSCCVertexMap.at(&e->getVertexSrc());
				auto dst = this->vertexToSCCVertexMap.at(&e->getVertexDst());
				auto &newE = sccG.createEdge(*src, *dst, e->getDistance(), e->getDependencyType());
				newE.setDelay(e->getDelay());
			}
		}

		// generate resource model
		for(auto v : this->g.Vertices()) {
			// skip vertices that are only in trivial SCCs
			if(this->vertexToSCCVertexMap.find(v) == this->vertexToSCCVertexMap.end()) continue;
			// get resource of vertex
			auto res = this->resourceModel.getResource(v);
			Resource* newRes;
			// only create new resource if it does not already exist
			try {
				newRes = sccR.getResource(res->getName());
			}
			catch(HatScheT::Exception&) {
				newRes = &sccR.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
			}
			// register vertex of tempG to new resource
			sccR.registerVertex(this->vertexToSCCVertexMap.at(v),newRes);
		}

		// check if there are even any non-trivial SCCs
		if (sccG.getNumberOfVertices() == 0) {
			this->scheduleFound = true;
			this->IIFeasible = true;
			return;
		}

		// schedule graph with SAT-based scheduler
		SATScheduler s(sccG, sccR, (int)this->II);
		s.setQuiet(this->quiet);
		s.setSolverTimeout(this->solverTimeout);
		s.setEarliestStartTimes(this->earliestStartTimes);
		s.setLatestStartTimeDifferences(this->latestStartTimeDifferences);
		s.setTargetLatency(this->sccGraphMaxLat);
		s.schedule();
		this->scheduleFound = s.getScheduleFound();
		if (!this->scheduleFound) {
			this->II = -1;
			this->IIFeasible = s.getTimeouts() > 0; // can't tell if II is feasible when the solver encounters a timeout :(
			return;
		}
		// get results
		this->IIFeasible = true;
		this->relativeSchedule.clear();
		auto sccSchedule = s.getSchedule();
		for (auto &v : sccG.Vertices()) {
			auto origV = this->sccVertexToVertexMap.at(v);
			auto t = sccSchedule.at(v);
			this->relativeSchedule[origV] = t;
			auto r = this->resourceModel.getResource(origV);
			if (!r->isUnlimited()) this->MRT.at(r).at(t % (int)this->II)++;
		}
	}

	void SATSCCScheduler::computeFinalSchedule() {
		for (auto &stage : this->topologicallySortedSCCs) {
			for (auto &scc : stage) {
				if (scc->getNumberOfVertices() == 1) {
					// trivial SCC -> schedule ASAP (while respecting resource constraints)
					auto v = scc->getVerticesOfSCC().front();
					int asapTime = 0;
					for (auto &e : this->g.Edges()) {
						if (&e->getVertexDst() != v) continue;
						auto vSrc = &e->getVertexSrc();
						auto tSrc = this->startTimes.at(vSrc);
						auto t = tSrc + e->getDelay() - (this->II * e->getDistance()) + this->resourceModel.getVertexLatency(vSrc);
						if (t > asapTime) asapTime = t;
					}
					auto r = this->resourceModel.getResource(v);
					auto offsetTime = 0;
					if (!r->isUnlimited()) {
						auto rLim = r->getLimit();
						for (int i=0; i<this->II; i++) {
							auto slot = (asapTime + i) % (int)this->II;
							if (this->MRT.at(r).at(slot) < rLim) {
								offsetTime = i;
								this->MRT.at(r).at(slot)++;
								break;
							}
						}
					}
					this->startTimes[v] = asapTime + offsetTime;
					continue;
				}
				int maxMinOffset = 0;
				auto verticesOfSCC = scc->getVerticesOfSCC();
				for (auto e : this->g.Edges()) {
					if (std::find(verticesOfSCC.begin(), verticesOfSCC.end(), &e->getVertexDst()) == verticesOfSCC.end()) {
						// skip edges that do not have an SCC vertex as sink
						continue;
					}
					if (std::find(verticesOfSCC.begin(), verticesOfSCC.end(), &e->getVertexSrc()) != verticesOfSCC.end()) {
						// skip edges that have an SCC vertex as source
						continue;
					}
					auto vSrc = &e->getVertexSrc();
					auto tSrc = this->startTimes.at(vSrc);
					auto distance = e->getDistance();
					auto delay = e->getDelay();
					auto minOffset = (int) std::ceil(((double) tSrc + this->resourceModel.getVertexLatency(vSrc) - delay) / ((double)this->II)) - distance;
					minOffset = std::max(minOffset, 0); // do not let the minimum offset be negative to prevent negative starting times
					maxMinOffset = std::max(maxMinOffset, minOffset);
				}
				// offset every vertex by the minimum offset
				for (auto vSrc : verticesOfSCC) {
					/*
					auto vSCCSrc = this->vertexToSCCVertexMap.at(vSrc);
					auto tSrc = (int)this->relativeSchedule.at(vSCCSrc) + (maxMinOffset * this->II);
					 */
					auto tSrc = (int)this->relativeSchedule.at(vSrc) + (maxMinOffset * this->II);
					this->startTimes[vSrc] = tSrc;
				}
			}
		}
	}

	bool SATSCCScheduler::getIIFeasible() const {
		return this->IIFeasible;
	}

	void SATSCCScheduler::orderSCCs() {
// topologically sort sccs based on an asap schedule
		// create resource model
		ResourceModel sccRm;
		auto res = &sccRm.makeResource("res",UNLIMITED,1);
		// create graph
		Graph sccG;
		// create vertices
		std::map<const Vertex*, SCC*> vertexSCCMap;
		for (auto scc : this->sccs) {
			// create a vertex for each scc
			auto sccVertex = &sccG.createVertex(scc->getId());
			if(!this->quiet) {
				std::cout << "SATSCCScheduler::orderSCCs: created vertex with id " << sccVertex->getId() << std::endl;
			}
			sccRm.registerVertex(sccVertex, res);
			vertexSCCMap[sccVertex] = scc;
		}
		// create edges
		for (auto e : this->g.Edges()) {
			// check if e is inside one of the sccs
			bool insideSCC = false;
			for (auto scc : this->sccs) {
				auto sccEdges = scc->getSCCEdges();
				insideSCC = (std::find(sccEdges.begin(), sccEdges.end(), e) != sccEdges.end());
				if (insideSCC) break;
			}
			if (insideSCC) continue;
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			int srcSCCId = -1;
			int dstSCCId = -1;
			// find src and dst scc id
			for (auto scc : this->sccs) {
				auto sccVertices = scc->getVerticesOfSCC();
				for (auto v : sccVertices) {
					if (srcSCCId < 0 and v == vSrc) {
						srcSCCId = scc->getId();
						if (!this->quiet) {
							std::cout << "SATSCCScheduler::orderSCCs: found src vertex '" << v->getName() << "' in SCC " << srcSCCId << std::endl;
						}
					}
					if (dstSCCId < 0 and v == vDst) {
						dstSCCId = scc->getId();
						if (!this->quiet) {
							std::cout << "SATSCCScheduler::orderSCCs: found dst vertex '" << v->getName() << "' in SCC " << dstSCCId << std::endl;
						}
					}
					if (srcSCCId >= 0 and dstSCCId >= 0) break;
				}
				if (srcSCCId >= 0 and dstSCCId >= 0) break;
			}
			if (srcSCCId < 0 or dstSCCId < 0) {
				throw HatScheT::Exception("Failed to find source or destination vertex of edge inside any of the SCCs - this must be a bug!");
			}
			// create edge
			if (!this->quiet) {
				std::cout << "SATSCCScheduler::orderSCCs: src scc id: " << srcSCCId << std::endl;
				std::cout << "SATSCCScheduler::orderSCCs: dst scc id: " << dstSCCId << std::endl;
			}
			auto &sccSrc = sccG.getVertexById(srcSCCId);
			auto &sccDst = sccG.getVertexById(dstSCCId);
			sccG.createEdge(sccSrc, sccDst, 0);
		}
		// asap schedule
		ASAPScheduler asap(sccG, sccRm);
		asap.schedule();
		// get order
		auto sccSchedule = asap.getSchedule();
		this->topologicallySortedSCCs.resize(asap.getScheduleLength());
		for (auto it : sccSchedule) {
			this->topologicallySortedSCCs[it.second].emplace_back(vertexSCCMap[it.first]);
		}
	}

	SATSCCScheduler::~SATSCCScheduler() {
		for (auto &it : this->sccs) {
			delete it;
		}
	}

	void SATSCCScheduler::postProcessSchedule() {
		double minTime = std::numeric_limits<double>::infinity();
		for (auto &it : this->startTimes) {
			minTime = std::min(minTime, (double)it.second);
		}
		if (minTime == 0) {
			// do nothing if earliest schedule time is 0
			return;
		}
		if (!this->quiet) {
			std::cout << "SATSCCScheduler::postProcessSchedule: detected minimum schedule time = " << minTime << std::endl;
		}
		for (auto v : this->g.Vertices()) {
			this->startTimes[v] -= (int)minTime;
		}
	}

	double SATSCCScheduler::getSolvingTime() const {
		return this->solvingTime;
	}

	void SATSCCScheduler::computeEarliestAndLatestStartTimes() {
		/* Define the following ...
		 * std::map<Vertex*, int> earliestStartTimes;
		 * std::map<Vertex*, int> latestStartTimes;
		 * std::map<Vertex*, int> latestStartTimeDifferences;
		 * std::map<SCC*, int> sccMaxLat;
		 * int sccGraphMaxLat;
		 */
		this->sccGraphMaxLat = 0;
		for (auto &scc : this->sccs) {
			auto sccVertices = scc->getVerticesOfSCC();
			auto sccEdges = scc->getSCCEdges();
			int sccLat = 0;
			// perform DFS for cycle enumeration starting at an arbitrary vertex
			// we can start at an arbitrary one because each SCC is one large cycle anyways
			auto *vStart = sccVertices.front();
			///////////////////////////////////
			// start DFS beginning at vStart //
			///////////////////////////////////
			// container to track the queue in which the graph is searched
			std::list<std::pair<Edge*, int>> queue;
			for (auto &e : sccEdges) {
				if (&e->getVertexSrc() != vStart) continue;
				queue.emplace_front(e, 0);
			}
			// container to track the current path through the graph
			std::list<Edge*> path;
			while (!queue.empty()) {
				// get next queue element
				auto nextQueueIt = queue.front();
				auto *vDst = &nextQueueIt.first->getVertexDst();
				queue.pop_front();
				// resize path according to queue element
				path.resize(nextQueueIt.second);
				// check for loop
				bool isLoop = false;
				for (auto &e : path) {
					if (&e->getVertexSrc() == vDst) {
						isLoop = true;
					}
				}
				// adjust path
				path.emplace_back(nextQueueIt.first);
				if (!this->quiet) {
					std::cout << "Current path:" << std::endl;
					for (auto &e : path) {
						std::cout << "  " << e->getVertexSrcName() << " -> " << e->getVertexDstName() << std::endl;
					}
				}
				// handle loop
				if (isLoop) {
					// we got a loop here!!
					if (!this->quiet) {
						std::cout << "SATSCCScheduler: found loop" << std::endl;
					}
					int loopDistance = 0;
					int loopLatency = 0;
					for (auto &e : path) {
						if (!this->quiet) {
							std::cout << "  edge: " << e->getVertexSrcName() << "->" << e->getVertexDstName() << " - distance " << e->getDistance() << " - delay " << e->getDelay() << std::endl;
						}
						loopDistance += e->getDistance();
						loopLatency += e->getDelay();
						loopLatency += this->resourceModel.getVertexLatency(&e->getVertexSrc());
					}
					if (!this->quiet) {
						std::cout << "  loop distance: " << loopDistance << std::endl;
						std::cout << "  loop latency: " << loopLatency << std::endl;
						std::cout << "  minII = " << loopLatency << "/" << loopDistance << " = " << (float)loopLatency / (float)loopDistance << std::endl;
					}
					// adjust max latency of this scc
					if (this->II * loopDistance > sccLat) sccLat = (int)this->II * loopDistance;
					// adjust start times
					for (auto &e : path) {
						auto *v = &e->getVertexSrc();
						this->earliestStartTimes[v] = 0; // todo: think about more sophisticated lower bound
						auto lst = (int)this->II * loopDistance - e->getDelay() - this->resourceModel.getVertexLatency(v);
						if (lst > this->latestStartTimes[v]) this->latestStartTimes[v] = lst;
					}
					// go to next element in queue
					continue;
				}
				// seems like we did not encounter a loop (yet)
				// -> push all outgoing edges into queue
				for (auto &e : sccEdges) {
					if (&e->getVertexSrc() != vDst) continue;
					queue.emplace_front(e, path.size());
				}
			}
			// set maximum latency of whole graph if needed
			this->sccMaxLat[scc] = sccLat;
			if (sccLat > this->sccGraphMaxLat) this->sccGraphMaxLat = sccLat;
		}
		for (auto &v : this->g.Vertices()) {
			// earliest start time
			if (this->earliestStartTimes.find(v) == this->earliestStartTimes.end()) this->earliestStartTimes[v] = 0;
			// latest start time
			if (this->latestStartTimes.find(v) == this->latestStartTimes.end()) this->latestStartTimes[v] = this->sccGraphMaxLat - this->resourceModel.getVertexLatency(v);
			this->latestStartTimeDifferences[v] = this->sccGraphMaxLat - this->latestStartTimes.at(v);
		}
		// adjust max latency if the user also requested a maximum latency
		if (this->maxLatencyConstraint >= 0) {
			this->sccGraphMaxLat = min(this->sccGraphMaxLat, this->maxLatencyConstraint);
		}
	}

	void SATSCCScheduler::initMRT() {
		for (auto &r : this->resourceModel.Resources()) {
			if (r->isUnlimited()) continue;
			for (int i=0; i<this->II; i++) {
				this->MRT[r][i] = 0;
			}
		}
	}
}
#endif