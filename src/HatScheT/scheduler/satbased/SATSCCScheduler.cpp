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
#include <unordered_map>
#include <ScaLP/Solver.h>
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
		this->createSCCGraphAndRM();
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
		//this->sccs = k.getSCCs();
		auto tempSCCs = k.getSCCs();
		// sort SCCs by size
		while(!tempSCCs.empty()) {
			// find largest SCC
			auto largestSCCIt = tempSCCs.begin();
			for (auto it = largestSCCIt; it != tempSCCs.end(); it++) {
				if ((*it)->getNumberOfVertices() > (*largestSCCIt)->getNumberOfVertices()) largestSCCIt = it;
			}
			// put it into this->sccs
			this->sccs.emplace_back(*largestSCCIt);
			// delete it from tempSCCs
			tempSCCs.erase(largestSCCIt);
		}
		// sanity check
		/*
		for (auto &e : this->g.Edges()) {
			if (!e->isDataEdge()) {
				if (e->getDelay() < 1) {
					std::cout << "Found weird chaining edge with delay "+std::to_string(e->getDelay())+": "+e->getVertexSrcName()+" -> "+e->getVertexDstName() << std::endl;
					//throw Exception("Found weird chaining edge with delay "+std::to_string(e->getDelay())+": "+e->getVertexSrcName()+" -> "+e->getVertexDstName());
				}
				continue; // only check data edges...
			}
			auto* vSrc = &e->getVertexSrc();
			auto* vDst = &e->getVertexDst();
			bool shouldBeInSCC = false;
			int shouldBeInSCCIdx = -1;
			bool foundEdge = false;
			for (auto &scc : this->sccs) {
				auto sccVertices = scc->getVerticesOfSCC();
				auto sccEdges = scc->getSCCEdges();
				bool srcInSCC = false;
				bool dstInSCC = false;
				for (auto &v : sccVertices) {
					if (v == vSrc) srcInSCC = true;
					if (v == vDst) dstInSCC = true;
				}
				if (!srcInSCC or !dstInSCC) continue;
				shouldBeInSCC = true;
				shouldBeInSCCIdx = scc->getId();
				std::cout << "Edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "' should be in SCC_" << shouldBeInSCCIdx << std::endl;
				for (auto &sccEdge : sccEdges) {
					if (sccEdge == e) {
						foundEdge = true;
						std::cout << "Found it" << std::endl;
						break;
					}
				}
			}
			if (shouldBeInSCC and !foundEdge) {
				std::cout << "Failed to find edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "' in SCC (should have been in SCC #" << shouldBeInSCCIdx << ")" << std::endl;
				throw Exception("Oh no!");
			}
		}
		 */
	}

	void SATSCCScheduler::computeSCCSchedule() {
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
		std::unordered_map<Vertex*, std::vector<Edge*>> outgoingEdges;
#if 1
		int sccLat;
		int loopCounter;
		std::unordered_map<Vertex*, bool> visited;
		std::list<Edge*> path;
		std::function<void(Vertex*, Vertex*, int)> dfs = [&](Vertex* v, Vertex* start, int level) -> void {
			if (visited.at(v)) {
				if (v == start) {
					// found cycle
					if (!this->quiet) {
						std::cout << "SATSCCScheduler: found loop!" << std::endl;
					}
					loopCounter++;
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
					auto maxLoopLength = loopDistance * (int)this->II;
					for (auto &e : path) {
						auto *vSrc = &e->getVertexSrc();
						auto sccV = this->vertexToSCCVertexMap.at(vSrc);
						this->earliestStartTimes[sccV] = 0;
						this->latestStartTimes[sccV] = std::max(this->latestStartTimes[sccV], maxLoopLength - this->resourceModel.getVertexLatency(vSrc));
					}
					if (!this->quiet) {
						std::cout << "  loop distance: " << loopDistance << std::endl;
						std::cout << "  loop latency: " << loopLatency << std::endl;
						std::cout << "  minII = " << loopLatency << "/" << loopDistance << " = " << (float)loopLatency / (float)loopDistance << std::endl;
					}
					// adjust max latency of this scc
					if (maxLoopLength > sccLat) sccLat = maxLoopLength;
				}
				return;
			}
			visited.at(v) = true;
			for (auto &e : outgoingEdges.at(v)) {
				path.resize(level);
				path.emplace_back(e);
				dfs(&e->getVertexDst(), start, level+1);
			}
			visited.at(v) = false;
		};

		int maxSCCLatencyCounter = 0;

		auto getMaxSCCLatency = [&](const list<Vertex*> &sccVertices, const list<Edge*> &sccEdges) {
			// check if solution is trivial
			if (sccVertices.size() == 1 or sccEdges.empty()) {
				int maxSCCLatency = -1;
				for (auto &v : sccVertices) {
					auto l = this->resourceModel.getVertexLatency(v);
					if (maxSCCLatency < 0 or l > maxSCCLatency) maxSCCLatency = l;
				}
				return maxSCCLatency;
			}
			// use SDC schedule without resource constraints with ScaLP
			ScaLP::Solver s({"Gurobi", "CPLEX", "LPSolve", "SCIP"});
			std::unordered_map<Vertex*, ScaLP::Variable> t;
			auto supersink = ScaLP::newIntegerVariable("supersink", 0.0, ScaLP::INF());
			auto supersource = ScaLP::newIntegerVariable("supersource", -ScaLP::INF(), 0.0);
			ScaLP::status stat;
			std::map<ScaLP::Variable, double> results;
			for (auto &v : sccVertices) {
				t[v] = ScaLP::newIntegerVariable(v->getName(), 0.0, ScaLP::INF());
			}
			std::vector<Vertex*> sources;
			std::vector<Vertex*> sinks;
			// compute min latency of SCC
			/*
			for (auto &v : sccVertices) {
				auto var = t.at(v);
				auto l = this->resourceModel.getVertexLatency(v);
				s.addConstraint(supersink - var >= l);
			}
			for (auto &e : sccEdges) {
				auto vSrc = &e->getVertexSrc();
				auto vDst = &e->getVertexDst();
				auto lSrc = this->resourceModel.getVertexLatency(vSrc);
				s.addConstraint(t.at(vDst) - t.at(vSrc) >= lSrc + e->getDelay() - (e->getDistance() * this->II));
			}
			s.setObjective(ScaLP::minimize(supersink));
			stat = s.solve();
			if (stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE) {
				throw Exception("SATSCCScheduler: failed to compute min schedule length for SCC");
			}
			results = s.getResult().values;
			int minSCCLat = (int)std::round(results.at(supersink));
			if (!this->quiet) {
				std::cout << "SATSCCScheduler: min SCC latency = " << minSCCLat << std::endl;
			}
			// calculate min times
			s.reset();
			ScaLP::Term minVarSum;
			for (auto &v : sccVertices) {
				auto var = t.at(v);
				minVarSum += var;
				auto l = this->resourceModel.getVertexLatency(v);
				s.addConstraint(var <= minSCCLat-l);
			}
			for (auto &e : sccEdges) {
				auto vSrc = &e->getVertexSrc();
				auto vDst = &e->getVertexDst();
				auto lSrc = this->resourceModel.getVertexLatency(vSrc);
				s.addConstraint(t.at(vDst) - t.at(vSrc) >= lSrc + e->getDelay() - (e->getDistance() * this->II));
			}
			s.setObjective(ScaLP::minimize(minVarSum));
			stat = s.solve();
			if (stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE) {
				throw Exception("SATSCCScheduler: failed to compute min times for SCC");
			}
			results = s.getResult().values;
			std::unordered_map<Vertex*, int> minTimes;
			for (auto &v : sccVertices) {
				minTimes[v] = (int) std::round(results[t.at(v)]);
			}

			// calculate max times
			s.reset();
			ScaLP::Term maxVarSum;
			for (auto &v : sccVertices) {
				auto var = t.at(v);
				maxVarSum += var;
				auto l = this->resourceModel.getVertexLatency(v);
				s.addConstraint(var <= minSCCLat-l);
			}
			for (auto &e : sccEdges) {
				auto vSrc = &e->getVertexSrc();
				auto vDst = &e->getVertexDst();
				auto lSrc = this->resourceModel.getVertexLatency(vSrc);
				s.addConstraint(t.at(vDst) - t.at(vSrc) >= lSrc + e->getDelay() - (e->getDistance() * this->II));
			}
			s.setObjective(ScaLP::maximize(maxVarSum));
			stat = s.solve();
			if (stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE) {
				throw Exception("SATSCCScheduler: failed to compute max times for SCC");
			}
			results = s.getResult().values;
			std::unordered_map<Vertex*, int> maxTimes;
			for (auto &v : sccVertices) {
				maxTimes[v] = (int) std::round(results[t.at(v)]);
			}
			*/

			// from min and max times compute sources and sinks
			for (auto &v : sccVertices) {
				bool canBeSource = true;
				bool canBeSink = true;
				for (auto &e : sccEdges) {
					// it is not a source if it the sink of any edge with zero distance
					if (v == &e->getVertexDst() and (e->isDataEdge() and e->getDistance() == 0)) {
						canBeSource = false;
					}
					// it is not a sink if it the source of any edge with zero distance
					if (v == &e->getVertexSrc() and (e->isDataEdge() and e->getDistance() == 0)) {
						canBeSink = false;
					}
				}
				if (canBeSource) { //  and maxTimes.at(v) == 0
					sources.emplace_back(v);
					if (!this->quiet) {
						std::cout << "SATSCCScheduler: vertex '" << v->getName() << "' is a source" << std::endl;
					}
				}
				if (canBeSink) { //  and minTimes.at(v) + this->resourceModel.getVertexLatency(v) == minSCCLat
					sinks.emplace_back(v);
					if (!this->quiet) {
						std::cout << "SATSCCScheduler: vertex '" << v->getName() << "' is a sink" << std::endl;
					}
				}
			}

			// use sources and sinks to compute max SCC latency
			s.reset();
			std::unordered_map<Vertex*, ScaLP::Variable> supersourceActivators;
			ScaLP::Term sourceTerm;
			std::unordered_map<Vertex*, ScaLP::Variable> supersinkActivators;
			ScaLP::Term sinkTerm;
			double bigM = 1000000.0; // something huge... -> maybe think about something that always works
			for (auto &v : sources) {
				auto actVar = ScaLP::newBinaryVariable("source_var_"+v->getName());
				supersourceActivators[v] = actVar;
				sourceTerm += actVar;
				auto var = t.at(v);
				s.addConstraint((bigM * (1-actVar)) + supersource - var >= 0);
			}
			s.addConstraint(sourceTerm >= 1);
			for (auto &v : sinks) {
				auto actVar = ScaLP::newBinaryVariable("sink_var_"+v->getName());
				supersinkActivators[v] = actVar;
				sinkTerm += actVar;
				auto var = t.at(v);
				auto l = this->resourceModel.getVertexLatency(v);
				s.addConstraint(-(bigM * (1-actVar)) + supersink - var <= l);
			}
			s.addConstraint(sinkTerm >= 1);
			for (auto &e : sccEdges) {
				auto vSrc = &e->getVertexSrc();
				auto vDst = &e->getVertexDst();
				auto lSrc = this->resourceModel.getVertexLatency(vSrc);
				s.addConstraint(t.at(vDst) - t.at(vSrc) >= lSrc + e->getDelay() - (e->getDistance() * this->II));
			}
			s.setObjective(ScaLP::maximize(supersink - supersource));
			stat = s.solve();
			if (stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE) {
				throw Exception("SATSCCScheduler: failed to compute max schedule length for SCC (status: "+ScaLP::showStatus(stat)+")");
			}
			results = s.getResult().values;
			int maxSCCLat = 0;
			for (auto &v : sccVertices) {
				auto var = t.at(v);
				auto l = this->resourceModel.getVertexLatency(v);
				auto tV = round(results.at(var)) + l;
				if (tV > maxSCCLat) maxSCCLat = tV;
			}
			if (maxSCCLatencyCounter < this->II-1) {
				maxSCCLatencyCounter++;
			}

			//return 7;
			return maxSCCLat + maxSCCLatencyCounter;
		};

		this->sccGraphMaxLat = 0;
		for (auto &scc : this->sccs) {
			// get scc vertices and edges
			auto sccVertices = scc->getVerticesOfSCC();
			auto sccEdges = scc->getSCCEdges();
			// clear containers
			visited.clear();
			outgoingEdges.clear();
			// init visited container
			for (auto &v : sccVertices) {
				if (!this->quiet) {
					//std::cout << "  " << v->getName() << std::endl;
				}
				visited[v] = false;
				outgoingEdges[v] = {};
			}
			// init outgoing edges container
			int totalDistance = 0;
			int dataEdgeCounter = 0;
			for (auto &e : sccEdges) {
				if (!e->isDataEdge()) continue; // ignore chaining edges
				if (!this->quiet) {
					//std::cout << "  " << e->getVertexSrcName() << " -> " << e->getVertexDstName() << std::endl;
				}
				dataEdgeCounter++;
				outgoingEdges[&e->getVertexSrc()].emplace_back(e);
				totalDistance += e->getDistance();
			}
			if (!this->quiet) {
				std::cout << "SATSCCScheduler: start enumerating loops of SCC_" << scc->getId() << " with "
									<< sccVertices.size() << " vertices and " << sccEdges.size() << " edges (" << dataEdgeCounter
									<< " data edges)" << std::endl;
			}
#if 1 // 1: max latency; 0: DFS
			sccLat = getMaxSCCLatency(sccVertices, sccEdges);
			std::cout << "  SCC latency = " << sccLat << " (dTotal*II = " << totalDistance << "*" << this->II << ")" << std::endl;
			for (auto &v : sccVertices) {
				if (this->vertexToSCCVertexMap.find(v) == this->vertexToSCCVertexMap.end()) continue; // skip trivial SCCs
				auto sccV = this->vertexToSCCVertexMap.at(v);
				this->earliestStartTimes[sccV] = 0;
				this->latestStartTimes[sccV] = sccLat - this->resourceModel.getVertexLatency(v);
				this->latestStartTimeDifferences[sccV] = this->resourceModel.getVertexLatency(v);
			}
#else
			// perform DFS starting at each vertex of the SCC
			loopCounter = 0;
			for (auto &v : sccVertices) {
				if (!this->quiet) {
					std::cout << "  starting DFS at vertex " << v->getName() << std::endl;
				}
				dfs(v, v, 0);
			}
			if (!this->quiet) {
				std::cout << "  found " << loopCounter << " loops" << std::endl;
			}
#endif
			// set maximum latency of the whole graph if needed
			this->sccMaxLat[scc] = sccLat;
			if (sccLat > this->sccGraphMaxLat) this->sccGraphMaxLat = sccLat;
		}

#else
		this->sccGraphMaxLat = 0;
		for (auto &scc : this->sccs) {
			auto sccVertices = scc->getVerticesOfSCC();
			auto sccEdges = scc->getSCCEdges();
			int sccLat = 0;
			// compute outgoing data edges of each vertex
			int dataEdgeCounter = 0;
			for (auto &e : sccEdges) {
				if ((!e->isDataEdge()) and (e->getDelay() > 0)) continue; // ignore chaining edges
				outgoingEdges[&e->getVertexSrc()].emplace_back(e);
				dataEdgeCounter++;
			}
			if (!this->quiet) {
				std::cout << "starting DFS on SCC_" << scc->getId() << " with " << sccVertices.size() << " vertices and "
				  << sccEdges.size() << " edges (" << dataEdgeCounter << " data edges)" << std::endl;
			}
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
				std::vector<Edge*> loopPath(path.size()+1);
				int loopPathSize = 0;
				for (auto &e : path) {
					if (&e->getVertexSrc() == vDst) {
						isLoop = true;
					}
					if (isLoop) {
						loopPath[loopPathSize] = e;
						loopPathSize++;
					}
				}
				// adjust path
				path.emplace_back(nextQueueIt.first);
				if (!this->quiet) {
					for (auto &e : path) {
						std::cout << "  " << e->getVertexSrcName() << " -> " << e->getVertexDstName() << std::endl;
					}
				}
				// handle loop
				if (isLoop) {
					loopPath[loopPathSize] = nextQueueIt.first;
					loopPathSize++;
					loopPath.resize(loopPathSize);
					// we got a loop here!!
					if (!this->quiet) {
						std::cout << "SATSCCScheduler: found loop around " << vDst->getName() << " (loop path size: " << loopPathSize << ")" << std::endl;
					}
					int loopDistance = 0;
					int loopLatency = 0;
					for (auto &e : loopPath) {
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
					for (auto &e : loopPath) {
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
				//for (auto &e : sccEdges) {
				for (auto &e : outgoingEdges[vDst]) {
					//if (&e->getVertexSrc() != vDst) continue;
					queue.emplace_front(e, path.size());
				}
			}
			// set maximum latency of whole graph if needed
			this->sccMaxLat[scc] = sccLat;
			if (sccLat > this->sccGraphMaxLat) this->sccGraphMaxLat = sccLat;
		}

#endif
		// adjust max latency if the user also requested a maximum latency
		if (this->maxLatencyConstraint >= 0) {
			this->sccGraphMaxLat = min(this->sccGraphMaxLat, this->maxLatencyConstraint);
		}
		// define remaining stuff
		for (auto &v : this->g.Vertices()) {
			if (this->vertexToSCCVertexMap.find(v) == this->vertexToSCCVertexMap.end()) continue; // skip non-SCC vertices
			auto sccV = this->vertexToSCCVertexMap.at(v);
			// earliest start time
			if (this->earliestStartTimes.find(sccV) == this->earliestStartTimes.end()) this->earliestStartTimes[sccV] = 0;
			// latest start time
			if (this->latestStartTimes.find(sccV) == this->latestStartTimes.end()) this->latestStartTimes[sccV] = this->sccGraphMaxLat - this->resourceModel.getVertexLatency(v);
			this->latestStartTimeDifferences[sccV] = this->sccGraphMaxLat - this->latestStartTimes.at(sccV);
			if (!this->quiet) {
				std::cout << "SATSCCScheduler: vertex '" << sccV->getName() << "':" << std::endl;
				std::cout << "  earliest start time: " << this->earliestStartTimes.at(sccV) << std::endl;
				std::cout << "  latest start time: " << this->latestStartTimes.at(sccV) << std::endl;
				std::cout << "  latest start time diff: " << this->latestStartTimeDifferences.at(sccV) << std::endl;
			}
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

	void SATSCCScheduler::createSCCGraphAndRM() {
		// insert vertices into scc graph
		for(auto scc : this->sccs) {
			// insert scc into tempG
			// insert vertices into tempG
			auto vertices = scc->getVerticesOfSCC();
			// skip trivial SCCs
			if (vertices.size() < 2) continue;
			for (auto v : vertices) {
				auto &newV = this->sccG.createVertex(v->getId());
				this->vertexToSCCVertexMap[v] = &newV;
				this->sccVertexToVertexMap[&newV] = v;
				this->sccVertexToSCCMap[&newV] = scc;
			}
			// insert edges into tempG
			for (auto e : scc->getSCCEdges()) {
				auto src = this->vertexToSCCVertexMap.at(&e->getVertexSrc());
				auto dst = this->vertexToSCCVertexMap.at(&e->getVertexDst());
				auto &newE = this->sccG.createEdge(*src, *dst, e->getDistance(), e->getDependencyType());
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
				newRes = this->sccR.getResource(res->getName());
			}
			catch(HatScheT::Exception&) {
				newRes = &this->sccR.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
			}
			// register vertex of tempG to new resource
			this->sccR.registerVertex(this->vertexToSCCVertexMap.at(v),newRes);
		}
	}
}
#endif