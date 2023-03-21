//
// Created by nfiege on 6/27/22.
//

#include "SATSCCRatIIScheduler.h"
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <HatScheT/scheduler/satbased/SATRatIIScheduler.h>
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/utility/Utility.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>
#include <unordered_map>
#ifdef USE_CADICAL
#ifdef USE_SCALP
#include <ScaLP/Solver.h>
namespace HatScheT {

	SATSCCRatIIScheduler::SATSCCRatIIScheduler(Graph &g, ResourceModel &resourceModel, int M, int S) :
		RationalIISchedulerLayer(g, resourceModel, M, S), IIFeasible(true), solvingTime(-1.0) {
	}

	void SATSCCRatIIScheduler::scheduleIteration() {
		this->solvingTime = 0.0;
		auto timerStart = std::chrono::steady_clock::now();
		this->initiationIntervals = RationalIISchedulerLayer::getOptimalInitiationIntervalSequence(this->samples,this->modulo,this->quiet);
		this->latencySequence = RationalIISchedulerLayer::getLatencySequenceFromInitiationIntervals(this->initiationIntervals,this->modulo);
		this->relativeSchedule.clear();
		this->relativeSchedule.resize(this->samples);
		this->startTimesVector.clear();
		this->startTimesVector.resize(this->samples);
		this->calcDeltaMins();
		this->initMRT();
		this->computeSCCs();
		this->createSCCGraphsAndRMs();
		this->computeEarliestAndLatestStartTimes();
		this->computeComplexSCCSchedule();
		if (!this->scheduleFound) {
			this->solvingTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timerStart).count() / 1000.0;
			return;
		}
		this->computeBasicSCCSchedules();
		this->orderSCCs();
		this->computeFinalSchedule();
		this->postProcessSchedule();
		if (!this->startTimesVector.empty()) this->startTimes = this->startTimesVector.at(0);
		this->solvingTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timerStart).count() / 1000.0;
	}

	void SATSCCRatIIScheduler::computeSCCs() {
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
	}

	void SATSCCRatIIScheduler::computeComplexSCCSchedule() {
		// check if there are even any non-trivial SCCs
		if (this->complexSCCG.getNumberOfVertices() == 0) {
			this->scheduleFound = true;
			this->IIFeasible = true;
			return;
		}
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler: start computing complex SCC schedule now" << std::endl;
		}

		// schedule graph with SAT-based scheduler
		SATRatIIScheduler s(this->complexSCCG, this->complexSCCR, this->modulo, this->samples);
		s.setQuiet(this->quiet);
		s.setSolverTimeout(this->solverTimeout);
		s.setEarliestStartTimes(this->earliestStartTimes);
		s.setLatestStartTimeDifferences(this->latestStartTimeDifferences);
		s.setTargetLatency(this->sccGraphMaxLat);
		s.disableVerifier();
		s.schedule();
		this->scheduleFound = s.getScheduleFound();
		if (!this->scheduleFound) {
			if (!this->quiet) {
				std::cout << "SATSCCRatIIScheduler: failed to find schedule for II=" << this->II << std::endl;
			}
			this->II = -1;
			this->IIFeasible = s.getTimeouts() > 0; // can't tell if II is feasible when the solver encounters a timeout :(
			return;
		}
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler: found schedule for II=" << this->II << std::endl;
		}
		// get results
		this->IIFeasible = true;
		this->relativeSchedule.clear();
		this->relativeSchedule.resize(this->samples);
		auto sccSchedule = s.getStartTimeVector();
		for (auto &v : this->complexSCCG.Vertices()) {
			auto origV = this->sccVertexToVertexMap.at(v);
			for (auto i=0; i<this->samples; i++) {
				auto t = sccSchedule.at(i).at(v);
				this->relativeSchedule.at(i)[origV] = t;
				auto r = this->resourceModel.getResource(origV);
				if (!r->isUnlimited()) this->MRT.at(r).at(t % (int)this->modulo)++;
			}
		}
	}

	void SATSCCRatIIScheduler::computeFinalSchedule() {
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler: start computing final schedule" << std::endl;
		}
		for (auto &stage : this->topologicallySortedSCCs) {
			for (auto &scc : stage) {
				if (!this->quiet) {
					std::cout << "SATSCCRatIIScheduler: next SCC contains following vertices:" << std::endl;
					for (auto &it : scc->getVerticesOfSCC()) {
						std::cout << "  " << it->getName() << std::endl;
					}
				}
				auto sccType = scc->getSccType(&this->resourceModel);
				switch (sccType) {
					case scctype::trivial: {
						if (!this->quiet) {
							std::cout << "SCC type: trivial" << std::endl;
						}
						for (int s=0; s<this->samples; s++) {
							// schedule ASAP (while respecting resource constraints)
							auto v = scc->getVerticesOfSCC().front();
							int asapTime = 0;
							for (auto &e : this->g.Edges()) {
								if (&e->getVertexDst() != v) continue;
								auto vSrc = &e->getVertexSrc();
								auto sampleIdxAndOffset = Utility::getSampleIndexAndOffset(e->getDistance(), s, this->samples, this->modulo);
								auto tSrc = this->startTimesVector.at(sampleIdxAndOffset.first).at(vSrc);
								auto t = tSrc + e->getDelay() - sampleIdxAndOffset.second + this->resourceModel.getVertexLatency(vSrc);
								if (t > asapTime) asapTime = t;
							}
							auto r = this->resourceModel.getResource(v);
							auto offsetTime = 0;
							if (!r->isUnlimited()) {
								auto rLim = r->getLimit();
								for (int i=0; i<this->modulo; i++) {
									auto slot = (asapTime + i) % this->modulo;
									if (this->MRT.at(r).at(slot) < rLim) {
										offsetTime = i;
										this->MRT.at(r).at(slot)++;
										break;
									}
								}
							}
							this->startTimesVector.at(s)[v] = asapTime + offsetTime;
						}
						break;
					}
					case scctype::basic: {
						if (!this->quiet) {
							std::cout << "SCC type: basic" << std::endl;
						}
						auto verticesOfSCC = scc->getVerticesOfSCC();
						for (int s=0; s<this->samples; s++) {
							int maxMinOffset = 0;
							for (auto e : this->g.Edges()) {
								if (std::find(verticesOfSCC.begin(), verticesOfSCC.end(), &e->getVertexDst()) == verticesOfSCC.end()) {
									// skip edges that do not have an SCC vertex as sink
									continue;
								}
								if (std::find(verticesOfSCC.begin(), verticesOfSCC.end(), &e->getVertexSrc()) != verticesOfSCC.end()) {
									// skip edges that have an SCC vertex as source
									continue;
								}
								auto sampleIdxAndOffset = Utility::getSampleIndexAndOffset(e->getDistance(), s, this->samples, this->modulo);
								auto vSrc = &e->getVertexSrc();
								auto vDst = &e->getVertexDst();
								auto lSrc = this->resourceModel.getVertexLatency(vSrc);
								auto tSrc = this->startTimesVector.at(sampleIdxAndOffset.first).at(vSrc);
								auto tDstRel = this->relativeSchedule.at(s).at(vDst);
								auto delay = e->getDelay();
								//auto minOffset = (int) std::ceil(((double) tSrc - tDstRel + lSrc + delay - int(this->deltaMins.at(distance))) / (this->II));
								auto minOffset = (int) std::ceil(((double) tSrc - tDstRel + lSrc + delay - sampleIdxAndOffset.second) / (this->modulo));
								minOffset = std::max(minOffset, 0); // do not let the minimum offset be negative to prevent negative starting times
								maxMinOffset = std::max(maxMinOffset, minOffset);
							}
							// offset every vertex by the minimum offset
							for (auto vSrc : verticesOfSCC) {
								auto tSrc = (int)this->relativeSchedule.at(s).at(vSrc) + maxMinOffset;
								this->startTimesVector.at(s)[vSrc] = tSrc;
							}
						}
						break;
					}
					case scctype::complex: {
						if (!this->quiet) {
							std::cout << "SCC type: complex" << std::endl;
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
							auto vDst = &e->getVertexDst();
							auto lSrc = this->resourceModel.getVertexLatency(vSrc);
							auto distance = e->getDistance();
							auto delay = e->getDelay();

							for (int s=0; s<this->samples; s++) {
								auto sampleIdxAndOffset = Utility::getSampleIndexAndOffset(distance, s, this->samples, this->modulo);
								auto tSrc = this->startTimesVector.at(sampleIdxAndOffset.first).at(vSrc);
								auto tDstRel = this->relativeSchedule.at(s).at(vDst);
								auto minOffset = (int) std::ceil(((double) tSrc - tDstRel + lSrc + delay - sampleIdxAndOffset.second) / ((double)this->modulo));
								maxMinOffset = std::max(maxMinOffset, minOffset);
							}
						}
						// offset every vertex by the minimum offset
						for (auto v : verticesOfSCC) {
							for (int s=0; s<this->samples; s++) {
								auto t = (int)this->relativeSchedule.at(s).at(v) + (maxMinOffset * this->modulo);
								this->startTimesVector.at(s)[v] = t;
							}
						}
						break;
					}
					default: {
						throw HatScheT::Exception("SATSCCRatIIScheduler: detected unknown SCC type while computing final schedule");
					}
				}
			}
		}
	}

	bool SATSCCRatIIScheduler::getIIFeasible() const {
		return this->IIFeasible;
	}

	void SATSCCRatIIScheduler::orderSCCs() {
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler: start topologically sorting SCCs" << std::endl;
		}
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
				std::cout << "SATSCCRatIIScheduler::orderSCCs: created vertex with id " << sccVertex->getId() << std::endl;
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
							std::cout << "SATSCCRatIIScheduler::orderSCCs: found src vertex '" << v->getName() << "' in SCC " << srcSCCId << std::endl;
						}
					}
					if (dstSCCId < 0 and v == vDst) {
						dstSCCId = scc->getId();
						if (!this->quiet) {
							std::cout << "SATSCCRatIIScheduler::orderSCCs: found dst vertex '" << v->getName() << "' in SCC " << dstSCCId << std::endl;
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
				std::cout << "SATSCCRatIIScheduler::orderSCCs: src scc id: " << srcSCCId << std::endl;
				std::cout << "SATSCCRatIIScheduler::orderSCCs: dst scc id: " << dstSCCId << std::endl;
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
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler::orderSCCs: finished" << std::endl;
			int i=0;
			std::cout << "num stages = " << this->topologicallySortedSCCs.size() << std::endl;
			for (auto &it : this->topologicallySortedSCCs) {
				std::cout << "  stage " << i++ << std::endl;
				for (auto &it2 : it) {
					std::cout << "    SCC " << it2->getId() << std::endl;
				}
			}
		}
	}

	SATSCCRatIIScheduler::~SATSCCRatIIScheduler() {
		for (auto &it : this->sccs) {
			delete it;
		}
	}

	void SATSCCRatIIScheduler::postProcessSchedule() {
		double minTime = std::numeric_limits<double>::infinity();
		for (auto &it : this->startTimesVector) {
			for (auto &it2 : it) {
				minTime = std::min(minTime, (double) it2.second);
			}
		}
		if (minTime == 0) {
			// do nothing if earliest schedule time is 0
			return;
		}
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler::postProcessSchedule: detected minimum schedule time = " << minTime << std::endl;
		}
		for (auto v : this->g.Vertices()) {
			for (auto s=0; s<this->samples; s++) {
				this->startTimesVector.at(s)[v] -= (int)minTime;
			}
		}
	}

	double SATSCCRatIIScheduler::getSolvingTime() const {
		return this->solvingTime;
	}

	void SATSCCRatIIScheduler::computeEarliestAndLatestStartTimes() {
		/* Define the following ...
		 * std::map<Vertex*, int> earliestStartTimes;
		 * std::map<Vertex*, int> latestStartTimes;
		 * std::map<Vertex*, int> latestStartTimeDifferences;
		 * std::map<SCC*, int> sccMaxLat;
		 * int sccGraphMaxLat;
		 */
		std::unordered_map<Vertex*, std::vector<Edge*>> outgoingEdges;
		int sccLat;

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
			// from min and max times compute sources and sinks
			for (auto &v : sccVertices) {
				bool canBeSource = true;
				bool canBeSink = true;
				for (auto &e : sccEdges) {
					// it is not a source if it the sink of any edge with zero distance
					if (v == &e->getVertexDst() and (e->isDataEdge() and int(this->deltaMins[e->getDistance()]) == 0)) {
						canBeSource = false;
					}
					// it is not a sink if it the source of any edge with zero distance
					if (v == &e->getVertexSrc() and (e->isDataEdge() and int(this->deltaMins[e->getDistance()]) == 0)) {
						canBeSink = false;
					}
				}
				if (canBeSource) { //  and maxTimes.at(v) == 0
					sources.emplace_back(v);
					if (!this->quiet) {
						std::cout << "SATSCCRatIIScheduler: vertex '" << v->getName() << "' is a source" << std::endl;
					}
				}
				if (canBeSink) { //  and minTimes.at(v) + this->resourceModel.getVertexLatency(v) == minSCCLat
					sinks.emplace_back(v);
					if (!this->quiet) {
						std::cout << "SATSCCRatIIScheduler: vertex '" << v->getName() << "' is a sink" << std::endl;
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
				s.addConstraint(t.at(vDst) - t.at(vSrc) >= lSrc + e->getDelay() - int(this->deltaMins[e->getDistance()]));
			}
			s.setObjective(ScaLP::maximize(supersink - supersource));
			stat = s.solve();
			if (stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE) {
				throw Exception("SATSCCRatIIScheduler: failed to compute max schedule length for SCC (status: "+ScaLP::showStatus(stat)+")");
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

			return maxSCCLat + maxSCCLatencyCounter;
		};

		this->sccGraphMaxLat = 0;
		for (auto &scc : this->sccs) {
			// get scc vertices and edges
			auto sccVertices = scc->getVerticesOfSCC();
			auto sccEdges = scc->getSCCEdges();
			// clear containers
			outgoingEdges.clear();
			// init visited container
			for (auto &v : sccVertices) {
				outgoingEdges[v] = {};
			}
			// init outgoing edges container
			int totalDistance = 0;
			int dataEdgeCounter = 0;
			for (auto &e : sccEdges) {
				if (!e->isDataEdge()) continue; // ignore chaining edges
				dataEdgeCounter++;
				outgoingEdges[&e->getVertexSrc()].emplace_back(e);
			}
			if (!this->quiet) {
				std::cout << "SATSCCRatIIScheduler: start enumerating loops of SCC_" << scc->getId() << " with "
									<< sccVertices.size() << " vertices and " << sccEdges.size() << " edges (" << dataEdgeCounter
									<< " data edges)" << std::endl;
			}
			sccLat = getMaxSCCLatency(sccVertices, sccEdges);
			auto minMaxSCCStartTimes = this->getMinMaxSCCStartTimes(sccVertices, sccEdges, sccLat);
			for (auto &v : sccVertices) {
				if (this->vertexToSCCVertexMap.find(v) == this->vertexToSCCVertexMap.end()) continue; // skip trivial SCCs
				auto sccV = this->vertexToSCCVertexMap.at(v);
				this->earliestStartTimes[sccV] = minMaxSCCStartTimes.first.at(v);
				this->latestStartTimes[sccV] = minMaxSCCStartTimes.second.at(v);
				this->latestStartTimeDifferences[sccV] = sccLat - minMaxSCCStartTimes.second.at(v);
			}
			// set maximum latency of the whole graph if needed
			this->sccMaxLat[scc] = sccLat;
			if (sccLat > this->sccGraphMaxLat) this->sccGraphMaxLat = sccLat;
		}

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
				std::cout << "SATSCCRatIIScheduler: vertex '" << sccV->getName() << "':" << std::endl;
				std::cout << "  earliest start time: " << this->earliestStartTimes.at(sccV) << std::endl;
				std::cout << "  latest start time: " << this->latestStartTimes.at(sccV) << std::endl;
				std::cout << "  latest start time diff: " << this->latestStartTimeDifferences.at(sccV) << std::endl;
			}
		}
	}

	void SATSCCRatIIScheduler::initMRT() {
		for (auto &r : this->resourceModel.Resources()) {
			if (r->isUnlimited()) continue;
			for (int i=0; i<this->modulo; i++) {
				this->MRT[r][i] = 0;
			}
		}
	}

	void SATSCCRatIIScheduler::createSCCGraphsAndRMs() {
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler: creating SCC graphs and resource models now" << std::endl;
		}
		this->numBasicSCCs = 0;
		// insert vertices into scc graph
		for(auto scc : this->sccs) {
			auto sccType = scc->getSccType(&this->resourceModel);
			switch (sccType) {
				case scctype::trivial: {
					if (!this->quiet) {
						std::cout << "SATSCCRatIIScheduler: skipping trivial SCC" << std::endl;
					}
					continue;
				}
				case scctype::basic: {
					if (!this->quiet) {
						std::cout << "SATSCCRatIIScheduler: found basic SCC" << std::endl;
					}
					this->numBasicSCCs++;
					this->basicSCCG.emplace_back(std::make_shared<Graph>());
					this->basicSCCR.emplace_back(std::make_shared<ResourceModel>());
					auto g = this->basicSCCG.back();
					auto rm = this->basicSCCR.back();
					auto vertices = scc->getVerticesOfSCC();
					// put vertices into graph
					for (auto v : vertices) {
						auto &newV = g->createVertex(v->getId());
						this->vertexToSCCVertexMap[v] = &newV;
						this->sccVertexToVertexMap[&newV] =v;
					}
					// put edges into graph
					for (auto &e : scc->getSCCEdges()) {
						auto &src = this->vertexToSCCVertexMap.at(&e->getVertexSrc());
						auto &dst = this->vertexToSCCVertexMap.at(&e->getVertexDst());
						auto &newE = g->createEdge(*src, *dst, e->getDistance(), e->getDependencyType());
						newE.setDelay(e->getDelay());
					}
					// handle resources
					for (auto &v : vertices) {
						auto res = this->resourceModel.getResource(v);
						Resource* newRes;
						// only create new resource if it does not already exist
						try {
							newRes = rm->getResource(res->getName());
						}
						catch(HatScheT::Exception&) {
							newRes = &rm->makeResource(res->getName(), res->getLimit(), res->getLatency(), res->getBlockingTime());
						}
						// register vertex of tempG to new resource
						rm->registerVertex(this->vertexToSCCVertexMap.at(v), newRes);
					}
					break;
				}
				case scctype::complex: {
					if (!this->quiet) {
						std::cout << "SATSCCRatIIScheduler: found complex SCC" << std::endl;
					}
					// insert vertices into scc graph
					auto vertices = scc->getVerticesOfSCC();
					for (auto v : vertices) {
						auto &newV = this->complexSCCG.createVertex(v->getId());
						this->vertexToSCCVertexMap[v] = &newV;
						this->sccVertexToVertexMap[&newV] = v;
					}
					// insert edges into scc graph
					for (auto &e : scc->getSCCEdges()) {
						auto &src = this->vertexToSCCVertexMap.at(&e->getVertexSrc());
						auto &dst = this->vertexToSCCVertexMap.at(&e->getVertexDst());
						auto &newE = this->complexSCCG.createEdge(*src, *dst, e->getDistance(), e->getDependencyType());
						newE.setDelay(e->getDelay());
					}
					// handle resources
					for (auto &v : vertices) {
						auto res = this->resourceModel.getResource(v);
						Resource* newRes;
						// only create new resource if it does not already exist
						try {
							newRes = this->complexSCCR.getResource(res->getName());
						}
						catch(HatScheT::Exception&) {
							newRes = &this->complexSCCR.makeResource(res->getName(), res->getLimit(), res->getLatency(), res->getBlockingTime());
						}
						// register vertex of tempG to new resource
						this->complexSCCR.registerVertex(this->vertexToSCCVertexMap.at(v), newRes);
					}
					break;
				}
				default: {
					throw Exception("SATSCCRatIIScheduler: unknown SCC type");
				}
			}
		}
	}

	std::pair<std::map<Vertex *, int>, std::map<Vertex *, int>>
	SATSCCRatIIScheduler::getMinMaxSCCStartTimes(const list<Vertex *> &sccVertices, const list<Edge *> &sccEdges,
																					const int &maxLat) {
		ScaLP::Solver s({"Gurobi", "CPLEX", "SCIP", "LPSolve"});
		std::map<Vertex*, ScaLP::Variable> vars;
		ScaLP::Term varSum;
		for (auto &v : sccVertices) {
			auto var = ScaLP::newIntegerVariable(v->getName(), 0.0, maxLat - this->resourceModel.getVertexLatency(v));
			varSum += var;
			vars[v] = var;
		}
		for (auto &e : sccEdges) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			s.addConstraint(vars.at(vDst) + int(this->deltaMins[e->getDistance()]) - (vars.at(vSrc) + lSrc + e->getDelay()) >= 0.0);
		}
		s.setObjective(ScaLP::minimize(varSum));
		s.solve();
		auto results = s.getResult().values;
		std::map<Vertex*, int> earliestSCCStartTimes;
		for (auto &v : sccVertices) {
			earliestSCCStartTimes[v] = (int)std::round(results.at(vars.at(v)));
		}
		s.setObjective(ScaLP::maximize(varSum));
		s.solve();
		results = s.getResult().values;
		std::map<Vertex*, int> latestSCCStartTimes;
		for (auto &v : sccVertices) {
			latestSCCStartTimes[v] = (int)std::round(results.at(vars.at(v)));
		}
		// return results
		return {earliestSCCStartTimes, latestSCCStartTimes};
	}

	void SATSCCRatIIScheduler::computeBasicSCCSchedules() {
		if (!this->quiet) {
			std::cout << "SATSCCRatIIScheduler: start computing basic SCC schedule now" << std::endl;
		}
		for (int i=0; i<this->numBasicSCCs; i++) {
			auto g = this->basicSCCG.at(i);
			auto rm = this->basicSCCR.at(i);
			if (!this->quiet) {
				std::cout << "SATSCCRatIIScheduler: found basic SCC with following vertices" << std::endl;
			}
			std::unordered_map<Vertex*, std::vector<ScaLP::Variable>> vars;
			ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink", 0.0, ScaLP::INF());
			auto solver = ScaLP::Solver({"Gurobi", "CPLEX", "SCIP", "LPSolve"});
			for (auto &v : g->Vertices()) {
				if (!this->quiet) {
					std::cout << "  " << v->getName() << std::endl;
				}
				vars[v].resize(this->samples);
				auto lat = rm->getVertexLatency(v);
				for (int s=0; s<this->samples; s++) {
					auto var = ScaLP::newIntegerVariable(v->getName()+"_"+std::to_string(s), 0.0, ScaLP::INF());
					vars[v][s] = var;
					solver.addConstraint(supersink - var >= lat);
				}
			}
			for (auto &e : g->Edges()) {
				auto distance = e->getDistance();
				auto *vSrc = &e->getVertexSrc();
				auto *vDst = &e->getVertexDst();
				for (int s=0; s<this->samples; s++) {
					auto sampleIdxAndOffset = Utility::getSampleIndexAndOffset(distance, s, this->samples, this->modulo);
					auto varSrc = vars.at(vSrc).at(sampleIdxAndOffset.first);
					auto varDst = vars.at(vDst).at(s);
					solver.addConstraint(varDst - varSrc >= rm->getVertexLatency(vSrc) + e->getDelay() - sampleIdxAndOffset.second);
				}
			}
			solver.setObjective(ScaLP::minimize(supersink));
			solver.solve();
			auto result = solver.getResult().values;
			for (auto &v : g->Vertices()) {
				auto originalV = this->sccVertexToVertexMap.at(v);
				for (int s=0; s<this->samples; s++) {
					auto var = vars.at(v).at(s);
					auto t = std::round(result.at(var));
					this->relativeSchedule[s][originalV] = t;
				}
			}
		}
	}

	void SATSCCRatIIScheduler::calcDeltaMins() {if(this->initiationIntervals.empty() or this->latencySequence.empty())
			throw HatScheT::Exception("SATSCCRatIIScheduler::calcDeltaMins: need to specify initiation intervals/latency sequence");
		if(this->samples<=0 or this->modulo<=0)
			throw HatScheT::Exception("SATSCCRatIIScheduler::calcDeltaMins: need to specify samples and modulo");
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
			int minDelta = 10000000; // 'infinity'
			for(auto offset=0; offset<this->samples; ++offset) {
				int delta = 0;
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
}
#endif // USE_SCALP
#endif // USE_CADICAL
