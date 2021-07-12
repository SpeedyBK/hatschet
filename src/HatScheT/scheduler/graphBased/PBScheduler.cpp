//
// Created by nfiege on 7/9/21.
//

#include "PBScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/dev/IntegerIINonRectScheduler.h"

#include <iostream>
#include <algorithm>
#include <map>
#include <cmath>

namespace HatScheT {
	PBScheduler::PBScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
		std::list<std::string> solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist),
		sw(solverWishlist) {
		// reset previous solutions
		this->II = -1;
		this->timeouts = 0;
		this->startTimes.clear();
		this->scheduleFound = false;
		this->optimalResult = true;
		computeMinII(&g, &resourceModel);
		this->minII = ceil(this->minII);
		computeMaxII(&g, &resourceModel);
	}

	void PBScheduler::schedule() {
		this->solvingTime = 0.0;
		if(!this->quiet){
			std::cout << "PBS: min/maxII = " << this->minII << " " << this->maxII << ", (minResII/minRecII " << this->resMinII << " / " << this->recMinII << ")" << std::endl;
			std::cout << "PBS: solver timeout = " << this->solverTimeout << " (sec)" << endl;
		}

		//set maxRuns, e.g., maxII - minII, iff value if not -1
		if(this->maxRuns > 0){
			int runs = this->maxII - this->minII + 1;
			if(runs > this->maxRuns) this->maxII = this->minII + this->maxRuns - 1;
			if(this->quiet==false) std::cout << "PBS: maxII changed due to maxRuns value set by user!" << endl;
			if(this->quiet==false) std::cout << "PBS: min/maxII = " << this->minII << " " << this->maxII << std::endl;
			std::cout << "#q# maxRuns = " << this->maxRuns << std::endl;
			std::cout << "#q# minII = " << this->minII << std::endl;
			std::cout << "#q# maxII = " << this->maxII << std::endl;
		}

		if (this->minII > this->maxII)
			throw HatScheT::Exception("Inconsistent II bounds");

		// find subgraphs based on SCCs
		this->createSCCs();
		this->orderSCCs();
		// print topo sorted SCCs for debugging
		if (!this->quiet) {
			for (int stage=0; stage<this->topoSortedSCCs.size(); stage++) {
				std::cout << "Stage " << stage << std::endl;
				for (auto scc : this->topoSortedSCCs[stage]) {
					std::cout << "  SCC " << scc->getId() << std::endl;
					for (auto v : scc->getVerticesOfSCC()) {
						std::cout << "    " << v->getName() << " (ID " << v->getId() << ")" << std::endl;
					}
				}
			}
		}

		// iterative modulo scheduling loop
		for (int candII = this->minII; candII <= this->maxII; ++candII) {
			// try scheduling
			this->scheduleAttempt(candII);
			// free all that garbage that we just created
			this->freeMemory();
			// check if we found a solution
			if (this->scheduleFound) {
				this->II = candII;
				auto solution = this->solver->getResult().values;

				if(!this->quiet) {
					std::cout << "PBS: found " << (this->optimalResult ? "optimal" : "feasible") << " solution with II=" << this->II << std::endl;
					for(auto it : this->startTimes) {
						std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
					}
				}
				break;
			}
			if(!this->scheduleFound) if(this->quiet==false) cout << "  II=" << candII << " : " << this->stat << endl;
		}
		if(scheduleFound == false) this->II = -1;
		if(this->quiet==false) std::cout << "PBS: solving time was " << this->solvingTime << " seconds" << std::endl;
	}

	void PBScheduler::scheduleAttempt(int candidateII) {
		// set to false at timeout
		this->scheduleFound = true;
		// time management
		this->begin = clock();

		// WHAT TO DO:
		// create subgraphs from SCCs
		this->partitionSCCGraph(candidateII);
		// debug info
		if (!this->quiet) {
			std::cout << "PBScheduler::scheduleAttempt: info" << std::endl;
			// print II
			std::cout << "  candidate II = " << candidateII << std::endl;
			// print subgraphs
			std::cout << "  subgraphs:" << std::endl;
			for (auto subgraph : this->subgraphs) {
				std::cout << "    new subgraph:" << std::endl;
				std::cout << "      vertices:" << std::endl;
				for (auto sv : subgraph->Vertices()) {
					auto v = this->subgraphVertexToVertexMap[sv];
					std::cout << "        '" << v->getName() << "' (id " << v->getId() << ")" << std::endl;
				}
				std::cout << "      edges:" << std::endl;
				for (auto e : subgraph->Edges()) {
					auto vSrc = this->subgraphVertexToVertexMap[&e->getVertexSrc()];
					auto vDst = this->subgraphVertexToVertexMap[&e->getVertexDst()];
					auto dist = e->getDistance();
					std::cout << "        '" << vSrc->getName() << "' -" << dist << "-> '" << vDst->getName() << "'" << std::endl;
				}
			}
		}
		// schedule subgraphs
		this->scheduleSubgraphs(candidateII);
		if (!this->scheduleFound) {
			// timeout :(
			this->end = clock();
			this->solvingTime += ((double) this->end - this->begin) / ((double) CLOCKS_PER_SEC);
			this->optimalResult = false; // I would be shocked if we ever found an optimum
			this->II = -1;
		}
		// offset relative schedules to obey dependency constraints
		this->sortSubgraphs(candidateII);
		// postprocessing because the smallest start time might be greater than 0
		this->postProcessSchedule();
		// VARIABLES TO SET:
		//   this->begin
		//   this->end
		//   this->scheduleFound
		//   this->optimalResult
		//   this->startTimes
		//   this->solvingTime
		this->end = clock();
		this->solvingTime += ((double) this->end - this->begin) / ((double) CLOCKS_PER_SEC);
		this->optimalResult = false; // I would be shocked if we ever found an optimum
		this->II = candidateII;
	}

	void PBScheduler::createSCCs() {
		// use kosaraju scc finder to find sccs
		KosarajuSCC k(this->g);
		k.setQuiet(this->quiet);
		this->sccs = k.getSCCs();
	}

	void PBScheduler::orderSCCs() {
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
				std::cout << "PBScheduler::orderSCCs: created vertex with id " << sccVertex->getId() << std::endl;
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
							std::cout << "PBScheduler::orderSCCs: found src vertex '" << v->getName() << "' in SCC " << srcSCCId << std::endl;
						}
					}
					if (dstSCCId < 0 and v == vDst) {
						dstSCCId = scc->getId();
						if (!this->quiet) {
							std::cout << "PBScheduler::orderSCCs: found dst vertex '" << v->getName() << "' in SCC " << dstSCCId << std::endl;
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
				std::cout << "PBScheduler::orderSCCs: src scc id: " << srcSCCId << std::endl;
				std::cout << "PBScheduler::orderSCCs: dst scc id: " << dstSCCId << std::endl;
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
		this->topoSortedSCCs.resize(asap.getScheduleLength());
		for (auto it : sccSchedule) {
			this->topoSortedSCCs[it.second].emplace_back(vertexSCCMap[it.first]);
		}
	}

	void PBScheduler::partitionSCCGraph(int candidateII) {
		// calculate maximal subgraph size
		if (this->maximalSubgraphSize <= 0) {
			this->maximalSubgraphSize = round(sqrt(this->g.getNumberOfVertices()));
		}
		if (!this->quiet) {
			std::cout << "PBScheduler::partitionSCCGraph: start partitioning graph with max subgraph size = "
				<< this->maximalSubgraphSize << std::endl;
		}
		// keep track which SCCs were already assigned to a subgraph
		// at the beginning mark every one as unassigned
		std::map<SCC*, bool> assigned;
		for (auto scc : this->sccs) {
			assigned[scc] = false;
		}
		// keep adding sccs to subgraphs until each one was added
		this->subgraphs.clear();
		this->subgraphs.emplace_back(new Graph());
		this->subgraphs.back()->setName("subgraph_0");
		int currentStage = 0;
		while (currentStage < this->topoSortedSCCs.size()) {

			// check if every SCC in this stage was added to a subgraph
			bool allAdded = true;
			for (auto scc : this->topoSortedSCCs[currentStage]) {
				if (!assigned[scc]) {
					allAdded = false;
					break;
				}
			}
			if (allAdded) {
				if (!this->quiet) {
					std::cout << "PBScheduler::partitionSCCGraph: all SCCs of stage " << currentStage << " added to subgraphs" << std::endl;
				}
				// increment current stage
				currentStage++;
				// go into next iteration
				continue;
			}
			// reference to current subgraph that we are adding stuff to
			auto subgraph = this->subgraphs.back();
			// get the largest SCC in the current stage that still leads to a subgraph within the maximal subgraph size bound
			std::vector<SCC*> bestSCCs;
			for (auto scc : this->topoSortedSCCs[currentStage]) {
				// skip SCC if it was already assigned
				if (assigned[scc]) {
					continue;
				}
				// add current SCC if it is the first one
				if (bestSCCs.empty()) {
					bestSCCs.emplace_back(scc);
					continue;
				}
				// check if subgraph size would overflow due to adding this SCC
				if (subgraph->getNumberOfVertices() + scc->getNumberOfVertices() > this->maximalSubgraphSize) {
					continue;
				}
				// check if it fits better
				if (scc->getNumberOfVertices() > bestSCCs.front()->getNumberOfVertices())  {
					bestSCCs.clear();
					bestSCCs.emplace_back(scc);
				}
				// check if it fits just as good as the current best ones
				if (scc->getNumberOfVertices() == bestSCCs.front()->getNumberOfVertices()) {
					bestSCCs.emplace_back(scc);
				}
				// do nothing if it fits worse
			}
			// if there is only one best SCC, choosing one is trivial
			SCC* bestSCC = bestSCCs.front();
			// if there are multiple: choose any one at random
			if (bestSCCs.size() > 1) {
				bestSCC = *Utility::selectRandomElement(bestSCCs.begin(), bestSCCs.end());
			}
			// create new subgraph if no SCC in current stage fits
			if (subgraph->getNumberOfVertices() + bestSCC->getNumberOfVertices() > this->maximalSubgraphSize) {
				std::cout << "PBScheduler::partitionSCCGraph: creating new subgraph because no SCC in stage " << currentStage << " fits" << std::endl;
				this->subgraphs.emplace_back(new Graph());
				subgraph = this->subgraphs.back();
				subgraph->setName("subgraph_"+to_string(this->subgraphs.size()-1));
			}
			// add SCC to subgraph
			// add vertices
			for (auto v : bestSCC->getVerticesOfSCC()) {
				auto newV = &subgraph->createVertex(v->getId());
				// add info to internal maps
				this->vertexToSubgraphMap[v] = subgraph;
				this->vertexToSubgraphVertexMap[v] = newV;
				this->subgraphVertexToVertexMap[newV] = v;
			}
			// add edges
			for (auto e : bestSCC->getSCCEdges()) {
				auto vSrc = this->vertexToSubgraphVertexMap[&e->getVertexSrc()];
				auto vDst = this->vertexToSubgraphVertexMap[&e->getVertexDst()];
				auto distance = e->getDistance();
				auto type = e->getDependencyType();
				if (type == Edge::Precedence) {
					distance = e->getDelay();
				}
				subgraph->createEdge(*vSrc, *vDst, distance, type);
				if (!this->quiet) {
					std::cout << "PBScheduler::partitionSCCGraph: added edge to subgraph: '" << vSrc->getName() << "' -" << distance
						<< "-> '" << vDst->getName() << "'" << std::endl;
				}
			}
			// mark SCC as "assigned"
			assigned[bestSCC] = true;
		}
		// assign resources to all subgraphs after graph partitioning finished
		this->partitionMRTs(candidateII);
	}

	void PBScheduler::scheduleSubgraphs(int candidateII) {
		for (auto subgraph : this->subgraphs) {
			auto elapsedTime = ((double)(clock() - this->begin)) / CLOCKS_PER_SEC;
			if (!this->quiet) {
				std::cout << "start scheduling " << subgraph->getName() << " - elapsed time so far: " << elapsedTime << " sec"
					<< std::endl;
			}
			auto rm = this->resourceModels[subgraph];
			IntegerIINonRectScheduler scheduler(*subgraph, *rm, this->sw);
			scheduler.setQuiet(this->quiet);
			scheduler.setSolverTimeout(this->getSolverTimeout() - round(elapsedTime));
			scheduler.setCandidateII(candidateII);
			scheduler.schedule();
			if (!scheduler.getScheduleFound()) {
				this->scheduleFound = false;
				if (!this->quiet) {
					std::cout << "PBScheduler::scheduleSubgraphs: backbone scheduler failed to find solution "
						<< "with ScaLP status: " << scheduler.getScaLPStatus() << std::endl;
				}
				break;
			}
			this->subgraphSchedule[subgraph] = scheduler.getSchedule();
		}
		// debug
		if (!this->quiet) {
			for (auto subgraph : this->subgraphs) {
				std::cout << "Schedule for " << subgraph->getName() << std::endl;
				for (auto it : this->subgraphSchedule[subgraph]) {
					std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
				}
			}
		}
	}

	void PBScheduler::sortSubgraphs(int candidateII) {
		for (auto stage : this->topoSortedSCCs) {
			for (auto scc : stage) {
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
					auto subgraphSrc = this->vertexToSubgraphMap[vSrc];
					auto subgraphDst = this->vertexToSubgraphMap[vDst];
					auto vSubSrc = this->vertexToSubgraphVertexMap[vSrc];
					auto vSubDst = this->vertexToSubgraphVertexMap[vDst];
					auto tSrc = this->startTimes[vSrc];
					auto tDst = this->subgraphSchedule[subgraphDst][vSubDst];
					auto distance = e->getDistance();
					auto delay = e->getDelay();
					auto minOffset = (int) ceil(((double) tSrc + this->resourceModel.getVertexLatency(vSrc) - delay) / ((double)candidateII)) + distance;
					maxMinOffset = std::max(maxMinOffset, minOffset);
				}
				// offset every vertex by the minimum offset
				for (auto vSrc : verticesOfSCC) {
					auto subgraphSrc = this->vertexToSubgraphMap[vSrc];
					auto vSubSrc = this->vertexToSubgraphVertexMap[vSrc];
					auto tSrc = this->subgraphSchedule[subgraphSrc][vSubSrc] + (maxMinOffset * candidateII);
					this->startTimes[vSrc] = tSrc;
				}
			}
		}
	}

	void PBScheduler::postProcessSchedule() {
		int minTime = 0x7FFFFFFF; // max int
		for (auto &it : this->startTimes) {
			if (!this->g.isSourceVertex(it.first)) {
				// skip non-sources
				continue;
			}
			minTime = std::min(minTime, it.second);
		}
		if (minTime == 0) {
			// do nothing if earliest schedule time is 0
			return;
		}
		if (!this->quiet) {
			std::cout << "PBScheduler::postProcessSchedule: detected minimum schedule time = " << minTime << std::endl;
		}
		for (auto v : this->g.Vertices()) {
			this->startTimes[v] -= minTime;
		}
	}

	void PBScheduler::partitionMRTs(int candidateII) {
		// keep track of free MRT slots
		std::map<std::pair<std::string, int>, int> freeSlots;
		for (auto res : this->resourceModel.Resources()) {
			for (int slot = 0; slot < candidateII; slot++) {
				freeSlots[{res->getName(), slot}] = res->getLimit();
			}
		}
		for (auto subgraph : this->subgraphs) {
			// create a container to keep track which resources are needed
			// map <pair <resource name, congruence class>> -> limit
			std::map<std::pair<std::string, int>, int> neededResources;
			// for each vertex select a random (FREE!) mrt slot
			for (auto sv : subgraph->Vertices()) {
				auto v = this->subgraphVertexToVertexMap[sv];
				auto res = this->resourceModel.getResource(v);
				if (res->isUnlimited()) {
					// skip unlimited resources because we do not need to partition them
					continue;
				}
				std::vector<int> limits;
				for (int slot = 0; slot < candidateII; slot++) {
					if (freeSlots[{res->getName(), slot}] > 0) {
						limits.emplace_back(slot);
					}
				}
				int slot = *Utility::selectRandomElement(limits.begin(), limits.end());
				// add modulo slot to needed resources
				neededResources[{res->getName(), slot}] += 1;
				// subtract modulo slot from free resources if it is not unlimited
				freeSlots[{res->getName(), slot}] -= 1;
			}
			// create resource model for that subgraph
			auto rm = new ResourceModel();
			this->resourceModels[subgraph] = rm;
			for (auto res : this->resourceModel.Resources()) {
				auto &newRes = rm->makeResource(res->getName(), res->getLimit(), res->getLatency(), res->getBlockingTime());
				// specify MRT heights
				for (int slot = 0; slot < candidateII; slot++) {
					auto limit = neededResources[{newRes.getName(), slot}];
					if (newRes.isUnlimited()) {
						limit = UNLIMITED;
					}
					newRes.setNonRectLimit(slot, limit);
				}
			}
			// register vertices of that subgraph
			for (auto v : subgraph->Vertices()) {
				auto originalResource = this->resourceModel.getResource(this->subgraphVertexToVertexMap[v]);
				rm->registerVertex(v, rm->getResource(originalResource->getName()));
			}
		}
		// randomly assign remaining free slots
		// Todo: do it.
	}

	void PBScheduler::freeMemory() {
		for(auto it : this->resourceModels) {
			// delete subgraph
			delete it.first;
			// delete its resource model
			delete it.second;
		}
		this->resourceModels.clear();
		this->subgraphs.clear();
	}
}

