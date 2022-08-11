/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "HatScheT/utility/Utility.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <map>
#include <ios>
#include <fstream>
#include <ctime>
#include <cstddef>
#include <iomanip>
#include <deque>
#include <queue>
#include <HatScheT/utility/writer/DotWriter.h>

#include "HatScheT/scheduler/ASAPScheduler.h"

#ifdef USE_SCALP

#include "HatScheT/base/ILPSchedulerBase.h"
#include "HatScheT/scheduler/ilpbased/ASAPILPScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/scheduler/ULScheduler.h"

#endif

namespace HatScheT {

	int Utility::getNoOfResConstrVertices(ResourceModel *rm, Graph *g) {
		int count = 0;
		for (auto it = g->verticesBegin(); it != g->verticesEnd(); ++it) {
			Vertex *v = *it;
			const Resource *r = rm->getResource(v);
			if (r->getLimit() > 0) count++;
		}
		return count;
	}

	bool Utility::everyVertexisRegistered(HatScheT::Graph &g, HatScheT::ResourceModel &rm) {
		for (auto it = g.verticesBegin(); it != g.verticesEnd(); ++it) {
			Vertex *v = *it;

			rm.getResource(v);
		}
		return true;
	}

	int Utility::getNoOfInputsWithoutRegs(Graph *g, const Vertex *v) {
		int no = 0;

		for (auto it = g->edgesBegin(); it != g->edgesEnd(); ++it) {
			Edge *e = *it;
			Vertex *dstV = &e->getVertexDst();

			if (dstV == v && e->getDistance() == 0) no++;
		}

		return no;
	}

	int Utility::getNoOfInputs(Graph *g, const Vertex *v) {
		int no = 0;

		for (auto it = g->edgesBegin(); it != g->edgesEnd(); ++it) {
			Edge *e = *it;
			Vertex *dstV = &e->getVertexDst();

			if (dstV == v) no++;
		}

		return no;
	}

	int Utility::getNoOfOutputsWithoutDistance(Graph *g, const Vertex *v) {
		int outputs = 0;

		for (auto it = g->edgesBegin(); it != g->edgesEnd(); ++it) {
			Edge *e = *it;
			Vertex *vSrc = &e->getVertexSrc();

			if (vSrc == v && e->getDistance() == 0) outputs++;
		}

		return outputs;
	}

	int Utility::getNoOfOutputs(Graph *g, const Vertex *v) {
		int outputs = 0;

		for (auto it = g->edgesBegin(); it != g->edgesEnd(); ++it) {
			Edge *e = *it;
			Vertex *vSrc = &e->getVertexSrc();

			if (vSrc == v) outputs++;
		}

		return outputs;
	}

	int Utility::getCyclesOfLongestPath(HatScheT::Graph *g, HatScheT::ResourceModel *rm, double II) {
		int length = 0;

		//identify inputs
		vector<Vertex *> inputs;
		for (auto it = g->verticesBegin(); it != g->verticesEnd(); ++it) {
			Vertex *v = *it;

			if (g->getPredecessors(v).size() == 0) inputs.push_back(v);
		}

		// check if there are vertices, which only have input edges with distance>0
		if (inputs.empty()) {
			for (auto v : g->Vertices()) {
				bool pushMeBack = true;
				for (auto e : g->Edges()) {
					if (&e->getVertexDst() != v) continue;
					if (e->getDistance() < 1) {
						pushMeBack = false;
						break;
					}
				}
				if (pushMeBack) inputs.emplace_back(v);
			}
		}

		//find seed edges
		vector<const Edge *> seeds;
		//iterate over inputs
		for (auto it = inputs.begin(); it != inputs.end(); ++it) {
			Vertex *in = *it;

			set<Vertex *> succ = g->getSuccessors(in);

			for (auto it2 : succ) {
				list<const Edge *> s = g->getEdges(in, it2);

				for (auto it3 : s) {
					seeds.push_back(it3);
				}
			}
		}

		//start recursion
		for (auto it : seeds) {
			vector<Vertex *> visited;
			int l = rm->getVertexLatency(&it->getVertexSrc());
			l += std::ceil(II * it->getDistance());
			Utility::cycle(it, visited, l, g, rm, II);

			if (l > length) length = l;
		}

		return length;
	}

	void Utility::cycle(const HatScheT::Edge *e, vector<HatScheT::Vertex *> &visited, int &currLength, Graph *g,
											ResourceModel *rm, double II) {
		Vertex *succ = &e->getVertexDst();

		//check whether succ was already visited (cycle ends)
		for (auto it : visited) {
			if (succ == it)
				return;
		}

		//put to visited
		visited.push_back(succ);
		//increase length
		currLength += rm->getVertexLatency(succ);

		//collect all outgoing edges
		set<Vertex *> s = g->getSuccessors(succ);
		vector<const Edge *> outgoing;
		for (auto it : s) {
			list<const Edge *> o = g->getEdges(succ, it);

			for (auto it2 : o) {
				outgoing.push_back(it2);
			}
		}

		//check if outport found
		if (outgoing.size() == 0) return;

		//continue recursion
		for (auto it : outgoing) {
			currLength += std::ceil(II * it->getDistance());
			Utility::cycle(it, visited, currLength, g, rm, II);
		}
	}

	bool Utility::IIisRational(double II) {
		double intpart;
		if (modf(II, &intpart) != 0.0) {
			return true;
		}
		return false;
	}

	double Utility::calcMinII(double minResII, double minRecII) {
		if (minResII > minRecII) return minResII;

		return minRecII;
	}

#ifdef USE_SCALP

	double Utility::calcResMII(ResourceModel *rm, Target *t) {
		double resMII = 1.0f;
		//standard case without using a hardware target
		if (t == nullptr) {
			for (auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it) {
				Resource *r = *it;
				//skip unlimited resources
				if (r->getLimit() == -1)continue;
				int opsUsingR = rm->getNumVerticesRegisteredToResource(r);
				int avSlots = r->getLimit();

				if (avSlots <= 0) throw HatScheT::Exception("Utility.calcResMII: avSlots <= 0 : " + to_string(avSlots));
				double tempMax =
					((double) opsUsingR) / ((double) avSlots); // + (double)(opsUsingR % avSlots != 0); < -- ?? patrick

				if (tempMax > resMII) resMII = tempMax;
			}
		}
			//a target is provided
		else {
			//throw Exception("Utility.calcResMII: ERROR calculating resMinII using a target is not supported yet");
			cout << "Utility.calcResMII: WARNING calculating resMinII using a target is not supported yet" << endl;
		}

		return resMII;
	}

	int Utility::getILPASAPScheduleLength(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
		HatScheT::ASAPScheduler a(*g, *rm);
		a.schedule();
		int criticalPath = a.getScheduleLength();

		ASAPILPScheduler asap(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
		asap.setMaxLatencyConstraint(criticalPath);
		asap.schedule();

		return asap.getScheduleLength();
	}

	int Utility::getASAPScheduleLength(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
		HatScheT::ASAPScheduler a(*g, *rm);
		a.schedule();

		return a.getScheduleLength();
	}

	int Utility::getASAPNoHCScheduleLength(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
		map<HatScheT::Resource *, int> limits;

		//save old limits
		for (auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it) {
			HatScheT::Resource *r = *it;
			limits.insert(make_pair(r, r->getLimit()));

			r->setLimit(-1);
		}

		//determine schedule
		ASAPScheduler asap(*g, *rm);
		asap.schedule();
		int lat = asap.getScheduleLength();

		//restore old limits
		for (auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it) {
			HatScheT::Resource *r = *it;

			r->setLimit(limits[r]);
		}

		return lat;
	}

	int Utility::getCriticalPath(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
		map<Resource *, int> restoreLimits;
		//set all resource unlimited for critical path calculation, store old values
		for (auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it) {
			Resource *r = *it;
			if (r->isUnlimited() == true) continue;

			restoreLimits.insert(make_pair(r, r->getLimit()));
			r->setLimit(-1);
		}

		//do critical path calculation
		ASAPScheduler asap(*g, *rm);
		asap.schedule();
		int limitlat = asap.getScheduleLength();

		ASAPILPScheduler ilp(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
		ilp.setMaxLatencyConstraint(limitlat);
		ilp.schedule();

		int criticalPath = ilp.getScheduleLength();

		//restore old resource model
		for (auto it = restoreLimits.begin(); it != restoreLimits.end(); ++it) {
			Resource *r = it->first;
			r->setLimit(it->second);
		}

		return criticalPath;
	}

	pair<int, int> Utility::splitRational(double x) {
		//int[] A =new int[12];
		//int[] B = new int[12];
		//int[] k = new int[12];
		int A0 = 1;
		int A1 = 0;
		int B0 = 0;
		int B1 = 1;
		for (int i = 0; i < 1000; i++) {
			//k[i+2] = (int) ((double)1/x);
			int k = (int) ((double) 1 / x);
			//A[i+2] = A[i] + k[i+2] * A[i+1];
			int temp = A1;
			A1 = A0 + k * A1;
			A0 = temp;
			//B[i+2] = B[i] + k[i+2] * B[i+1];
			temp = B1;
			B1 = B0 + k * B1;
			B0 = temp;
			x = ((double) 1 / x) - k;
			if (abs(x) <= 1.0E-3) {
				return make_pair(A1, B1);
			};
		}

		return make_pair(0, 0);
	}

	int Utility::safeRoundDown(double x) {
		if (x >= 0) {
			double error = x - (int) x;
			if (error < 0.999) return (int) x;
			return (int) ceil(x);
		}
		double error = (int) x - x;
		if (error < 0.001) return (int) x;
		return (int) floor(x);
	}

	int Utility::calcAbsolutMaxII(Graph *g, ResourceModel *rm) {
		map<Resource *, int> limit_map;

		//store initial values
		for (auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it) {
			Resource *r = *it;

			limit_map.insert(make_pair(r, r->getLimit()));

			//set temporarily to limit 1
			if (r->getName() != "special_virtual") r->setLimit(1);
		}

		//determine minimum possible latency
		HatScheT::ASAPScheduler asap(*g, *rm);
		asap.schedule();
		int criticalPath = asap.getScheduleLength();

		if (g->getNumberOfVertices() > 200) {
			if (!HatScheT::verifyModuloSchedule(*g, *rm, asap.getSchedule(), asap.getII())) {
				throw HatScheT::Exception("Utility.calcMaxII: ASAP scheduler found invalid result!");
			}
		} else {
			//get optimal critical path using asap ilp scheduler
			HatScheT::ASAPILPScheduler *asapilp = new HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
			asapilp->setMaxLatencyConstraint(criticalPath);
			asapilp->schedule();
			criticalPath = asapilp->getScheduleLength();

			delete asapilp;

			//error in non ilp-based asap detected
			//starting new asap ilp
			if (criticalPath <= 0) {
				HatScheT::ASAPILPScheduler *asapilp = new HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
				asapilp->setMaxLatencyConstraint(g->getNumberOfVertices() * (rm->getMaxLatency() + 1));
				asapilp->schedule();
				criticalPath = asapilp->getScheduleLength();

				delete asapilp;
			}
		}

		//restore values
		for (auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it) {
			Resource *r = *it;
			r->setLimit(limit_map[r]);
		}

		return criticalPath;
	}

	int Utility::calcMaxII(Graph *g, ResourceModel *rm) {
		//determine minimum possible latency
		HatScheT::ASAPScheduler asap(*g, *rm);
		asap.schedule();
		int criticalPath = asap.getScheduleLength();

		if (g->getNumberOfVertices() > 200) {
			if (!HatScheT::verifyModuloSchedule(*g, *rm, asap.getSchedule(), asap.getII())) {
				throw HatScheT::Exception("Utility.calcMaxII: ASAP scheduler found invalid result!");
			}
			return criticalPath;
		}

		//get optimal critical path using asap ilp scheduler
		auto asapilp = HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
		asapilp.setMaxLatencyConstraint(criticalPath);
		asapilp.schedule();
		criticalPath = asapilp.getScheduleLength();

		//error in non ilp-based asap detected
		//starting new asap ilp
		if (criticalPath <= 0) {
			auto newasapilp = HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
			newasapilp.setMaxLatencyConstraint(g->getNumberOfVertices() * (rm->getMaxLatency() + 1));
			newasapilp.schedule();
			criticalPath = newasapilp.getScheduleLength();
		}

		return criticalPath;
	}

	double Utility::calcRecMII(Graph *g, ResourceModel *rm) {
		ScaLP::Solver solver({"Gurobi", "CPLEX"});

		// construct decision variables
		auto II = ScaLP::newRealVariable("II", 0, ScaLP::INF());
		std::map<Vertex *, ScaLP::Variable> t;
		for (auto it = g->verticesBegin(), end = g->verticesEnd(); it != end; it++) {
			auto v = *it;
			t[v] = ScaLP::newRealVariable("t_" + to_string(v->getId()), 0, std::numeric_limits<int>::max());
		}

		// construct constraints
		for (auto it = g->edgesBegin(), end = g->edgesEnd(); it != end; it++) {
			auto e = *it;
			auto i = &e->getVertexSrc();
			auto j = &e->getVertexDst();

			solver << ((t[i] + rm->getVertexLatency(i) + e->getDelay() - t[j] - (e->getDistance() * II)) <= 0);
		}

		// construct objective
		solver.setObjective(ScaLP::minimize(II));

		auto status = solver.solve();
		if (status != ScaLP::status::OPTIMAL && status != ScaLP::status::FEASIBLE) {
			cout << "Utility.calcRecMII: ERROR No solution found!" << endl;
			cout << "Utility.calcRecMII: ScaLP::STATUS " << status << endl;
			throw HatScheT::Exception("RecMII computation failed!");
		}

		return max(1.0, (double) (solver.getResult().objectiveValue)); // RecMII could be 0 if instance has no backedges.
	}

#endif

	int Utility::sumOfStarttimes(std::map<Vertex *, int> &startTimes) {
		int sum = 0;

		for (auto &startTime : startTimes) {
			sum += startTime.second;
		}

		return sum;
	}

	bool
	Utility::resourceAvailable(std::map<Vertex *, int> &startTimes, ResourceModel *rm, const Resource *r, Vertex *checkV,
														 int timeStep) {
		//unlimited
		if (r->getLimit() == -1) return true;

		int instancesUsed = 0;

		for (auto &startTime : startTimes) {
			if (startTime.second == timeStep) {
				Vertex *v = startTime.first;
				if (checkV != v && rm->getResource(v) == r) instancesUsed++;
			}
		}

		if (instancesUsed < r->getLimit()) return true;
		return false;
	}

	bool Utility::edgeIsInGraph(Graph *g, const Edge *e) {
		for (auto iterE : g->Edges()) {
			if (iterE == e) return true;
		}
		return false;
	}

	bool Utility::isInput(Graph *g, Vertex *v) {
		for (auto e : g->Edges()) {
			Vertex *vDst = &e->getVertexDst();

			if (v == vDst && e->getDistance() == 0) return false;
		}

		return true;
	}

	bool Utility::existEdgeBetweenVertices(Graph *g, Vertex *Vsrc, Vertex *Vdst) {
		for (auto iterE : g->Edges()) {
			Vertex *iterSrc = &iterE->getVertexSrc();
			Vertex *iterDst = &iterE->getVertexDst();

			if ((iterSrc == Vsrc) && (iterDst == Vdst)) {
				return true;
			}
			if ((iterSrc == Vdst) && (iterDst == Vsrc)) {
				return true;
			}
		}
		return false;
	}


	bool Utility::occurrencesAreConflictFree(Occurrence *occ1, Occurrence *occ2) {
		vector<Vertex *> occ1Set = occ1->getVertices();

		for (auto it:occ1Set) {
			Vertex *v = it;

			if (!occ2->vertexIsNew(v)) {
				return false;
			}
		}
		return true;
	}

	bool Utility::vertexInOccurrence(Occurrence *occ, Vertex *v) {
		vector<Vertex *> vVec = occ->getVertices();

		for (auto vIter : vVec) {
			if (vIter == v) return true;
		}
		return false;
	}

	bool Utility::resourceModelAndTargetValid(HatScheT::ResourceModel &rm, HatScheT::Target &t) {
		for (auto it = rm.resourcesBegin(); it != rm.resourcesEnd(); ++it) {
			Resource *r = *it;
			//skip unlimited resources
			//TODO: is this still valid?? valid in this context?? (patrick)
			//if(r->getLimit() == -1) continue;
			map<string, double> &costs = r->getHardwareCosts();

			for (auto &cost : costs) {
				if (!t.elementExists(cost.first)) {
					cout << "Utility.resourceModelAndTargetValid: ERROR demanded hardware element '" << cost.first
							 << "' of resource '" << r->getName() << "' is not available on this target:" << endl;
					cout << t << endl;
					return false;
				}
			}
		}

		return true;
	}

	int Utility::calcUsedOperationsOfBinding(map<const Vertex *, int> &binding, ResourceModel &rm, Resource *r) {
		int opsUsed = 0;
		//unlimited resources used in paralell
		if (r->getLimit() <= 0) {
			return rm.getNumVerticesRegisteredToResource(r);
		}

		vector<bool> usedOp;
		usedOp.resize(r->getLimit());
		for (int i = 0; i < usedOp.size(); i++) {
			usedOp[i] = false;
		}

		for (auto it:binding) {
			const Vertex *v = it.first;
			if (rm.getResource(v) == r) {
				int bind = binding[v];
				usedOp[bind] = true;
			}
		}

		for (int i = 0; i < usedOp.size(); i++) {
			if (usedOp[i]) opsUsed++;
		}

		return opsUsed;
	}

	void Utility::printBinding(map<const Vertex *, int> &binding, ResourceModel &rm) {
		cout << "-------Print Binding Start-------" << endl;
		for (auto it:binding) {
			const Vertex *v = it.first;
			const Resource *r = rm.getResource(v);
			cout << "Vertex " << v->getName() << " bound to unit " << it.second << " of Resource " << r->getName()
					 << " with limit " << r->getLimit() << endl;
		}
		cout << "-------Print Binding Finished-------" << endl;
	}

	void Utility::printSchedule(map<Vertex *, int> &schedule) {
		cout << "Utility::printSchedule: Start" << endl;
		for (auto it:schedule) {
			Vertex *v = it.first;
			cout << "Scheduled " << v->getName() << " at " << it.second << endl;
		}
		cout << "Utility::printSchedule: Finished" << endl;
	}

	bool Utility::allInputsAreRegisters(Graph *g, Vertex *v) {
		for (auto it = g->edgesBegin(); it != g->edgesEnd(); ++it) {
			Edge *e = *it;
			if (e->getDistance() == 0 && &e->getVertexDst() == v) return false;
		}
		return true;
	}

	bool Utility::vertexInOccurrenceSet(OccurrenceSet *occS, Vertex *v) {
		set<Occurrence *> occSet = occS->getOccurrences();

		for (auto it:occSet) {
			Occurrence *occIter = it;

			if (Utility::vertexInOccurrence(occIter, v)) return true;
		}
		return false;
	}

	bool Utility::occurenceSetsAreConflictFree(OccurrenceSet *occs1, OccurrenceSet *occs2) {
		set<Occurrence *> occs1Set = occs1->getOccurrences();
		set<Occurrence *> occs2Set = occs2->getOccurrences();

		for (auto it:occs1Set) {
			Occurrence *occ = it;
			for (auto it2:occs2Set) {
				Occurrence *occ2 = it2;

				if (!Utility::occurrencesAreConflictFree(occ, occ2)) return false;
			}
		}

		return true;
	}

	void Utility::printRationalIIMRT(map<HatScheT::Vertex *, int> sched,
																	 vector<map<const HatScheT::Vertex *, int> > ratIIbindings,
																	 HatScheT::ResourceModel *rm, int modulo, vector<int> initIntervalls) {
		for (auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it) {
			const Resource *r = *it;

			//operation start time -> operation, unit
			map<int, vector<pair<string, int> > > MRT;

			//insert MRT information over modulo slots
			for (int i = 0; i < modulo; i++) {
				vector<pair<string, int> > slot;
				for (auto it : sched) {
					auto v = it.first;
					if (r != rm->getResource(v)) continue;

					//track the offset for the samples
					int offset = 0;
					//iterate over samples
					for (int j = 0; j < initIntervalls.size(); j++) {
						//increase offset
						if (j > 0) offset += initIntervalls[j - 1];

						int moduloTime = (it.second + offset) % modulo;

						if (moduloTime != i) continue;
						slot.push_back(make_pair(v->getName() + "_s" + to_string(j), ratIIbindings[j][v]));
					}
				}
				MRT.insert(make_pair(i, slot));
			}

			//do the printing
			cout << "-----" << endl;
			cout << "MRT after binding for resource " << r->getName() << " (limit " << r->getLimit() << ")" << endl;
			for (auto &itMrt:MRT) {
				cout << to_string(itMrt.first) << ": ";

				for (auto &it2:itMrt.second) {
					cout << "(" << it2.first << ", unit " << to_string(it2.second) << ") ";
				}
				if (itMrt.second.empty()) cout << "-----";
				cout << endl;
			}
			cout << "-----" << endl;
		}
	}

	std::pair<int, int>
	Utility::getSampleIndexAndOffset(int distance, const int &sample, const int &samples, const int &modulo) {
		auto sAndD = Utility::getSampleIndexAndDistance(distance, sample, samples);
		return std::make_pair(sAndD.first, sAndD.second * modulo);
	}

	std::pair<int, int> Utility::getSampleIndexAndDistance(int distance, const int &sample, const int &samples) {
		// trivial case
		if (distance == 0) return {sample, 0};
		// non-trivial case
		int sampleIndex = sample;
		int newDistance = 0;
		while (distance > 0) {
			if (sampleIndex == 0) {
				sampleIndex = samples - 1;
				newDistance++;
			} else {
				--sampleIndex;
			}
			--distance;
		}
		return std::make_pair(sampleIndex, newDistance);
	}

	std::map<Vertex *, Vertex *> Utility::transposeGraph(Graph *g, Graph *h) {

		std::map<Vertex *, Vertex *> m;

		//Copy all verticies from graph g to graph h. And creating a map with the corresponding verticies.
		//(gVertex : hVertex).
		for (auto V:g->Vertices()) {
			m[V] = &h->createVertex(V->getId());
		}

		//Creating the edges
		for (auto E:g->Edges()) {
			auto &newE = h->createEdge(*m[&E->getVertexDst()], *m[&E->getVertexSrc()], E->getDistance(),
																 E->getDependencyType());
			newE.setDelay(E->getDelay());
		}


		return m;
	}

	bool Utility::iscyclic(Graph *g) {

		map<Vertex *, bool> visited;
		map<Vertex *, bool> recStack;

		for (auto &v : g->Vertices()) {
			visited[v] = false;
			recStack[v] = false;
		}

		for (auto &it : g->Vertices()) {
			if (iscyclicHelper(g, it, visited, recStack)) {
				return true;
			}
		}

		return false;
	}

	bool Utility::iscyclicHelper(Graph *g, Vertex *V, map<Vertex *, bool> &visited, map<Vertex *, bool> &recStack) {

		if (!visited[V]) {

			visited[V] = true;
			recStack[V] = true;

			for (auto &it : g->getSuccessors(V)) {
				if (!visited[it] && iscyclicHelper(g, it, visited, recStack)) {
					return true;
				} else if (recStack[it]) {
					return true;
				}
			}
		}

		recStack[V] = false;
		return false;

	}

	int Utility::hFunction(double n, double M, int tau) {
		if (tau == 0) return (int) ceil(n / M);
		else return hFunction(n - ceil(n / M), M - 1, tau - 1);
	}

	double Utility::getNumberOfEquivalent2x1Muxs(int numFUConnections, Graph *g, ResourceModel *rm,
																							 int numRegsWithPossibleMuxInputs) {
		for (auto &r : rm->Resources()) {
			// number of FUs for this resource
			int numFUs;
			if (r->isUnlimited()) {
				numFUs = rm->getNumVerticesRegisteredToResource(r);
			} else {
				numFUs = r->getLimit();
			}
			// number of inputs for each FU
			int numInputs = 0;
			for (auto &v : rm->getVerticesOfResource(r)) {
				//auto numVertexInputs = g->getPredecessors(v).size(); // nfiege: this does not work...
				int numVertexInputs = 0;
				// count number of inputs
				// using g->getPredecessors does not work because a vertex can have multiple edges
				// coming from the same predecessor
				for (auto &e : g->Edges()) {
					if (not e->isDataEdge()) continue;
					if (v != &e->getVertexDst()) continue;
					numVertexInputs++;
				}
				if (numVertexInputs > numInputs) numInputs = numVertexInputs;
			}
			// subtract the number of input ports * the number of FUs
			// e.g. an FU has 2 inputs to port number 1 and 6 inputs to port number 2
			// then one 2x1 mux is needed for port 1 and 5 2x1 muxs are needed on port number 2
			// this means that #2x1 muxs = 1+5 = 6 = 2+6-#ports = 2+6-2
			// since this holds for each implemented FU, we must multiply the number of inputs with the number of FUs
			numFUConnections -= (numInputs * numFUs);
		}
		return numFUConnections - numRegsWithPossibleMuxInputs;
	}

	std::pair<int, int> Utility::getMaxRegsAndMuxs(Graph *g, ResourceModel *rm, std::map<Vertex *, int> times, int II) {
		int maxRegs = 0;
		int maxConnections = 0;
		int maxMuxs = 0;
		// upper bound for the number of registers
		std::map<const HatScheT::Vertex *, int> sortedLifetimesVertices;
		std::map<const HatScheT::Resource *, std::list<int>> sortedLifetimesResources;
		try {
			for (auto &e : g->Edges()) {
				if (!e->isDataEdge()) continue;
				auto *vSrc = &e->getVertexSrc();
				auto *vDst = &e->getVertexDst();
				auto *rSrc = rm->getResource(vSrc);
				auto *rDst = rm->getResource(vDst);
				auto tSrc = times.at(vSrc);
				auto tDst = times.at(vDst);
				auto latSrc = rm->getVertexLatency(vSrc);
				auto distance = e->getDistance();
				auto lifetime = tDst - tSrc - latSrc + (II * distance);
				if (sortedLifetimesVertices[vSrc] < lifetime) sortedLifetimesVertices[vSrc] = lifetime;
			}
			for (auto &it : sortedLifetimesVertices) {
				auto &v = it.first;
				auto res = rm->getResource(v);
				auto &lifetime = it.second;
				sortedLifetimesResources[res].emplace_front(lifetime);
				sortedLifetimesResources[res].sort();
			}
			for (auto &it : sortedLifetimesResources) {
				auto &res = it.first;
				int numIt;
				if (res->isUnlimited()) {
					numIt = rm->getNumVerticesRegisteredToResource(res);
				} else {
					numIt = res->getLimit();
				}
				auto &sortedStuff = it.second;
				auto listIt = sortedStuff.end();
				int regs = 0;
				for (int i = 0; i < numIt; i++) {
					listIt--;
					regs += (*listIt);
				}
				maxRegs += regs;
			}
		}
		catch (std::out_of_range &) {
			throw Exception("Utility::getMaxRegsAndMuxs: allocation or schedule corrupt");
		}
		// upper bound for the number of interconnect lines
		for (auto &e : g->Edges()) {
			if (!e->isDataEdge()) continue;
			maxConnections += 1;
		}
		// calculate upper bound for multiplexers from upper bound of interconnect lines
		maxMuxs = (int) HatScheT::Utility::getNumberOfEquivalent2x1Muxs(maxConnections, g, rm);
		return {maxRegs, maxMuxs};
	}

	double Utility::getNumberOfFUConnections(int num2x1Muxs, Graph *g, ResourceModel *rm) {
		for (auto &r : rm->Resources()) {
			// here we can NOT skip unlimited resources
			// even though they will never add muxs to the design, they do need input connections
			auto numFUs = r->getLimit();
			if (r->isUnlimited()) {
				numFUs = rm->getNumVerticesRegisteredToResource(r);
			}
			// calculate the number of inputs for this FU
			int numInputs = 0;
			for (auto &v : rm->getVerticesOfResource(r)) {
				auto numVertexInputs = g->getPredecessors(v).size();
				if (numVertexInputs > numInputs) numInputs = numVertexInputs;
			}
			// add the number of input ports * the number of FUs
			// e.g. an FU has 2 inputs to port number 1 and 6 inputs to port number 2
			// then one 2x1 mux is needed for port 1 and 5 2x1 muxs are needed on port number 2
			// this means that #connections = 2+6 = 8 = 1+5+#ports = 1+5+2
			// since this holds for each implemented FU, we must multiply the number of inputs with the number of FUs
			num2x1Muxs += (numInputs * numFUs);
		}
		return num2x1Muxs;
	}

	bool Utility::iequals(const std::string &s1, const std::string &s2) {
		auto sz = s1.size();
		if (s2.size() != sz) return false;
		for (unsigned int i = 0; i < sz; i++) {
			if (std::tolower(s1[i]) != std::tolower(s2[i])) return false;
		}
		return true;
	}

	Binding::BindingContainer Utility::convertBindingContainer(Graph *g, ResourceModel *rm, const int &II,
																														 const Binding::RegChainBindingContainer &bChain,
																														 std::map<Vertex *, int> sched) {
		Binding::BindingContainer b;

		// trivial copies
		//b.multiplexerCosts = bChain.multiplexerCosts; // can't just copy mux costs
		b.registerCosts = bChain.registerCosts;
		b.solutionStatus = bChain.solutionStatus;
		//b.portAssignments = bChain.portAssignments; // not so trivial anymore eh?
		//b.resourceBindings = bChain.resourceBindings; // easy but copying doesn't work because of different data structures
		for (auto &it : bChain.resourceBindings) {
			b.resourceBindings[it.first] = {it.second};
		}

		// copy port assignments and assume that the output port of the src vertex is always zero
		// i.e. all operators have exactly one output port
		for (auto &p : bChain.portAssignments) {
			b.portAssignments[p.first] = {0, p.second};
		}

		// calc lifetimes for all vertices
		std::map<Vertex *, int> vertexLifetimes;
		std::map<Edge *, int> edgeLifetimes;
		for (auto &e : g->Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto tSrc = sched[vSrc];
			auto tDst = sched[vDst];
			auto lSrc = rm->getVertexLatency(vSrc);
			auto lifetime = tDst - tSrc - lSrc + (II * e->getDistance());
			if (lifetime > vertexLifetimes[vSrc]) {
				vertexLifetimes[vSrc] = lifetime;
			}
			edgeLifetimes[e] = lifetime;
		}

		// count number of registers after each FU
		std::map<std::pair<std::string, int>, int> regChainLength;
		for (auto &it : bChain.resourceBindings) {
			regChainLength[{it.first, it.second}] = 0;
		}
		for (auto &connection : bChain.fuConnections) {
			auto rSrcName = connection.first.first.first;
			auto fuSrcIndex = connection.first.first.second;
			auto rDstName = connection.first.second.first;
			auto fuDstIndex = connection.first.second.second;
			auto numRegs = connection.second.first;
			auto dstPort = connection.second.second;
			regChainLength[{rSrcName, fuSrcIndex}] = std::max(regChainLength[{rSrcName, fuSrcIndex}], numRegs);
		}

		// uniquely number registers
		int registerCounter = 0;
		std::map<std::pair<std::string, int>, int> regOffsets;
		for (auto &it : regChainLength) {
			regOffsets[it.first] = registerCounter;
			registerCounter += it.second;
		}

		// check if register costs are ok
		if (bChain.registerCosts != registerCounter) {
			throw Exception("Utility::convertBindingContainer: detected invalid register costs");
		}

		// enable all registers in all timesteps
		for (int regIndex = 0; regIndex < registerCounter; regIndex++) {
			for (int i = 0; i < II; i++) {
				b.registerEnableTimes[regIndex].insert(i);
			}
		}

		// now comes the tricky part...
		// we must re-create all connections
		// define helper function
		// this checks if a specific connection already exists and creates it if not
		auto checkConnection = [](
			std::list<std::tuple<std::string, int, int, std::string, int, int, std::set<int>>> *conn,
			const std::string &srcRes, const int &srcFU, const int &srcPort, const std::string &dstRes, const int &dstFU,
			const int &dstPort) {
			for (auto &c : *conn) {
				if (std::get<0>(c) != srcRes) continue;
				if (std::get<1>(c) != srcFU) continue;
				if (std::get<2>(c) != srcPort) continue;
				if (std::get<3>(c) != dstRes) continue;
				if (std::get<4>(c) != dstFU) continue;
				if (std::get<5>(c) != dstPort) continue;
				return &c;
			}
			conn->push_front({srcRes, srcFU, srcPort, dstRes, dstFU, dstPort, {}});
			return &conn->front();
		};
		// start with FU->register and register->register connections
		for (auto &it : regChainLength) {
			auto rSrcName = it.first.first;
			auto fuSrcIndex = it.first.second;
			auto numRegs = it.second;

			// check if there are any FU->register connections after this FU
			if (numRegs <= 0) continue;

			// get registers
			auto regOffset = regOffsets[it.first];

			// compute times when this FU produces variables with lifetime > 0
			/*
			std::set<int> variableProductionTimes;
			for (auto &vFu : bChain.resourceBindings) {
				auto *v = &g->getVertexByName(vFu.first);
				if (vertexLifetimes[v] == 0) continue;
				auto fuIndex = vFu.second;
				auto *r = rm->getResource(v);
				if (r->getName() != rSrcName or fuIndex != fuSrcIndex) continue;
				auto t = sched.at(v);
				auto productionTime = t + r->getLatency();
				variableProductionTimes.insert(productionTime % II);
			}
			 */

			// create FU->register connection
			auto *cFUReg = checkConnection(&b.connections, rSrcName, fuSrcIndex, 0, "register", regOffset, 0);
			/*
			for (auto t : variableProductionTimes) {
				std::get<6>(*cFUReg).insert(t);
			}
			 */
			for (int t = 0; t < II; t++) {
				std::get<6>(*cFUReg).insert(t);
			}

			// check if there are any register->register connections after this FU
			if (numRegs <= 1) continue;

			// create register->register connections
			for (int i = 0; i < numRegs - 1; i++) {
				/*
				std::set<int> updatedVariableProductionTimes;
				for (auto &vFu : bChain.resourceBindings) {
					auto *v = &g->getVertexByName(vFu.first);
					if (vertexLifetimes[v] <= i) continue;
					auto fuIndex = vFu.second;
					auto *r = rm->getResource(v);
					if (r->getName() != rSrcName or fuIndex != fuSrcIndex) continue;
					auto t = sched.at(v);
					auto productionTime = t + r->getLatency();
					updatedVariableProductionTimes.insert((productionTime + i + 1) % II);
				}
				 */
				auto *cRegReg = checkConnection(&b.connections, "register", regOffset + i, 0, "register", regOffset + i + 1, 0);
				/*
				for (auto t : updatedVariableProductionTimes) {
					std::get<6>(*cRegReg).insert(t);
				}
				 */
				for (int t = 0; t < II; t++) {
					std::get<6>(*cRegReg).insert(t);
				}

			}
		}

		// now do the remaining register->FU and FU->FU connections
		for (auto &connection : bChain.fuConnections) {
			auto rSrcName = connection.first.first.first;
			auto fuSrcIndex = connection.first.first.second;
			auto rDstName = connection.first.second.first;
			auto fuDstIndex = connection.first.second.second;
			auto numRegs = connection.second.first;
			auto dstPort = connection.second.second;

			if (numRegs > 0) {
				// register -> FU
				std::set<int> variableReadTimes;
				for (auto &e : g->Edges()) {
					auto *vSrc = &e->getVertexSrc();
					auto *vDst = &e->getVertexDst();
					if (rm->getResource(vSrc)->getName() != rSrcName or
							rm->getResource(vDst)->getName() != rDstName or
							bChain.resourceBindings.at(vSrc->getName()) != fuSrcIndex or
							bChain.resourceBindings.at(vDst->getName()) != fuDstIndex or
							bChain.portAssignments.at(e) != dstPort) {
						continue;
					}
					// only consider edges with lifetime = numRegs
					if (edgeLifetimes[e] != numRegs) {
						continue;
					}
					variableReadTimes.insert(sched.at(vDst) % II);
				}
				auto srcRegisterIndex = regOffsets.at({rSrcName, fuSrcIndex}) + numRegs - 1;
				auto *cRegFU = checkConnection(&b.connections, "register", srcRegisterIndex, 0, rDstName, fuDstIndex, dstPort);
				for (auto t : variableReadTimes) {
					std::get<6>(*cRegFU).insert(t);
				}
			} else {
				// FU -> FU
				std::set<int> variablePassTimes;
				for (auto &e : g->Edges()) {
					// only consider edges with lifetime = 0 (otherwise we would have at least one reg in between)
					if (edgeLifetimes[e] != 0) continue;
					auto *vSrc = &e->getVertexSrc();
					auto *vDst = &e->getVertexDst();
					if (rm->getResource(vSrc)->getName() != rSrcName or
							rm->getResource(vDst)->getName() != rDstName or
							bChain.resourceBindings.at(vSrc->getName()) != fuSrcIndex or
							bChain.resourceBindings.at(vDst->getName()) != fuDstIndex or
							bChain.portAssignments.at(e) != dstPort) {
						continue;
					}
					variablePassTimes.insert(sched.at(vDst) % II);
				}
				auto *cFUFU = checkConnection(&b.connections, rSrcName, fuSrcIndex, 0, rDstName, fuDstIndex, dstPort);
				for (auto t : variablePassTimes) {
					std::get<6>(*cFUFU).insert(t);
				}
			}
		}

		// now set mux costs
		b.multiplexerCosts = b.connections.size();

		// debugging
		std::cout << "FU bindings:" << std::endl;
		for (auto &it : bChain.resourceBindings) {
			std::cout << "  '" << it.first << "' -> '" << rm->getResource(&g->getVertexByName(it.first))->getName() << "' ("
								<< it.second << ")" << std::endl;
		}
		std::cout << "connections in register chain binding container:" << std::endl;
		for (auto &connection : bChain.fuConnections) {
			auto rSrcName = connection.first.first.first;
			auto fuSrcIndex = connection.first.first.second;
			auto rDstName = connection.first.second.first;
			auto fuDstIndex = connection.first.second.second;
			auto numRegs = connection.second.first;
			auto dstPort = connection.second.second;
			std::cout << "  '" << rSrcName << "' (" << fuSrcIndex << ") -> '" << rDstName << "' (" << fuDstIndex << ") port "
								<< dstPort << " over " << numRegs << " registers" << std::endl;
		}

		std::cout << "connections in general binding container:" << std::endl;
		for (auto &connection : b.connections) {
			auto rSrcName = std::get<0>(connection);
			auto fuSrcIndex = std::get<1>(connection);
			auto srcPort = std::get<2>(connection);
			auto rDstName = std::get<3>(connection);
			auto fuDstIndex = std::get<4>(connection);
			auto dstPort = std::get<5>(connection);
			std::cout << "  '" << rSrcName << "' (" << fuSrcIndex << ") port " << srcPort << " -> '" << rDstName << "' ("
								<< fuDstIndex << ") port " << dstPort << std::endl;
			for (auto t : std::get<6>(connection)) {
				std::cout << "    active in t=" << t << std::endl;
			}
		}

		std::cout << "register enable times:" << std::endl;
		for (auto &it : b.registerEnableTimes) {
			std::cout << "  reg #" << it.first << ": " << std::endl;
			for (auto &t : it.second) {
				std::cout << "    " << t << std::endl;
			}
		}

		return b;
	}

	// Todo: fix bugs before usage - this does not work, yet!
	double Utility::calcRecMIIDFS(Graph *g, ResourceModel *rm) {
		double minII = 0.0;
		for (auto &vStart : g->Vertices()) {
			// start a DFS originating from this vertex and check for loops
			// this container tracks the current path through the graph
			// for convenience we always start with a nullptr that stands for the start of the path
			// each element consists of the following tuple:
			//   [0] the vertex
			//   [1] the cumulative distance until this vertex
			//   [2] the cumulative delay until this vertex (INCLUDING its own delay)
			// here, delay = vertexLatency + edgeDelay (edgeDelay != 0 for chaining edges)
			std::vector<std::tuple<Vertex *, double, double>> path = {{nullptr, 0.0, 0.0}};
			// this container tracks the stack that controls the DFS
			// each element consists of the following pair:
			//   [0] the vertex on the stack
			//   [1] its input
			//   [2] the distance on the edge from the input to the vertex on the stack
			//   [3] the delay on the edge from the input to the vertex on the stack (delay defined as above)
			std::list<std::tuple<Vertex *, Vertex *, double, double>> stack{{vStart, nullptr, 0.0, 0.0}};
			while (!stack.empty()) {
				// get current element from stack
				auto stackTuple = stack.front();
				auto *currVertex = std::get<0>(stackTuple);
				auto *lastVertex = std::get<1>(stackTuple);
				auto additionalDistance = std::get<2>(stackTuple);
				auto additionalDelay = std::get<3>(stackTuple);
				stack.pop_front();
				// check if we have to re-calculate the path (that happens after we found a recurrence or the end on a path)
				if (std::get<0>(path.back()) != lastVertex) {
					std::vector<std::tuple<Vertex *, double, double>> newPath;
					for (auto &it : path) {
						newPath.emplace_back(it);
						if (std::get<0>(it) == lastVertex) break;
					}
					path = newPath;
				}
				// check if we found a loop
				bool foundLoop = false;
				for (auto &it : path) {
					if (std::get<0>(it) != currVertex) {
						continue;
					}
					foundLoop = true;
					auto q = std::get<2>(it) / std::get<1>(it);
					std::cout << "found loop with q = " << q << " = " << std::get<2>(it) << "/" << std::get<1>(it) << std::endl;
					if (q > minII) {
						minII = q;
					}
					break;
				}
				// continue with next iteration if we find a loop
				if (foundLoop) {
					continue;
				}
				// get last path element
				auto lastPathTuple = path.back();
				// insert current element into path
				double newDistance = std::get<1>(lastPathTuple) + additionalDistance;
				double newDelay = std::get<2>(lastPathTuple) + additionalDelay;
				path.emplace_back(currVertex, newDistance, newDelay);
				// loop over outgoing edges and put everything we can find on the stack
				for (auto &e : g->Edges()) {
					auto *vSrc = &e->getVertexSrc();
					if (vSrc != currVertex) continue;
					auto *vDst = &e->getVertexDst();
					double edgeDistance = e->getDistance();
					double edgeDelay = e->getDelay() + rm->getVertexLatency(vSrc);
					std::tuple<Vertex *, Vertex *, double, double> nextStackElement{vDst, vSrc, edgeDistance, edgeDelay};
					stack.emplace_front(nextStackElement);
				}
			}
		}
		return max(1.0, minII); // RecMII could be 0 if instance has no loops
	}

	int Utility::calcMinNumRegs(Graph *g, ResourceModel *rm, const std::vector<std::map<Vertex *, int>> &schedule,
															const int &modulo) {
		if (schedule.empty()) return 0;
		if (modulo <= 0) return 0;
		int samples = (int) schedule.size();
		std::map<int, int> numAlive;
		for (auto &vSrc : g->Vertices()) {
			for (auto s = 0; s < samples; s++) {
				auto tSrc = schedule.at(s).at(vSrc);
				auto lSrc = rm->getVertexLatency(vSrc);
				auto latestReadTime = tSrc + lSrc;
				for (auto &e : g->Edges()) {
					if (&e->getVertexSrc() != vSrc) continue;
					auto vDst = &e->getVertexDst();
					for (auto dstSample = 0; dstSample < samples; dstSample++) {
						auto srcSample = dstSample;
						auto delta = 0;
						auto edgeDistance = e->getDistance();
						for (int sampleCnt = 0; sampleCnt < edgeDistance; sampleCnt++) {
							if (srcSample == 0) {
								srcSample = samples - 1;
								delta++;
							} else {
								srcSample--;
							}
						}
						if (srcSample != s) continue;
						auto tDst = schedule.at(dstSample).at(vDst);
						auto readTime = tDst + (delta * modulo);
						if (readTime > latestReadTime) latestReadTime = readTime;
					}
				}
				for (int t = tSrc + lSrc + 1; t <= latestReadTime; t++) {
					numAlive[t % (int) modulo]++;
				}
			}
		}
		auto minNumRegs = 0;
		for (auto &it : numAlive) {
			if (it.second > minNumRegs) minNumRegs = it.second;
		}
		return minNumRegs;
	}

	int Utility::calcMinNumRegs(Graph *g, ResourceModel *rm, const std::map<Vertex *, int> &schedule, const int &II) {
		return Utility::calcMinNumRegs(g, rm, std::vector<std::map<Vertex *, int>>(1, schedule), II);
	}

	int Utility::calcMinNumChainRegs(Graph *g, ResourceModel *rm, const vector<std::map<Vertex *, int>> &schedule,
																	 const vector<std::map<const Vertex *, int>> &binding, const int &modulo) {
		if (schedule.empty()) return 0;
		if (modulo <= 0) return 0;
		int samples = (int) schedule.size();
		std::map<const Resource *, int> resourceLimits;
		std::map<std::pair<const Resource *, int>, int> resourceLifetimes;
		for (auto &r : rm->Resources()) {
			if (r->isUnlimited()) {
				resourceLimits[r] = (int) rm->getNumVerticesRegisteredToResource(r) * samples;
			} else {
				resourceLimits[r] = r->getLimit();
			}
			for (int l = 0; l < resourceLimits.at(r); l++) {
				resourceLifetimes[{r, l}] = 0;
			}
		}
		for (auto &e : g->Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto *rSrc = rm->getResource(vSrc);
			auto *rDst = rm->getResource(vDst);
			auto lSrc = rSrc->getLatency();
			for (int sDst = 0; sDst < samples; sDst++) {
				int sSrc = sDst;
				int delta = 0;
				for (int i = 0; i < e->getDistance(); i++) {
					if (sSrc == 0) {
						sSrc = samples - 1;
						delta++;
					} else {
						sSrc--;
					}
				}
				int tSrc;
				int tDst;
				int bSrc;
				try {
					tSrc = schedule.at(sSrc).at(vSrc);
					tDst = schedule.at(sDst).at(vDst);
					bSrc = binding.at(sSrc).at(vSrc);
				}
				catch (std::out_of_range &) {
					// invalid schedule/binding -> unable to calculate #regs
					return -1;
				}
				if (bSrc >= resourceLimits.at(rSrc)) {
					// invalid binding provided -> unable to calculate #regs
					std::cout << "Detected invalid binding for vertex '" << vSrc->getName() << "': FU index is '" << bSrc
										<< "' but resource limit is " << resourceLimits.at(rSrc) << std::endl;
					return -1;
				}
				auto lifetime = tDst + (delta * modulo) - (tSrc + lSrc);
				if (lifetime > resourceLifetimes.at({rSrc, bSrc})) {
					resourceLifetimes.at({rSrc, bSrc}) = lifetime;
				}
			}
		}
		int minNumRegs = 0;
		for (auto &it : resourceLifetimes) {
			minNumRegs += it.second;
		}
		return minNumRegs;
	}

	int Utility::calcMinNumChainRegs(Graph *g, ResourceModel *rm, const map<Vertex *, int> &schedule,
																	 const map<const Vertex *, int> &binding, const int &II) {
		return Utility::calcMinNumChainRegs(g, rm, std::vector<std::map<Vertex *, int>>(1, schedule),
																				std::vector<std::map<const Vertex *, int>>(1, binding), II);
	}

	void Utility::unroll(Graph *gUnroll, ResourceModel *rmUnroll, const int &numSamples, const int &cycleLength, Graph *g,
											 ResourceModel *rm, std::map<Vertex *, std::vector<Vertex *>> *vertexMappings,
											 const bool &quiet) {
		// clear vertex mappings just to be safe
		vertexMappings->clear();
		// unroll vertices
		for (auto &v : g->Vertices()) {
			if (!quiet) {
				std::cout << "Utility::unroll: unrolling vertex '" << v->getName() << "'" << std::endl;
			}
			const HatScheT::Resource *r = rm->getResource(v);
			Resource *r_new;
			vector<Vertex *> v_mapping(numSamples);
			try {
				r_new = rmUnroll->getResource(r->getName());
			}
			catch (Exception &) {
				// resource does not exist yet -> create it
				r_new = &rmUnroll->makeResource(r->getName(), r->getLimit(), r->getLatency(), r->getBlockingTime());
			}
			for (int i = 0; i < numSamples; i++) {
				Vertex *v_new = &gUnroll->createVertex();
				v_new->setName(v->getName() + "_" + to_string(i));
				rmUnroll->registerVertex(v_new, r_new);
				v_mapping[i] = v_new;
			}
			(*vertexMappings)[v] = v_mapping;
		}
		// unroll edges
		for (auto e : g->Edges()) {
			Vertex *v_src = &e->getVertexSrc();
			Vertex *v_dst = &e->getVertexDst();
			if (!quiet) {
				std::cout << "Utility::unroll: unrolling edge '" << v_src->getName() << "' -> '" << v_dst->getName() << "'"
									<< std::endl;
			}
			vector<Vertex *> v_src_mappings = vertexMappings->at(v_src);
			vector<Vertex *> v_dst_mappings = vertexMappings->at(v_dst);
			for (int i = 0; i < v_src_mappings.size(); i++) {
				if (e->getDistance() == 0) {
					auto &newE = gUnroll->createEdge(*v_src_mappings[i], *v_dst_mappings[i], 0, e->getDependencyType());
					newE.setDelay(e->getDelay());
				} else {
					int distance = e->getDistance();
					auto sampleIndexDistance = Utility::getSampleIndexAndDistance(distance, i, numSamples);
					auto &newE = gUnroll->createEdge(*v_src_mappings[sampleIndexDistance.first], *v_dst_mappings[i],
																					 sampleIndexDistance.second, e->getDependencyType());
					newE.setDelay(e->getDelay());
				}
			}
		}
	}

	int Utility::getMinLatency(Graph *g, ResourceModel *rm,
                               const map<Vertex *, int> &tMin,
                               const map<Vertex *, int> &tMax,
                               const int &II, const std::list<std::string> &sw,
                               const unsigned int &timeout,
                               const int &maxScheduleLength,
                               const int8_t &threads,
                               bool quiet) {
#ifdef USE_SCALP
		ScaLP::Solver s{sw};
		if (threads > 0) {
            s.threads = (uint8_t) threads;
        }
		std::map<Vertex*, ScaLP::Variable> y;
		std::map<std::pair<Vertex*, int>, ScaLP::Variable> b;
		std::map<Vertex*, ScaLP::Variable> t;
		int latestEndTime = -1;
		auto tau = ScaLP::newIntegerVariable("tau", 0, ScaLP::INF());
		for (auto &v : g->Vertices()) {
			// check if tMin/tMax are set
			if (tMin.find(v) == tMin.end() or tMax.find(v) == tMax.end()) {
				throw Exception("Utility::getMinLatency: tMin/tMax missing for vertex '"+v->getName()+"'");
			}
			auto r = rm->getResource(v);
			if (r->isUnlimited()) {
				auto &tVar = t[v] = ScaLP::newIntegerVariable("t_"+v->getName(), 0, ScaLP::INF());
				s.addConstraint(tVar - tau <= tMax.at(v));
				s.addConstraint(tVar >= tMin.at(v));
			}
			else {
				auto &yVar = y[v] = ScaLP::newIntegerVariable("y_"+v->getName(), 0, ScaLP::INF());
				ScaLP::Term bSum;
				ScaLP::Term bSumWeighted;
				for (int m=0; m<II; m++) {
					auto &bVar = b[{v, m}] = ScaLP::newBinaryVariable("b_"+v->getName()+"_"+std::to_string(m));
					bSum += bVar;
					bSumWeighted += (m * bVar);
				}
				s.addConstraint(bSum == 1);
				s.addConstraint(bSumWeighted + II*yVar - tau <= tMax.at(v));
				s.addConstraint(bSumWeighted + II*yVar >= tMin.at(v));
			}
			auto tMaxV = tMax.at(v) + r->getLatency();
			if (maxScheduleLength != -1 and (latestEndTime == -1 or latestEndTime < tMaxV)) {
				latestEndTime = tMaxV;
			}
		}
		if (!quiet) {
            std::cout << "max schedule length = " << maxScheduleLength << std::endl;
            std::cout << "latest end time = " << latestEndTime << std::endl;
            std::cout << "tau max = " << maxScheduleLength - latestEndTime << std::endl;
        }
		if (maxScheduleLength != -1) {
			s.addConstraint(tau <= maxScheduleLength - latestEndTime);
		}
		for (auto &r : rm->Resources()) {
			if (r->isUnlimited()) continue;
			auto vertices = rm->getVerticesOfResource(r);
			for (int m=0; m<II; m++) {
				ScaLP::Term bSum;
				for (auto &vc : vertices) {
					auto *v = const_cast<Vertex*>(vc);
					bSum += b[{v, m}];
				}
				s.addConstraint(bSum <= r->getLimit());
			}
		}
		s.setObjective(ScaLP::minimize(tau));
		s.timeout = timeout;
		auto stat = s.solve();
		if (stat == ScaLP::status::INFEASIBLE or stat == ScaLP::status::TIMEOUT_INFEASIBLE or stat == ScaLP::status::INVALID or stat == ScaLP::status::ERROR or stat == ScaLP::status::INFEASIBLE_OR_UNBOUND or stat == ScaLP::status::UNKNOWN) {
			std::cout << "ScaLP failed to find latency estimation in " << timeout << " sec" << std::endl;
			std::cout << "ScaLP status: " << ScaLP::showStatus(stat) << std::endl;
			return -1;
		}
		return (int)std::round(s.getResult().values.at(tau));
#else
		throw Exception("Utility::getMinLatency: need ScaLP for minLatency estimation");
#endif
	}

  pair<int, int> Utility::getLatencyEstimation(Graph *g, ResourceModel *rm, double II, Utility::latencyBounds lb, bool quiet) {

	  // Calculate earliest and latest possible starttimes:
	  auto aslap = Utility::getSDCAsapAndAlapTimes(g, rm, II, quiet);

	  // Strange stuff which allows me to reuse the getSDCScheduleLength() function... :roll_eyes:
	  // ToDo: If there is time, we can rewrite getSDCScheduleLength() to get rid of this ugly
	  //       piece of code.
	  unordered_map<Vertex*, Vertex*> helperMap;
      unordered_map<Vertex*, int> tMaxUnordered;
      for (auto &v : aslap.second){
          helperMap[v.first] = v.first;
          tMaxUnordered[v.first] = v.second;
      }

      // Getting the latency of our SDC-Schedules:
      int modAsapLength = getSDCScheduleLength(tMaxUnordered, helperMap, rm);

      int increment = 0;
      int maxLatency = calcMaxLatencyEstimation(g, rm, aslap.first, aslap.second, (int) II);;
      int minLatency = modAsapLength;

      // lb = maxLatency returns {0, maxLatency}
      // lb = minLatency returns {minLatency, 0}
      // lb = minLatency returns {minLatency, maxLatency}
      switch (lb) {
          case latencyBounds::maxLatency:
              // Just return maxLatency... looks obvious...
              return {0, maxLatency};

          case latencyBounds::minLatency:
              // Latency can't be less then II, otherwise it would be possible to just reduce the II.
              // So minLatency is set to II.
              if ((int)II > modAsapLength){
                  minLatency = (int) II;
                  //Update latest start times.
                  for (auto &it : aslap.second){
                      it.second = it.second + (II - modAsapLength);
                  }
              }
              // Latency calculation loop.
              // Runs untill a solution is found, or if minLatency is
              while (minLatency < maxLatency){
                  auto feasilbe = calcMinLatencyEstimation(g, rm,
                                                           aslap.first,
                                                           aslap.second,
                                                           (int)II,
                                                           increment,
                                                           minLatency,
                                                           modAsapLength,
                                                           quiet);
                  if (feasilbe){
                      if (!quiet) { cout << "Min Latency: " << minLatency << endl; }
                      break;
                  }
                  //Adding the calculated increment to latest start times and minLatency.
                  for (auto &it : aslap.second){
                      it.second += increment;
                  }
                  if (!quiet) { cout << "Min Lat:" << minLatency << " Increment: " << increment << endl; }
                  minLatency += increment;
              }
              return {minLatency, 0};

          case latencyBounds::both:
              // Latency can't be less then II, otherwise it would be possible to just reduce the II.
              // So minLatency is set to II.
              if ((int)II > minLatency){
                  minLatency = (int) II;
                  //Update latest start times.
                  for (auto &it : aslap.second){
                      int temp = it.second;
                      it.second = it.second + (II - modAsapLength);
                  }
              }
              // Latency calculation loop.
              // Runs untill a solution is found, or if minLatency is
              while (minLatency < maxLatency){
                  auto feasilbe = calcMinLatencyEstimation(g, rm,
                                                           aslap.first,
                                                           aslap.second,
                                                           (int) II,
                                                           increment,
                                                           minLatency,
                                                           modAsapLength,
                                                           quiet);
                  if (feasilbe){
                      if (!quiet) { cout << "Min Latency: " << minLatency << endl; }
                      break;
                  }
                  //Adding the calculated increment to latest start times and minLatency.
                  for (auto &it : aslap.second){
                      it.second += increment;
                  }
                  if (!quiet) { cout << "Min Lat:" << minLatency << " Increment: " << increment << endl; }
                  minLatency += increment;
              }
              return {minLatency, maxLatency};
      }
      throw(HatScheT::Exception("Utility::getLatencyEstimation(): Reached end of the switch-statement."));
      return {0,0};
  }

  pair<map<Vertex*, int>, map<Vertex*, int>> Utility::getSDCAsapAndAlapTimes(Graph* g, ResourceModel* rm,
                                                                             const double &II, bool quiet) {

      // Create SDC-Graphs (Transposed with Helper for ALAP-Starttimes
      // and original with Helper for ASAP-Starttimes) as well as a mapping
      // to the Vertices of our original Graph:
      Graph sdcGraphAlap;
      Graph sdcGraphAsap;
      unordered_map<Vertex*, int> vertexLatencyAlap;
      unordered_map<Vertex*, int>vertexDistanceAlap;
      unordered_map<Vertex*, Vertex*> oldToNewVertexAlap;
      unordered_map<Vertex*, Vertex*> newToOldVertexAlap;
      unordered_map<Vertex*, int> vertexLatencyAsap;
      unordered_map<Vertex*, int> vertexDistanceAsap;
      unordered_map<Vertex*, Vertex*> oldToNewVertexAsap;
      unordered_map<Vertex*, Vertex*> newToOldVertexAsap;

      for (auto &v : g->Vertices()){
          Vertex* vSdcAlap = &sdcGraphAlap.createVertex(v->getId());
          Vertex* vSdcAsap = &sdcGraphAsap.createVertex(v->getId());
          vSdcAlap->setName(v->getName());
          vSdcAsap->setName(v->getName());
          vertexLatencyAlap[vSdcAlap] = rm->getVertexLatency(v);
          vertexLatencyAsap[vSdcAsap] = rm->getVertexLatency(v);
          oldToNewVertexAlap[v] = vSdcAlap;
          oldToNewVertexAsap[v] = vSdcAsap;
          newToOldVertexAlap[vSdcAlap] = v;
          newToOldVertexAsap[vSdcAsap] = v;
      }

      for (auto &e : g->Edges()){
          auto vSrcAlap = oldToNewVertexAlap.at(&e->getVertexSrc());
          auto vSrcAsap = oldToNewVertexAsap.at(&e->getVertexSrc());
          auto vDstAlap = oldToNewVertexAlap.at(&e->getVertexDst());
          auto vDstAsap = oldToNewVertexAsap.at(&e->getVertexDst());
          sdcGraphAlap.createEdge(*vDstAlap, *vSrcAlap, -vertexLatencyAlap.at(vSrcAlap)
                                                        - e->getDelay()
                                                        + e->getDistance() * (int) II);
          sdcGraphAsap.createEdge(*vSrcAsap, *vDstAsap, -vertexLatencyAsap.at(vSrcAsap)
                                                        - e->getDelay()
                                                        + e->getDistance() * (int) II);

      }

      //Bellmann-Ford Step 1:
      //Create Helper Vertex and connect it to all other Vertices with distance 0:
      //Initialization of Bellmann-Ford Algorithm with INT_MAX for all Vertices except Helper
      //to solve the SDC-System:
      Vertex& helperAlap = sdcGraphAlap.createVertex();
      Vertex& helperAsap = sdcGraphAsap.createVertex();
      helperAlap.setName("Helper" + to_string(helperAlap.getId()));
      helperAsap.setName("Helper" + to_string(helperAsap.getId()));
      for (auto &v : sdcGraphAlap.Vertices()){
          if (v == &helperAlap){
              vertexDistanceAlap[&helperAlap] = 0;
              continue;
          }
          sdcGraphAlap.createEdge(helperAlap, *v, -rm->getVertexLatency(newToOldVertexAlap.at(v)));
          vertexDistanceAlap[v] = INT32_MAX;
      }
      for (auto &v : sdcGraphAsap.Vertices()){
          if (v == &helperAsap){
              vertexDistanceAsap[&helperAsap] = 0;
              continue;
          }
          sdcGraphAsap.createEdge(helperAsap, *v, 0);
          vertexDistanceAsap[v] = INT32_MAX;
      }

      //Bellmann-Ford Step 2:
      //Searching shortest path:
      //A simple shortest path from src to any other vertex can have
      //at-most |V| - 1 edges
      try {
          unsigned int numOfVerticesAlap = sdcGraphAlap.getNumberOfVertices();
          for (int i = 0; i < numOfVerticesAlap - 1; i++) {
              for (auto &e : sdcGraphAlap.Edges()) {
                  auto t = &e->getVertexSrc();
                  auto u = &e->getVertexDst();
                  int weight = e->getDistance();
                  if ((vertexDistanceAlap.at(t) != INT32_MAX) &&
                      (vertexDistanceAlap.at(t) + weight < vertexDistanceAlap.at(u))) {
                      vertexDistanceAlap.at(u) = vertexDistanceAlap.at(t) + weight;
                  }
              }
          }
      }catch (std::out_of_range&) {
          throw (std::out_of_range("Utility: getSDCAsapAndAlapTimes(): Bellmann-Ford, Step 2, ALAP Loop"));
      }
      try{
          unsigned int numOfVerticesAsap = sdcGraphAsap.getNumberOfVertices();
          for (int i = 0; i < numOfVerticesAsap - 1; i++) {
              for (auto &e : sdcGraphAsap.Edges()) {
                  auto t = &e->getVertexSrc();
                  auto u = &e->getVertexDst();
                  int weight = e->getDistance();
                  if ((vertexDistanceAsap.at(t) != INT32_MAX) &&
                      (vertexDistanceAsap.at(t) + weight < vertexDistanceAsap.at(u))) {
                      vertexDistanceAsap.at(u) = vertexDistanceAsap.at(t) + weight;
                  }
              }
          }
      } catch (std::out_of_range&) {
          throw (std::out_of_range("Utility: getSDCAsapAndAlapTimes(): Bellmann-Ford, Step 2, ASAP Loop"));
      }

      //Checking for negativ Cycles:
      for (auto &e : sdcGraphAlap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertexDistanceAlap.at(t) != INT32_MAX) && (vertexDistanceAlap.at(t) + weight < vertexDistanceAlap.at(u))){
              cout << "Negative Cycle Detected ALAP:" << endl;
              cout << "Infeasibility without Ressource Constraints: Check if II is correct." << endl;
              throw(HatScheT::Exception("Utility::getSDCAsapAndAlapTimes()"));
          }
      }
      for (auto &e : sdcGraphAsap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertexDistanceAsap.at(t) != INT32_MAX) && (vertexDistanceAsap.at(t) + weight < vertexDistanceAsap.at(u))){
              cout << "Negative Cycle Detected ASAP" << endl;
              cout << "Infeasibility without Ressource Constraints: Check if II is correct." << endl;
              throw(HatScheT::Exception("Utility::getSDCAsapAndAlapTimes()"));
          }
      }

      auto min = std::min_element(vertexDistanceAlap.begin(), vertexDistanceAlap.end(), [](const pair<Vertex*,int> &x, const pair<Vertex*, int> &y) {
        return x.second < y.second;
      })->second;

      map<Vertex*, int> startTimesAlap;
      map<Vertex*, int> startTimesAsap;

      for (auto &it : vertexDistanceAlap) {
          //Removing HELPER_ALAP-Vertex from ALAP-Starttimes:
          if (it.first == &helperAlap){
              continue;
          }
          it.second += abs(min);
          startTimesAlap[newToOldVertexAlap[it.first]]=it.second;
      }

      for (auto &it : vertexDistanceAsap) {
          //Removing HELPER_ASAP-Vertex from ALAP-Starttimes:
          if (it.first == &helperAsap) {
              continue;
          }
          it.second = abs(it.second);
          startTimesAsap[newToOldVertexAsap[it.first]] = it.second;
      }

      auto success = vertexDistanceAlap.erase(&helperAlap);
      if (!success){
          throw (HatScheT::Exception("Utility::getSDCAsapAndAlapTimes(): Deleting ALAP-Helper-Vertex failed!"));
      }
      success = vertexDistanceAsap.erase(&helperAsap);
      if (!success){
          throw (HatScheT::Exception("Utility::getSDCAsapAndAlapTimes(): Deleting ASAP-Helper-Vertex failed!"));
      }

      //Saving schedule length
      int modAlapLength = getSDCScheduleLength(vertexDistanceAlap, newToOldVertexAlap, rm);
      int modAsapLength = getSDCScheduleLength(vertexDistanceAsap, newToOldVertexAsap, rm);
      if (!quiet) { cout << "SDC-ALAP-Latency: " << modAlapLength << endl; }
      if (!quiet) { cout << "SDC-ASAP-Latency: " << modAsapLength << endl; }
      if (!quiet) { cout << endl; }
      if (modAsapLength != modAlapLength ){
          throw (HatScheT::Exception("Hatschet::Utility::getSDCAsapAndAlapTimes(), SDC-ALAP-Latency and SDC-ASAP-Latency not equal"));
      }

      //If no valid schedule is found, we continue our latency estimation with the found
      //MOD-ASAP- and MOD-ALAP-Starttimes
      return {startTimesAsap, startTimesAlap};
  }

  int Utility::getSDCScheduleLength(const unordered_map<Vertex *, int> &vertexStarttimes,
                                    const unordered_map<Vertex *, Vertex *> &newToOld,
                                    ResourceModel* rm, bool quiet) {
      int maxTime = -1;
      for (std::pair<Vertex *, int> vtPair : vertexStarttimes) {
          try {
              Vertex *v = vtPair.first;
              if ((vtPair.second + rm->getVertexLatency(newToOld.at(v))) > maxTime) {
                  maxTime = (vtPair.second + rm->getVertexLatency(newToOld.at(v)));
                  //Debugging:
                  if (!quiet) {
                      cout << vtPair.first->getName() << ": " << vtPair.second << " + "
                           << rm->getVertexLatency(newToOld.at(v)) << " = " << maxTime << endl;
                  }
              }
          }catch(std::out_of_range&){
              cout << vtPair.first->getName() << ": " << vtPair.second << endl;
              throw (HatScheT::Exception("Utility::getSDCScheduleLength(): OUT_OF_RANGE"));
          }
      }
      return maxTime;
  }

  int Utility::calcMaxLatencyEstimation(Graph* g, ResourceModel* rm,
                                        const map<Vertex*, int> &tMin,
                                        const map<Vertex*, int> &tMax,
                                        const int &II, bool quiet) {

      // Algorithm to estimate max latency based on earliest and latest possible start times
      // as well as on ressource constraints.
      map<pair<Vertex*, int>, bool> vertexTimeslot;
      unordered_map<Vertex*, bool> checked;

      // Strange stuff which allows me to reuse the getSDCScheduleLength() function... :roll_eyes:
      // ToDo: If there is time, we can rewrite getSDCScheduleLength() to get rid of this ugly
      //       piece of code.
      unordered_map<Vertex*, Vertex*> helperMap;
      unordered_map<Vertex*, int> tMaxUnordered;
      for (auto &v : tMax){
          helperMap[v.first] = v.first;
          tMaxUnordered[v.first] = v.second;
      }

      int length = getSDCScheduleLength(tMaxUnordered, helperMap, rm);

      // Generates a matrix with the information wether a Vertex can be schedules in a timeslot or not,
      // based on dependency constraints.
      for (auto &v : g->Vertices()){
          checked.insert(std::make_pair(v,false));
          int tAsap = tMin.at(v);
          int tAlap = tMax.at(v);
          for (int i = 0; i < tAsap; i++){
              vertexTimeslot[{v, i}] = false;
          }
          for (int i = tAsap; i <= tAlap; i++){
              vertexTimeslot[{v, i}] = true;
          }
          for (int i = tAlap + 1; i < length; i++){
              vertexTimeslot[{v, i}] = false;
          }
      }

      // Trys to satisfy ressource constraints. If they can not be satisfied, there will be operations
      // that can not be scheduled.
      for (auto &r : rm->Resources()){
          if (r->isUnlimited()){
              continue;
          }
          vector<int>usedFuInModslot;
          usedFuInModslot.resize(II);
          for (int i = 0; i < length; i++){
              for (auto &cvp : rm->getVerticesOfResource(r)){
                  auto v = (Vertex*) cvp;
                  if (!vertexTimeslot.at(std::make_pair(v, i))){
                      continue;
                  }
                  // Check conflicts with other mod-slots.
                  if (usedFuInModslot.at(i % II) < r->getLimit() and !checked.at(v)){
                      //Check vertex and count up used Resources:
                      checked.at(v) = true;
                      usedFuInModslot.at(i % II)++;
                      continue;
                  }
                  // Set vertices of resource in the same timeslot to 0, because there are no more FUs.
                  vertexTimeslot.at(std::make_pair(v, i)) = false;
              }
          }
          usedFuInModslot.clear();
          usedFuInModslot.shrink_to_fit();
      }
      //Counting the not scheduled operations to expand the schedule by that amount.
      deque<int>criticalRessources;
      for (auto &r : rm->Resources()) {
          if (r->isUnlimited()){
              continue;
          }
          int count2 = 0;
          for (auto &v : rm->getVerticesOfResource(r)) {
              int count = 0;
              for (int i = 0; i < length; i++) {
                  count += (int)vertexTimeslot.at(std::make_pair((Vertex*)v, i));
              }
              if (count == 0){
                  count2++;
              }
          }
          criticalRessources.push_back(count2);
      }

      int maximum = *std::max_element(criticalRessources.begin(), criticalRessources.end());

      int maxLatency = maximum + length;
      if ( !quiet ) { cout << "Max Latency Suggestion: " << maximum + length << endl; }

      return maxLatency;
  }

  bool Utility::calcMinLatencyEstimation(Graph* g, ResourceModel* rm,
                                         const map<Vertex*, int> &tMin,
                                         const map<Vertex*, int> &tMax,
                                         const int &II,
                                         int &increment,
                                         int &minLatency,
                                         int &modAsapLength,
                                         bool quiet){

	  // Comperator to compare pairs of <Vertex*, int>, needed for priority queue (pq).
      struct VertexIntCompLess {
        constexpr bool operator()(
            pair<Vertex*, int> const &a,
            pair<Vertex*, int> const &b)
        const noexcept {
            return a.second < b.second;
        }
      };

      // Comperator to compare pairs of <int, int>, needed for priority queue (fspq).
      struct IntIntComp {
        bool operator()(
            pair<int, int> const &a,
            pair<int, int> const &b)
        const noexcept {
            return a.second < b.second;
        }
      };

      // Algorithm to estimate min latency based on earliest and latest possible start times
      // as well as on ressource constraints.
      // It creates a schedule based on SDC-starttimes and on resource contraints.
      map<pair<Vertex*, int>, bool> vertexTimeslot;
      unordered_map<Resource*, bool> checkedResource;

      for (auto &r : rm->Resources()){
          checkedResource[r] = false;
      }

      //Generates a matrix with the information wether a Vertex can be schedules in a timeslot or not,
      //based on dependency constraints.
      for (auto &v : g->Vertices()){
          int t_asap = tMin.at(v);
          int t_alap = tMax.at(v);
          for (int i = 0; i < t_asap; i++){
              vertexTimeslot[{v, i}] = false;
          }
          for (int i = t_asap; i <= t_alap; i++){
              vertexTimeslot[{v, i}] = true;
          }
          for (int i = t_alap + 1; i < minLatency; i++){
              vertexTimeslot[{v, i}] = false;
          }
      }

      // Schedules all operation with satisfied resource constraints. So we can get a requiered min. latency
      // for a scheduling.
      for (auto &r : rm->Resources()) {
          if (r->isUnlimited()) {
              continue;
          }

          auto verticesOfThisResource = rm->getVerticesOfResource(r);

          // Preprocessing: Reducing the problem to one itteration interval.
          // to make sure resource Contraints are satisfied for each modulo slot.
          for (int x = 0; x < min(II, minLatency); x++) {
              for (auto &cvp : verticesOfThisResource) {
                  auto vp = (Vertex *) cvp;
                  if (vertexTimeslot.at({vp, x})) {
                      for (int i = x + 1; i < minLatency; i++) {
                          try {
                              if ((i % II) != x) {
                                  continue;
                              }
                              vertexTimeslot.at({vp, i}) = false;
                          } catch (std::out_of_range &) {
                              cout << "i: " << i << " Out of Range A" << endl;
                              cout << "Utility::calcMinLatencyEstimation(): Out of Range, " << vp->getName()
                                   << " " << x << endl << "Min. Latency: " << minLatency << endl;
                              throw (HatScheT::Exception("Utility::calcMinLatencyEstimation(): Out of Range A"));
                          }
                      }
                  } else {
                      for (int i = x + 1; i < minLatency; i++) {
                          try {
                              if ((i % II) != x) {
                                  continue;
                              }
                              if (vertexTimeslot.at({vp, i})) {
                                  vertexTimeslot.at({vp, i}) = false;
                                  vertexTimeslot.at({vp, x}) = true;
                              }
                          } catch (std::out_of_range &) {
                              cout << "i: " << i << endl;
                              cout << "Utility::calcMinLatencyEstimation(): Out of Range, " << vp->getName()
                                   << " " << x << endl << "Min. Latency: " << minLatency << endl;
                              throw (HatScheT::Exception(
                                  "Utility::calcMinLatencyEstimation(): Out of Range B"));
                          }
                      }
                  }
              }
          }

          // 1. Count how many FUs are potentially used in each modulo timeslot.
          // 2. Count how many potential timeslots for each vertex are existing.
          vector<int> usedFuInModslot;
          usedFuInModslot.resize(II);
          map<Vertex *, int> availibleSlots;
          for (auto &cvp : verticesOfThisResource) {
              auto vp = (Vertex*) cvp;
              availibleSlots[vp] = 0;
              for (int i = 0; i < minLatency; i++) {
                  if (!vertexTimeslot.at({vp, i})) {
                      continue;
                  }
                  usedFuInModslot.at(i % II)++;
                  availibleSlots.at(vp)++;
              }
          }

          // Check if resource constraints are satisfied:
          for (int x = 0; x < min(II, minLatency); x++) {
              if (usedFuInModslot.at(x) <= r->getLimit()) {
                  // No conflict, do nothing.
                  continue;
              }
              // Potential conflict:
              // If there is a potential conflict, we have to create a mapping between the operations in the
              // mod-slot and their "mobility".
              // These operation are now sorted in a priority queue based on their mobility (availible slots).
              // Higher mobility means an earlier position in the PQ.
              // Then we take the operation from the PQ and move them to the mod-slot with the fewest used FUs,
              // until the resource contraint for the actual slot id satisfied or there are no more available slots
              // where the operation can be moved.
              priority_queue<pair<Vertex *, int>, vector<pair<Vertex *, int>>, VertexIntCompLess> pq;
              for (auto &cvp : verticesOfThisResource) {
                  auto vp = (Vertex *) cvp;
                  pq.push({vp, availibleSlots.at(vp)});
              }
              // We track used FUs for each mod-slot so we can compare the used FUs from the last iteration to the used
              // FUs of the current iteration. If used FUs of the previous iteration are the same as the used FUs in
              // Current iteration, we know that we can not satisfy the resource constraint, and we have to increase
              // min. Latency
              int usedFUsAtActualSlot = 0;

              // While-Loop to reduce used FUs until resource constraints are satisfied or no further reduction of used
              // FUs is possible.
              while (usedFuInModslot.at(x) > r->getLimit()) {
                  if (usedFuInModslot.at(x) == usedFUsAtActualSlot) {
                      // If resource constraints can't be satisfied, then we calculate an increment. Which will be added
                      // to min. latency and break the loop.
                      increment = (int)floor(usedFuInModslot.at(x)/r->getLimit()) - r->getLimit();
                      if (increment < 1){
                          increment = 1;
                      }
                      if (!quiet) { cout << "break equal" << endl; }
                      break;
                  }
                  usedFUsAtActualSlot = usedFuInModslot.at(x);
                  // Operation with most available slot will be moved away from the actual slot, to the slot with
                  // least amount uf used FUs.
                  while(!pq.empty()) {
                      if (usedFuInModslot.at(x) <= r->getLimit()){
                          break;
                      }
                      auto currOp = pq.top();
                      pq.pop();
                      Vertex *y = currOp.first;
                      if (!vertexTimeslot.at({y, x})){
                          continue;
                      }
                      priority_queue<pair<int, int>, vector<pair<int, int>>, IntIntComp> fspq;
                      for (int i = 0; i < min((int)usedFuInModslot.size(), minLatency); i++) {
                          int temp = (r->getLimit() - usedFuInModslot.at(i)) * (int) vertexTimeslot.at({y, i});
                          fspq.push({i, temp});
                      }
                      while (!fspq.empty()) {
                          auto curr = fspq.top();
                          fspq.pop();
                          if (curr.first == x) {
                              continue;
                          }
                          if (vertexTimeslot.at({y, curr.first})) {
                              if (vertexTimeslot.at({y, x})) {
                                  vertexTimeslot.at({y, x}) = false;
                                  usedFuInModslot.at(x)--;
                                  availibleSlots.at(y)--;
                              }
                          }
                      }
                  }
              }
              // Check if resource constraints can be satisfied
              if (usedFuInModslot.at(x) > r->getLimit()){
                  //If not, break and increase latency
                  if (!quiet) {
                      cout << "Conflict at " << x << " Used FUs: " << usedFuInModslot.at(x) << endl;
                      cout << "Not Feasiable... Increase Latency!" << endl;
                  }
                  return false;
              }else{
                  checkedResource.at(r) = true;
                  for (auto &cvp : verticesOfThisResource){
                      auto vp = (Vertex*)cvp;
                      if (vertexTimeslot.at({vp, x})){
                          for (int i = 0; i < min(II, minLatency); i++){
                              if (i == x){
                                  continue;
                              }
                              if (vertexTimeslot.at({vp, i})) {
                                  vertexTimeslot.at({vp, i}) = false;
                                  availibleSlots.at(vp)--;
                                  usedFuInModslot.at(i)--;
                              }
                          }
                      }
                  }
              }
          }
      }
      return true;
  }
}
