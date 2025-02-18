/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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

#include <HatScheT/utility/Verifier.h>
#include <HatScheT/utility/Utility.h>

#include <vector>
#include <sstream>
#include <map>
#include <iostream>
#include <cmath>

using namespace HatScheT;
using namespace std;

bool HatScheT::verifyResourceConstrainedSchedule(HatScheT::Graph &g, HatScheT::ResourceModel &rm,
                                                 map<HatScheT::Vertex *, int> &schedule, int SL, bool quiet)
{
  return HatScheT::verifyModuloSchedule(g, rm, schedule, SL+1, quiet); // +1 to be safe (depends on the interpretation of SL)
}

bool HatScheT::verifyModuloSchedule(Graph &g, ResourceModel &rm,
                                    std::map<Vertex *, int> &schedule, int II, bool quiet)
{
  if(II<=0){
    if (!quiet) cout << "HatScheT.verifyModuloSchedule Error invalid II passed to verifier: "  << II << endl;
    return false;
  }
  if(schedule.empty()==true){
    if (!quiet)cout << "HatScheT.verifyModuloSchedule Error empty schedule provided to verifier!"  << endl;
    return false;
  }
  auto &S = schedule; // alias
  bool ok;

  /* 1) precedence edges */
  for (auto it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
    auto e = *it;
    auto i = &e->getVertexSrc();
    auto j = &e->getVertexDst();

    ok = S[i] + rm.getVertexLatency(i) + e->getDelay() <= S[j] + e->getDistance() * II;
    if (! ok) {
      if (!quiet) cout << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance() << "*" << II << endl;
      if (!quiet) cerr << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance() << "*" << II << endl;
      return false;
    }
  }

  /* 2) modulo resource constraints */
  vector<map<const Resource *, vector<Vertex *>>> congruenceClassesByRes(II);
  for (auto it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
    auto v = *it;
    congruenceClassesByRes[ /* modulo slot: */ S[v] % II         ]
                          [ /* resource:    */ rm.getResource(v) ]
                          .push_back(v);
  }
  for (int m = 0; m < II; ++m) {
    auto &cc = congruenceClassesByRes[m];
    for (auto &entry : cc) {
      auto  res = entry.first;
      auto &vs  = entry.second;

      ok = res->getLimit() == UNLIMITED || vs.size() <= res->getLimit();
      if (! ok) {
        if (!quiet) cout << "The following " << vs.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in congruence class " << m << " (mod " << II << "):" << endl;
        if (!quiet) cerr << "The following " << vs.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in congruence class " << m << " (mod " << II << "):" << endl;
        for (auto v : vs){
            if (!quiet) cout << *v << " (t=" << S[v] << ")" << endl;
            if (!quiet) cerr << *v << " (t=" << S[v] << ")" << endl;
        }
        return false;
      }
    }
  }


  /* 3) TODO: cycle-time constraints are obeyed */

  return true;
}

bool HatScheT::verifyRationalIIModuloSchedule2(HatScheT::Graph &g, HatScheT::ResourceModel &rm,
                                              vector<map<HatScheT::Vertex *, int>> &schedule, vector<int> latencySequence,
                                              int scheduleLength, bool quiet) {

    if (latencySequence.size() == 0) {
        if (!quiet) cout << "HatScheT.verifyRationalIIModuloSchedule Error empty II vector passed to verifier!" << endl;
        return false;
    }
    if (schedule.size() == 0) {
        if (!quiet) cout << "HatScheT.verifyRationalIIModuloSchedule Error empty schedule provided to verifier!" << endl;
        return false;
    }
    if (schedule.size() != latencySequence.size()) {
        if (!quiet) cout << "HatScheT.verifyRationalIIModuloSchedule Error schedule and II vector of incoherent  size provided!" << endl;
        return false;
    }
    if (!quiet) cout << "Start verify rational II schedule: < ";
    for (int i = 0; i < latencySequence.size(); i++) {
        cout << latencySequence[i] << " ";
        if (latencySequence[i] <= 0) {
            if (!quiet) cout << "HatScheT.verifyRationalIIModuloSchedule Error wrong latency between IIs provided: " << latencySequence[i] << endl;
            return false;
        }
    }
    if (!quiet) cout << " >" << endl;

    int m = 0;

    for(auto it:latencySequence) {
      m += it;
    }

    /* 1) precedence edges are obeyed */
    for (int s = 0; s < schedule.size(); s++) {
        auto &S = schedule[s]; // alias
        bool ok;

        for (auto it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
            int II = 0;
            auto e = *it;
            auto i = &e->getVertexSrc();
            auto j = &e->getVertexDst();

            //determine II based on the edges distance and rational II insertions, if distance==0 omit II for this check
            if (e->getDistance() > 0) {
                int stepsBack = e->getDistance();
                int currIIPosition = s;

                while (stepsBack != 0) {
                    if (currIIPosition == 0) {
                      currIIPosition=latencySequence.size()-1;
                      II += latencySequence.back();
                    }
                    else if(currIIPosition > 0){
                      II += latencySequence[currIIPosition-1];
                      currIIPosition-=1;
                    }

                    stepsBack--;
                }

                //cout << "Distance: " << e->getDistance() << " results in a latency II sequnce of " << II << " cycles" << endl;
            }

            ok = S[i] + rm.getVertexLatency(i) + e->getDelay() <= S[j] +  II;
            if (!ok) {
                if (!quiet) cout << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay()
                     << " <= " << S[j] << " + " << II << "(sample " << s << ")" << endl;
                cerr << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay()
                     << " <= " << S[j] << " + " << II << "(sample " << s << ")" << endl;
                return false;
            }
        }
    }

    /* 2)  modulo resource constraints*/

    for (auto it = rm.resourcesBegin(); it != rm.resourcesEnd(); ++it) {
        Resource *r = *it;
        //unlimited
        if (r->getLimit() == -1) continue;

        //iterate over every timestep
        //TODO resource limit in timestep mod m !!
        for (int timeStep = 0; timeStep <= m; timeStep++) {
            int instancesUsed = 0;
            //iterate over schedules
            for (int iloop = 0; iloop < schedule.size(); iloop++) {
                auto &S = schedule[iloop]; // alias

                //iterate over vertices
                for (auto it2 = g.verticesBegin(); it2 != g.verticesEnd(); ++it2) {
                    Vertex *v = *it2;
                    //vertex of other resource
                    if (rm.getResource(v) != r) continue;
                    //other timestep assigned
                    if ((S[v] % m) != timeStep) continue;
                    else instancesUsed++;
                }
            }

            if (instancesUsed > r->getLimit()) {
                if (!quiet) cout << "Resource Constraint violated for " << r->getName() << " in modulo slot " << timeStep << " mod " << m << ": used " << instancesUsed << " of " << r->getLimit() << endl;
                return false;
            }
        }
    }

    return true;
}

pair<int,int> splitRational(double x);
int safeRoundDown(double x);

bool HatScheT::verifyModuleScheduleRational(Graph &g, ResourceModel &rm,
                                    std::map<Vertex *, double> &schedule, double II, bool quiet)
{
    if(II<=0){
        if (!quiet) cout << "HatScheT.verifyModuloScheduleRational Error invalid II passed to verifier: "  << II << endl;
        return false;
    }
    if(schedule.empty()==true){
        if (!quiet) cout << "HatScheT.verifyModuloScheduleRational Error empty schedule provided to verifier!"  << endl;
        return false;
    }
    auto &S = schedule; // alias
    bool ok;

    int latest = 0;
    for (auto it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
        auto v = *it;
        latest = max(latest, safeRoundDown(S[v]) + rm.getVertexLatency(v) );
    }
    //How many Iterations should be tested? up until phase of the latest vertex + denominator of II
    //Equivalent to cycle of latest vertex + numerator of II, divided by II

    //Before splitting the II, we check for possible double imprecisions. if the error is smaller than 0,999 we round
    if (fabs(II - (round(II))) < 0.001) { II = round(II);}
    pair<int,int> split = splitRational(II);
    if (split.second == 0) {
        //cout << "HatScheT.verifyModuloScheduleRational Error couldn't deduce rational number from: " + to_string(II) << endl;
        return false;
    }
    int max_it = latest/II + split.second;
    /* 1) precedence edges */
    for (int i = 0; i <= max_it; i++) {
        for (auto it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
            auto e = *it;
            auto a = &e->getVertexSrc();
            auto b = &e->getVertexDst();

            int a_start = safeRoundDown(S[a] + II*i);
            int b_start = safeRoundDown(S[b] + II*(i+e->getDistance()));

            ok = a_start + rm.getVertexLatency(a) + e->getDelay() <= b_start;
            if (! ok) {
                if (!quiet) cout << *e << " violated: " << a_start << " + " << rm.getVertexLatency(a) << " + " << e->getDelay() << " <= " << b_start<< " In Iteration: " << i << endl;
                cerr << *e << " violated: " << a_start << " + " << rm.getVertexLatency(a) << " + " << e->getDelay() << " <= " << b_start<< " In Iteration: " << i << endl;
                return false;
            }
        }
    }

    /* 2) modulo resource constraints are obeyed */
    map<pair<const Resource *,int>, vector<Vertex *>> ressourceSlotByCycle;
    for (int i = 0; i <= max_it; i++) {
        for (auto it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
            auto v = *it;
            if (!quiet) cout.precision(17);
            if (!quiet) cout << v->getName() << " i =  " <<  i << " i*II = " << i*II << " Start = " << S[v] << " sum = " << safeRoundDown(S[v] + i*II) << endl;
            ressourceSlotByCycle[make_pair(rm.getResource(v), safeRoundDown(S[v] + i*II) )].push_back(v);
        }
    }
    //this time, we need the actual cycles
    max_it = latest + split.first;
    for (int i = 0; i <= max_it; i++) {
        for(auto it=rm.resourcesBegin();it != rm.resourcesEnd();++it) {
            Resource *res = *it;
            auto used = ressourceSlotByCycle[make_pair(res,i)];

            ok = res->getLimit() == UNLIMITED || used.size() <= res->getLimit();
            if (! ok) {
                if (!quiet) cout << "The following " << used.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in cycle " << i << ":" << endl;
                cerr << "The following " << used.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in cycle " << i << ":" << endl;
                for (auto v : used){
                    cout << *v << " (t=" << S[v] << ")" << endl;
                    cerr << *v << " (t=" << S[v] << ")" << endl;}
                return false;
            }
        }
    }


    /* 3) TODO: cycle-time constraints are obeyed */

    return true;
}

bool HatScheT::verifyRationalIIModuloSchedule(Graph &g, ResourceModel &rm, vector<std::map<Vertex *, int>> &schedule, int samples, int modulo, bool quiet) {
  if (schedule.empty()) {
    cout << "HatScheT.verifyRationalIIModuloSchedule Error empty schedule provided to verifier!" << endl;
    return false;
  }

  /* 1) precedence edges are obeyed */
  for (auto *e : g.Edges()) {
    Vertex *vSrc = &e->getVertexSrc();
    Vertex *vDst = &e->getVertexDst();

    for (int i = 0; i < samples; i++) {
      auto so = Utility::getSampleIndexAndOffset(e->getDistance(), i, samples, modulo);
      auto sampleIndex = so.first;
      auto offset = so.second;
      if (schedule[i][vDst] - schedule[sampleIndex][vSrc] - rm.getVertexLatency(vSrc) + offset < 0) {
          if (!quiet)cout << *e << " violated: " << schedule[i][vDst] << " - " << schedule[sampleIndex][vSrc] << " - "
             << rm.getVertexLatency(vSrc) << " + " << offset << " >= 0 (sample " << i << ")" << endl;
        cerr << *e << " violated: " << schedule[i][vDst] << " - " << schedule[sampleIndex][vSrc] << " - "
             << rm.getVertexLatency(vSrc) << " + " << offset << " >= 0 (sample " << i << ")" << endl;
        return false;
      }
    }
  }

  /* 2)  modulo resource constraints*/
	std::map<const Resource*, std::map<int, int>> resInstancesUsed;
	for (auto &v : g.Vertices()) {
		auto r = rm.getResource(v);
		if (r->isUnlimited()) continue;
		for (int s=0; s<samples; s++) {
			auto t = schedule[s][v];
			auto m = t % modulo;
			resInstancesUsed[r][m]++;
			if (resInstancesUsed[r][m] > r->getLimit()) {
                if (!quiet) cout << "Resource Constraint violated for " << r->getName() << " in modulo slot " << t << " mod " << modulo << " = " << m << ": used "
						 << resInstancesUsed[r][m] << " of " << r->getLimit() << endl;
				return false;
			}
		}
	}

  return true;
}

bool HatScheT::verifyIntIIBinding(Graph *g, ResourceModel *rm, map<Vertex *, int> sched, int II,
																	Binding::RegChainBindingContainer bind, set<const Resource *> commutativeOps,
																	bool quiet) {
	// check for empty binding
	if (bind.resourceBindings.empty()) {
		if (!quiet) {
			std::cout << "HatScheT::verifyIntIIBinding: detected empty binding" << std::endl;
		}
		return false;
	}
	// check if operations of unlimited resources are executed on unique FUs
	for(auto &it : bind.resourceBindings) {
		auto v = it.first;
		auto fu = it.second;
		auto res = rm->getResource(&g->getVertexByName(v));
		if(!res->isUnlimited()) continue;
		for(auto &it2 : bind.resourceBindings) {
			auto v2 = it2.first;
			if(v == v2) continue;
			auto fu2 = it2.second;
			auto res2 = rm->getResource(&g->getVertexByName(v2));
			if(res != res2) continue;
			if(fu == fu2) {
				// FUs are equal, this should not happen!
				if (!quiet) {
					std::cout << "Operations '" << v << "' and '" << v2
										<< "' are unlimited and bound to the same resource - that should never happen!" << std::endl;
				}
				return false;
			}
		}
	}
	// check if a resource is assigned more than one operation per modulo slot
	// map <pair<resource type, FU number> -> map of modulo slots in which this FU is busy to the vertex that occupies it>
	std::map<std::pair<const Resource*,int>,std::map<int,Vertex*>> busyResources;
	for(auto &it : bind.resourceBindings) {
		auto vName = it.first;
		auto v = &g->getVertexByName(vName);
		auto fu = it.second;
		auto t = sched[v];
		auto m = t % II;
		auto r = rm->getResource(v);
		auto alreadyBusy = busyResources[{r,fu}];
		try {
			auto conflictVertex = alreadyBusy.at(m);
			if (!quiet) {
				std::cout << "Found resource conflict for resource '" << r->getName() << "', FU '" << fu << "' in modulo slot '"
									<< m << "': vertices '" << conflictVertex->getName() << "' and '" << v->getName()
									<< "' occupy it in the same time slot" << std::endl;
			}
			return false;
		}
		catch(std::out_of_range&) {
			busyResources[{r,fu}][m] = v;
		}
		catch(...) {
			throw HatScheT::Exception("HatScheT::verifyIntIIBinding: something went TERRIBLY wrong when searching for resource conflicts");
		}
	}

	// check if port assignments for non-commutative operations are obeyed
	if(bind.portAssignments.empty() or bind.fuConnections.empty()) {
		if (!quiet) {
			std::cout << "HatScheT::verifyIntIIBinding: no port assignments passed - will skip evaluation for those" << std::endl;
		}
		return true;
	}

	for (auto &v : g->Vertices()) {
		// iterate through all incoming edges of that vertex
		auto resDst = rm->getResource(v);
		// skip commutative operation types
		if(commutativeOps.find(resDst) != commutativeOps.end()) continue;
		for (auto &e : g->Edges()) {
			// ignore chaining edges
			if (!e->isDataEdge()) continue;
			// make sure to only consider incoming edges
			if (&e->getVertexDst() != v) continue;
			// check if a valid connection exists
			bool foundConnection = false;
			for (auto &it : bind.fuConnections) {
				if (it.first.second.first != resDst->getName()) continue;
				auto resSrc = rm->getResource(&e->getVertexSrc());
				if (it.first.first.first != resSrc->getName()) continue;
				auto fuSrc = bind.resourceBindings[e->getVertexSrcName()];
				if (it.first.first.second != fuSrc) continue;
				auto fuDst = bind.resourceBindings[e->getVertexDstName()];
				if (it.first.second.second != fuDst) continue;
				auto latSrc = rm->getVertexLatency(const_cast<Vertex*>(&e->getVertexSrc()));
				auto tSrc = sched[const_cast<Vertex*>(&e->getVertexSrc())];
				auto tDst = sched[const_cast<Vertex*>(&e->getVertexDst())];
				auto lifetime = tDst - tSrc - latSrc + (II * e->getDistance());
				if (it.second.first != lifetime) continue;
				auto port = bind.portAssignments[e];
				if (it.second.second != port) continue;
				foundConnection = true;
				break;
			}
			if (!foundConnection) {
				if (!quiet) {
					std::cout << "Could not find a port assignment for edge '" << e->getVertexSrcName() << "' -> '"
										<< e->getVertexDstName()
										<< "' (non-commutative)" << std::endl;
				}
				return false;
			}
		}
	}

	// check if commutative operations have valid port assignments
	for(auto &vDst : g->Vertices()) {
		auto rDst = rm->getResource(vDst);
		// skip non-commutative operation types
		if(commutativeOps.find(rDst) == commutativeOps.end()) continue;
		std::set<int> actualPorts;
		std::set<int> requestedPorts;
		for(auto &e : g->Edges()) {
			// skip chaining edges
			if(e->getDependencyType() != Edge::DependencyType::Data) continue;
			// skip edges that are not of interest
			if(&e->getVertexDst() != vDst) continue;
			// now let's check if everything is ok
			auto vSrc = &e->getVertexSrc();
			auto rSrc = rm->getResource(vSrc);
			auto fuSrc = bind.resourceBindings[vSrc->getName()];
			auto fuDst = bind.resourceBindings[vDst->getName()];
			auto tSrc = sched[vSrc];
			auto tDst = sched[vDst];
			auto latSrc = rSrc->getLatency();
			auto dist = e->getDistance();
			auto wantedPort = bind.portAssignments[e];
			if (requestedPorts.find(wantedPort) != requestedPorts.end()) {
				if (!quiet) {
					std::cout << "Corrupt port assignment container - Multiple edges are connected to port '" << wantedPort
										<< "' of '" << vDst->getName() << "'" << std::endl;
				}
			}
			requestedPorts.insert(wantedPort);
			auto lifetime = tDst - tSrc - latSrc + dist*II;
			bool foundIt = false;
			for(auto &it : bind.fuConnections) {
				if(it.first.first.first != rSrc->getName()) continue;
				if(it.first.first.second != fuSrc) continue;
				if(it.first.second.first != rDst->getName()) continue;
				if(it.first.second.second != fuDst) continue;
				if(it.second.first != lifetime) continue;
				foundIt = true;
				actualPorts.insert(it.second.second);
				// nfiege: do not break here because multiple connections can lead to different ports
				// and we might miss some if we break...
				//break;
			}
			if(!foundIt) {
				if (!quiet) {
					std::cout << "Could not find a port assignment for edge '" << vSrc->getName() << "' -> '" << vDst->getName()
										<< "' (commutative)" << std::endl;
				}
				return false;
			}
		}
		if(requestedPorts.size() != actualPorts.size()) {
			if (!quiet) {
				std::cout << "Found invalid port assignments for commutative operation '" << vDst->getName() << "'"
									<< std::endl;
			}
			return false;
		}
	}

	return true;
}

bool HatScheT::verifyRatIIBinding(Graph *g, ResourceModel *rm, std::vector<map<Vertex *, int>> sched, int samples, int modulo,
																	Binding::RatIIRegChainBindingContainer bind, map<Edge *, int> portAssignments,
																	set<const Resource *> commutativeOps) {
	// check for empty binding
	if (bind.resourceBindings.empty()) {
		std::cout << "HatScheT::verifyRatIIBinding: detected empty binding" << std::endl;
		return false;
	}
	// check if operations of unlimited resources are executed on unique FUs
	for(int s=0; s<samples; s++) {
		for(auto it : bind.resourceBindings[s]) {
			auto v = it.first;
			auto fu = it.second;
			auto res = rm->getResource(&g->getVertexByName(v));
			if(!res->isUnlimited()) continue;
			for(int s2=0; s2<samples;s2++) {
				for(auto it2 : bind.resourceBindings[s2]) {
					auto v2 = it2.first;
					if(v == v2 and s == s2) continue;
					auto fu2 = it2.second;
					auto res2 = rm->getResource(&g->getVertexByName(v2));
					if(res != res2) continue;
					if(fu == fu2) {
						// FUs are equal, this should not happen!
						std::cout << "Operation '" << v << "' of sample '" << s << "' and operation '" << v2
											<< "' of sample '" << s2
											<< "' are unlimited and bound to the same resource - that should never happen!" << std::endl;
						return false;
					}
				}
			}
		}
	}

	// check if a resource is assigned more than one operation per modulo slot
	// map <pair<resource type, FU number> -> map of modulo slots in which this FU is busy to the vertex that occupies it>
	std::map<std::pair<const Resource*,int>,std::map<int,std::pair<Vertex*,int>>> busyResources;
	for(int s=0; s<samples;s++) {
		for(auto it : bind.resourceBindings[s]) {
			auto vName = it.first;
			auto v = &g->getVertexByName(vName);
			auto fu = it.second;
			auto t = sched[s][v];
			auto m = t % modulo;
			auto r = rm->getResource(v);
			auto alreadyBusy = busyResources[{r,fu}];
			try {
				auto conflictVertex = alreadyBusy.at(m).first;
				auto conflictSample = alreadyBusy.at(m).second;
				std::cout << "Found resource conflict for resource '" << r->getName() << "' FU '" << fu << "' in modulo slot '"
					<< m << "': vertices '" << conflictVertex->getName() << "' of sample '" << conflictSample << "' and '"
					<< v->getName() << "' of sample '" << s << "' occupy it in the same time slot" << std::endl;
				return false;
			}
			catch(std::out_of_range&) {
				busyResources[{r,fu}][m] = {v,s};
			}
			catch(...) {
				throw HatScheT::Exception("HatScheT::verifyRatIIBinding: something went TERRIBLY wrong when searching for resource conflicts");
			}
		}
	}

	// check if port assignments for non-commutative operations are obeyed
	if(portAssignments.empty() or bind.fuConnections.empty()) {
		std::cout << "HatScheT::verifyRatIIBinding: no port assignments passed - will skip evaluation for those" << std::endl;
		return true;
	}

	for(auto &e : g->Edges()) {
		auto vSrc = &e->getVertexSrc();
		auto vDst = &e->getVertexDst();
		auto rSrc = rm->getResource(vSrc);
		auto rDst = rm->getResource(vDst);
		// skip commutative operation types
		if(commutativeOps.find(rDst) != commutativeOps.end()) continue;
		// skip chaining edges
		if(e->getDependencyType() != Edge::DependencyType::Data) continue;
		// continue check for non-commutative operations
		for(int sDst=0; sDst<samples; sDst++) {
			auto dist = e->getDistance();
			auto sSrc = (sDst - dist) % samples;
			auto delta = ceil((double) (dist - sDst) / (double) samples);
			auto fuSrc = bind.resourceBindings[sSrc][vSrc->getName()];
			auto fuDst = bind.resourceBindings[sDst][vDst->getName()];
			auto tSrc = sched[sSrc][vSrc];
			auto tDst = sched[sDst][vDst];
			auto latSrc = rSrc->getLatency();
			auto wantedPort = portAssignments[e];
			auto lifetime = tDst - tSrc - latSrc + delta*modulo;
			bool allGood = false;
			for(auto &it : bind.fuConnections) {
				if(it.first.first.first != rSrc->getName()) continue;
				if(it.first.first.second != fuSrc) continue;
				if(it.first.second.first != rDst->getName()) continue;
				if(it.first.second.second != fuDst) continue;
				if(it.second.first != lifetime) continue;
				if(it.second.second == wantedPort) {
					allGood = true;
					break;
				}
				else {
					std::cout << "Found illegal port assignment for non-commutative operation '" << vDst->getName()
										<< "' of type '" << rDst->getName() << "' - wanted port '" << wantedPort << "', actual port '"
										<< it.second.second << "'" << std::endl;
					return false;
				}
			}
			if(!allGood) {
				std::cout << "Could not find a port assignment for edge '" << vSrc->getName() << "' -> '" << vDst->getName()
									<< "' (non-commutative)" << std::endl;
				return false;
			}
		}
	}

	// check if commutative operations have valid port assignments
	for(auto &vDst : g->Vertices()) {
		auto rDst = rm->getResource(vDst);
		// skip non-commutative operation types
		if(commutativeOps.find(rDst) == commutativeOps.end()) continue;
		for(int sDst=0; sDst<samples; sDst++) {
			std::set<int> actualPorts;
			std::set<int> requestedPorts;
			for(auto &e : g->Edges()) {
				// skip chaining edges
				if(e->getDependencyType() != Edge::DependencyType::Data) continue;
				// skip edges that are not of interest
				if(&e->getVertexDst() != vDst) continue;
				// now let's check if everything is ok
				auto dist = e->getDistance();
				auto sSrc = (sDst - dist) % samples;
				if(sSrc < 0) sSrc += samples; // negative arguments are handled in a weird way on some machines
				auto delta = (int)ceil((double) (dist - sDst) / (double) samples);
				auto vSrc = &e->getVertexSrc();
				auto rSrc = rm->getResource(vSrc);
				auto fuSrc = bind.resourceBindings[sSrc][vSrc->getName()];
				auto fuDst = bind.resourceBindings[sDst][vDst->getName()];
				auto tSrc = sched[sSrc][vSrc];
				auto tDst = sched[sDst][vDst];
				auto latSrc = rSrc->getLatency();
				auto wantedPort = portAssignments[e];
				requestedPorts.insert(wantedPort);
				auto lifetime = tDst - tSrc - latSrc + delta*modulo;
				bool foundIt = false;
				for(auto &it : bind.fuConnections) {
					if(it.first.first.first != rSrc->getName()) continue;
					if(it.first.first.second != fuSrc) continue;
					if(it.first.second.first != rDst->getName()) continue;
					if(it.first.second.second != fuDst) continue;
					if(it.second.first != lifetime) continue;
					foundIt = true;
					actualPorts.insert(it.second.second);
					// nfiege: do not break here because multiple connections can lead to different ports
					// and we might miss some if we break...
					// break;
				}
				if(!foundIt) {
					std::cout << "Could not find a port assignment for edge '" << vSrc->getName() << "' -> '" << vDst->getName()
										<< "' (commutative)" << std::endl;
					return false;
				}
			}
			if(requestedPorts.size() != actualPorts.size()) {
				std::cout << "Found invalid port assignments for commutative operation '" << vDst->getName() << "'" << std::endl;
				return false;
			}
		}
	}

	return true;
}

bool HatScheT::verifyIntIIBinding(Graph *g, ResourceModel *rm, map<Vertex *, int> sched, int II,
																	Binding::BindingContainer bind, std::set<const Resource *> commutativeOps,
																	bool quiet) {

	// check if all edges have a *valid* port assignment (i.e., all input ports of each vertex must be occupied)
	// ATTENTION: this function expects that the port assignments container was updated
	// if ports were switched for commutative operations
	for (auto &vDst : g->Vertices()) {
		int numInputEdges = 0;
		std::set<int> occupiedInputPorts;
		for (auto &e : g->Edges()) {
			if (vDst != &e->getVertexDst()) continue;
			try {
				occupiedInputPorts.insert(bind.portAssignments.at(e).second);
			}
			catch (std::out_of_range&) {
				// edge has no port assignment
				// oh no, binding is invalid :(
				if (!quiet) {
					std::cout << "Missing port assignment for edge '" << e->getVertexSrcName() << "' -(" << e->getDistance()
										<< ")-> '" << e->getVertexDstName() << "'" << std::endl;
				}
				return false;
			}
			numInputEdges++;
		}
		if (numInputEdges != occupiedInputPorts.size()) {
			// not all input ports are occupied
			// oh no, binding is invalid :(
			if (!quiet) {
				std::cout << "Not all ports of vertex '" << vDst->getName() << "' are occupied (" << occupiedInputPorts.size()
									<< "/" << numInputEdges << ")" << std::endl;
			}
			return false;
		}
	}

	// check if all connections have valid active times
	std::map<std::tuple<std::string, int, int>, std::set<int>> activeTimes;
	for (auto &it : bind.connections) {
		for (auto t : std::get<6>(it)) {
			if (t >= II) {
				if (!quiet) {
					std::cout << "Detected invalid active time of connection '" << std::get<0>(it) << "' (" << std::get<1>(it)
										<< ") port " << std::get<2>(it) << " -> '" << std::get<3>(it) << "' (" << std::get<4>(it)
										<< ") port " << std::get<5>(it) << ": t >= II (" << t << " >= " << II << ")" << std::endl;
				}
				return false;
			}
			std::tuple<std::string, int, int> dst = {std::get<3>(it), std::get<4>(it), std::get<5>(it)};
			if (activeTimes[dst].find(t) == activeTimes[dst].end()) {
				activeTimes[dst].insert(t);
			}
			else {
				if (!quiet) {
					std::cout << "Detected duplicate active time of '" << std::get<3>(it) << "' (" << std::get<4>(it)
										<< ") input " << std::get<5>(it) << std::endl;
				}
				return false;
			}
		}
	}

	// check if all vertices are assigned to an FU
	for (auto v : g->Vertices()) {
		if (bind.resourceBindings.find(v->getName()) == bind.resourceBindings.end()) {
			// oh no, binding is invalid :(
			if (!quiet) {
				std::cout << "Vertex '" << v->getName() << "' is not assigned to an FU" << std::endl;
			}
			return false;
		}
	}

	// check if conflicting vertices are bound to the same FU
	for (auto v1 : g->Vertices()) {
		for (auto v2 : g->Vertices()) {
			// skip potential conflicts with itself
			if (v1 == v2) continue;
			// conflicts are only possible if both resource types are equal
			if (rm->getResource(v1) != rm->getResource(v2)) continue;
			// conflicts are only possible if both congruence classes are also equal
			if (sched.at(v1) % II != sched.at(v2) % II) continue;
			// we got a conflict if they are executed by the same FU
			for (auto &fu1 : bind.resourceBindings.at(v1->getName())) {
				for (auto &fu2 : bind.resourceBindings.at(v2->getName())) {
					if (fu1 == fu2) {
						// oh no, binding is invalid :(
						if (!quiet) {
							std::cout << "Conflicting vertices '" << v1->getName() << "' and '" << v2->getName()
												<< "' are bound to the same FU" << std::endl;
						}
						return false;
					}
				}
			}
		}
	}

	// check if all dependencies for all vertices are ok
	bool allDependenciesOk = true;
	for (auto &vDst : g->Vertices()) {
		// get info about this vertex
		auto rDst = rm->getResource(vDst);
		auto tDst = sched.at(vDst);
		auto fuDst = bind.resourceBindings.at(vDst->getName());
		// this is needed for commutative operation types
		// keep track which edge is connected to which input port of the FU that executes this operation
		std::map<Edge*, std::set<int>> vertexInputs;
		std::vector<Edge*> inputEdges;
		// check all input edges
		for (auto e : g->Edges()) {
			// skip input edges of other vertices
			if (vDst != &e->getVertexDst()) continue;
			// register edge as input edges of this vertex
			inputEdges.emplace_back(e);
			// just get all info about src vertex that we may potentially need
			auto vSrc = &e->getVertexSrc();
			auto rSrc = rm->getResource(vSrc);
			auto tSrc = sched.at(vSrc);
			auto fuSrc = bind.resourceBindings.at(vSrc->getName());
			// calculate lifetime
			int lifetime = tDst - tSrc - rSrc->getLatency() + e->getDistance() * II;
			// keep track of where the variable is at which time step
			std::set<std::pair<std::string, int>> currentSources; // = {{rSrc->getName(), fuSrc}};
			for (auto &it : fuSrc) {
				currentSources.insert({rSrc->getName(), it});
			}
			// number of considered lifetime register stages
			int numLifetimeRegs = 0;
			// the time step in which the variable was created
			int currentTimestep = tSrc + rSrc->getLatency();
			// search the location of the variable in the time step when the dst operation is executed
			while (lifetime > numLifetimeRegs) {
				// container to store where the variable will be in the next time step
				std::set<std::pair<std::string, int>> nextSources;

				// check all locations of the variable in the current time step
				for (auto &source : currentSources) {
					// check if it is passed to another register
					for (auto &connection : bind.connections) {
						// only check connections for which the source matches
						// skip connections for source resource type mismatch
						if (std::get<0>(connection) != source.first) continue;
						// also skip it if FU or register index does not match
						if (std::get<1>(connection) != source.second) continue;
						// also skip it if destination is not a register
						if (std::get<3>(connection) != "register") continue;
						// shortcut for the register index
						auto regIndex = std::get<4>(connection);
						// also skip it if the register is not enabled in the current time step
						if (bind.registerEnableTimes[regIndex].find(currentTimestep % II) == bind.registerEnableTimes[regIndex].end()) continue;
						// looks like we found a register that holds the variable in the next time step
						nextSources.insert({"register", regIndex});
					}

					// check if it remains where it is
					bool overwritesValue = false;
					if (source.first == "register") {
						for (auto enableTime : bind.registerEnableTimes[source.second]) {
							if (enableTime == currentTimestep % II) {
								overwritesValue = true;
								break;
							}
						}
					}
					else {
						// FUs always produce a new value
						overwritesValue = true;
					}
					if (!overwritesValue) {
						nextSources.insert(source);
					}
				}

				// update sources
				currentSources = nextSources;

				// update time step counter
				currentTimestep++;

				// update register stage counter
				numLifetimeRegs++;
			}

			// check if all destinations get their data from any of the sources
			std::set<int> foundConnection;
			for (auto &source : currentSources) {
				int expectedSrcOutputPort;
				if (source.first == "register") {
					expectedSrcOutputPort = 0;
				}
				else {
					expectedSrcOutputPort = bind.portAssignments.at(e).first;
				}
				for (auto &fu : fuDst) {
					for (auto connection : bind.connections) {
						// skip connections for source resource type mismatch
						if (std::get<0>(connection) != source.first) continue;
						// skip if FU or register index does not match
						if (std::get<1>(connection) != source.second) continue;
						// skip if src port does not match
						if (std::get<2>(connection) != expectedSrcOutputPort) continue;
						// skip if destination does not match
						if (std::get<3>(connection) != rDst->getName()) continue;
						if (std::get<4>(connection) != fu) continue;
						// skip if dst port does not match
						if (std::get<5>(connection) != bind.portAssignments.at(e).second) continue;
						// connection seems fine...
						foundConnection.insert(fu);
					}
				}
			}
			if (foundConnection != fuDst) {
				// oh no, binding is invalid :(
				if (!quiet) {
					std::cout << "Failed to find all necessary connection paths for edge '" << e->getVertexSrcName() << "' port "
										<< bind.portAssignments.at(e).first << " -(" << e->getDistance() << ")-> '" << e->getVertexDstName()
										<< "' port " << bind.portAssignments.at(e).second << std::endl;
				}
				allDependenciesOk = false;
			}
		}
	}
	return allDependenciesOk;
}


///return numerator and denominator of a rational number stored in double
///If the number is not rational, or a specific budget is hit, it returns the pair <0,0>
///As no rational number has a 0 as a denominator, this can be used to test the result
pair<int,int> splitRational(double x) {
    //int[] A =new int[12];
    //int[] B = new int[12];
    //int[] k = new int[12];
    int A0 = 1;
    int A1 = 0;
    int B0 = 0;
    int B1 = 1;
    for(int i = 0; i <1000; i++) {
        //k[i+2] = (int) ((double)1/x);
        int k = (int) ((double)1/x);
        //A[i+2] = A[i] + k[i+2] * A[i+1];
        int temp = A1;
        A1 = A0 + k*A1;
        A0 = temp;
        //B[i+2] = B[i] + k[i+2] * B[i+1];
        temp = B1;
        B1 = B0 + k*B1;
        B0 = temp;
        x = ((double)1/x) - k;
        if (abs(x)<=1.0E-3) {
            return make_pair(A1,B1);
        };
    }

    return make_pair(0,0);
}

///Safely rounds a double down to the next integer, taking precision issues into account
///Inputing 3.9999999999999968 will return 4
int safeRoundDown(double x) {
    if (x>=0) {
        double error = x - (int) x;
        if (error < 0.999) return (int) x;
        return (int) ceil(x);
    }
    double error = (int) x -x;
    if (error < 0.001) return (int) x;
    return (int) floor(x);
}