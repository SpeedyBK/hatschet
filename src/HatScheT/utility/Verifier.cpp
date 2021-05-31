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
                                                 map<HatScheT::Vertex *, int> &schedule, int SL)
{
  return HatScheT::verifyModuloSchedule(g, rm, schedule, SL+1); // +1 to be safe (depends on the interpretation of SL)
}

bool HatScheT::verifyModuloSchedule(Graph &g, ResourceModel &rm,
                                    std::map<Vertex *, int> &schedule, int II)
{
  if(II<=0){
    cout << "HatScheT.verifyModuloSchedule Error invalid II passed to verifier: "  << II << endl;
    return false;
  }
  if(schedule.empty()==true){
    cout << "HatScheT.verifyModuloSchedule Error empty schedule provided to verifier!"  << endl;
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
      cout << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance() << "*" << II << endl;
      cerr << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance() << "*" << II << endl;    
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
        cout << "The following " << vs.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in congruence class " << m << " (mod " << II << "):" << endl;
        cerr << "The following " << vs.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in congruence class " << m << " (mod " << II << "):" << endl;      
        for (auto v : vs){
          cout << *v << " (t=" << S[v] << ")" << endl;
          cerr << *v << " (t=" << S[v] << ")" << endl;}
        return false;
      }
    }
  }


  /* 3) TODO: cycle-time constraints are obeyed */

  return true;
}

bool HatScheT::verifyRationalIIModuloSchedule2(HatScheT::Graph &g, HatScheT::ResourceModel &rm,
                                              vector<map<HatScheT::Vertex *, int>> &schedule, vector<int> latencySequence, int scheduleLength) {

    if (latencySequence.size() == 0) {
        cout << "HatScheT.verifyRationalIIModuloSchedule Error empty II vector passed to verifier!" << endl;
        return false;
    }
    if (schedule.size() == 0) {
        cout << "HatScheT.verifyRationalIIModuloSchedule Error empty schedule provided to verifier!" << endl;
        return false;
    }
    if (schedule.size() != latencySequence.size()) {
        cout << "HatScheT.verifyRationalIIModuloSchedule Error schedule and II vector of incoherent  size provided!"
             << endl;
        return false;
    }
    cout << "Start verify rational II schedule: < ";
    for (int i = 0; i < latencySequence.size(); i++) {
        cout << latencySequence[i] << " ";
        if (latencySequence[i] <= 0) {
            cout << "HatScheT.verifyRationalIIModuloSchedule Error wrong latency between IIs provided: " << latencySequence[i] << endl;
            return false;
        }
    }
    cout << " >" << endl;

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
                cout << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay()
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
                cout << "Resource Constraint violated for " << r->getName() << " in modulo slot " << timeStep << " mod " << m << ": used "
                     << instancesUsed << " of " << r->getLimit() << endl;
                return false;
            }
        }
    }

    return true;
}

pair<int,int> splitRational(double x);
int safeRoundDown(double x);

bool HatScheT::verifyModuleScheduleRational(Graph &g, ResourceModel &rm,
                                    std::map<Vertex *, double> &schedule, double II)
{
    if(II<=0){
        cout << "HatScheT.verifyModuloScheduleRational Error invalid II passed to verifier: "  << II << endl;
        return false;
    }
    if(schedule.empty()==true){
        cout << "HatScheT.verifyModuloScheduleRational Error empty schedule provided to verifier!"  << endl;
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
                cout << *e << " violated: " << a_start << " + " << rm.getVertexLatency(a) << " + " << e->getDelay() << " <= " << b_start<< " In Iteration: " << i << endl;
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
            cout.precision(17);
            cout << v->getName() << " i =  " <<  i << " i*II = " << i*II << " Start = " << S[v] << " sum = " << safeRoundDown(S[v] + i*II) << endl;
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
                cout << "The following " << used.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in cycle " << i << ":" << endl;
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

bool HatScheT::verifyRationalIIModuloSchedule(Graph &g, ResourceModel &rm, vector<std::map<Vertex *, int>> &schedule, int samples, int modulo) {
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
        cout << *e << " violated: " << schedule[i][vDst] << " - " << schedule[sampleIndex][vSrc] << " - "
             << rm.getVertexLatency(vSrc) << " + " << offset << " >= 0 (sample " << i << ")" << endl;
        cerr << *e << " violated: " << schedule[i][vDst] << " - " << schedule[sampleIndex][vSrc] << " - "
             << rm.getVertexLatency(vSrc) << " + " << offset << " >= 0 (sample " << i << ")" << endl;
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
    for (int timeStep = 0; timeStep <= modulo; timeStep++) {
      int instancesUsed = 0;
      //iterate over schedules
      for (auto S : schedule) {

        //iterate over vertices
        for (auto it2 = g.verticesBegin(); it2 != g.verticesEnd(); ++it2) {
          Vertex *v = *it2;
          //vertex of other resource
          if (rm.getResource(v) != r) continue;
          //other timestep assigned
          if ((S[v] % modulo) != timeStep) continue;
          else instancesUsed++;
        }
      }

      if (instancesUsed > r->getLimit()) {
        cout << "Resource Constraint violated for " << r->getName() << " in modulo slot " << timeStep << " mod " << modulo << ": used "
             << instancesUsed << " of " << r->getLimit() << endl;
        return false;
      }
    }
  }

  return true;
}

bool HatScheT::verifyIntIIBinding(Graph *g, ResourceModel *rm, map<Vertex *, int> sched, int II,
		Binding::BindingContainer bind, map<Edge *, int> portAssignments, set<const Resource *> commutativeOps) {
	// check if a resource is assigned more than one operation per modulo slot
	// map <pair<resource type, FU number> -> map of modulo slots in which this FU is busy to the vertex that occupies it>
	std::map<std::pair<const Resource*,int>,std::map<int,Vertex*>> busyResources;
	for(auto it : bind.resourceBindings) {
		auto v = const_cast<Vertex*>(it.first);
		auto fu = it.second;
		auto t = sched[v];
		auto m = t % II;
		auto r = rm->getResource(v);
		auto alreadyBusy = busyResources[{r,fu}];
		try {
			auto conflictVertex = alreadyBusy.at(m);
			std::cout << "Found resource conflict for resource '" << r->getName() << "' in modulo slot '" << m
								<< "': vertices '" << conflictVertex->getName() << "' and '" << v->getName()
								<< "' occupy it in the same time slot" << std::endl;
			return false;
		}
		catch(std::out_of_range) {
			busyResources[{r,fu}][m] = v;
		}
		catch(...) {
			throw HatScheT::Exception("HatScheT::verifyIntIIBinding: something went TERRIBLY wrong when searching for resource conflicts");
		}
	}

	// check if port assignments for non-commutative operations are obeyed
	if(portAssignments.empty() or bind.fuConnections.empty()) {
		std::cout << "HatScheT::verifyIntIIBinding: no port assignments passed - will skip evaluation for those" << std::endl;
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
		auto fuSrc = bind.resourceBindings[vSrc];
		auto fuDst = bind.resourceBindings[vDst];
		auto tSrc = sched[vSrc];
		auto tDst = sched[vDst];
		auto latSrc = rSrc->getLatency();
		auto dist = e->getDistance();
		auto wantedPort = portAssignments[e];
		auto lifetime = tDst - tSrc - latSrc + dist*II;
		bool allGood = false;
		for(auto &it : bind.fuConnections) {
			if(it.first.first.first != rSrc) continue;
			if(it.first.first.second != fuSrc) continue;
			if(it.first.second.first != rDst) continue;
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
			auto fuSrc = bind.resourceBindings[vSrc];
			auto fuDst = bind.resourceBindings[vDst];
			auto tSrc = sched[vSrc];
			auto tDst = sched[vDst];
			auto latSrc = rSrc->getLatency();
			auto dist = e->getDistance();
			auto wantedPort = portAssignments[e];
			requestedPorts.insert(wantedPort);
			auto lifetime = tDst - tSrc - latSrc + dist*II;
			bool foundIt = false;
			for(auto &it : bind.fuConnections) {
				if(it.first.first.first != rSrc) continue;
				if(it.first.first.second != fuSrc) continue;
				if(it.first.second.first != rDst) continue;
				if(it.first.second.second != fuDst) continue;
				if(it.second.first != lifetime) continue;
				foundIt = true;
				actualPorts.insert(it.second.second);
				break;
			}
			if(!foundIt) {
				std::cout << "Could not find a port assignment for edge '" << vSrc->getName() << "' -> '" << vDst->getName()
									<< "' (commutative)" << std::endl;
				return false;
			}
		}
		if(requestedPorts.size() != actualPorts.size()) {
			std::cout << "Found invalid port assignments for commutative operation '" << vDst->getName() << "'" << std::endl;
		}
	}

	return true;
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