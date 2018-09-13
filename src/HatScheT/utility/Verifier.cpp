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

#include <vector>
#include <map>
#include <iostream>
#include <cmath>

using namespace HatScheT;
using namespace std;

using VertexIter = set<Vertex *>::const_iterator;
using EdgeIter   = set<Edge *>::const_iterator;

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

  /* 1) precedence edges are obeyed */
  for (EdgeIter it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
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

  /* 2) modulo resource constraints are obeyed */
  vector<map<const Resource *, vector<Vertex *>>> congruenceClassesByRes(II);
  for (VertexIter it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
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
    for (VertexIter it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
        auto v = *it;
        latest = max(latest, safeRoundDown(S[v]) + rm.getVertexLatency(v) );
    }
    //How many Iterations should be tested? up until phase of the latest vertex + denominator of II
    //Equivalent to cycle of latest vertex + numerator of II, divided by II
    pair<int,int> split = splitRational(II);
    if (split.second == 0) {
        cout << "HatScheT.verifyModuloScheduleRational Error couldn't deduce rational number from: " + to_string(II) << endl;
        return false;
    }
    int max_it = latest/II + split.second;
    /* 1) precedence edges are obeyed */
    for (int i = 0; i <= max_it; i++) {
        for (EdgeIter it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
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
        for (VertexIter it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
            auto v = *it;
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



//    /* 1) precedence edges are obeyed */
//    for (EdgeIter it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
//        auto e = *it;
//        auto i = &e->getVertexSrc();
//        auto j = &e->getVertexDst();
//
//        ok = S[i] + rm.getVertexLatency(i) + e->getDelay() <= S[j] + e->getDistance() * II;
//        if (! ok) {
//            cout << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance() << "*" << II << endl;
//            cerr << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance() << "*" << II << endl;
//            return false;
//        }
//    }

//    /* 2) modulo resource constraints are obeyed */
//    vector<map<const Resource *, vector<Vertex *>>> congruenceClassesByRes(II);
//    for (VertexIter it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
//        auto v = *it;
//        congruenceClassesByRes[ /* modulo slot: */ S[v] % II         ]
//        [ /* resource:    */ rm.getResource(v) ]
//                .push_back(v);
//    }
//    for (int m = 0; m < II; ++m) {
//        auto &cc = congruenceClassesByRes[m];
//        for (auto &entry : cc) {
//            auto  res = entry.first;
//            auto &vs  = entry.second;
//
//            ok = res->getLimit() == UNLIMITED || vs.size() <= res->getLimit();
//            if (! ok) {
//                cout << "The following " << vs.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in congruence class " << m << " (mod " << II << "):" << endl;
//                cerr << "The following " << vs.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in congruence class " << m << " (mod " << II << "):" << endl;
//                for (auto v : vs){
//                    cout << *v << " (t=" << S[v] << ")" << endl;
//                    cerr << *v << " (t=" << S[v] << ")" << endl;}
//                return false;
//            }
//        }
//    }


    /* 3) TODO: cycle-time constraints are obeyed */

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
        if (abs(x)<=1.0E-7) {
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
        if (error < 0.999999999999) return (int) x;
        return (int) ceil(x);
    }
    double error = (int) x -x;
    if (error < 0.000000000001) return (int) x;
    return (int) floor(x);
}