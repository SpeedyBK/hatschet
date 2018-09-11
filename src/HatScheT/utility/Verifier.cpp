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

  /* 1) precedence edges are obeyed */
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

  /* 2) modulo resource constraints are obeyed */
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


bool HatScheT::verifyRationalIIModuloSchedule(HatScheT::Graph &g, HatScheT::ResourceModel &rm,
                                              vector<map<HatScheT::Vertex *, int>> &schedule, vector<int> IIs) {

  /* 1) precedence edges are obeyed */

  /* 2) modulo resource constraints are obeyed */

  /* 3) TODO: cycle-time constraints are obeyed */
}