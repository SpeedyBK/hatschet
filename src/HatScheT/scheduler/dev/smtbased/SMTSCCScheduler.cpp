//
// Created by bkessler on 8/11/22.
//

#include <cmath>
#include <z3++.h>

#include "SMTSCCScheduler.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"

namespace HatScheT {

  SMTSCCScheduler::SMTSCCScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {

      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);

  }

  void SMTSCCScheduler::schedule() {

      // Beispiel:
      //   SRC     Kante     DST
      // Vertex_A -- 2 --> Vertex_B

      // Daraus folgt die Ungleichung:
      // B - A >= 2

      z3::context c;
      z3::expr_vector ev(c); // Vector für Dependency Constr
      z3::expr_vector oneslot_a(c); // One Slot Constraint für A
      z3::expr_vector oneslot_b(c); // One Slot Constraint für A
      vector<int>ones; // Gewichte für One Slot Constraint (immer 1)
      vector<int>factors; // Gewichte für Dependency Constraint
      for (int i = 0; i < 5; i++){ // Schedule Länge 5, willkürlich gewählt zum testen.
          factors.push_back(-(i)); // Offset muss hier sein um die 0 zu eliminieren,
          ones.push_back(1);         // da 0 * x immer 0 ist und somit keine aussage über x getroffen werden kann.
          std::stringstream expr_name;
          expr_name << "A" << i;
          z3::expr e(c.bool_const(expr_name.str().c_str()));
          ev.push_back(e);
          oneslot_a.push_back(e);
      }
      for (int i = 0; i < 5; i++){
          factors.push_back((i));
          std::stringstream expr_name;
          expr_name << "B" << i;
          z3::expr f(c.bool_const(expr_name.str().c_str()));
          ev.push_back(f);
          oneslot_b.push_back(f);
      }

      z3::solver s(c);

      if (ev.size() != factors.size() or ev.empty()){
          throw(std::out_of_range("Vector sizes not equal!"));
      }

      s.add(z3::pbeq(oneslot_a, &ones.at(0), 1)); //Nur ein Zeitslot für Vertex A
      s.add(z3::pbeq(oneslot_b, &ones.at(0), 1)); //Nur ein Zeitslot für Vertex B
      s.add(z3::pbge(ev, &factors.at(0), 2)); // Dependency Constraint

      cout << s << endl;
      cout << s.check() << endl;
      auto m = s.get_model();
      for(auto it : ev){
          cout << it << ": " << m.eval(it).is_true() << endl;
      }

      throw(HatScheT::Exception("blub"));

      modifyResourceModel();

      computeSCCs();

      cout << "Trivial: " << endl;
      for (auto &it : trivialSCCs){
          cout << it->getId() << ": " << it->getNumberOfVertices() << endl;
      }

      cout << "Basic: " << endl;
      for (auto &it : basicSCCs){
          cout << it->getId() << ": " << it->getNumberOfVertices() << endl;
      }

      cout << "Complex: " << endl;
      for (auto &it : complexSCCs){
          cout << it->getId() << ": " << it->getNumberOfVertices() << endl;
      }

      resetResourceModel();
  }

  void SMTSCCScheduler::computeSCCs() {

      KosarajuSCC kscc(g);
      auto sccs = kscc.getSCCs();

      for (auto &it : sccs) {
          if (it->getSccType(&resourceModel) == trivial) {
              trivialSCCs.insert(it);
          }
          if (it->getSccType(&resourceModel) == basic) {
              basicSCCs.insert(it);
          }
          if (it->getSccType(&resourceModel) == complex) {
              complexSCCs.insert(it);
          }
      }
  }

  void SMTSCCScheduler::modifyResourceModel() {
      for (auto &r : resourceModel.Resources()){
          resourceLimits[r]=r->getLimit();
          if (resourceModel.getNumVerticesRegisteredToResource(r) <= r->getLimit()){
              r->setLimit(UNLIMITED);
          }
      }
  }

  void SMTSCCScheduler::resetResourceModel() {

      for (auto &r : resourceLimits){
          r.first->setLimit(resourceLimits.at(r.first));
      }
  }
}