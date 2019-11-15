//
// Created by sittel on 15/11/19.
//

#include "UnrollRationalIIScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "math.h"


namespace HatScheT {

  UnrollRationalIIScheduler::UnrollRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
  : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {

    throw Exception("UnrollRationalIIScheduler::UnrollRationalIIScheduler: scheduler not finished yet!");

    this->scheduleFound = false;
    this->scheduler = ED97;

    this->computeMinII(&g,&resourceModel);
    this->minII = ceil(this->minII);
    this->computeMaxII(&g,&resourceModel);
    if (this->minII >= this->maxII) this->maxII = this->minII+1;

  }

  void UnrollRationalIIScheduler::schedule(){
    this->scheduleFound = false;
  }

  void UnrollRationalIIScheduler::unroll(int s) {
    Graph *new_g = new Graph();
    ResourceModel *new_rm = new ResourceModel();

    map<Vertex *, vector<Vertex *> > mappings;

    for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
      HatScheT::Vertex *v = *it;
      const HatScheT::Resource *r = this->resourceModel.getResource(v);

      Resource *r_new;

      vector<Vertex *> v_mapping;

      if (new_rm->resourceExists(r->getName()) == true) {
        r_new = new_rm->getResource(r->getName());
      } else {
        r_new = &new_rm->makeResource(r->getName(), r->getLimit(), r->getLatency(), r->getBlockingTime());
      }

      for (int i = 0; i < s; i++) {
        Vertex *v_new = &new_g->createVertex();
        v_new->setName(v->getName() + "_" + to_string(i));
        new_rm->registerVertex(v_new, r_new);
        v_mapping.push_back(v_new);
      }

      mappings.insert(make_pair(v, v_mapping));
    }

    for (auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
      Edge *e = *it;
      Vertex *v_src = &e->getVertexSrc();
      Vertex *v_dst = &e->getVertexDst();

      vector<Vertex *> v_src_mappings = mappings[v_src];
      vector<Vertex *> v_dst_mappings = mappings[v_dst];

      for (int i = 0; i < v_src_mappings.size(); i++) {
        if (e->getDistance() == 0)
          new_g->createEdge(*v_src_mappings[i], *v_dst_mappings[i], 0, e->getDependencyType());
        else {
          int distance = e->getDistance();

          if (distance > i) {
            int new_distance = distance - i;
            int new_port = new_distance % s;
            new_distance = ceil((double) new_distance / (double) distance);

            new_g->createEdge(*v_src_mappings[new_port], *v_dst_mappings[i], new_distance, e->getDependencyType());
          } else {
            new_g->createEdge(*v_src_mappings[i - distance], *v_dst_mappings[i], 0, e->getDependencyType());
          }
        }
      }
    }
  }
}