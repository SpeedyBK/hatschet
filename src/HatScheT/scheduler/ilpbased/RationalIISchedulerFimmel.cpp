#include <HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ULScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <iostream>
#include <ScaLP/Solver.h>
#include <HatScheT/utility/Verifier.h>
#include <cmath>
#include <math.h>

namespace HatScheT {
  RationalIISchedulerFimmel::RationalIISchedulerFimmel(Graph &g, ResourceModel &resourceModel,
                                                       std::list<std::string> solverWishlist)
    : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {

  }


  void RationalIISchedulerFimmel::generateTestSetup() {
    cout << "Generating Test Graph" << endl;

    HatScheT::Graph *testGraph = new HatScheT::Graph();
    Vertex *S[11];
    for (int i = 1; i <= 10; i++) {
      Vertex *v = &testGraph->createVertex(i);
      v->setName("S" + to_string(i));
      S[i] = v;
    }
    testGraph->createEdge(*S[1], *S[2], 1);
    testGraph->createEdge(*S[1], *S[2], 0);
    testGraph->createEdge(*S[2], *S[3], 1);
    testGraph->createEdge(*S[2], *S[3], 0);
    testGraph->createEdge(*S[3], *S[4], 1);
    testGraph->createEdge(*S[3], *S[4], 0);
    testGraph->createEdge(*S[4], *S[5], 2);
    testGraph->createEdge(*S[4], *S[5], 1);
    testGraph->createEdge(*S[5], *S[6], 2);
    testGraph->createEdge(*S[5], *S[6], 1);
    testGraph->createEdge(*S[6], *S[7], 0);
    testGraph->createEdge(*S[7], *S[8], 0);
    testGraph->createEdge(*S[8], *S[1], 1);
    testGraph->createEdge(*S[4], *S[9], 0);
    testGraph->createEdge(*S[9], *S[10], 0);
    testGraph->createEdge(*S[10], *S[9], 1);

    g = *testGraph;

    HatScheT::ResourceModel *testResource = new HatScheT::ResourceModel();
    Resource *rm = &testResource->makeResource("Add", 2, 2);
    for (int i = 1; i <= 10; i++) {
      testResource->registerVertex(S[i], rm);
    }
    resourceModel = *testResource;

    this->computeMinII(&g, &resourceModel);
    this->minII = ceil(this->minII);
    cout << "minII: " << this->minII << endl;

    cout << "Finished Generating Test Graph" << endl;
  }

  void RationalIISchedulerFimmel::generateTestSetup2() {

    HatScheT::Graph *testGraph = new HatScheT::Graph();
    Vertex *S[4];
    for (int i = 1; i <= 3; i++) {
      Vertex *v = &testGraph->createVertex(i);
      v->setName("S" + to_string(i));
      S[i] = v;
    }
    testGraph->createEdge(*S[1], *S[2], 0);
    testGraph->createEdge(*S[2], *S[3], 0);
    testGraph->createEdge(*S[3], *S[1], 1);
    g = *testGraph;

    HatScheT::ResourceModel *testResource = new HatScheT::ResourceModel();
    Resource *a = &testResource->makeResource("A", 1, 0, 0);
    Resource *b = &testResource->makeResource("B", 2, 1, 0);
    testResource->registerVertex(S[1], a);
    testResource->registerVertex(S[2], a);
    testResource->registerVertex(S[3], b);
    resourceModel = *testResource;

  }

  void RationalIISchedulerFimmel::schedule() {
    this->computeMinII(&g, &resourceModel);
    this->computeMaxII(&g, &resourceModel);

    auto s = solver;

    ScaLP::Variable Lambda = ScaLP::newRealVariable("Lambda", 1, ScaLP::INF());

    s->setObjective(ScaLP::minimize(Lambda));

    int p_max = 0;
    if (pmax >= 0)
      p_max = pmax;
    else {
      p_max = ceil((double) (2 * maxII) / minII);
      pmax = p_max;
    }

    if(this->quiet==false) cout << "Starting Fimmel Scheduler of " << g.getName() << " with pmax = " << pmax << " and maxII = " << maxII
         << endl;

    int Lambda_max = maxII;

    map<Vertex *, ScaLP::Variable> C;
    map<Vertex *, ScaLP::Variable> c;
    map<Vertex *, ScaLP::Variable> k;

    map<tuple<Vertex *, int>, ScaLP::Variable> beta;

    map<tuple<const Resource *, Vertex *, int>, ScaLP::Variable> kappa;

    //6.1
    for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
      Vertex *v = *it;

      int id = v->getId();
      ScaLP::Variable curr_C = ScaLP::newRealVariable("C[" + to_string(id) + "]", 0, ScaLP::INF());
      ScaLP::Variable curr_C_ = ScaLP::newRealVariable("C_[" + to_string(id) + "]", 0, ScaLP::INF());
      ScaLP::Variable curr_c = ScaLP::newRealVariable("c[" + to_string(id) + "]", 0, ScaLP::INF());
      ScaLP::Variable curr_k = ScaLP::newIntegerVariable("k[" + to_string(id) + "]", 0, p_max);

      C[v] = curr_C;
      c[v] = curr_c;
      s->addConstraint(curr_C_ + curr_c - curr_C == 0);

      //MaxLatencyConstraint
      if (this->maxLatencyConstraint >= 0)
        s->addConstraint(curr_C + this->resourceModel.getVertexLatency(v) <= this->maxLatencyConstraint);


      ScaLP::Term term = 0;

      /// C_ == k*Lambda
      for (int k = 0; k <= p_max; k++) {
        ScaLP::Variable curr_beta = ScaLP::newBinaryVariable("beta[" + to_string(id) + "][" + to_string(k) + "]", 0, 1);
        s->addConstraint(k * Lambda - (1 - curr_beta) * k * Lambda_max - curr_C_ <= 0);
        s->addConstraint(k * Lambda + (1 - curr_beta) * (p_max - k) * Lambda_max - curr_C_ >= 0);
        term = term + curr_beta;
      }
      s->addConstraint(term == 1);

      //6.2
      const Resource *r = this->resourceModel.getResource(v);

      term = 0;
      for (int i = 1; i <= r->getLimit(); i++) {
        ScaLP::Variable curr_kappa = ScaLP::newBinaryVariable(
          "kappa[" + r->getName() + "][" + to_string(v->getId()) + "][" + to_string(i) + "]", 0, 1);
        term = term + curr_kappa;

        kappa[make_tuple(r, v, i)] = curr_kappa;

      }
      if (r->getLimit() != -1) s->addConstraint(term == 1);

    }

    //6.3
    for (auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
      Edge *e = *it;
      Vertex *source = &e->getVertexSrc();
      Vertex *dest = &e->getVertexDst();
      int distance = e->getDistance();

      if (this->resourceModel.getResource(source)->getLimit() != -1) {
        ScaLP::Term term = 0;
        auto r = this->resourceModel.getResource(source);
        for (int i = 1; i <= r->getLimit(); i++) {
          term =
            term + kappa[make_tuple(r, source, i)] * (this->resourceModel.getVertexLatency(source) + e->getDelay());
        }
        s->addConstraint(term - (Lambda * distance + C[dest] - C[source]) <= 0);
      } else {
        int dis = this->resourceModel.getVertexLatency(source) + e->getDelay();
        s->addConstraint(Lambda * distance + C[dest] - C[source] >= dis);
      }
    }

    //6.4
    for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
      Vertex *v = *it;
      for (auto it2 = this->g.verticesBegin(); it2 != this->g.verticesEnd(); ++it2) {
        Vertex *w = *it2;
        auto res = this->resourceModel.getResource(v);
        if (w != v && res == this->resourceModel.getResource(w) && res->getLimit() > 0) {
          ScaLP::Variable curr_Schlange = ScaLP::newBinaryVariable(
            "Schlange[" + to_string(v->getId()) + "][" + to_string(w->getId()) + "]", 0, 1);
          for (int i = 1; i <= res->getLimit(); i++) {
            int curr_OuS = 1;

            int lat = resourceModel.getVertexLatency(w);
            if (lat == 0) curr_OuS = 1;

            s->addConstraint(curr_OuS -
                             (curr_Schlange + 2 - kappa[make_tuple(res, v, i)] - kappa[make_tuple(res, w, i)]) * 2 *
                             Lambda_max - (c[v] - c[w]) <= 0);
            s->addConstraint(curr_OuS - (2 - kappa[make_tuple(res, v, i)] - kappa[make_tuple(res, w, i)]) * curr_OuS -
                             (Lambda + c[w] - c[v]) <= 0);
            s->addConstraint(curr_OuS - (2 - kappa[make_tuple(res, v, i)] - kappa[make_tuple(res, w, i)]) * curr_OuS -
                             (Lambda + c[v] - c[w]) <= 0);
            s->addConstraint(curr_OuS -
                             (1 - curr_Schlange + 2 - kappa[make_tuple(res, v, i)] - kappa[make_tuple(res, w, i)]) * 2 *
                             Lambda_max - (c[w] - c[v]) <= 0);
          }
        }
      }
    }

    if(this->quiet==false) {
      cout << "##############" << endl;
      cout << "Solver start, timeout = " << this->solverTimeout << endl;
    }

//    //timestamp
//    this->begin = clock();
//    //solve
//    ScaLP::status r = s->solve();
//    //timestamp
//    this->end = clock();
//
//    //log time
//    if (this->solvingTime == -1.0) this->solvingTime = 0.0;
//    this->solvingTime += (double) (this->end - this->begin) / CLOCKS_PER_SEC;

    //timestamp
    startTimeTracking();
    //solve
    ScaLP::status r = s->solve();
    //timestamp
    endTimeTracking();

    ScaLP::Result res = s->getResult();
    if(this->quiet==false) {
      cout.precision(17);
      cout << "Solver finished" << endl;
      cout << "Found II = " << res.values[Lambda] << endl;
      cout << "##############" << endl;
    }


    map<ScaLP::Variable, double> variableMap = res.values;
    double lambdaValue = variableMap[Lambda];
    map<Vertex *, double> vertexValues;

    if(this->quiet==false)  cout << "Start times: " << endl;

    for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
      Vertex *v = *it;
      vertexValues[v] = variableMap[C[v]];
      this->startTimes.insert(make_pair(v, variableMap[C[v]]));
      if(this->quiet==false) cout.precision(16);
      if(this->quiet==false) cout << "C[" << v->getName() << "] = " << vertexValues[v] << endl;
    }

    this->II = lambdaValue;

    if(this->II > 0) {
      this->scheduleFound = true;
    } else {
      this->II = -1;
      if (this->quiet == false) cout << "No Schedule found!" << endl;
    }

    this->stat = r;
    if(this->quiet==false) cout << "##############" << endl;
  }

}
