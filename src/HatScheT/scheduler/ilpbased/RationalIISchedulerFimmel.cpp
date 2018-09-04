#include <HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h>
//#include <HatScheT/scheduler/ASAPScheduler.h>
//#include <HatScheT/utility/Utility.h>
#include <iostream>
#include <ScaLP/Solver.h>

namespace HatScheT
{
RationalIISchedulerFimmel::RationalIISchedulerFimmel(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
        : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
{

}

void RationalIISchedulerFimmel::generateTestSetup() {
    cout << "Generating Test Graph" << endl;
    HatScheT::Graph* testGraph = new  HatScheT::Graph();
    Vertex* S[11];
    for (int i = 1; i <= 10; i++) {
        Vertex* v = &testGraph->createVertex(i);
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
    Resource* rm = &testResource->makeResource("Add", 2, 2);
    for (int i = 1; i <= 10; i++) {
        testResource->registerVertex(S[i], rm);
    }
    resourceModel = *testResource;

    this->minII = this->computeMinII(&g,&resourceModel);
    cout << "minII: " << this->minII << endl;

    cout << "Finished Generating Test Graph" << endl;
}

void RationalIISchedulerFimmel::schedule() {
    this->generateTestSetup();

    ScaLP::Solver s{"CPLEX", "Gurobi", "SCIP"};

    ScaLP::Variable Lambda = ScaLP::newRealVariable("Lambda", 1, ScaLP::INF());

    s.setObjective(ScaLP::minimize(Lambda));

    int p_max = 3;
    int Lambda_max = 100;

    map<Vertex*, ScaLP::Variable > C;
    //map<Vertex*, ScaLP::Variable > C_;
    map<Vertex*, ScaLP::Variable > c;
    map<Vertex*, ScaLP::Variable > k;

    map<tuple<Vertex*,int>,ScaLP::Variable > beta;

    map<tuple<Resource*,Vertex*, int>, ScaLP::Variable> kappa;
    //map<tuple<Vertex*,Vertex*>, int> Schlange;


    cout << "a" << endl;


    //6.1
    for(auto it=this->g.verticesBegin();it!=this->g.verticesEnd();++it){
        Vertex* v = *it;
        cout << *v << endl;
        int id = v->getId();
        ScaLP::Variable curr_C = ScaLP::newRealVariable("C[" + to_string(id) + "]", 0, ScaLP::INF());
        ScaLP::Variable curr_C_ = ScaLP::newRealVariable("C_[" + to_string(id) + "]",0, ScaLP::INF());
        ScaLP::Variable curr_c = ScaLP::newRealVariable("c[" + to_string(id) + "]",0, ScaLP::INF());
        ScaLP::Variable curr_k = ScaLP::newIntegerVariable("k[" + to_string(id) + "]", 0, p_max);

        C[v] = curr_C;
        c[v] = curr_c;
        s << (curr_C_ + curr_c - curr_C == 0);


        ScaLP::Term term = 0;

        /// C_ == k*Lambda
        for (int k = 0; k <=p_max; k++) {
            ScaLP::Variable curr_beta = ScaLP::newBinaryVariable("beta[" + to_string(id) + "][" + to_string(k) + "]", 0, 1);
            s << (k*Lambda - (1 - curr_beta)*k*Lambda_max - curr_C_ <= 0);
            s << (k*Lambda + (1-curr_beta)*(p_max-k)*Lambda_max - curr_C_ >= 0);
            term = term + curr_beta;
            //ScaLP::Variable curr_beta
        }
        s << (term == 1);




        //6.2
        term = 0;
        for(auto it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesEnd();++it){
            Resource* r= *it;
            for(int u = 1; u <= r->getLimit(); u++) {
                ScaLP::Variable curr_kappa = ScaLP::newBinaryVariable("kappa[" + r->getName() + "][" + to_string(v->getId()) + "][" + to_string(u) + "]", 0, 1);
                term = term + curr_kappa;

                kappa[make_tuple(r,v,u)] = curr_kappa;
            }
        }

        s << (term == 1);



    }

    cout << "b" << endl;


    //6.3
    for(auto it=this->g.edgesBegin();it!=this->g.edgesEnd();++it){
        Edge* e= *it;
        Vertex* source = &e->getVertexSrc();
        Vertex* dest = &e->getVertexDst();
        int distance = e->getDistance();
        ScaLP::Term term = 0;
        for(auto it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesEnd();++it){
            Resource* r= *it;
            for(int i=1; i <= r->getLimit(); i++) {
                term = term + kappa[make_tuple(r,source,i)]*r->getLatency();
            }
        }
        s << (term - (Lambda*distance + C[dest] - C[source]) <= 0);
    }



    cout << "c" << endl;


    //6.4
    for(auto it=this->g.verticesBegin();it!=this->g.verticesEnd();++it) {
        Vertex *v = *it;
        for (auto it2 = this->g.verticesBegin(); it2 != this->g.verticesEnd(); ++it2) {
            Vertex *w = *it2;
            if (w != v) {
                ScaLP::Variable curr_Schlange = ScaLP::newBinaryVariable("Schlange[" + to_string(v->getId()) + "][" + to_string(w->getId()) + "]", 0, 1);
                for(auto it3=this->resourceModel.resourcesBegin();it3!=this->resourceModel.resourcesEnd();++it3){
                    Resource* r= *it3;
                    set<const Vertex*> calculates = resourceModel.getVerticesOfResource(r);
                    if (calculates.find(v) != calculates.end() && calculates.find(w) != calculates.end() ) {
                        for (int i = 1; i <= r->getLimit(); i++) {
                            int curr_OuS = 1;       //TODO: Dont hardcode this!!!
                            s << (curr_OuS - (curr_Schlange + 2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*2*Lambda_max - (c[v] - c[w] ) <= 0);
                            s << (curr_OuS - (2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*curr_OuS - (Lambda + c[w] - c[v]) <= 0);
                            s << (curr_OuS - (2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*curr_OuS - (Lambda + c[v] - c[w]) <= 0);
                            s << (curr_OuS - (1 - curr_Schlange + 2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*2*Lambda_max - (c[w] - c[v]) <= 0);
                        }
                    }
                }
            }
        }
    }


    //cout << "#######" << endl;
    //for(auto it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesEnd();++it){
    //    Resource* r= *it;
    //}


    /*
    ScaLP::Variable* C = new ScaLP::Variable[g.getNumberOfEdges()]();

    s.setObjective(ScaLP::minimize(Lambda));



    s << (Lambda >= 0);
    for(auto it=this->g.verticesBegin();it!=this->g.verticesEnd();++it){
        Vertex* v = *it;
        C[v->getId() -1] = ScaLP::newRealVariable("C[" + to_string(v->getId() -1) +"]");
        s << (C[v->getId() - 1] >= 0);
    }
    for(auto it=this->g.edgesBegin();it!=this->g.edgesEnd();++it){
        Edge* e= *it;
        Vertex* source = &e->getVertexSrc();
        s << (e->getDistance()*Lambda + C[e->getVertexDst().getId()-1] - C[source->getId()-1] >= resourceModel.getVertexLatency(source));
    }

    for(auto it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesBegin();++it){
        Resource* r= *it;
    }
    */
    cout << s.showLP() << endl;
    cout << "##############" << endl;
    cout << "Solver start" << endl;
    ScaLP::status r = s.solve();
    ScaLP::Result res = s.getResult();
    cout << res << endl;
    cout << r << endl;

    //delete[] C;
}

}
