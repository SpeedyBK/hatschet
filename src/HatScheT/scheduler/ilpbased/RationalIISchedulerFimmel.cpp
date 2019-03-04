#include <HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ULScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <iostream>
#include <ScaLP/Solver.h>
#include <HatScheT/utility/Verifier.h>
#include <cmath>
#include <math.h>

namespace HatScheT
{
RationalIISchedulerFimmel::RationalIISchedulerFimmel(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
        : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
{

}



    void RationalIISchedulerFimmel::generateTestSetup() {
        cout << "Generating Test Graph" << endl;


        /*
        cout << "§§§§§§§§§§§§§§§§§§§§§§§§§§§§" << endl;
        ScaLP::Solver s{"CPLEX", "Gurobi", "SCIP"};

        int lambda_max = 100;
        int p_max = 1;

        ScaLP::Variable Lambda = ScaLP::newRealVariable("Lambda", 1, ScaLP::INF());

        s.setObjective(ScaLP::minimize(Lambda));

//S1

        ScaLP::Variable C1 = ScaLP::newRealVariable("C1", 0, ScaLP::INF());
        ScaLP::Variable C_1 = ScaLP::newRealVariable("C_1", 0, ScaLP::INF());
        ScaLP::Variable c1 = ScaLP::newRealVariable("c1", 0, ScaLP::INF());

        s << (C_1 + c1 - C1 == 0);

        ScaLP::Variable beta10 = ScaLP::newBinaryVariable("beta10", 0, 1);
        ScaLP::Variable beta11 = ScaLP::newBinaryVariable("beta11", 0, 1);

        s << (0*Lambda - (1 - beta10)*0*lambda_max - C_1 <= 0);
        s << (1*Lambda - (1 - beta11)*1*lambda_max - C_1 <= 0);

        s << (0*Lambda + (1-beta10)*(p_max - 0)*lambda_max - C_1 >= 0);
        s << (1*Lambda + (1-beta11)*(p_max - 1)*lambda_max - C_1 >= 0);

        s << (beta10 + beta11 == 1);

        ScaLP::Variable kappa1A1 = ScaLP::newBinaryVariable("kappa1A1", 0, 1);
        ScaLP::Variable kappa1B1 = ScaLP::newBinaryVariable("kappa1B1", 0, 1);
        ScaLP::Variable kappa2B1 = ScaLP::newBinaryVariable("kappa2B1", 0, 1);

        s << (kappa1A1 + kappa1B1 + kappa2B1 == 1);


//S2

        ScaLP::Variable C2 = ScaLP::newRealVariable("C2", 0, ScaLP::INF());
        ScaLP::Variable C_2 = ScaLP::newRealVariable("C_2", 0, ScaLP::INF());
        ScaLP::Variable c2 = ScaLP::newRealVariable("c2", 0, ScaLP::INF());

        s << (C_2 + c2 - C2 == 0);

        ScaLP::Variable beta20 = ScaLP::newBinaryVariable("beta20", 0, 1);
        ScaLP::Variable beta21 = ScaLP::newBinaryVariable("beta21", 0, 1);

        s << (0*Lambda - (1 - beta20)*0*lambda_max - C_2 <= 0);
        s << (1*Lambda - (1 - beta21)*1*lambda_max - C_2 <= 0);

        s << (0*Lambda + (1-beta20)*(p_max - 0)*lambda_max - C_2 >= 0);
        s << (1*Lambda + (1-beta21)*(p_max - 1)*lambda_max - C_2 >= 0);

        s << (beta20 + beta21 == 1);

        ScaLP::Variable kappa1A2 = ScaLP::newBinaryVariable("kappa1A2", 0, 1);
        ScaLP::Variable kappa1B2 = ScaLP::newBinaryVariable("kappa1B2", 0, 1);
        ScaLP::Variable kappa2B2 = ScaLP::newBinaryVariable("kappa2B2", 0, 1);

        s << (kappa1A2 + kappa1B2 + kappa2B2 == 1);

//S3

        ScaLP::Variable C3 = ScaLP::newRealVariable("C3", 0, ScaLP::INF());
        ScaLP::Variable C_3 = ScaLP::newRealVariable("C_3", 0, ScaLP::INF());
        ScaLP::Variable c3 = ScaLP::newRealVariable("c3", 0, ScaLP::INF());

        s << (C_3 + c3 - C3 == 0);

        ScaLP::Variable beta30 = ScaLP::newBinaryVariable("beta30", 0, 1);
        ScaLP::Variable beta31 = ScaLP::newBinaryVariable("beta31", 0, 1);

        s << (0*Lambda - (1 - beta30)*0*lambda_max - C_3 <= 0);
        s << (1*Lambda - (1 - beta31)*1*lambda_max - C_3 <= 0);

        s << (0*Lambda + (1-beta30)*(p_max - 0)*lambda_max - C_3 >= 0);
        s << (1*Lambda + (1-beta31)*(p_max - 1)*lambda_max - C_3 >= 0);

        s << (beta30 + beta31 == 1);

        ScaLP::Variable kappa1A3 = ScaLP::newBinaryVariable("kappa1A3", 0, 1);
        ScaLP::Variable kappa1B3 = ScaLP::newBinaryVariable("kappa1B3", 0, 1);
        ScaLP::Variable kappa2B3 = ScaLP::newBinaryVariable("kappa2B3", 0, 1);

        s << (kappa1A3 + kappa1B3 + kappa2B3 == 1);


//edges

        s << (Lambda*0 + C3 - C1  - (kappa1A1 * 1 + kappa1B1 * 0 + kappa2B1 * 0) >= 0);

        s << (Lambda*0 + C3 - C2  - (kappa1A2 * 1 + kappa1B2 * 0 + kappa2B2 * 0) >= 0);

        s << (Lambda*1 + C1 - C3  - (kappa1A3 * 1 + kappa1B3 * 0 + kappa2B3 * 0) >= 0);

        s << (Lambda*1 + C2 - C3  - (kappa1A3 * 1 + kappa1B3 * 0 + kappa2B3 * 0) >= 0);


//Ressource, only for pairs <S1,S2> and <S2,S1>

        ScaLP::Variable Schlange12 = ScaLP::newBinaryVariable("Schlange12", 0, 1);
        ScaLP::Variable Schlange21 = ScaLP::newBinaryVariable("Schlange21", 0, 1);

//s=1 s'=2

        s << (1 - (Schlange12 + 2 - kappa1A1 - kappa1A2)*2*lambda_max - (c1 - c2) <= 0);
        s << (1 - (2 - kappa1A1 - kappa1A2)*1 - (Lambda + c2 - c1) <= 0);
        s << (1 - (2 - kappa1A1 - kappa1A2)*1 - (Lambda + c1 - c2) <= 0);
        s << (1 - (1 - Schlange12 + 2 - kappa1A1 - kappa1A2)*2*lambda_max - (c2 - c1) <= 0);

//s=2 s'=1

        s << (1 - (Schlange21 + 2 - kappa1A2 - kappa1A1)*2*lambda_max - (c2 - c1) <= 0);
        s << (1 - (2 - kappa1A2 - kappa1A1)*1 - (Lambda + c1 - c2) <= 0);
        s << (1 - (2 - kappa1A2 - kappa1A1)*1 - (Lambda + c2 - c1) <= 0);
        s << (1 - (1 - Schlange21 + 2 - kappa1A2 - kappa1A1)*2*lambda_max - (c1 - c2) <= 0);

        s.writeLP("example2.lp");
        ScaLP::status r = s.solve();
        ScaLP::Result res = s.getResult();

        cout << "Solver finished" << endl;
        cout << "Found II = " << res.values[Lambda] << endl;
        cout << "##############" << endl;

        cout << res << endl;
        cout << r << endl;


        cout << "§§§§§§§§§§§§§§§§§§§§§§§§§§§§" << endl;
        */















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

        /*
        HatScheT::Graph* testGraph = new  HatScheT::Graph();
        Vertex* S[6];
        for (int i = 1; i <= 5; i++) {
            if (i == 4 || i == 3) continue;
            Vertex* v = &testGraph->createVertex(i);
            v->setName("S" + to_string(i));
            S[i] = v;

        }

        testGraph->createEdge(*S[1], *S[5], 0);
        testGraph->createEdge(*S[2], *S[5], 0);
        //testGraph->createEdge(*S[3], *S[5], 0);
        //testGraph->createEdge(*S[4], *S[5], 0);
        testGraph->createEdge(*S[5], *S[1], 1);
        testGraph->createEdge(*S[5], *S[2], 1);
        g = *testGraph;

        HatScheT::ResourceModel *testResource = new HatScheT::ResourceModel();
        Resource* a = &testResource->makeResource("A", 1, 1);
        //Resource* b = &testResource->makeResource("B", 1, 1);
        Resource* c = &testResource->makeResource("C", 2, 0);
        testResource->registerVertex(S[1], a);
        testResource->registerVertex(S[2], a);
        //testResource->registerVertex(S[3], b);
        //testResource->registerVertex(S[4], b);
        testResource->registerVertex(S[5], c);
        resourceModel = *testResource;
        */
        this->computeMinII(&g,&resourceModel);
        this->minII = ceil(this->minII);
        cout << "minII: " << this->minII << endl;

        cout << "Finished Generating Test Graph" << endl;
    }

    void RationalIISchedulerFimmel::generateTestSetup2() {

        HatScheT::Graph* testGraph = new HatScheT::Graph();
        Vertex* S[4];
        for (int i = 1; i <= 3; i++) {
            Vertex* v = &testGraph->createVertex(i);
            v->setName("S" + to_string(i));
            S[i] = v;
        }
        testGraph->createEdge(*S[1], *S[2], 0);
        testGraph->createEdge(*S[2], *S[3], 0);
        testGraph->createEdge(*S[3], *S[1], 1);
        g = *testGraph;

        HatScheT::ResourceModel *testResource = new HatScheT::ResourceModel();
        Resource* a = &testResource->makeResource("A", 1, 0,0);
        Resource* b = &testResource->makeResource("B", 2, 1,0);
        testResource->registerVertex(S[1], a);
        testResource->registerVertex(S[2], a);
        testResource->registerVertex(S[3], b);
        resourceModel = *testResource;

    }

void RationalIISchedulerFimmel::schedule() {
    //this->generateTestSetup();
    //this->generateTestSetup2();

    //cout << "Starting Fimmel Scheduler of " << g.getName() << " with "
    this->computeMinII(&g,&resourceModel);
    //cout << "minII: " << this->minII << endl;


    //HatScheT::ASAPScheduler* asap = new HatScheT::ASAPScheduler(g,resourceModel);
    //asap->schedule();
    this->computeMaxII(&g, &resourceModel);
    //this->maxII = asap->getII();
    //delete asap;
    //cout << "maxII: " << this->maxII << endl;

    auto s = solver;

    ScaLP::Variable Lambda = ScaLP::newRealVariable("Lambda", 1, ScaLP::INF());

    s->setObjective(ScaLP::minimize(Lambda));

    /*
    this->minII = this->computeMinII(&g,&resourceModel);
    this->maxII = Utility::calcMaxII(&g, &resourceModel);
    if (minII >= maxII) maxII = minII+1;
    */


    int p_max = 0;
    if (pmax >=0)
        p_max = pmax;
    else {
        p_max = ceil((double) (2 * maxII) / minII);
        pmax = p_max;
    }

    cout << "Starting Fimmel Scheduler of " << g.getName() << " with pmax = " << pmax << " and maxII = " << maxII << endl;

    int Lambda_max = maxII;

    map<Vertex*, ScaLP::Variable > C;
    //map<Vertex*, ScaLP::Variable > C_;
    map<Vertex*, ScaLP::Variable > c;
    map<Vertex*, ScaLP::Variable > k;

    map<tuple<Vertex*,int>,ScaLP::Variable > beta;

    map<tuple<const Resource*,Vertex*, int>, ScaLP::Variable> kappa;
    //map<tuple<Vertex*,Vertex*>, int> Schlange;


    //cout << "a" << endl;


    //6.1
    for(auto it=this->g.verticesBegin();it!=this->g.verticesEnd();++it){
        Vertex* v = *it;
        //cout << *v << endl;
        int id = v->getId();
        ScaLP::Variable curr_C = ScaLP::newRealVariable("C[" + to_string(id) + "]", 0, ScaLP::INF());
        ScaLP::Variable curr_C_ = ScaLP::newRealVariable("C_[" + to_string(id) + "]",0, ScaLP::INF());
        ScaLP::Variable curr_c = ScaLP::newRealVariable("c[" + to_string(id) + "]",0, ScaLP::INF());
        ScaLP::Variable curr_k = ScaLP::newIntegerVariable("k[" + to_string(id) + "]", 0, p_max);

        C[v] = curr_C;
        c[v] = curr_c;
        s->addConstraint(curr_C_ + curr_c - curr_C == 0);

        //MaxLatencyConstraint
        if (this->maxLatencyConstraint >= 0)
            s->addConstraint(curr_C + this->resourceModel.getVertexLatency(v) <= this->maxLatencyConstraint);


        ScaLP::Term term = 0;

        /// C_ == k*Lambda
        for (int k = 0; k <=p_max; k++) {
            ScaLP::Variable curr_beta = ScaLP::newBinaryVariable("beta[" + to_string(id) + "][" + to_string(k) + "]", 0, 1);
            s->addConstraint(k*Lambda - (1 - curr_beta)*k*Lambda_max - curr_C_ <= 0);
            s->addConstraint(k*Lambda + (1-curr_beta)*(p_max-k)*Lambda_max - curr_C_ >= 0);
            term = term + curr_beta;
            //ScaLP::Variable curr_beta
        }
        s->addConstraint(term == 1);




        //6.2
        const Resource* r= this->resourceModel.getResource(v);
        //cout << "$" << v->getName() << " " << r->getName() << " " << r->getLimit() << " " << r->getLatency() << endl;
        term = 0;
        for (int i = 1; i <= r->getLimit(); i++) {
            ScaLP::Variable curr_kappa = ScaLP::newBinaryVariable(
                    "kappa[" + r->getName() + "][" + to_string(v->getId()) + "][" + to_string(i) + "]", 0, 1);
            term = term + curr_kappa;

            kappa[make_tuple(r, v, i)] = curr_kappa;

        }
        if (r->getLimit() != -1)  s->addConstraint(term == 1);

        /*
        if (r->getLimit() != -1) {
            term = 0


            for(auto it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesEnd();++it){
                r = *it;
                for (int u = 1; u <= r->getLimit(); u++) {
                ScaLP::Variable curr_kappa = ScaLP::newBinaryVariable(
                        "kappa[" + r->getName() + "][" + to_string(v->getId()) + "][" + to_string(u) + "]", 0, 1);
                term = term + curr_kappa;

                kappa[make_tuple(r, v, u)] = curr_kappa;
            }
            }

            s << (term == 1);

        }
*/

    }

    //cout << "b" << endl;

    //6.3
    for(auto it=this->g.edgesBegin();it!=this->g.edgesEnd();++it){
        Edge* e= *it;
        Vertex* source = &e->getVertexSrc();
        Vertex* dest = &e->getVertexDst();
        int distance = e->getDistance();

        if (this->resourceModel.getResource(source)->getLimit() != -1) {
            ScaLP::Term term = 0;
            //for (auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
                auto r = this->resourceModel.getResource(source);
                for (int i = 1; i <= r->getLimit(); i++) {

                    //cout << r->getName()  << source->getName() << i << endl;
                    //cout << kappa[make_tuple(r, source, i)] << endl;
                    term = term + kappa[make_tuple(r, source, i)] * (this->resourceModel.getVertexLatency(source)+e->getDelay());
                }
            //}
            s->addConstraint(term - (Lambda * distance + C[dest] - C[source]) <= 0);
        } else {
            int dis = this->resourceModel.getVertexLatency(source)+e->getDelay();
            s->addConstraint(Lambda*distance +  C[dest] - C[source]  >= dis);
            //cout << "§§§§" << C[dest] << C[source] << this->resourceModel.getVertexLatency(source) << e->getDelay() << endl;

        }
    }

    //cout << "c" << endl;

    //6.4
    for(auto it=this->g.verticesBegin();it!=this->g.verticesEnd();++it) {
        Vertex *v = *it;
        for (auto it2 = this->g.verticesBegin(); it2 != this->g.verticesEnd(); ++it2) {
            Vertex *w = *it2;
            auto res = this->resourceModel.getResource(v);
            if (w != v && res == this->resourceModel.getResource(w) && res->getLimit() > 0) {
                ScaLP::Variable curr_Schlange = ScaLP::newBinaryVariable("Schlange[" + to_string(v->getId()) + "][" + to_string(w->getId()) + "]", 0, 1);
                for (int i = 1; i <= res->getLimit(); i++) {
                    int curr_OuS = 1;       //TODO: Dont hardcode this!!!
                    //int curr_OuS_ = 1;

                    int lat = resourceModel.getVertexLatency(w);
                    if (lat == 0 ) curr_OuS = 1;

                    s->addConstraint(curr_OuS - (curr_Schlange + 2 - kappa[make_tuple(res,v,i)] - kappa[make_tuple(res,w,i)])*2*Lambda_max - (c[v] - c[w] ) <= 0);
                    s->addConstraint(curr_OuS - (2 - kappa[make_tuple(res,v,i)] - kappa[make_tuple(res,w,i)])*curr_OuS - (Lambda + c[w] - c[v]) <= 0);
                    s->addConstraint(curr_OuS - (2 - kappa[make_tuple(res,v,i)] - kappa[make_tuple(res,w,i)])*curr_OuS - (Lambda + c[v] - c[w]) <= 0);
                    s->addConstraint(curr_OuS - (1 - curr_Schlange + 2 - kappa[make_tuple(res,v,i)] - kappa[make_tuple(res,w,i)])*2*Lambda_max - (c[w] - c[v]) <= 0);
                }





                /* Original, in case the possibility of multiple ressources for one Vertex gets added
                for(auto it3=this->resourceModel.resourcesBegin();it3!=this->resourceModel.resourcesEnd();++it3){
                    Resource* r= *it3;
                    set<const Vertex*> calculates = resourceModel.getVerticesOfResource(r);
                    if (calculates.find(v) != calculates.end() && calculates.find(w) != calculates.end() ) {
                        for (int i = 1; i <= r->getLimit(); i++) {
                            cout << i << endl;
                            int curr_OuS = 1;       //TODO: Dont hardcode this!!!
                            //int curr_OuS_ = 1;

                            int lat = resourceModel.getVertexLatency(w);
                            if (lat == 0 ) curr_OuS = 0;

                            s << (curr_OuS - (curr_Schlange + 2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*2*Lambda_max - (c[v] - c[w] ) <= 0);
                            s << (curr_OuS - (2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*curr_OuS - (Lambda + c[w] - c[v]) <= 0);
                            s << (curr_OuS - (2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*curr_OuS - (Lambda + c[v] - c[w]) <= 0);
                            s << (curr_OuS - (1 - curr_Schlange + 2 - kappa[make_tuple(r,v,i)] - kappa[make_tuple(r,w,i)])*2*Lambda_max - (c[w] - c[v]) <= 0);
                        }
                    }
                }
                 */
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
    //cout << s.showLP() << endl;
    cout << "##############" << endl;
    cout << "Solver start, timeout = " << this->solverTimeout << endl;

    //s.timeout = this->solverTimeout;
    //s->writeLP("example.lp");
    //s.threads = 1;
    ScaLP::status r = s->solve();
    ScaLP::Result res = s->getResult();

    cout.precision(17);
    cout << "Solver finished" << endl;
    cout << "Found II = " << res.values[Lambda] << endl;
    cout << r << endl;
    cout << "##############" << endl;

    cout << res << endl;

    cout << "##############" << endl;

    map<ScaLP::Variable, double> variableMap = res.values;
    double lambdaValue = variableMap[Lambda];
    map<Vertex*, double> vertexValues;
    cout << "Start times: " << endl;
    for(auto it=this->g.verticesBegin();it!=this->g.verticesEnd();++it) {
        Vertex *v = *it;
        vertexValues[v] = variableMap[C[v]];
        this->startTimes.insert(make_pair(v, variableMap[C[v]]));
        cout.precision(16);
        cout << "C["<< v->getName() << "] = " << vertexValues[v] << endl;
    }
    this->II = lambdaValue;

    bool ok = verifyModuleScheduleRational(g,resourceModel,vertexValues,lambdaValue);
    if (ok) {
        cout << "Schedule is correct" << endl;
        this->scheduleFound = true;
    } else {
        cout << "Schedule is faulty" << endl;
        this->II = -1;
        this->startTimes.clear();
    }
    this->stat = r;
    cout << "##############" << endl;
    //delete[] C;
}

}
