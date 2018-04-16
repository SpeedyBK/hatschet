#include <HatScheT/scheduler/graphBased/graphBasedMs.h>
#include <vector>
#include <HatScheT/Vertex.h>
#include <HatScheT/scheduler/ASAPScheduler.h>


namespace HatScheT
{

GraphBasedMs::GraphBasedMs(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
{

}

// ModuloSchedExample
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/ModuloSchedExampleRM.xml --graph=../hatschet/graphMLFiles/example/ModuloSchedExample.graphml

// MoovacExample
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/MoovacExampleRM.xml --graph=../hatschet/graphMLFiles/example/MoovacExample.graphml

// ASAPHCExample
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/ASAPHCExampleRM.xml --graph=../hatschet/graphMLFiles/example/ASAPHCExample.graphml

void GraphBasedMs::schedule()
{
    //..insert lennarts project work
    cout << "Hello: Here is Lennarts scheduler!" << endl;
    cout << "The inserted graph has " << g.getNumberOfVertices() << " vertices!" << endl;

    if(this->g.isEmpty()==true) cout << "The inserted graph is empty!" << endl;
    if(this->g.getNumberOfVertices()>=1) cout << "The inserted graph has more than one vertex!" << endl;

    // Create new ASAPScheduler pointer and schedule current graph
    ASAPScheduler* asap = new ASAPScheduler(g,resourceModel);

    asap->schedule();


    // Get Asap schedule length
    int n = asap->getScheduleLength();

    cout << "Scheduling length is " << n << endl;
    cout << "Start times of vertices: " << endl;


//    for (auto it=g.verticesBegin(); it!=g.verticesEnd();++it){
//        Vertex* anotherVertexPointer = *it;
//        cout << "Start time of " << anotherVertexPointer->getName() << " : " << asap->getStartTime(*anotherVertexPointer) << endl;
//        const Resource* r= this->resourceModel.getResource(anotherVertexPointer);
//        if(r->getLimit() != -1){
//            cout << r->getName() << " is limited resource with limit " << r->getLimit() << endl;
//        }
//    }

    vector<Vertex*> Adders;
    vector<Vertex*> Multipliers;
    vector<Vertex*> Gayn;

    vector<string> constrRessources;

    for (int i = 1; i!=g.getNumberOfVertices()+1;++i){
        Vertex& anotherVertexPointer = g.getVertexById(i);
        cout << "Start time of " << anotherVertexPointer.getName() << " : " << asap->getStartTime(anotherVertexPointer) << endl;
        const Resource* r= this->resourceModel.getResource(&anotherVertexPointer);

        bool isMember = false;
        for (int j = 0; j!= constrRessources.size();++j){
            if (constrRessources[j] == r->getName()){
                isMember = true;
            }
        }
        if (!isMember){
                if (r->getLimit() != -1){
                constrRessources.push_back(r->getName());
                }
        }


        if(r->getLimit() != -1){
            cout << r->getName() << " is limited resource with limit " << r->getLimit() << endl;

            if (r->getName() == "Adder"){
                Adders.push_back(&anotherVertexPointer);
            }
            else if (r->getName() == "Multiplier"){
                Multipliers.push_back(&anotherVertexPointer);
            }
            else if (r->getName() == "Gain"){
                Gayn.push_back(&anotherVertexPointer);
            }
        }
    }

    cout << "Number of constrained Ressources: " << constrRessources.size() << endl;
    //    n = 7;

    vector<vector<int>> sampleResults(constrRessources.size());

    for (int constrIt = 0; constrIt != constrRessources.size();++constrIt){

        // create vector of vectors (matrix) for conflicts and vector(s) for constrained ressources (missing!)
        vector<vector<int>> conflMatrix(n, vector<int>(n));
        vector<int> sampleVec(n);
        int tempSort;

        vector<Vertex*>* currRessource = NULL;

        if (constrRessources[constrIt] == "Adder"){
            currRessource = &Adders;
        }
        else if (constrRessources[constrIt] == "Multiplier"){
            currRessource = &Multipliers;
        }
        else if (constrRessources[constrIt] == "Gain"){
            currRessource = &Gayn;
        }


        for (int j = 0; j != n; ++j){
            sampleVec[j] = 0;
        }

        for (int j = 0; j != currRessource->size(); ++j){
            Vertex* tempVertex = NULL;

            tempVertex = currRessource->at(j);
            sampleVec[asap->getStartTime(*tempVertex)] = 1;
        }


        // write columnwise to matrix
        for (int j = 0;j!=n;j++)
        {
            for (int i = 0;i!=n;i++)
            {
                conflMatrix[i][j] = sampleVec[i];
            }

            // shuffle all elements in timing vector down by one and copy last sample to top
            tempSort = sampleVec[6];
            for (int i = n-1;i>=1;i--)
            {
                sampleVec[i] = sampleVec[i-1];
            }
            sampleVec[0] = tempSort;
        }

        //    conflMatrix[0][4] = 1;
        //    conflMatrix[2][3] = 1;

        // print matrix row-wise
        for (int j = 0; j<n; j++)
        {
            for (int i = 0; i<n; i++)
            {
                cout << conflMatrix[j][i] << "\t";
            }
            cout << endl;
        }

        int debug = 0;

        if (debug == 0)
        {

            // create new graph pointer and vector of vertices
            HatScheT::Graph* conflGraph = new HatScheT::Graph();
            vector<HatScheT::Vertex*> conflVertices;

            // create a vertice for each sample
            for (int i = 0; i<n;i++)
            {
                conflVertices.push_back(&conflGraph->createVertex(i));
            }

            for (int j = 0;j<n;j++)
            {
                int firstOne = -1;

                for (int i = 0;i<n;i++){

                    // search for first one in row and skip to next column
                    if ((conflMatrix[j][i] == 1) & (firstOne == -1)){firstOne = i; continue;}
                    if (conflMatrix[j][i] == 1)
                    {
                        // create edges between first found sample and all following samples
                        conflGraph->createEdge(*conflVertices[firstOne],*conflVertices[i]);

                        cout << "Created Edge between Sample " << firstOne << " and Sample " << i << endl;
                    }
                }
            }


            cout << endl << "Number of overall connections: " << conflGraph->getNumberOfEdges() << endl;

            // create vector for "used" vertices in heuristic approach
            vector<Vertex*> sampleVertices;

            //        sampleVertices.push_back(conflVertices[3]);

            vector<int> NoOfConnCheck(n);

            for (int i = 0; i!=conflGraph->getNumberOfVertices();++i){
                NoOfConnCheck[i] = GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[i],sampleVertices);
                cout << "Number of connections for vertex_" << i << ": " << NoOfConnCheck[i] << endl;
            }

            // create vector for recursive function call
            vector<int> sampleTimes;

            int ConnSum = 0;

            for (int i = 0; i!= NoOfConnCheck.size(); ++i){
                ConnSum = ConnSum + NoOfConnCheck[i];
            }

            if (ConnSum != 0){

                cout << "Vertex with the least amount of connections is: " << GraphBasedMs::getVertexWithLeastNoOfConnections(conflGraph,sampleVertices)->getName() << endl;

                cout << endl << endl;

                cout << "Starting heuristic now:" << endl << endl;

                // call recursive heuristic function
                GraphBasedMs::getSampleTimesWithHeuristic(conflGraph, sampleVertices, sampleTimes);
            }

            else {
                cout << endl << "There are no conflicts for this constrained resource. All sample times are available." << endl;

                for (int i = 0; i != n; ++i){
                    sampleTimes.push_back(i);
                }
            }

            //        for (int i = sampleTimes.size()-1; i >= 0; --i){
            //            cout << sampleTimes[i] << "\t";}

            cout << endl;

            // print result
            for (int i = 0; i != sampleTimes.size(); ++i){
                cout << sampleTimes[i] << "\t";
            }

            sampleResults[constrIt] = sampleTimes;

            cout << endl << endl << "End of scheduling for " << constrRessources[constrIt] <<endl<<endl<<endl;
        }
    }

//    for (int i = 0;i!= constrRessources.size();++i){
//        for (int j = 0; j!= sampleResults[i].size();++j){

//            cout << sampleResults[i][j] << "\t";

//        }
//        cout << endl;
//    }


    vector<int> result;

    if (constrRessources.size() != 1){
        vector<int> tempSampleTimes = sampleResults[0];

//        for (int i = 0; i!= tempSampleTimes.size(); ++i){
//            cout << tempSampleTimes[i] << "\t";
//        } cout << endl;

//        cout << tempSampleTimes.size() << endl;

        for (int i = 1; i!=constrRessources.size();++i)
        {
            for (int j = 0; j!=tempSampleTimes.size();++j)
            {
                if (isMemberInVector<int>(sampleResults[i],tempSampleTimes[j]))
                {
                    if (!isMemberInVector<int>(result,tempSampleTimes[j]))
                    {
                        result.push_back(tempSampleTimes[j]);
                    }
                }
            }
            tempSampleTimes = result;
        }
    }
    else {result = sampleResults[0];}

    for (int i = 0; i!= result.size(); ++i){
        cout << result[i] << "\t";
    } cout << endl <<endl;
}


Vertex* GraphBasedMs::getVertexWithLeastNoOfConnections(Graph *g, vector<Vertex*> vVec)
{
    Vertex* returnVertex = NULL;
    int edgeCount=g->getNumberOfEdges()+1;

    // iterate through all vertices
    for(auto it=g->verticesBegin(); it!=g->verticesEnd(); ++it){
        Vertex* v = *it;

        // used vertices can not exceed total number of vertices
        if ( vVec.size() < g->getNumberOfVertices()){

            // if vertex hasn't been used already
            if (!GraphBasedMs::isVertexInVector(vVec,v)){
                cout << "default edges: " << edgeCount << endl;
                int tempEdgeCount = this->getNoOfConnections(g,v,vVec);

                // compare previous edgecount with current (temp)edgecount
//                if ((tempEdgeCount<edgeCount) && (tempEdgeCount != g->getNumberOfEdges())){
                if (tempEdgeCount<=edgeCount){
                    edgeCount= tempEdgeCount;
                    returnVertex = v;
                }
            }
        }
    }
    cout << "Vertex with least edges: " << returnVertex->getName() << endl;
    return returnVertex;
}

int GraphBasedMs::getNoOfConnections(Graph *g, Vertex *v, vector<Vertex*> vVec)
{
    int number=0;
    // iterate through all edges
    for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it){
        Edge* e = *it;

        // used vertices can not exceed total number of vertices
        if ( vVec.size() < g->getNumberOfVertices()){



            // if current vertex is either source or destination of iterating edge then
            if(v==&e->getVertexSrc() || v==&e->getVertexDst()){

                // if current vertex is used (shouldn't happen) return 0
                if (this->isVertexInVector(vVec,v))
                {return 0;}

                ++number;

                // if vertex is connected to used vertex, reduce number of connections by 1
                if ((this->isVertexInVector(vVec,&e->getVertexSrc())) ||
                    (this->isVertexInVector(vVec,&e->getVertexDst())))
                { --number;}
            }
        }
    }

    return number;
}


void GraphBasedMs::getSampleTimesWithHeuristic(Graph *g, vector<Vertex*> vVec, vector<int> &sampleTimes){

    if (vVec.size() < g->getNumberOfVertices()){

        // print number of all connections
        for (int i = 0; i!=g->getNumberOfVertices();++i){
            cout << "Number of connections for vertex_" << i << ": " << GraphBasedMs::getNoOfConnections(g,&g->getVertexById(i),vVec) << endl;
        }
        cout << endl;

        // print used vertices
        for (int i = 0;i != vVec.size();++i)
        {cout << "Used vertice nr." << i << " : " << vVec[i]->getName() << endl; }
        cout << endl;

        Vertex* tempVertex = GraphBasedMs::getVertexWithLeastNoOfConnections(g, vVec);

        // add vertex with least number of connections to "used" vertices
        vVec.push_back(tempVertex);

        GraphBasedMs::removeUsedVertices(g,tempVertex,vVec);

        // get last vector size
        int lastVecSize = sampleTimes.size();

        cout << "Current sample: " << tempVertex->getId() << endl;

        // add sample time of vertex to sampling time vector
        sampleTimes.push_back(tempVertex->getId());

        // if there are any samples
        if (sampleTimes.size() != 0){
            cout << "vVec size: " << vVec.size() << endl;
            cout << "sampleTimes size: " << sampleTimes.size() <<endl;
            cout << "lastVecSize : " << lastVecSize << endl << endl;

            // if nothing changed between last function call and current function call
            if ((lastVecSize == sampleTimes.size()) || (sampleTimes.size() == g->getNumberOfVertices()) ){
                return;
            }
            else GraphBasedMs::getSampleTimesWithHeuristic(g,vVec,sampleTimes);
        }
        else{
            sampleTimes.push_back(0);
            return;
        }
    }
}

void GraphBasedMs::removeUsedVertices(Graph* g, Vertex* v, vector<Vertex*> &vVec){

    // iterate through all edges
    for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it){
        Edge* e = *it;

        int vecSize = vVec.size();

        // used vertices can not exceed total number of vertices
        if ( vecSize <= g->getNumberOfVertices()){

            // if current vertex is either source or destination of iterating edge then add to used list
            if(v==&e->getVertexSrc()){

                // if vertex is not found, add to vector
                if (!GraphBasedMs::isVertexInVector(vVec,&e->getVertexDst())){
                    vVec.push_back(&e->getVertexDst());
                    cout << "Removed destination vertex : " << e->getVertexDstName() << endl;
                    vecSize = vVec.size();
                }
            }
            else if (v==&e->getVertexDst()){

                // if vertex is not found, add to vector
                if (!GraphBasedMs::isVertexInVector(vVec,&e->getVertexSrc())){
                    vVec.push_back(&e->getVertexSrc());
                    cout << "Removed source vertex : " << e->getVertexSrcName() << endl;
                    vecSize = vVec.size();
                }
            }
        }
    }
}

bool GraphBasedMs::isVertexInVector(vector<Vertex*> vVec, Vertex *v){

    for (int i=0;i!=vVec.size(); ++i){
        if (vVec[i] == v) {return true;}
    }
    return false;
}

template<typename T> bool GraphBasedMs::isMemberInVector(vector<T> vVec, T v){

    for (int i=0;i!=vVec.size(); ++i){
        if (vVec[i] == v) {return true;}
    }

    return false;
}
}
