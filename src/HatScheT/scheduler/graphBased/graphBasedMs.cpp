#include <HatScheT/scheduler/graphBased/graphBasedMs.h>
#include <vector>
#include <HatScheT/Vertex.h>
#include <HatScheT/scheduler/ASAPScheduler.h>


namespace HatScheT
{

GraphBasedMs::GraphBasedMs(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
{

}


// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/ModuloSchedExampleRM.xml --graph=../hatschet/graphMLFiles/example/ModuloSchedExample.graphml

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

    n = 7;

    // create vector of vectors (matrix) for conflicts and vector(s) for constrained ressources (missing!)
    vector<vector<int>> conflMatrix(n, vector<int>(n));
    vector<int> sampleVec(n);
    int tempSort;

    sampleVec[0] = 1;
    sampleVec[1] = 0;
    sampleVec[2] = 0;
    sampleVec[3] = 0;
    sampleVec[4] = 0;
    sampleVec[5] = 1;
    sampleVec[6] = 0;

    // write columnwise to matrix
    for (int j = 0;j<n;j++)
    {
        for (int i = 0;i<n;i++)
        {
            conflMatrix[i][j] = sampleVec[i];
        }

        // shuffle all elements down by one and copy last sample to top
        tempSort = sampleVec[6];
        for (int i = n-1;i>=1;i--)
        {
            sampleVec[i] = sampleVec[i-1];
        }
        sampleVec[0] = tempSort;
    }

    conflMatrix[0][4] = 1;
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
        //        vector<int> numEdges(n);
        //    vector<int> conflVertices(7);

        //create a vertice for each sample
        for (int i = 0; i<n;i++)
        {
            //       Vertex *conflVertices[i] = new Vertex;
            conflVertices.push_back(&conflGraph->createVertex(i));
        }

        for (int j = 0;j<n;j++)
        {
            int firstOne = -1;

            for (int i = 0;i<n;i++)
            {   //search for first one in row and skip to next column
                if ((conflMatrix[j][i] == 1) & (firstOne == -1)){firstOne = i; continue;}
                if (conflMatrix[j][i] == 1)
                {
                    // create edges between first found sample and all following samples
                    conflGraph->createEdge(*conflVertices[firstOne],*conflVertices[i]);

                    cout << "Created Edge between Sample " << firstOne << " and Sample " << i << endl;
                }
            }
        }


        cout << "Number of overall connections: " << conflGraph->getNumberOfEdges() << endl;

        // create vector for "used" vertices in heuristic approach
        vector<Vertex*> sampleVertices;

        //        sampleVertices.push_back(conflVertices[3]);

        cout << "Number of connections for vertex_0: " << GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[0],sampleVertices) << endl;
        cout << "Number of connections for vertex_1: " << GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[1],sampleVertices) << endl;
        cout << "Number of connections for vertex_2: " << GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[2],sampleVertices) << endl;
        cout << "Number of connections for vertex_3: " << GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[3],sampleVertices) << endl;
        cout << "Number of connections for vertex_4: " << GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[4],sampleVertices) << endl;
        cout << "Number of connections for vertex_5: " << GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[5],sampleVertices) << endl;
        cout << "Number of connections for vertex_6: " << GraphBasedMs::getNoOfConnections(conflGraph,conflVertices[6],sampleVertices) << endl;

        cout << "Vertex with the least amount of connections is: " << GraphBasedMs::getVertexWithLeastNoOfConnections(conflGraph,sampleVertices)->getName() << endl;

        cout << endl << endl;

        cout << "Starting heuristic now:" << endl << endl;

        // create vector for
        vector<int> sampleTimes;

        // call recursive heuristic function
        GraphBasedMs::getSampleTimesWithHeuristic(conflGraph, sampleVertices, sampleTimes);

        for (int i = 0; i != sampleTimes.size(); ++i){
            cout << sampleTimes[i] << "\t";}
        cout << endl;



    }
}

Vertex* GraphBasedMs::getVertexWithLeastNoOfConnections(Graph *g, vector<Vertex*> vVec)
{
    Vertex* returnVertex;
    int edgeCount=g->getNumberOfEdges();

    // iterate through all vertices
    for(auto it=g->verticesBegin(); it!=g->verticesEnd(); ++it){
        Vertex* v = *it;

        // used vertices can not exceed total number of vertices
        if ( vVec.size() <= g->getNumberOfVertices()){

            int match = -1;

            // compare current vertex to used vertices
            for (int k = 0; k!=vVec.size(); ++k)
            {if (v==vVec[k]) {match = 1;}}

            // if there is no conflict, get number of connections
            if (match != 1) {
                int tempEdgeCount = this->getNoOfConnections(g,v,vVec);
                // compare previous edgecount with current (temp)edgecount
                if ((tempEdgeCount<edgeCount) & (tempEdgeCount != g->getNumberOfEdges())){
                    edgeCount= tempEdgeCount;
                    returnVertex = v;
                    cout << "Vertex with least edges: " << v->getName() << endl;
                }
            }
        }
    }

    return returnVertex;
}

int GraphBasedMs::getNoOfConnections(Graph *g, Vertex *v, vector<Vertex*> vVec)
{
    int number=0;
    // iterate through all edges
    for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it){
        Edge* e = *it;

        // used vertices can not exceed total number of vertices
        if ( vVec.size() <= g->getNumberOfVertices()){

            // if current vertex is either source or destination of iterating edge then
            if(v==&e->getVertexSrc() || v==&e->getVertexDst()){

                int match = -1;

                // compare current vertex to used vertices
                for (int k = 0; k!=vVec.size(); ++k)
                {if (v==vVec[k]) {match = 1; number = g->getNumberOfEdges();}}

                // if there is no conflict, increase number of connections
                if (match !=1) number++;
            }
        }
    }

    return number;
}


void GraphBasedMs::getSampleTimesWithHeuristic(Graph *g, vector<Vertex*> vVec, vector<int> &sampleTimes){

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
        sampleTimes.push_back(-1);
        return;
    }
}

void GraphBasedMs::removeUsedVertices(Graph* g, Vertex* v, vector<Vertex*> &vVec){

    // iterate through all edges
    for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it){
        Edge* e = *it;

        // used vertices can not exceed total number of vertices
        if ( vVec.size() <= g->getNumberOfVertices()){


            int vert = 0;
            int vecSize = vVec.size();

            // if current vertex is either source or destination of iterating edge then add to used list
            if(v==&e->getVertexSrc()){

//                for (int vert = 0; vert!=vVec.size(); ++vert){
                while (vert < vecSize){
                    if (vVec[vert] != &e->getVertexDst()){
                        vVec.push_back(&e->getVertexDst());
                        cout << "Removed destination vertex : " << e->getVertexDstName() << endl;
                        vecSize = vVec.size();
//                        vert = 0;
//                        --vert;
//                        break;
                    }
//                    else {continue;}
                    ++vert;
//                    cout << vert<< endl;
                }
            }
            else if (v==&e->getVertexDst()){

//                for (int vert = 0; vert!=vVec.size(); ++vert){
                while (vert < vecSize){
                    if (vVec[vert] != &e->getVertexSrc()){
                        vVec.push_back(&e->getVertexSrc());
                        cout << "Removed source vertex : " << e->getVertexSrcName() << endl;
                        vecSize = vVec.size();
//                        vert = 0;
//                        --vert;
//                        break;
                    }
//                    else {continue;}
                    ++vert;
//                    cout << vert<< endl;
                }
            }
        }
    }
}
}
