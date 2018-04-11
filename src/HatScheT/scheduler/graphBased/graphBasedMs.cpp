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

    ASAPScheduler* asap = new ASAPScheduler(g,resourceModel);

    asap->schedule();



    int n = asap->getScheduleLength();

    cout << "Scheduling length is " << n << endl;

    n = 7;

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

    for (int j = 0;j<n;j++)
    {
        for (int i = 0;i<n;i++)
        {
            conflMatrix[i][j] = sampleVec[i];
        }

        tempSort = sampleVec[6];
        for (int i = n-1;i>=1;i--)
        {
            sampleVec[i] = sampleVec[i-1];
        }
        sampleVec[0] = tempSort;
    }

    conflMatrix[0][4] = 1;
    //    conflMatrix[2][3] = 1;

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

        HatScheT::Graph* conflGraph = new HatScheT::Graph();
        vector<HatScheT::Vertex*> conflVertices;
        //        vector<int> numEdges(n);
        //    vector<int> conflVertices(7);

        for (int i = 0; i<n;i++)
        {
            //       Vertex *conflVertices[i] = new Vertex;
            conflVertices.push_back(&conflGraph->createVertex(i));
        }

        for (int j = 0;j<n;j++)
        {
            int firstOne = -1;

            for (int i = 0;i<n;i++)
            {
                if ((conflMatrix[j][i] == 1) & (firstOne == -1)){firstOne = i; continue;}
                if (conflMatrix[j][i] == 1)
                {

                    conflGraph->createEdge(*conflVertices[firstOne],*conflVertices[i]);

                    cout << "Created Edge between Sample " << firstOne << " and Sample " << i << endl;
                }
            }
        }


        cout << "Number of overall connections: " << conflGraph->getNumberOfEdges() << endl;

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



        vector<int> sampleTimes = this->getSampleTimesWithHeuristic(conflGraph, sampleVertices);

        for (int i = 0; i != sampleTimes.size(); ++i){
            cout << sampleTimes[i] << "\t";}
        cout << endl;



    }
}

Vertex* GraphBasedMs::getVertexWithLeastNoOfConnections(Graph *g, vector<Vertex*> vVec)
{
    Vertex* returnVertex;
    int edgeCount=g->getNumberOfEdges();

    for(auto it=g->verticesBegin(); it!=g->verticesEnd(); ++it){
        Vertex* v = *it;

        int match = -1;

        for (int k = 0; k!=vVec.size(); ++k)
        {if (v==vVec[k]) {match = 1;}}

        if (match != 1) {
            int tempEdgeCount = this->getNoOfConnections(g,v,vVec);
            if(tempEdgeCount<edgeCount){
                edgeCount= tempEdgeCount;
                returnVertex = v;
                cout << "Vertex with least edges: " << v->getName() << endl;
            }
        }
    }

    return returnVertex;
}

int GraphBasedMs::getNoOfConnections(Graph *g, Vertex *v, vector<Vertex*> vVec)
{
    int number=0;

    for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it){
        Edge* e = *it;

        if(v==&e->getVertexSrc() || v==&e->getVertexDst()){

            int match = -1;

            for (int k = 0; k!=vVec.size(); ++k)
            {if (v==vVec[k]) {match = 1;}}

            if (match !=1) number++;
        }
    }

    return number;
}


vector<int> GraphBasedMs::getSampleTimesWithHeuristic(Graph *g, vector<Vertex*> vVec){

    vector<int> sampleInsertTimes;

    Vertex* tempVertex = GraphBasedMs::getVertexWithLeastNoOfConnections(g, vVec);

    int lastVecSize = sampleInsertTimes.size();

    vVec.push_back(tempVertex);
    sampleInsertTimes.push_back(tempVertex->getId());

    cout << tempVertex->getName() << endl;

    if (sampleInsertTimes.size() != 0){
        cout << "vVec size: " << vVec.size() << endl;
        cout << "sampleInsertTimes size: " << sampleInsertTimes.size() <<endl;
        cout << "lastVecSize : " << lastVecSize << endl;
        if (lastVecSize == sampleInsertTimes.size()){
            return sampleInsertTimes;
        }
        else sampleInsertTimes = GraphBasedMs::getSampleTimesWithHeuristic(g,vVec);
    }
    else{
        sampleInsertTimes.push_back(-1);
        return sampleInsertTimes;
    }
}
}
