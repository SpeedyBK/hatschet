#include <HatScheT/scheduler/graphBased/graphBasedMs.h>
#include <vector>
#include <HatScheT/Vertex.h>
#include <algorithm>

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

    // int testArr[7][7];
    int n = 7;
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
        vector<HatScheT::Vertex> conflVertices;
        vector<int> numEdges(n);
    //    vector<int> conflVertices(7);

        for (int i = 0; i<n;i++)
        {
    //       Vertex *conflVertices[i] = new Vertex;
           conflVertices.push_back(conflGraph->createVertex(i));
        }

        for (int j = 0;j<n;j++)
        {
            int firstOne = -1;

            for (int i = 0;i<n;i++)
            {
                if ((conflMatrix[j][i] == 1) & (firstOne == -1)){firstOne = i; continue;}

    //                for (int k = 0;k<=6;k++)
    //                {
    //                    if ((k != i) & (k != j))
    //                    {
//                            if (conflMatrix[j][k] == 1)
                            if (conflMatrix[j][i] == 1)
                            {
    //                            if (tempEdge->getDistance() == 0)
    //                            {
//                                    conflGraph->createEdge(conflVertices[j],conflVertices[k]);
                                    conflGraph->createEdge(conflVertices[firstOne],conflVertices[i]);
                                    numEdges[firstOne]++;
                                    numEdges[i]++;
                                    cout << "Created Edge between Sample " << firstOne << " and Sample " << i << endl;
//                                    cout << "i is " << i << endl;
    //                            }
                            }
    //                    }
    //                }
            }
        }

        HatScheT::Vertex* tempVertex = NULL;
        tempVertex = &conflGraph->getVertexById(0);

        cout << "Number of edges: " << numEdges[tempVertex->getId()] << endl;
        cout << "Vertex ID: " << tempVertex->getName() << endl;
        cout << "Is sourceVertex Abfrage: " << conflGraph->isSourceVertex(tempVertex) << endl;

        std::vector<int>::iterator result = std::min_element(begin(numEdges), end(numEdges));

        cout << "Min element at: " << std::distance(begin(numEdges), result) << endl;
        cout << "The smallest value is " << *result << endl;

    }

//    HatScheT::Edge* tempEdge = NULL;

//    try
//    {
//        tempEdge = &conflGraph->getEdge(&conflVertices[0],&conflVertices[3]);
//        cout << tempEdge->getDistance() << endl;

//        HatScheT::Vertex* srcVertex;
//        HatScheT::Vertex* dstVertex;

//        srcVertex = &tempEdge->getVertexSrc();
//        dstVertex = &tempEdge->getVertexDst();

//        cout << "Source Vertex: " << srcVertex->getId() << " , Destination Vertex: " << dstVertex->getId() << endl;
//    }
//    catch (HatScheT::Exception* noEdge) {cout << "No edge exsits between 0 and 3!" << endl;};

//    }


}

Vertex* GraphBasedMs::getVertexWithLeastNoOfConnections(Graph *g)
{
    Vertex* returnVertex;
    int edgeCount=g->getNumberOfEdges();

    for(auto it=g->verticesBegin(); it!=g->verticesEnd(); ++it){
        Vertex* v = *it;
        int tempEdgeCount = this->getNoOfConnections(g,v);
        if(tempEdgeCount<edgeCount){
            edgeCount= tempEdgeCount;
            returnVertex = v;
        }
    }

    return returnVertex;
}

int GraphBasedMs::getNoOfConnections(Graph *g, Vertex *v)
{
    int number=0;

    for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it){
        Edge* e = *it;

        if(v==&e->getVertexSrc() || v==&e->getVertexDst()) number++;
    }

    return number;
}

}
