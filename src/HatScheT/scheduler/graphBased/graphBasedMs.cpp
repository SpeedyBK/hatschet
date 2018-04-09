#include <HatScheT/scheduler/graphBased/graphBasedMs.h>
#include <vector>

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
    vector<vector<int>> conflMatrix(10, vector<int>(10));
    vector<int> sampleVec(10);
    int tempSort;

    sampleVec[0] = 1;
    sampleVec[1] = 0;
    sampleVec[2] = 0;
    sampleVec[3] = 0;
    sampleVec[4] = 0;
    sampleVec[5] = 1;
    sampleVec[6] = 0;

    for (int j = 0;j<=6;j++)
    {
        for (int i = 0;i<=6;i++)
        {
            conflMatrix[i][j] = sampleVec[i];
        }

        tempSort = sampleVec[6];

        for (int i = 6;i>=1;i--)
        {
            sampleVec[i] = sampleVec[i-1];
        }

        sampleVec[0] = tempSort;


        for (int i = 0;i<=6;i++)
        {
            cout << sampleVec[i] << endl;
        }
        cout << "Next iteration." << endl;
    }

    HatScheT::Graph* conflGraph = new HatScheT::Graph();

    for (int i = 0; i<=6;i++)
    {
        conflGraph->createVertex(i);
    }

//    for (int j = 0;j<= 6;j++)
//    {
//        for (int i = 0;i<= 6;i++)
//        {
//            if (i!=j)
//            {
//                if (conflMatrix[j][i] == 1)
//                {
//                    ;
//                }
//            }
//        }
//    }

}

}
