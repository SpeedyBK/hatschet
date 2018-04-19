#include <HatScheT/scheduler/graphBased/graphBasedMs.h>
#include <vector>
#include <HatScheT/Vertex.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <algorithm>
#include <HatScheT/MoovacScheduler.h>

namespace HatScheT
{

GraphBasedMs::GraphBasedMs(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
{

}

// ModuloSchedExample
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/ModuloSchedExampleRM.xml --graph=../hatschet/graphMLFiles/example/ModuloSchedExample.graphml

// MoovacExample
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/MoovacExampleRM.xml --graph=../hatschet/graphMLFiles/example/MoovacExample.graphml

// MoovacExample_lennart
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/MoovacExampleRM_lennart.xml --graph=../hatschet/graphMLFiles/example/MoovacExample_lennart.graphml

// ASAPHCExample
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/ASAPHCExampleRM.xml --graph=../hatschet/graphMLFiles/example/ASAPHCExample.graphml

// BackedgeExample
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/example/BackedgeExampleRM.xml --graph=../hatschet/graphMLFiles/example/BackedgeExample.graphml

//create .dot graph by adding --dot to the end

void GraphBasedMs::schedule()
{
    //..insert lennarts project work
// cout << "Hello: Here is Lennarts scheduler!" << endl;
// cout << "The inserted graph has " << g.getNumberOfVertices() << " vertices!" << endl;
//    MoovacScheduler ms(g,resourceModel,{"CPLEX"});
//    ms.schedule();

//    cout << "Moovac II is " << ms.getII() << endl;

    // Create new ASAPScheduler pointer and schedule current graph
    ASAPScheduler* asap = new ASAPScheduler(g,resourceModel);

    asap->schedule();

//    asap->printStartTimes();
    // calculate minII from Graph
<<<<<<< HEAD
    int minII = Utility::calcRecMII(&resourceModel,&g);
=======
    int minII = Utility::calcRecMII(&resourceModel, &g);
>>>>>>> 5645e279967fd8a43960ab7ed06be6e9e4824f00
    cout << "calcRecMinII: " << minII << endl;

    // Get Asap schedule length
    int n = asap->getScheduleLength();

// cout << "Scheduling length is " << n << endl;
// cout << "Start times of vertices: " << endl;


    vector<string> constrRessources;

    // get number and names of constrained ressources
    for (int i = 1; i!=g.getNumberOfVertices()+1;++i){
        Vertex& anotherVertexPointer = g.getVertexById(i);
// cout << "Start time of " << anotherVertexPointer.getName() << " : " << asap->getStartTime(anotherVertexPointer) << endl;
        const Resource* r= this->resourceModel.getResource(&anotherVertexPointer);

        if (!isMemberInVector(constrRessources,r->getName())){
            if (r->getLimit() != -1){
                constrRessources.push_back(r->getName());
            }
        }
    }

    // add vertices to a constraint matrix
    vector<vector<Vertex*>> constrRessourceVertices(constrRessources.size());

    for (int i = 0; i!= constrRessources.size();++i){
        for (int j=0;j!=g.getNumberOfVertices();++j){

            Vertex& tempVertexPointer = g.getVertexById(j+1);
            const Resource* tempR= this->resourceModel.getResource(&tempVertexPointer);

            if (constrRessources[i] == tempR->getName()){
                constrRessourceVertices[i].push_back(&tempVertexPointer);
            }
        }
    }


    // create result matrix with number of constrained ressources as rows and variable length columns
//    vector<vector<int>> sampleResults(constrRessources.size());

    vector<Graph*> allGraphs;


    // iterate through constrained ressources
    for (int constrIt = 0; constrIt != constrRessources.size();++constrIt){

        // create vector of vectors (matrix) for conflicts and vector(s) for constrained ressources
        vector<vector<int>> conflMatrix(n, vector<int>(n));
        vector<int> sampleVec(n);


        // create pointer and set it to current constrained ressource vector
        vector<Vertex*>* currRessource = NULL;
        int currConstraint = -1;

        currRessource = &constrRessourceVertices[constrIt];

        const Resource* r= this->resourceModel.getResource(currRessource->at(0));
        currConstraint = r->getLimit();


        // create vector with sample times
        for (int j = 0; j != n; ++j){
            sampleVec[j] = 0;
        }

        // "inject" ones at ressource start times
        for (int j = 0; j != currRessource->size(); ++j){
            Vertex* tempVertex = NULL;

            tempVertex = currRessource->at(j);
            sampleVec[asap->getStartTime(*tempVertex)] = 1;
        }


        conflMatrix = this->createConflictMatrix(conflMatrix,sampleVec,n);


        // create new graph pointer and vector of vertices
        HatScheT::Graph* conflGraph = new HatScheT::Graph();
        vector<HatScheT::Vertex*> conflVertices;

        this->createConflictGraph(*conflGraph,conflVertices,conflMatrix,currConstraint,n,minII);

        allGraphs.push_back(conflGraph);

    }

    // create vector for "used" vertices in heuristic approach
    vector<vector<Vertex*>> usedVertices(allGraphs.size(),vector<Vertex*>());


        // get number of connections for all vertices once
        int NoOfConnCheck = 0;

        for (int j = 0; j != allGraphs.size();++j){
            Graph* conflGraph = allGraphs[j];

            for (int i = 0; i!=conflGraph->getNumberOfVertices();++i){
                Vertex* tmpVertex = &conflGraph->getVertexById(i);
//                cout << "Connections for vertex_" << i << " : " << this->getNoOfConnections(conflGraph,tmpVertex,usedVertices[j]) << endl;
                NoOfConnCheck += this->getNoOfConnections(conflGraph,tmpVertex,usedVertices[j]);
            }
        }

        // create result vector for recursive function call
        vector<int> sampleTimes;

        // create sum of all connections to check if any conflicts exist
//        int ConnSum = 0;

//        for (int i = 0; i!= NoOfConnCheck.size(); ++i){
//            ConnSum = ConnSum + NoOfConnCheck[i];
//        }

        if (NoOfConnCheck != 0){
            // call recursive heuristic function
            GraphBasedMs::getSampleTimesWithHeuristic4All(allGraphs, usedVertices, sampleTimes);
        }

        else {
            // if no conflicts exist, all sample times are available
            for (int i = 0; i != n; ++i){
                sampleTimes.push_back(i);
            }
        }

        cout << endl;

        // sort resulting sampleTimes
        std::sort(sampleTimes.begin(),sampleTimes.end());

        // print result
//        for (int i = 0; i != sampleTimes.size(); ++i){
//            cout << sampleTimes[i] << "\t";
//        }

        // write result for each resource to result matrix
//        sampleResults[constrIt] = sampleTimes;

//        cout << endl << endl << "End of scheduling for " << constrRessources[constrIt] <<endl<<endl<<endl;




    // calculate intersecting sample times of each resource to check which times
//    vector<int> result;

//    if (constrRessources.size() != 1){
//        vector<int> tempSampleTimes = sampleResults[0];

//        for (int i = 1; i!=constrRessources.size();++i)
//        {
//            if (tempSampleTimes.size() != 0){
//                for (int j = 0; j!=tempSampleTimes.size();++j)
//                {
//                    if (isMemberInVector<int>(sampleResults[i],tempSampleTimes[j]))
//                    {
//                        if (!isMemberInVector<int>(result,tempSampleTimes[j]))
//                        {
//                            result.push_back(tempSampleTimes[j]);
//                        }
//                    }
//                }
//                tempSampleTimes = result;
//            }
//            else {
//                result.clear();
//                result[0] = 0;
//                break;
//            }
//        }
//    }
//    else {result = sampleResults[0];}

//    cout << "Final result: " << endl;

    // check validity of the result
    bool resultIsTrue = true;

    if (sampleTimes.size() > 1){

        for (int i = 0; i!= sampleTimes.size(); ++i){
            cout << sampleTimes[i] << "\t";
            if (i!=sampleTimes.size()-1){
                if (sampleTimes[i+1]-sampleTimes[i]<minII){resultIsTrue = false;}
            }
            else {
                if (abs(sampleTimes[0]-sampleTimes[i])<minII){resultIsTrue = false;}
            }
        }

    }
    else{
        cout << sampleTimes[0] << "\t";
    }

    cout << "modulo " << n << endl;

    cout << "Result is valid?: " << resultIsTrue << endl;
    cout << endl << endl;

}





Vertex* GraphBasedMs::getVertexWithLeastNoOfConnections(Graph *g, vector<Vertex*> vVec)
{
    Vertex* returnVertex = NULL;
    unsigned int edgeCount=0;
    --edgeCount;

    // iterate through all vertices
    for(auto it=g->verticesBegin(); it!=g->verticesEnd(); ++it){
        Vertex* v = *it;

        // used vertices can not exceed total number of vertices
        if ( vVec.size() < g->getNumberOfVertices()){

            // if vertex hasn't been used already
            if (!GraphBasedMs::isVertexInVector(vVec,v)){
                int tempEdgeCount = this->getNoOfConnections(g,v,vVec);

                // compare previous edgecount with current (temp)edgecount
                // if ((tempEdgeCount<edgeCount) && (tempEdgeCount != g->getNumberOfEdges())){
                if (tempEdgeCount<edgeCount){
                    edgeCount = tempEdgeCount;
                    returnVertex = v;
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

//        cout << "vVec size: " << vVec.size() << " ; number of vertices: " << g->getNumberOfVertices() << endl;


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
/*
    if (vVec.size() < g->getNumberOfVertices()){

        Vertex* tempVertex = GraphBasedMs::getVertexWithLeastNoOfConnections(g, vVec);

        // add vertex with least number of connections to "used" vertices
        if (vVec.size() == 0){
            tempVertex = &g->getVertexById(0);
        }

        vVec.push_back(tempVertex);

        GraphBasedMs::removeUsedVertices(g,tempVertex,vVec);

        // get last vector size
        int lastVecSize = sampleTimes.size();

//        cout << "Current sample: " << tempVertex->getId() << endl;

        // add sample time of vertex to sampling time vector
        sampleTimes.push_back(tempVertex->getId());

        // if there are any samples
        if (sampleTimes.size() != 0){

            // if nothing changed between last function call and current function call
            if ((lastVecSize == sampleTimes.size()) || (sampleTimes.size() == g->getNumberOfVertices()) ){
                return;
            }
            else GraphBasedMs::getSampleTimesWithHeuristic(g,vVec,sampleTimes);
        }
        else{ // if no suitable vertices are found, return 0
            sampleTimes.push_back(0);
            return;
        }
    }
    */
}


void GraphBasedMs::getSampleTimesWithHeuristic4All(vector<Graph*> allGraphs, vector<vector<Vertex*>> vVec, vector<int> &sampleTimes){

    for (int it = 0; it!= allGraphs.size(); ++it){
        if (vVec[it].size() >= allGraphs[it]->getNumberOfVertices()){
//            cout << "vVec size: " << vVec[it].size() << "; allGraphs[it] vertices: " << allGraphs[it]->getNumberOfVertices() <<endl;
            return;
        }
    }

    vector<vector<int>> noOfConnections(allGraphs.size(),vector<int>());

    cout << endl;

    for (int row = 0; row!= allGraphs.size();++row){
        Graph* tempGraph = allGraphs[row];

        noOfConnections[row] = this->getAllVertexConnections(tempGraph,vVec[row]);

        for (int column = 0; column!= noOfConnections[row].size();++column){
            cout << noOfConnections[row][column] << "\t";
        }
        cout << endl;
    }

    int minColumn = this->getVectorMatrixColumnWithMinSum(noOfConnections,sampleTimes);

    // add vertex with least number of connections to "used" vertices
    //        if (vVec.size() == 0){
    //            tempVertex = &g->getVertexById(0);
    //        }


    Vertex* tempVertex = NULL;

    for (int it = 0; it!= allGraphs.size(); ++it){
        Vertex* tempVertex = &allGraphs[it]->getVertexById(minColumn);

        vVec[it].push_back(tempVertex);

        GraphBasedMs::removeUsedVertices(allGraphs[it],tempVertex,vVec[it]);
    }

    // get last vector size
    int lastVecSize = sampleTimes.size();

    // cout << "Current sample: " << tempVertex->getId() << endl;

    // add sample time of vertex to sampling time vector
    sampleTimes.push_back(minColumn);

    // if there are any samples
    if (sampleTimes.size() != 0){

        // if nothing changed between last function call and current function call
        if ((lastVecSize == sampleTimes.size()) || (sampleTimes.size() == allGraphs[0]->getNumberOfVertices()) ){
            return;
        }
        else GraphBasedMs::getSampleTimesWithHeuristic4All(allGraphs,vVec,sampleTimes);
    }
    else{ // if no suitable vertices are found, return 0
        sampleTimes.push_back(0);
        return;
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
//                    cout << "Removed destination vertex : " << e->getVertexDstName() << endl;
                    vecSize = vVec.size();
                }
            }
            else if (v==&e->getVertexDst()){

                // if vertex is not found, add to vector
                if (!GraphBasedMs::isVertexInVector(vVec,&e->getVertexSrc())){
                    vVec.push_back(&e->getVertexSrc());
//                    cout << "Removed source vertex : " << e->getVertexSrcName() << endl;
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

vector<vector<int>> GraphBasedMs::createConflictMatrix(vector<vector<int>> conflMatrix, vector<int> sampleVec, int n){
    int tempSort;

    // write columnwise to matrix
    for (int j = 0;j!=n;j++)
    {
        for (int i = 0;i!=n;i++)
        {
            conflMatrix[i][j] = sampleVec[i];
        }

        // shuffle all elements in timing vector down by one and copy last sample to top
        tempSort = sampleVec[n-1];
        for (int i = n-1;i>=1;i--)
        {
            sampleVec[i] = sampleVec[i-1];
        }
        sampleVec[0] = tempSort;
    }

    // print matrix row-wise
    for (int j = 0; j<n; j++)
    {
        for (int i = 0; i<n; i++)
        {
            cout << conflMatrix[j][i] << "\t";
        }
        cout << endl;
    }

    cout << endl << endl;

    return conflMatrix;
}

void GraphBasedMs::createConflictGraph(Graph &conflGraph, vector<Vertex *> &conflVertices,vector<vector<int>> conflMatrix, int currConstraint, int n, int minII){
    // create a vertice for each sample
    for (int i = 0; i<n;i++)
    {
        conflVertices.push_back(&conflGraph.createVertex(i));
    }

    for (int j = 0;j<n;j++)
    {
        int firstOne = -1;
        int maxConflicts = currConstraint -1;

        for (int i = 0;i<n;i++){
            // search for first one in row and skip to next column
            if ((conflMatrix[j][i] == 1) && (firstOne == -1)){firstOne = i; continue;}

            // if one is found and no more conflicts are skipped
            if ((conflMatrix[j][i] == 1) && (maxConflicts == 0))
            {
                // create edges between first found sample and all following samples
                conflGraph.createEdge(*conflVertices[firstOne],*conflVertices[i]);
                cout << "created edge between Sample " << firstOne << " and Sample " << i <<endl;

            } // if a one is found, but there are still ressources available, ignore and decrease parallel resource count
            else if ((conflMatrix[j][i] == 1) && (maxConflicts != 0)){--maxConflicts;}
        }
    }

    // account for backedges through minimum initiation interval
    if (minII > 1)
    for (int j=0;j!=n;++j){
        for (int i = 0;i!=minII-1;++i){ // connect sample j with all samples in minII-1 reach
            int offset = (j+1+i)%(n);
            conflGraph.createEdge(*conflVertices[j],*conflVertices[offset]);
 cout << "Created Edge between Sample " << j << " and Sample " << offset << endl;
        }
    }
    cout << endl;
}

vector<int> GraphBasedMs::getAllVertexConnections(Graph *g, vector<Vertex*> vVec){

    vector<int> noOfConnections;
    for (int row = 0; row!= g->getNumberOfVertices();++row){
        Vertex* tempVertex = &g->getVertexById(row);

        noOfConnections.push_back(this->getNoOfConnections(g,tempVertex,vVec));
    }
    return noOfConnections;
}




int GraphBasedMs::getVectorMatrixColumnWithMinSum(vector<vector<int>> matrix, vector<int> sampleTimes){
    unsigned int minCount = 0;
    int minColumn = -1;
    --minCount;

    for (int column = 0; column!=matrix[0].size();++column){
        int tempMinCount = 0;

        if (!isMemberInVector(sampleTimes,column)){
            for (int row = 0; row != matrix.size();++row){
                tempMinCount += matrix[row][column];
            }
            if (tempMinCount < minCount){
                minCount = tempMinCount;
                minColumn = column;
            }
        }
    }
    cout << "minCount: " << minCount << endl;
    cout << "minColumn: " << minColumn << endl;


    return minColumn;
}
}
