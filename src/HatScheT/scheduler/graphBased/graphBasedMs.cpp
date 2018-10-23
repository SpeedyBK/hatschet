/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Lennart Henne

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <HatScheT/scheduler/graphBased/graphBasedMs.h>
#include <vector>
#include <HatScheT/Vertex.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <algorithm>
#include <HatScheT/scheduler/ilpbased/MoovacScheduler.h>
#include <fstream>
//#include <boost/algorithm/string/split.hpp>
#include <cstdio>
#include <ctime>
#include <chrono>
#include <cmath>

using Clock = std::chrono::high_resolution_clock;

namespace HatScheT {

    GraphBasedMs::GraphBasedMs(SchedulerBase *sched, Graph &g, ResourceModel &resourceModel, float moduloLowerBound, float moduloUpperBound) : SchedulerBase(g, resourceModel) {
        this->moduloLowerBound = moduloLowerBound;
        this->moduloUpperBound = moduloUpperBound;
        this->scheduler = sched;

        if ((moduloLowerBound <= 0) || (moduloLowerBound > 2))
        {
            throw HatScheT::Exception("GraphBasedMs.schedule(): Invalid modulo lower bound!");
        }

        if ((moduloUpperBound <= 0) || (moduloUpperBound > 2))
        {
            throw HatScheT::Exception("GraphBasedMs.schedule(): Invalid modulo upper bound!");
        }
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

//aes2/graph1
// ./hatschet --lennart=1 --resource=../hatschet/graphMLFiles/MachSuite/aes2/graph1_RM.xml --graph=../hatschet/graphMLFiles/MachSuite/aes2/graph1.graphml

//create .dot graph by adding --dot to the end

void GraphBasedMs::schedule()
{
    //..insert lennarts project work

    cout << "Lower Bound: " << this->moduloLowerBound << endl;
    cout << "Upper Bound: " << this->moduloUpperBound << endl;

    // Create new ASAPScheduler pointer and schedule current graph
//    ASAPScheduler* asap = new ASAPScheduler(g,resourceModel);

//    asap->schedule();

    auto start = Clock::now();

    bool doMoovac = false;
    int moovacII = 1;
    int moovacLat = 1;


    if (doMoovac){
        MoovacScheduler ms(g,resourceModel,{"CPLEX"});
        ms.setSolverTimeout(180);
        ms.setMaxLatencyConstraint(scheduler->getScheduleLength());
        ms.schedule();

        moovacII = ms.getII();
        moovacLat = ms.getScheduleLength();
        //        cout << endl << "Moovac II is: " << ms.getII() << endl;
        //        cout << "Moovac schedule length is: " << ms.getScheduleLength() << endl << endl;
    }

    auto stop = Clock::now();

    int durationMoovac = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

    // calculate minII from Graph
    int minII = Utility::calcRecMII(&g, &resourceModel);
//    cout << "calcRecMinII: " << minII << endl;

    // Get schedule length
    int n = scheduler->getScheduleLength();

//     int n = asap->getScheduleLength();
//     this->moduloLowerBound = 0.9;
//     this->moduloUpperBound = 1.1;


    cout << "schedule length: " << scheduler->getScheduleLength() << endl;

    if (n <= 1)
    {
        cout << "GraphBasedMs.schedule(): ERROR scheduler fucked up" << endl;
        throw HatScheT::Exception("GraphBasedMs.schedule(): ERROR scheduler fucked up");
    }


    vector<string> constrRessources;

    // get number and names of constrained ressources
    for (auto it = g.verticesBegin(); it!= g.verticesEnd(); ++it){
        Vertex* anotherVertexPointer = *it;
        // cout << "Start time of " << anotherVertexPointer.getName() << " : " << scheduler->getStartTime(anotherVertexPointer) << endl;
        const Resource* r= this->resourceModel.getResource(anotherVertexPointer);

        if (!isMemberInVector(constrRessources,r->getName())){
            if (r->getLimit() != -1){
                constrRessources.push_back(r->getName());
            }
        }
    }

    // add vertices to a constraint matrix
    vector<vector<Vertex*>> constrRessourceVertices(constrRessources.size());

    for (int i = 0; i!= constrRessources.size();++i){
        for (auto it = g.verticesBegin(); it!= g.verticesEnd(); ++it){

            // Vertex& tempVertexPointer = g.getVertexById(j+1);
            Vertex* tempVertexPointer = *it;
            const Resource* tempR= this->resourceModel.getResource(tempVertexPointer);

            if (constrRessources[i] == tempR->getName()){
                constrRessourceVertices[i].push_back(tempVertexPointer);
            }
        }
    }

//    int boundary=5;
//    if (n < 10) boundary = floor(n/2);


    for (int modulo = floor(n*moduloLowerBound) ;modulo!= ceil(n*moduloUpperBound)+1;++modulo){
//    for (int modulo = 80;modulo!=401;++modulo){

        start= Clock::now();

        // create result matrix with number of constrained ressources as rows and variable length columns
        vector<Graph*> allGraphs;


        // iterate through constrained ressources
        for (int constrIt = 0; constrIt != constrRessources.size();++constrIt){

            int rows = modulo;

            if (modulo<n) {rows = n;};

            // create vector of vectors (matrix) for conflicts and vector(s) for constrained ressources
            vector<vector<int>> conflMatrix(rows, vector<int>(modulo));
            vector<int> sampleVec(rows);


            // create pointer and set it to current constrained ressource vector
            vector<Vertex*>* currRessource = NULL;
            int currConstraint = -1;

            currRessource = &constrRessourceVertices[constrIt];

            const Resource* r= this->resourceModel.getResource(currRessource->at(0));
            currConstraint = r->getLimit();


            // create vector with sample times
            for (int j = 0; j != rows; ++j)
            {
                sampleVec[j] = 0;
            }

            // "inject" ones at ressource start times
            for (int j = 0; j != currRessource->size(); ++j)
            {
                Vertex* tempVertex = NULL;

                tempVertex = currRessource->at(j);
                sampleVec[scheduler->getStartTime(*tempVertex)] += 1;
            }

            //        cout << "currConstraint: " << currConstraint << endl;
            //        cout << "n: " << n << endl;
            //        cout << "minII: " << minII << endl << endl;

            //        cout << endl << "Resource: " << constrRessources[constrIt] << endl << endl;
            conflMatrix = this->createConflictMatrix(conflMatrix,sampleVec,modulo);


            // create new graph pointer and vector of vertices
            HatScheT::Graph* conflGraph = new HatScheT::Graph();
            vector<HatScheT::Vertex*> conflVertices;

            this->createConflictGraph(*conflGraph,conflVertices,conflMatrix,currConstraint,minII);

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

//        cout << endl;

        // sort resulting sampleTimes
        std::sort(sampleTimes.begin(),sampleTimes.end());


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
//            cout << sampleTimes[0] << "\t";
            cout << "0" << "\t";
        }

        cout << "modulo " << modulo << endl;

//        cout << endl << "Result is (minII) valid?: " << resultIsTrue << endl;

        bool resultModIsTrue = true;
        for (int it = 0;it!=constrRessources.size();++it){
            const Resource* r= this->resourceModel.getResource(constrRessourceVertices[it][0]);

            if ((sampleTimes.size()*(constrRessourceVertices[it].size()/r->getLimit())) > modulo){
                resultModIsTrue = false;
            }
        }

//        cout << "Result is (mod) valid?: " << resultModIsTrue << endl;
//        cout << endl << endl;

        bool isBetterThanScheduler = false;
        //    if ((double)1/scheduler->getScheduleLength() > (double)sampleTimes.size()/modulo) isBetterThanScheduler = true;
        if ((double)1/scheduler->getScheduleLength() > (double)sampleTimes.size()/n) isBetterThanScheduler = true;

        double graphBasedThrougput = (double)sampleTimes.size()/modulo;

        stop = Clock::now();

        int durationGraphBased = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

        /////////////////////////////////////////////////////


        vector<string> splitNames;

        //boost::split(splitNames, g.getName(), [](char del){return del == '/';});

        std::ofstream resultfile;
        resultfile.open("Results.txt",ios::app);
        if (resultfile.is_open()){
            resultfile << "Graph name: " << splitNames[splitNames.size()-2] << " -> " << splitNames.back() << endl;
            resultfile << "Moovac throughput:\t\t 1/" << moovacII << endl;
            resultfile << "GraphBasedMs throughput: " << sampleTimes.size() << "/" << modulo << endl;
            resultfile << "Moovac duration: \t\t " << durationMoovac << "ms" << endl;
            resultfile << "GraphBasedMs duration: \t " << durationGraphBased << "ms" << endl;
            resultfile << "Modulo: \t\t" << modulo << endl;
            resultfile << "Latency: \t\t" << scheduler->getScheduleLength() << endl;
            resultfile << "Is result valid?:  \t" << resultModIsTrue << endl << endl;
            resultfile.close();
        }

        resultfile.open("Results.csv",ios::app);
        if (resultfile.is_open()){
            resultfile << splitNames[splitNames.size()-2] << "," << splitNames.back() << ","; //Names
            resultfile << scheduler->getScheduleLength() << ",";
            resultfile << moovacLat << ","; //Moovac Schedule length
            resultfile << (double)1/moovacII << ","; //Moovac throughput
            resultfile << modulo << ","; //modulo
            resultfile << graphBasedThrougput << ","; //GraphBased throughput
            resultfile << durationMoovac << "," << durationGraphBased << ",";
            resultfile << resultModIsTrue << endl; //validity
            resultfile.close();
        }
    }
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



void GraphBasedMs::getSampleTimesWithHeuristic4All(vector<Graph*> allGraphs, vector<vector<Vertex*>> vVec, vector<int> &sampleTimes){

    // check if used vertices exceed total number of vertices
    for (int it = 0; it!= allGraphs.size(); ++it){
        if (vVec[it].size() >= allGraphs[it]->getNumberOfVertices()){
            //            cout << "vVec size: " << vVec[it].size() << "; allGraphs[it] vertices: " << allGraphs[it]->getNumberOfVertices() <<endl;
            return;
        }
    }
    //  cout << endl;

    vector<vector<int>> noOfConnections(allGraphs.size(),vector<int>());


    for (int row = 0; row!= allGraphs.size();++row){
        Graph* tempGraph = allGraphs[row];

        noOfConnections[row] = this->getAllVertexConnections(tempGraph,vVec[row]);

        //        for (int column = 0; column!= noOfConnections[row].size();++column){
        //            cout << noOfConnections[row][column] << "\t";
        //        }
        //        cout << endl;
    }

    int minColumn = this->getVectorMatrixColumnWithMinSum(noOfConnections,allGraphs,vVec);

    unsigned int limit = 0;

    if ((minColumn<0) || (minColumn == (limit -1))) return;

    // add vertex with least number of connections to "used" vertices
    Vertex* tempVertex = NULL;

    // remove used vertices for all graphs
    for (int it = 0; it!= allGraphs.size(); ++it){
        tempVertex = &allGraphs[it]->getVertexById(minColumn);

        vVec[it].push_back(tempVertex);

        //        for (int i = 0; i!= vVec[it].size();++i){
        //            cout << *vVec[it][i] << "\t";
        //        }

        //        cout << endl;

        GraphBasedMs::removeUsedVertices(allGraphs[it],tempVertex,vVec[it]);
    }

    // get last vector size
    int lastVecSize = sampleTimes.size();

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
                    //                    cout << "Removed destination vertex : " << e->getVertexDstName() << endl;
                    vVec.push_back(&e->getVertexDst());
                    vecSize = vVec.size();
                }
            }
            else if (v==&e->getVertexDst()){

                // if vertex is not found, add to vector
                if (!GraphBasedMs::isVertexInVector(vVec,&e->getVertexSrc())){
                    //                    cout << "Removed source vertex : " << e->getVertexSrcName() << endl;
                    vVec.push_back(&e->getVertexSrc());
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

vector<vector<int>> GraphBasedMs::createConflictMatrix(vector<vector<int>> conflMatrix, vector<int> sampleVec, int modulo){
    int tempSort;

    if (modulo >= sampleVec.size()){

//        for (int i = 0; i<modulo-sampleVec.size();++i){
//            sampleVec.push_back(0);
//            cout << "Did the " << i+1 << ". pushback" << endl;
//        }

        // write columnwise to matrix
        for (int j = 0;j!=modulo;j++)
        {
            for (int i = 0;i!=modulo;i++)
            {
                conflMatrix[i][j] = sampleVec[i];
            }

            // shuffle all elements in timing vector down by one and copy last sample to top
            tempSort = sampleVec.back();
            for (int i = sampleVec.size()-1;i>=1;i--)
            {
                sampleVec[i] = sampleVec[i-1];
            }
            sampleVec[0] = tempSort;
        }
    }
    else{
        // write columnwise to matrix
        for (int j = 0;j!=sampleVec.size();j++)
        {
            for (int i = 0;i!=sampleVec.size();i++)
            {
                conflMatrix[i][j%(modulo)] += sampleVec[i];
            }

            // shuffle all elements in timing vector down by one and copy last sample to top
            tempSort = sampleVec.back();
            for (int i = sampleVec.size()-1;i>=1;i--)
            {
                sampleVec[i] = sampleVec[i-1];
            }
            sampleVec[0] = tempSort;
        }
    }

//     print matrix row-wise
//        for (int j = 0; j<conflMatrix.size(); j++)
//        {
//            for (int i = 0; i<conflMatrix[0].size(); i++)
//            {
//                cout << conflMatrix[j][i] << "  ";
//            }
//            cout << endl;
//        }
//        cout << endl << endl;

    return conflMatrix;
}

void GraphBasedMs::createConflictGraph(Graph &conflGraph, vector<Vertex *> &conflVertices,vector<vector<int>> conflMatrix, int currConstraint, int minII){
    // create a vertice for each sample
    for (unsigned int i = 0; i<conflMatrix[0].size();i++)
    {
        conflVertices.push_back(&conflGraph.createVertex(i));
    }

    // iterate over rows
    for (unsigned int i=0;i!=conflMatrix.size();++i){
        // iterate over columns
        for (unsigned int j=0;j!=conflMatrix[0].size();++j){
            // iterate over columns for every j
            for (unsigned int k = j+1;k!=conflMatrix[0].size();++k){
                // if usages at both times exceed ressource limit, create conflict
                if(conflMatrix[i][j]+conflMatrix[i][k]>currConstraint){
                    if (!Utility::existEdgeBetweenVertices(&conflGraph,conflVertices[j],conflVertices[k])){
                        conflGraph.createEdge(*conflVertices[j],*conflVertices[k]);
                        // cout << "Created edge between Sample " << j << " and Sample " << k <<endl;
                    }
                }
            }
        }
    }

    //    for (int j = 0;j<n;j++){
    //        int maxConflicts = currConstraint -1;

    //        for (int i = 0;i<n;i++){

    //            // search for first one in row and skip to next column
    //            if (conflMatrix[j][i] == 1){
    //                for (int k = i+1;k!=n;++k){

    //                    // if one is found and no more conflicts are skipped
    //                    if ((conflMatrix[j][k] == 1) && (maxConflicts == 0))
    //                    {
    //                        // cout << "is edge in graph?: " << Utility::existEdgeBetweenVertices(&conflGraph,conflVertices[firstOne],conflVertices[i]) << endl;

    //                        if (!Utility::existEdgeBetweenVertices(&conflGraph,conflVertices[i],conflVertices[k])){
    //                            // create edges between first found sample and all following samples
    //                            conflGraph.createEdge(*conflVertices[i],*conflVertices[k]);
    //                            cout << "Created edge between Sample " << i << " and Sample " << k <<endl;
    //                        }
    //                    } // if a one is found, but there are still ressources available, ignore and decrease parallel resource count
    //                    else if ((conflMatrix[j][k] == 1) && (maxConflicts != 0)){--maxConflicts;}
    //                }
    //            }
    //        }
    //    }

    // account for backedges and component latency through minimum initiation interval
    if (minII > 1)
        for (unsigned int j=0;j!=conflMatrix[0].size();++j){
            for (unsigned int i = 0;i!=minII-1;++i){ // connect sample j with all samples in minII-1 reach
                int offset = (j+1+i)%(conflMatrix[0].size());

                conflGraph.createEdge(*conflVertices[j],*conflVertices[offset]);
                // cout << "Created minII edge between Sample " << j << " and Sample " << offset << endl;
            }
        }
    //    cout << endl;
}



vector<int> GraphBasedMs::getAllVertexConnections(Graph *g, vector<Vertex*> vVec){

    vector<int> noOfConnections;
    for (int row = 0; row!= g->getNumberOfVertices();++row){
        Vertex* tempVertex = &g->getVertexById(row);

        if (!isVertexInVector(vVec,tempVertex)){
            noOfConnections.push_back(this->getNoOfConnections(g,tempVertex,vVec));
        }
        else{
            noOfConnections.push_back(0);
            //            cout << *tempVertex << "is already used" << endl;
        }

    }
    return noOfConnections;
}




int GraphBasedMs::getVectorMatrixColumnWithMinSum(vector<vector<int>> matrix,vector<Graph*> allGraphs, vector<vector<Vertex*>> vVec){
    unsigned int minCount = 0;
    int minColumn = -1;
    --minCount;

    //iterate over all columns
    for (unsigned int column = 0; column!=matrix[0].size();++column){

        int tempMinCount = 0;
        bool isValid = true;

        for (int grIt = 0; grIt != allGraphs.size(); ++grIt){
            Vertex* tempVertex = &allGraphs[grIt]->getVertexById(column);
            // check if vertex is in vector, if yes, set to false
            if (isVertexInVector(vVec[grIt],tempVertex)) isValid = false;
        }

        //        cout << "is " << column << " in sampleTimes?: " << isVertexInVector(vVec,tempVertex) << endl;
        if (isValid){
            for (int row = 0; row != matrix.size();++row){
                tempMinCount += matrix[row][column];
            }
            if (tempMinCount < minCount){
                minCount = tempMinCount;
                minColumn = column;
            }
        }
    }
    //    cout << "minCount: " << minCount << endl;
    //    cout << "minColumn: " << minColumn << endl;


    return minColumn;
}
}
