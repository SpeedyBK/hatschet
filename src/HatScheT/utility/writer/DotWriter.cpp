#include "DotWriter.h"
#include <vector>
#include <sstream>

namespace HatScheT {

DotWriter::DotWriter(std::string path, Graph *g, ResourceModel *rm) : Writer(path)
{
  this->g = g;
  this->rm = rm;
  this->displayNames = false;
}

DotWriter::~DotWriter()
{

}

void DotWriter::write()
{
  vector<string> colors;

  //supported 7 different resources/colours
  colors.push_back("red");
  colors.push_back("green");
  colors.push_back("blue");
  colors.push_back("yellow");
  colors.push_back("orange");
  colors.push_back("violetred3");
  colors.push_back("purple");

  FILE* graphfilepointer = NULL;
  graphfilepointer = fopen((this->path+".dot").c_str(),"w");

  time_t mytime;
  time(&mytime);
  fprintf(graphfilepointer,"// File generated:  %s\n",ctime(&mytime));
  fprintf(graphfilepointer,"digraph DAG {\n");

  //write vertices to dot
  for(auto it = this->g->verticesBegin(); it != this->g->verticesEnd(); it++)
  {
    Vertex* v =  *it;
    stringstream node_string;
    string nodeName;

    if(displayNames==true) nodeName = v->getName();
    else if(displayNames==false) nodeName = to_string(v->getId());

    //bool limitedResource = this->rm->isResourceConstrained(v);
    unsigned int rId = 0;

    //if(limitedResource == true)
    {
      //Resource* r = this->rm->getResource(v);
      //rId = r->id;
    }

    node_string << "vertex" << v->getId() << "[label=\"";
    node_string << nodeName << "\\" << "n";
    node_string << "\",fontsize=10,shape=circle";
    //if(limitedResource == true) node_string << ",style=\"filled\", color=\"black\"" << ",fillcolor=\""+ colors[rId] +"\"";
    node_string << "];\n";

    fprintf(graphfilepointer,"%s",node_string.str().c_str());
  }

  //write edges to dot
  for(auto it = this->g->edgesBegin(); it != this->g->edgesEnd(); it++)
  {
    Edge* e = *it;
    Vertex& srcVertex = e->getVertexSrc();
    Vertex& dstVertex = e->getVertexDst();
    stringstream edge_string;

    int latency = e->getDelay();
    string latencyString;
    if(latency > 0) latencyString = to_string(latency);
    else latencyString = "";

    edge_string << "vertex" << srcVertex.getId() << " -> vertex" << dstVertex.getId();
    edge_string << " [label=\"";
    edge_string << latencyString;
    edge_string << "\",fontsize=12";
    edge_string << "]\n";
    fprintf(graphfilepointer,"%s",edge_string.str().c_str());
  }

  //close stream, write file
  fprintf(graphfilepointer,"}\n");
  fflush(graphfilepointer);
  fclose(graphfilepointer);
}

}

