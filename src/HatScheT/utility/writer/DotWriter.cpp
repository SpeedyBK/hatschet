/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

  //assign resource
  std::map<const Resource*, int> resColAssignments;

  int i = 0;
  for(auto it = this->rm->resourcesBegin(); it != this->rm->resourcesEnd(); ++it)
  {
    const Resource* r = *it;
    //skip unlimited
    if(r->getLimit() == -1) continue;

    resColAssignments.insert({r, i});
    i++;

    //only 7 supported
    if(i == 6) break;
  }

  FILE* graphfilepointer = NULL;
  graphfilepointer = fopen((this->path).c_str(),"w");

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
    const Resource* r = this->rm->getResource(v);

    if(r->getLimit() != -1)
    {     
      rId = resColAssignments.at(r);
    }

    node_string << "vertex" << v->getId() << "[label=\"";
    node_string << nodeName << "\\" << "n";
    node_string << "\",fontsize=10,shape=circle";
    if(r->getLimit() != -1) node_string << ",style=\"filled\", color=\"black\"" << ",fillcolor=\""+ colors[rId] +"\"";
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

    int latency = e->getDistance();
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

