/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/utility/Verifier.h>
#include <HatScheT/utility/Utility.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

namespace HatScheT
{

SchedulerBase::SchedulerBase(Graph& g, ResourceModel &resourceModel) : resourceModel(resourceModel), g(g)
{
  //check for valid registrations
  //throws exception when validation failed
  Utility::everyVertexisRegistered(g,resourceModel);
}

SchedulerBase::~SchedulerBase()
{

}

int SchedulerBase::getScheduleLength()
{
  int maxTime=-1;
  for(std::pair<Vertex*,int> vtPair : this->startTimes)
  {
    Vertex* v = vtPair.first;

    if((vtPair.second+resourceModel.getVertexLatency(v)) > maxTime) maxTime = (vtPair.second+resourceModel.getVertexLatency(v));
  }
  return maxTime;
}

std::map<Vertex*,int>&  SchedulerBase::getSchedule()
{
    if (startTimes.size() == 0) {
        cout << "SchedulerBase.getStartTimes: Empty schedule detected by SchedulerClass" << endl;
        //exit(-1); //Shouldnt ever terminate whole program!
        throw HatScheT::Exception("SchedulerBase.getStartTimes: Empty schedule detected by SchedulerClass");
    }
    if (II < 1) {
        cout << "SchedulerBase.getStartTimes: Invalid II detected by SchedulerClass, II = " << to_string(II) << endl;
        //exit(-1); //Shouldnt ever terminate whole program!
        throw HatScheT::Exception("SchedulerBase.getStartTimes: Invalid schedule detected by SchedulerClass, II = " + to_string(II));

    }

  return this->startTimes;
}

int SchedulerBase::getStartTime(Vertex &v)
{
  std::map<Vertex*,int>::iterator it = startTimes.find(&v);
  if(it != startTimes.end())
    return it->second;
  else
    return -1;
}

/*void SchedulerBase::printStartTimes()
{
  for(auto it:this->startTimes)
  {
    Vertex* v = it.first;
    cout << v->getName() << " (" << resourceModel.getResource(v)->getName() << ") at " << it.second << endl;
  }
}*/

void SchedulerBase::writeScheduleChart(string filename)
{
  std::string line;
  string templatePath = "templates/schedule_template.htm";

  stringstream htmlTableRows;

  int scheduleLength=getScheduleLength();

  htmlTableRows << "	<tr><td class=\"first\"></td>";
  for(int i=0; i < scheduleLength; i++)
  {
    htmlTableRows << "<td>" << i << "</td>";
  }
  htmlTableRows << "</tr>" << endl;

  for(auto it:startTimes)
  {
    Vertex* v = it.first;
    int scheduleTime = it.second;

    int maxLatency=resourceModel.getResource(v)->getLatency();

    htmlTableRows << "	<tr><td class=\"first\">" << v->getName() << " (" << resourceModel.getResource(v)->getName() << ")</td>";
    for(int i=0; i < scheduleLength; i++)
    {
      htmlTableRows << "<td";
      if(i == scheduleTime) htmlTableRows << " class=\"start\"";
      else if((i > scheduleTime) && (i < scheduleTime+maxLatency)) htmlTableRows << " class=\"active\"";
      htmlTableRows << "></td>";
    }
    htmlTableRows << "</tr>" << endl;
  }

  string graphName=getGraph()->getName();

  cout << "writing schedule chart to " << filename << endl;
  ofstream outputFile(filename);
  if(outputFile.is_open())
  {
  //open template file
    std::ifstream templateFile(templatePath);
    if(templateFile.is_open())
    {
      while(getline(templateFile,line))
      {
        if(line.find("{GRAPH}") != std::string::npos)
        {
          line.replace(line.find("{GRAPH}"), sizeof("{GRAPH}")-1, graphName);
        }
        if(line.find("{SCHEDULEROWS}") != std::string::npos)
        {
          line.replace(line.find("{SCHEDULEROWS}"), sizeof("{SCHEDULEROWS}")-1, htmlTableRows.str());
        }
        outputFile << line << '\n';
      }
      templateFile.close();
    }
    else throw HatScheT::Exception("Template file for schedule chart " + templatePath + " does not exist.");
  }
  else throw HatScheT::Exception("Could not open file " + filename + " for writing.");

  outputFile.close();
}

std::map<const Vertex *, int> SchedulerBase::getBindings()
{
  //generate new binding when no binding is available
  if(this->binding.size() == 0)
    this->binding = Utility::getSimpleBinding(this->getSchedule(),&this->resourceModel,(int)this->II);

  //throw exception when no binding was generated
  if(this->binding.size() == 0) throw Exception("SchedulerBase.getBindings: Error no binding could be generated! No schedule available?");

  //return the stored binding
  return this->binding;
}

std::map<Edge*,int> SchedulerBase::getLifeTimes()
{
  if(this->startTimes.size()==0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: cant return lifetimes! no startTimes determined!");
  if(this->II <= 0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: cant return lifetimes! no II determined!");

  std::map<Edge*,int> lifetimes;

  for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();
    Vertex* vDst = &e->getVertexDst();
    int lifetime = this->startTimes[vDst] - this->startTimes[vSrc] - this->resourceModel.getVertexLatency(vSrc) + e->getDistance()*this->II;

    if(lifetime < 0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: negative lifetime detected!");
    else lifetimes.insert(make_pair(e, lifetime));
  }
  return lifetimes;
}

}
