#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/utility/Verifier.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

namespace HatScheT
{

SchedulerBase::SchedulerBase(Graph& g, ResourceModel &resourceModel) : resourceModel(resourceModel), g(g)
{
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
  if(verifyModuloSchedule(this->g,this->resourceModel,this->startTimes,this->II)==false){
    this->printStartTimes();
    cout << "SchedulerBase.getStartTimes: Invalid schedule detected by SchedulerClass" << endl;
    exit(-1);
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

void SchedulerBase::printStartTimes()
{
  for(auto it:this->startTimes)
  {
    Vertex* v = it.first;
    cout << v->getName() << " (" << resourceModel.getResource(v)->getName() << ") at " << it.second << endl;
  }
}

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
    else throw new HatScheT::Exception("Template file for schedule chart " + templatePath + " does not exist.");
  }
  else throw new HatScheT::Exception("Could not open file " + filename + " for writing.");

  outputFile.close();
}

std::map<const Vertex *, int> SchedulerBase::getBindings()
{
  std::map<const Vertex*,int> bindings;
  std::map<const Resource*, int> naiveBindingCounter;

  for(auto it:this->startTimes){
    Vertex* v = it.first;
    const Resource* r = this->resourceModel.getResource(v);
    if(r->getLimit() == -1) throw new Exception("SchedulerBase.getBindings: resource not unlimited " + r->getName() + "! SchedulerBase does not support this behavior! Use a ResourceConstraint Scheduler!");

    if(naiveBindingCounter.find(r) == naiveBindingCounter.end()) naiveBindingCounter.insert(make_pair(r,0));
    //make naive binding
    bindings.insert(make_pair(v,naiveBindingCounter[r]));
    //increment naive binding counter for next possible binding
    naiveBindingCounter[r]++;
  }

  return bindings;
}

std::map<Edge*,int> SchedulerBase::getLifeTimes()
{
  if(this->startTimes.size()==0) throw new Exception("SchedulerBase.getLifeTimes: cant return lifetimes! no startTimes determined!");

  std::map<Edge*,int> lifetimes;

  for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();
    Vertex* vDst = &e->getVertexDst();
    int lifetime = this->startTimes[vDst] - this->startTimes[vSrc] - this->resourceModel.getVertexLatency(vSrc) + e->getDistance()*this->getScheduleLength();

    if(lifetime < 0) throw new Exception("SchedulerBase.getLifeTimes: negative lifetime detected!");
    else lifetimes.insert(make_pair(e, lifetime));
  }
  return lifetimes;
}

}
