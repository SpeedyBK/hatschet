#include <iostream>
#include <string.h>

#include <HatScheT/Exception.h>
#include <HatScheT/Graph.h>
#include <HatScheT/utility/reader/GraphMLGraphReader.h>
#include <HatScheT/utility/reader/GraphMLResourceReader.h>
#include <HatScheT/utility/writer/DotWriter.h>
#include <HatScheT/MoovacMinRegScheduler.h>
#include "HatScheT/utility/Tests.h"
#include "HatScheT/scheduler/graphBased/graphBasedMs.h"
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/Verifier.h"
#include "HatScheT/utility/Utility.h"

/**
 * Returns the value as string of a command line argument in syntax --key=value
 * @param argv the command line string
 * @param parameter the name of the parameter
 * @param value as string if parameter string was found
 * @return True if parameter string was found
 */
bool getCmdParameter(char* argv, const char* parameter, char* &value)
{
  if(strstr(argv, parameter))
  {
    value = argv+strlen(parameter);
    return true;
  }
  else
  {
    return false;
  }
}

void print_short_help()
{
    std::cout << "usage: hatschet [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "General Options:" << std::endl;
    std::cout << "Option                                  Meaning" << std::endl;
    std::cout << "--solver=[Gurobi|CPLEX|SCIP|LPSolve]    ILP solver (if available in ScaLP library), also a comma separated wish list of solvers can be provided" << std::endl;
    std::cout << "--timeout=[int]                         ILP solver Timeout in seconds" << std::endl;
    std::cout << "--threads=[int]                         Number of threads for the ILP solver" << std::endl;
    std::cout << "--resource=[string]                     Path to graphML Resource File you want to read. (Make sure XercesC is enabled)" << std::endl;
    std::cout << "--graph=[string]                        Path to graphML Graph File you want to read. (Make sure XercesC is enabled)" << std::endl;
    std::cout << "--dot=[string]                          Path to you desired dot file of your graph+resource model" << std::endl;
    std::cout << std::endl;

}
int main(int argc, char *args[])
{
  std::string ilpSolver="";
  int threads=0;
  int timeout=-1; //default -1 means no timeout
  bool lennart=false;
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;
  HatScheT::GraphMLResourceReader readerRes(&rm);

  //parse command line
  for (int i = 1; i < argc; i++) {
    char* value;
    if(getCmdParameter(args[i],"--timeout=",value))
    {
      timeout = atol(value);
    }
    else if(getCmdParameter(args[i],"--threads=",value))
    {        
      threads = atol(value);
    }   
    else if(getCmdParameter(args[i],"--solver=",value))
    {
      ilpSolver = std::string(value);
    }
    else if(getCmdParameter(args[i],"--lennart=",value))
    {
      if(std::string(value)=="1")
      lennart=true;
    }
    else if(getCmdParameter(args[i],"--resource=",value))
    {
      #ifdef USE_XERCESC
      string str = std::string(value);
      try
      {       
        rm = readerRes.readResourceModel(str.c_str());
      }
      catch(HatScheT::Exception &e)
      {
        std::cerr << "Error: " << e << std::endl;
      }

      #else
      cout << "Warning: You chose the graphml parameter, but XerseC was not found. Make to to prove the path to XercesC as CMAKE_PREFIX_PATH" << endl;
      #endif
    }
    else if(getCmdParameter(args[i],"--graph=",value))
    {
      #ifdef USE_XERCESC
      string str = std::string(value);
      try
      {
        if(rm.isEmpty() == false)
        {
          HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
          g = readerGraph.readGraph(str.c_str());
          std::string graphName = str.substr (0,str.length()-8);
          g.setName(graphName);
          cout << g << endl;
          cout << rm << endl;
        }

        else
        {
          cout << "Empty Resource Model Provided for graph parsing! Provide a valid resource model using  --graphmlrespath=" << endl;
        }


      }
      catch(HatScheT::Exception &e)
      {
        std::cerr << "Error: " << e << std::endl;
      }

      #else
      cout << "Warning: You chose the graphml parameter, but XerseC was not found. Make to to prove the path to XercesC as CMAKE_PREFIX_PATH" << endl;
      #endif
    }
    else if(getCmdParameter(args[i],"--dot=",value))
    {
      if(rm.isEmpty() == false && g.isEmpty() == false)
      {
        cout << "Writing to dotfile " << value << ".dot" << endl;
        HatScheT::DotWriter dw(value, &g, &rm);
        dw.setDisplayNames(true);
        dw.write();
      }    
    }
    else if(getCmdParameter(args[i],"--asap=",value))
    {
      if(rm.isEmpty() == false && g.isEmpty() == false)
      {
        cout << "Starting ASAP schedule" << endl;
        HatScheT::ASAPScheduler asap(g,rm);
        asap.schedule();
        cout << "Printing ASAP schedule" << endl;
        for(auto it=asap.getStartTimes().begin(); it!=asap.getStartTimes().end(); ++it){
          cout << it->first->getName() << " (" << rm.getResource(it->first)->getName() << ") " << " at " << it->second << endl;
        }
        cout << "Finished ASAP schedule" << endl;
      }
    }
    else if(getCmdParameter(args[i],"--alap=",value))
    {
      if(rm.isEmpty() == false && g.isEmpty() == false)
      {
        cout << "Starting ALAP schedule" << endl;
        HatScheT::ALAPScheduler alap(g,rm);
        alap.schedule();
        cout << "Printing ALAP schedule" << endl;
        for(auto it=alap.getStartTimes().begin(); it!=alap.getStartTimes().end(); ++it){
          cout << it->first->getName() << " (" << rm.getResource(it->first)->getName() << ") " << " at " << it->second << endl;
        }
        cout << "Finished ALAP schedule" << endl;
      }
    }
    else if(getCmdParameter(args[i],"--moovac=",value))
    {
      if(rm.isEmpty() == false && g.isEmpty() == false)
      {
        cout << "Starting Moovac schedule" << endl;
        std::list<std::string> wish = {"Gurobi"};
        HatScheT::MoovacScheduler mv(g, rm, wish);
        mv.schedule();

        if (mv.getScheduleFound()) {
          cout << "Printing Moovac schedule" << endl;
          for (auto it = mv.getStartTimes().begin(); it != mv.getStartTimes().end(); ++it) {
            cout << it->first->getName() << " (" << rm.getResource(it->first)->getName() << ") " << " at " << it->second
                 << endl;
          }
          cout << "Finished Moovac schedule" << endl;
          if (HatScheT::verifyModuloSchedule(g, rm, mv.getStartTimes(), mv.getII()))
            cout << ">>> Moovac schedule verified <<<" << endl;
        }
      }
    }
    else if(getCmdParameter(args[i],"--evalPaper=",value))
        {
          if(rm.isEmpty() == false && g.isEmpty() == false)
          {
            string str = std::string(value);
            HatScheT::Utility::evaluateSchedulers(g,rm, {"CPLEX"}, str);
          }
        }
    else if(getCmdParameter(args[i],"--adaptivePaper=",value))
        {
          if(rm.isEmpty() == false && g.isEmpty() == false)
          {
            string str = std::string(value);
            HatScheT::Utility::adaptiveScheduling(g,rm, {"CPLEX"}, str);
          }
        }
    //HatScheT Auto Test Function
    else if(getCmdParameter(args[i],"--test=",value))
    {
      string str = std::string(value);
      if(str=="READ" && HatScheT::Tests::readTest()==false) exit(-1);
      if(str=="MOOVAC" && HatScheT::Tests::moovacTest()==false) exit(-1);
      if(str=="MODULOSDC" && HatScheT::Tests::moduloSDCTest()==false) exit(-1);
      if(str=="API" && HatScheT::Tests::apiTest()==false) exit(-1);
      if(str=="ASAP" && HatScheT::Tests::asapTest()==false) exit(-1);
      if(str=="ASAPHC" && HatScheT::Tests::asapHCTest()==false) exit(-1);
      if(str=="ALAPHC" && HatScheT::Tests::alapHCTest()==false) exit(-1);
      if(str=="ULScheduler" && HatScheT::Tests::ulSchedulerTest()==false) exit(-1);
      if(str=="OCC" && HatScheT::Tests::occurrenceTest()==false) exit(-1);
      if(str=="OCCS" && HatScheT::Tests::occurrenceSetTest()==false) exit(-1);
      if(str=="OCCSC" && HatScheT::Tests::occurrenceSetCombinationTest()==false) exit(-1);
      if(str=="SGMS" && HatScheT::Tests::sgmSchedulerTest()==false) exit(-1);
    }
    else
    {
      std::cout << "Error: Illegal Option: " << args[i] << std::endl;
      print_short_help();
      exit(-1);
    }

  }

  std::cout << "settings:" << std::endl;
  std::cout << "timeout=" << timeout << std::endl;
  std::cout << "threads=" << threads << std::endl;

  if(lennart==true)
  {
    HatScheT::ASAPScheduler* asap = new HatScheT::ASAPScheduler(g,rm);
    cout << asap->getScheduleLength() << endl;
    HatScheT::GraphBasedMs* gbms = new HatScheT::GraphBasedMs(asap,g,rm,0.5,2);
      gbms->schedule();
  }

  return 0;
}
