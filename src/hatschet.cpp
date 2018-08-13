#include <iostream>
#include <string.h>
#include <HatScheT/utility/Exception.h>
#include <HatScheT/Graph.h>
#include <HatScheT/utility/writer/DotWriter.h>
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/utility/Utility.h"

#ifdef USE_XERCESC
#include <HatScheT/utility/reader/GraphMLGraphReader.h>
#include <HatScheT/utility/reader/GraphMLResourceReader.h>
#endif

#ifdef USE_SCALP
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include <HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h>
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/graphBased/graphBasedMs.h"
#include "HatScheT/utility/Tests.h"
#endif

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
  std::cout << "usage: hatschet [OPTIONS] <path to graphML file>" << std::endl;
  std::cout << std::endl;
  std::cout << "General Options:" << std::endl;
  std::cout << "Option                                                  Meaning" << std::endl;
  std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "--scheduler=[ASAP|ALAP|UL|MOOVAC|MODULOSDC|RATIONALII]  Scheduling algorithm, select one of the following:" << std::endl;
  std::cout << "                                                          ASAP: As-soon-as-possible scheduling" << std::endl;
  std::cout << "                                                          ALAP: As-last-as-possible scheduling" << std::endl;
  std::cout << "                                                          UL: Universal list scheduling" << std::endl;
  std::cout << "                                                          MOOVAC: Moovac ILP-based exact modulo scheduling" << std::endl;
  std::cout << "                                                          MODULOSDC: Modulo SDC modulo scheduling" << std::endl;
  std::cout << "                                                          RATIONALII: Experimental rational II scheduler" << std::endl;
  std::cout << "--solver=[Gurobi|CPLEX|SCIP|LPSolve]                    ILP solver (if available in ScaLP library), also a comma separated wish list of solvers can be provided" << std::endl;
  std::cout << "--timeout=[int]                                         ILP solver Timeout in seconds (default: -1, no timeout)" << std::endl;
  std::cout << "--threads=[int]                                         Number of threads for the ILP solver (default: 1)" << std::endl;
  std::cout << "--resource=[string]                                     Path to XML resource constrain file" << std::endl;
  std::cout << "--graph=[string]                        Path to graphML Graph File you want to read. (Make sure XercesC is enabled)" << std::endl;
  std::cout << "--dot=[string]                                          Optional path to dot file generated from graph+resource model (default: none)" << std::endl;
  std::cout << "--html=[string]                                         Optional path to html file for a schedule chart" << std::endl;
  std::cout << std::endl;

}
int main(int argc, char *args[])
{
#ifndef USE_XERCESC
  cout << "Error: XercesC not active! Without XML-parsing, this interface is disabled! Install XeresC for using the HatScheT binary" << endl;
  exit(0);
#else
  std::string ilpSolver="";
  int threads=1;
  int timeout=-1; //default -1 means no timeout

  enum Scheduler {ASAP, ALAP, UL, MOOVAC, MOOVACMINREG, MODULOSDC, RATIONALII, ASAPRATIONALII, NONE};
  Scheduler schedulerString = NONE;

  std::string graphMLFile="";
  std::string resourceModelFile="";
  std::string dotFile="";
  std::string htmlFile="";


  if(argc <= 1)
  {
      print_short_help();
      exit(0);
  }

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
      #ifndef USE_SCALP
      throw new HatScheT::Exception("HatScheT: ScaLP not active! Solver cant be chosen: " + std::string(value));
      #endif
      ilpSolver = std::string(value);
    }
    else if(getCmdParameter(args[i],"--resource=",value))
    {
      resourceModelFile = std::string(value);
    }
    else if(getCmdParameter(args[i],"--graph=",value))
    {
      graphMLFile = std::string(value);
    }
    else if(getCmdParameter(args[i],"--dot=",value))
    {
      dotFile = value;
    }
    else if(getCmdParameter(args[i],"--html=",value))
    {
      htmlFile = value;
    }
    else if(getCmdParameter(args[i],"--scheduler=",value))
    {
      std::string valueStr = std::string(value);//std::tolower(std::string(value));
      std::string valueLC; //lower case string

      valueLC.resize(valueStr.size());
      std::transform(valueStr.begin(),valueStr.end(),valueLC.begin(),::tolower);

      if(valueLC == "asap")
      {
        schedulerString = ASAP;
      }
      else if(valueLC == "alap")
      {
        schedulerString = ALAP;
      }
      else if(valueLC == "ul")
      {
        schedulerString = UL;
      }
      else if(valueLC == "moovac")
      {
        #ifndef USE_SCALP
        throw new HatScheT::Exception("HatScheT: ScaLP not active! This scheduler is not available:" + valueLC);
        #endif
        schedulerString = MOOVAC;
      }
      else if(valueLC == "moovacminreg")
      {
        #ifndef USE_SCALP
        throw new HatScheT::Exception("HatScheT: ScaLP not active! This scheduler is not available:" + valueLC);
        #endif
        schedulerString = MOOVACMINREG;
      }
      else if(valueLC == "modulosdc")
      {
        #ifndef USE_SCALP
        throw new HatScheT::Exception("HatScheT: ScaLP not active! This scheduler is not available:" + valueLC);
        #endif
        schedulerString = MODULOSDC;
      }
      else if(valueLC == "rationalii")
      {
        #ifndef USE_SCALP
        throw new HatScheT::Exception("HatScheT: ScaLP not active! This scheduler is not available:" + valueLC);
        #endif
        schedulerString = RATIONALII;
      }
      else if(valueLC == "asapRationalII")
      {
        schedulerString = ASAPRATIONALII;
      }
      else
      {
        cout << "Error: scheduler " << value << " unknown!" << endl;
        exit(0);
      }

    }
    //HatScheT Auto Test Function
    else if(getCmdParameter(args[i],"--test=",value))
    {
      #ifdef USE_SCALP
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
      #else
      throw new HatScheT::Exception("HatScheT: ScaLP not active! Test function disabled!");
      #endif
    }
    else if((args[i][0] != '-') && getCmdParameter(args[i],"",value))
    {
      string str = std::string(value);

      graphMLFile = std::string(value);

    }
    else
    {
      std::cout << "Error: Illegal Option: " << args[i] << std::endl;
      print_short_help();
      exit(-1);
    }

  }
  //output major settings:
  std::cout << "settings:" << std::endl;
  std::cout << "graph model=" << graphMLFile << endl;
  std::cout << "timeout=" << timeout << std::endl;
  std::cout << "threads=" << threads << std::endl;
  std::cout << "scheduler=";
  switch(schedulerString)
  {
    case ASAP:
      cout << "ASAP";
      break;
    case ALAP:
      cout << "ALAP";
      break;
    case UL:
      cout << "UL";
      break;
    case MOOVAC:
      cout << "MOOVAC";
      break;
    case MOOVACMINREG:
      cout << "MOOVACMINREG";
      break;
    case MODULOSDC:
      cout << "MODULOSDC";
      break;
    case RATIONALII:
      cout << "RATIONALII";
      break;
    case ASAPRATIONALII:
      cout << "ASAPRATIONALII";
      break;
    case NONE:
      cout << "NONE";
      break;
  }
  std::cout << std::endl;

  //read resource model:
  try
  {
    rm = readerRes.readResourceModel(resourceModelFile.c_str());
  }
  catch(HatScheT::Exception &e)
  {
    std::cerr << "Error: " << e << std::endl;
  }

  //read graph:
  try
  {
    if(rm.isEmpty() == false)
    {
      HatScheT::GraphMLGraphReader graphReader(&rm, &g);
      g = graphReader.readGraph(graphMLFile.c_str());
      std::string graphName = graphMLFile.substr(0,graphMLFile.length()-8);
      g.setName(graphName);
      cout << "graphName=" << graphName << endl;
      cout << g << endl;
      cout << rm << endl;
    }
    else
    {
      cout << "Error: Empty Resource Model Provided for graph parsing! Provide a valid resource model using --resource=" << endl;
      exit(0);
    }

    if(dotFile != "")
    {
      cout << "Writing to dotfile " << dotFile << endl;
      HatScheT::DotWriter dw(dotFile, &g, &rm);
      dw.setDisplayNames(true);
      dw.write();
    }

    HatScheT::SchedulerBase *scheduler;

    bool isModuloScheduler=false;
    std::list<std::string> solverWishList;
    if(ilpSolver.empty())
    {
      solverWishList = {"Gurobi","CPLEX","SCIP","LPSolve"};
    }
    else
    {
      solverWishList = {ilpSolver};
    }

    if(rm.isEmpty() == false && g.isEmpty() == false)
    {
      switch(schedulerString)
      {
        case ASAP:
          scheduler = new HatScheT::ASAPScheduler(g,rm);
          break;
        case ALAP:
          scheduler = new HatScheT::ALAPScheduler(g,rm);
          break;
        case UL:
          scheduler = new HatScheT::ULScheduler(g,rm);
          break;
        case MOOVAC:
          #ifdef USE_SCALP
          isModuloScheduler=true;
          scheduler = new HatScheT::MoovacScheduler(g,rm, solverWishList);
          if(timeout > 0) ((HatScheT::MoovacScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::MoovacScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::MoovacScheduler*) scheduler)->setSolverQuiet(true);
          #endif
          break;
        case MOOVACMINREG:
          #ifdef USE_SCALP
          isModuloScheduler=true;
          scheduler = new HatScheT::MoovacScheduler(g,rm, solverWishList);
          if(timeout > 0) ((HatScheT::MoovacMinRegScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::MoovacMinRegScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::MoovacMinRegScheduler*) scheduler)->setSolverQuiet(true);
          #endif
          break;
        case MODULOSDC:
          #ifdef USE_SCALP
          isModuloScheduler=true;
          scheduler = new HatScheT::ModuloSDCScheduler(g,rm,solverWishList);
          if(timeout>0) ((HatScheT::ModuloSDCScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::ModuloSDCScheduler*) scheduler)->setThreads(threads);
          #endif
          break;
        case RATIONALII:
          #ifdef USE_SCALP
          //not sure if you can verify this schedulers via the currently implemented function
          //isModuloScheduler=true;
          scheduler = new HatScheT::RationalIIScheduler(g,rm,solverWishList);
          if(timeout>0) ((HatScheT::RationalIIScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::RationalIIScheduler*) scheduler)->setThreads(threads);
          #endif
          break;
        case ASAPRATIONALII:{
          #ifdef USE_SCALP
          //not sure if you can verify this schedulers via the currently implemented function
          //isModuloScheduler=true;
          HatScheT::ASAPScheduler* asap = new HatScheT::ASAPScheduler(g,rm);
          scheduler = new HatScheT::GraphBasedMs(asap,g,rm,0.5,2);
          #endif
          break;}
        case NONE:
          cout << "Error, please select a scheduler using --scheduler=..." << endl;
          exit(-1);
        default:
          cout << "Error, scheduler not available!" << endl;
          exit(-1);
      }

      cout << "Performing schedule" << endl;
      scheduler->schedule();
      cout << "Finished schedule" << endl;

      if(isModuloScheduler)
      {
        if (HatScheT::verifyModuloSchedule(g, rm, scheduler->getSchedule(), scheduler->getII())){
          cout << "Modulo schedule verified successfully" << endl;
          cout << "Found II " << scheduler->getII() << " with sampleLatency " << scheduler->getScheduleLength() << endl;
        }
        else
        {
          cout << ">>> Modulo schedule verifivation failed! <<<" << endl;
          exit(-1);
        }
      }

      std::cout << "------------------------------------------------------------------------------------" << endl;
      std::cout << "---------------------------------- Schedule: ---------------------------------------" << endl;
      std::cout << "------------------------------------------------------------------------------------" << endl;
      std::cout << "latency = " << scheduler->getScheduleLength() << endl;
      scheduler->printStartTimes();

      if(htmlFile != "")
      {
        scheduler->writeScheduleChart(htmlFile);
      }

      delete scheduler;
    }


  }
  catch(HatScheT::Exception* e)
  {
    std::cerr << "Error: " << e->msg << std::endl;
  }

  return 0;
#endif
}
