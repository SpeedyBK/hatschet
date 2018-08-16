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

#include <iostream>
#include <string>
#include <algorithm>
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
  std::cout << "Option                   Meaning" << std::endl;
  std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "--scheduler=<algorithm>  Scheduling algorithm, select one of the following:" << std::endl;
  std::cout << "                            ASAP: As-soon-as-possible scheduling" << std::endl;
  std::cout << "                            ALAP: As-last-as-possible scheduling" << std::endl;
  std::cout << "                            UL: Universal list scheduling" << std::endl;
  std::cout << "                            MOOVAC: Moovac ILP-based exact modulo scheduling" << std::endl;
  std::cout << "                            MODULOSDC: Modulo SDC modulo scheduling" << std::endl;
  std::cout << "                            RATIONALII: Experimental rational II scheduler" << std::endl;
  std::cout << "--resource=[string]       Path to XML resource constrain file" << std::endl;
  std::cout << "--graph=[string]          graphML graph file you want to read. (Make sure XercesC is enabled)" << std::endl;
  std::cout << "--dot=[string]            Optional path to dot file generated from graph+resource model (default: none)" << std::endl;
  std::cout << "--html=[string]           Optional path to html file for a schedule chart" << std::endl;
  std::cout << std::endl;
  std::cout << "Options for ILP-based schedulers:" << std::endl;
  std::cout << std::endl;
  std::cout << "--solver=<solver>         ILP solver (as available in ScaLP library), typicall one of the following: Gurobi, CPLEX, SCIP, LPSolve" << std::endl;
  std::cout << "--timeout=[int]           ILP solver timeout in seconds (default: -1, no timeout)" << std::endl;
  std::cout << "--threads=[int]           Number of threads for the ILP solver (default: 1)" << std::endl;
  std::cout << "--show_ilp_solver_output  Shows the ILP solver output" << std::endl;
  std::cout << std::endl;
}
int main(int argc, char *args[])
{
#ifndef USE_XERCESC
  throw HatScheT::Exception("XercesC not active! Without XML-parsing, this interface is disabled! Install XeresC for using the HatScheT binary");
#else
  std::string ilpSolver="";
  int threads=1;
  int timeout=-1; //default -1 means no timeout

  bool solverQuiet=true;

  enum SchedulersSelection {ASAP, ALAP, UL, MOOVAC, MOOVACMINREG, MODULOSDC, RATIONALII, ASAPRATIONALII, NONE};
  SchedulersSelection schedulerSelection = NONE;
  string schedulerSelectionStr;

  std::string graphMLFile="";
  std::string resourceModelFile="";
  std::string dotFile="";
  std::string htmlFile="";


  if(argc <= 1)
  {
      print_short_help();
      exit(0);
  }

  try
  {
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
      else if(getCmdParameter(args[i],"--show_ilp_solver_output",value))
      {
        solverQuiet=false;
      }
      else if(getCmdParameter(args[i],"--solver=",value))
      {
        #ifndef USE_SCALP
        throw HatScheT::Exception("ScaLP not active! Solver cant be chosen: " + std::string(value));
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

        schedulerSelectionStr.resize(valueStr.size());
        std::transform(valueStr.begin(),valueStr.end(),schedulerSelectionStr.begin(),::tolower);

        if(schedulerSelectionStr == "asap")
        {
          schedulerSelection = ASAP;
        }
        else if(schedulerSelectionStr == "alap")
        {
          schedulerSelection = ALAP;
        }
        else if(schedulerSelectionStr == "ul")
        {
          schedulerSelection = UL;
        }
        else if(schedulerSelectionStr == "moovac")
        {
          schedulerSelection = MOOVAC;
        }
        else if(schedulerSelectionStr == "moovacminreg")
        {
          schedulerSelection = MOOVACMINREG;
        }
        else if(schedulerSelectionStr == "modulosdc")
        {
          schedulerSelection = MODULOSDC;
        }
        else if(schedulerSelectionStr == "rationalii")
        {
          schedulerSelection = RATIONALII;
        }
        else if(schedulerSelectionStr == "asapRationalII")
        {
          schedulerSelection = ASAPRATIONALII;
        }
        else
        {
          throw HatScheT::Exception("Scheduler " + valueStr + " unknown!");
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
        if(str=="ASAPHC" && HatScheT::Tests::asapHCTest()==false) exit(-1);
        if(str=="ALAPHC" && HatScheT::Tests::alapHCTest()==false) exit(-1);
        if(str=="ULScheduler" && HatScheT::Tests::ulSchedulerTest()==false) exit(-1);
        if(str=="OCC" && HatScheT::Tests::occurrenceTest()==false) exit(-1);
        if(str=="OCCS" && HatScheT::Tests::occurrenceSetTest()==false) exit(-1);
        if(str=="OCCSC" && HatScheT::Tests::occurrenceSetCombinationTest()==false) exit(-1);
        if(str=="SGMS" && HatScheT::Tests::sgmSchedulerTest()==false) exit(-1);
        #else
        throw HatScheT::Exception("ScaLP not active! Test function disabled!");
        #endif

        exit(0);
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
    switch(schedulerSelection)
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
    rm = readerRes.readResourceModel(resourceModelFile.c_str());

  //read graph:
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
      throw HatScheT::Exception("Empty Resource Model Provided for graph parsing! Provide a valid resource model using --resource=");
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
      switch(schedulerSelection)
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
#ifdef USE_SCALP
        case MOOVAC:
          isModuloScheduler=true;
          scheduler = new HatScheT::MoovacScheduler(g,rm, solverWishList);
          if(timeout > 0) ((HatScheT::MoovacScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::MoovacScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::MoovacScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case MOOVACMINREG:
          isModuloScheduler=true;
          scheduler = new HatScheT::MoovacScheduler(g,rm, solverWishList);
          if(timeout > 0) ((HatScheT::MoovacMinRegScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::MoovacMinRegScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::MoovacMinRegScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case MODULOSDC:
          isModuloScheduler=true;
          scheduler = new HatScheT::ModuloSDCScheduler(g,rm,solverWishList);
          if(timeout>0) ((HatScheT::ModuloSDCScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::ModuloSDCScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::ModuloSDCScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case RATIONALII:
          scheduler = new HatScheT::RationalIIScheduler(g,rm,solverWishList);
          if(timeout>0) ((HatScheT::RationalIIScheduler*) scheduler)->setSolverTimeout(timeout);
          ((HatScheT::RationalIIScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::RationalIIScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case ASAPRATIONALII:
        {
          HatScheT::ASAPScheduler* asap = new HatScheT::ASAPScheduler(g,rm);
          scheduler = new HatScheT::GraphBasedMs(asap,g,rm,0.5,2);
          delete asap;
          break;
        }
#else
        case MOOVAC:
        case MOOVACMINREG:
        case MODULOSDC:
        case RATIONALII:
        case ASAPRATIONALII:
          throw HatScheT::Exception("scheduler " + schedulerSelectionStr + " not available without SCALP library. Please build with SCALP.");
          break;
#endif //USE_SCALP
        case NONE:
          throw HatScheT::Exception("No scheduler given, please select a scheduler using --scheduler=...");
        default:
          throw HatScheT::Exception("Scheduler " + schedulerSelectionStr + " not available!");
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
  catch(HatScheT::Exception &e)
  {
    std::cerr << "Error: " << e.msg << std::endl;
    exit(-1);
  }
#ifdef USE_SCALP
  catch(ScaLP::Exception &e)
  {
    std::cerr << "Scalp Error: " << e.msg << std::endl;
    exit(-1);
  }
#endif //USE_SCALP

  return 0;
#endif
}
