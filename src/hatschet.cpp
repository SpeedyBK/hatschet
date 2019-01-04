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
#include <HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h>
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h>
#include "HatScheT/utility/Tests.h"
#include "HatScheT/scheduler/graphBased/graphBasedMs.h"
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/utility/Utility.h"

#ifdef USE_XERCESC
#include <HatScheT/utility/reader/GraphMLGraphReader.h>
#include <HatScheT/utility/reader/XMLResourceReader.h>
#include <HatScheT/utility/writer/GraphMLGraphWriter.h>
#include <HatScheT/utility/writer/XMLResourceWriter.h>
#endif

#ifdef USE_SCALP
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include <HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h>
#include <HatScheT/scheduler/ilpbased/MoovacResAwScheduler.h>
#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h"
#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11ResAwScheduler.h"
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/utility/reader/XMLTargetReader.h>
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/dev/ModSDC.h"
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
bool getCmdParameter(char* argv, const char* parameter, char* &value) {
  if(strstr(argv, parameter)) {
    value = argv+strlen(parameter);
    return true;
  } else {
    return false;
  }
}

void print_short_help() {
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
  std::cout << "                            ED97: ILP formulation by Eichenberger & Davidson" << std::endl;
  std::cout << "                            SH11: ILP formulation by Sucha & Hanzalek" << std::endl;
  std::cout << "                            MODULOSDC: Modulo SDC modulo scheduling" << std::endl;
  std::cout << "                            MODULOSDCFIEGE: Modulo SDC modulo scheduling (another implementation)" << std::endl;
  std::cout << "                            RATIONALII: Experimental rational II scheduler" << std::endl;
  std::cout << "                            RATIONALIIFIMMEL: Second experimental rational II scheduler" << std::endl;
  std::cout << "--resource=[string]       Path to XML resource constraint file" << std::endl;
  std::cout << "--target=[string]         Path to XML target constraint file" << std::endl;
  std::cout << "--graph=[string]          graphML graph file you want to read. (Make sure XercesC is enabled)" << std::endl;
  std::cout << "--dot=[string]            Optional path to dot file generated from graph+resource model (default: none)" << std::endl;
  std::cout << "--writegraph=[string]     Optional path to graphML file to write the graph model (default: none)" << std::endl;
  std::cout << "--writeresource=[string]  Optional path to xml file to write the resource model (default: none)" << std::endl;
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

int main(int argc, char *args[]) {
#ifndef USE_XERCESC
  throw HatScheT::Exception("XercesC not active! Without XML-parsing, this interface is disabled! Install XeresC for using the HatScheT binary");
#else
  std::string ilpSolver="";
  int threads=1;
  int timeout=-1; //default -1 means no timeout
  int maxLatency=-1;

  //variables for Rational II scheduling
  //experimental
  int samples=2; //default
  int modulo=2; //defaul

  bool solverQuiet=true;

  enum SchedulersSelection {ASAP, ALAP, UL, MOOVAC, MOOVACMINREG, RAMS, ED97, SH11, SH11RA, MODULOSDC, MODULOSDCFIEGE, RATIONALII, RATIONALIIFIMMEL, ASAPRATIONALII, NONE};
  SchedulersSelection schedulerSelection = NONE;
  string schedulerSelectionStr;

  std::string graphMLFile="";
  std::string writeGraphMLFile="";
  std::string resourceModelFile="";
  std::string writeResourceModelFile="";
  std::string targetFile="";
  std::string dotFile="";
  std::string htmlFile="";

  if(argc <= 1) {
      print_short_help();
      exit(0);
  }

  try {
    HatScheT::ResourceModel rm;
    HatScheT::Graph g;
    HatScheT::Target target;

    HatScheT::XMLResourceReader readerRes(&rm);
    HatScheT::XMLTargetReader readerTarget(&target);

    //parse command line
    for (int i = 1; i < argc; i++) {
      char* value;
      if(getCmdParameter(args[i],"--timeout=",value)) {
        timeout = atol(value);
      }
      else if(getCmdParameter(args[i],"--threads=",value)) {
        threads = atol(value);
      }
      else if(getCmdParameter(args[i],"--show_ilp_solver_output",value)) {
        solverQuiet=false;
      }
      else if(getCmdParameter(args[i],"--solver=",value)) {
        #ifndef USE_SCALP
        throw HatScheT::Exception("ScaLP not active! Solver cant be chosen: " + std::string(value));
        #endif
        ilpSolver = std::string(value);
      }
      else if(getCmdParameter(args[i],"--resource=",value)) {
        resourceModelFile = std::string(value);
      }
      else if(getCmdParameter(args[i],"--graph=",value)) {
        graphMLFile = std::string(value);
      }
      else if(getCmdParameter(args[i],"--target=",value)) {
        targetFile = std::string(value);
      }
      else if(getCmdParameter(args[i],"--dot=",value)) {
        dotFile = value;
      }
      else if(getCmdParameter(args[i],"--writegraph=",value)) {
        writeGraphMLFile = value;
      }
      else if(getCmdParameter(args[i],"--writeresource=",value)) {
        writeResourceModelFile = value;
      }
      else if(getCmdParameter(args[i],"--samples=",value)) {
        samples = atol(value);
      }
      else if(getCmdParameter(args[i],"--modulo=",value)) {
        modulo = atol(value);
      }
      else if(getCmdParameter(args[i],"--maxlatency=",value)) {
        maxLatency = atol(value);
      }
      else if(getCmdParameter(args[i],"--html=",value)) {
        htmlFile = value;
      }
      else if(getCmdParameter(args[i],"--scheduler=",value)) {
        std::string valueStr = std::string(value);

        schedulerSelectionStr.resize(valueStr.size());
        std::transform(valueStr.begin(),valueStr.end(),schedulerSelectionStr.begin(),::tolower);

        if(schedulerSelectionStr == "asap") {
          schedulerSelection = ASAP;
        }
        else if(schedulerSelectionStr == "alap") {
          schedulerSelection = ALAP;
        }
        else if(schedulerSelectionStr == "ul") {
          schedulerSelection = UL;
        }
        else if(schedulerSelectionStr == "moovac") {
          schedulerSelection = MOOVAC;
        }
        else if(schedulerSelectionStr == "moovacminreg") {
          schedulerSelection = MOOVACMINREG;
        }
        else if(schedulerSelectionStr == "rams") {
          schedulerSelection = RAMS;
        }
        else if(schedulerSelectionStr == "ed97") {
          schedulerSelection = ED97;
        }
        else if(schedulerSelectionStr == "sh11") {
          schedulerSelection = SH11;
        }
        else if(schedulerSelectionStr == "sh11ra") {
          schedulerSelection = SH11RA;
        }
        else if(schedulerSelectionStr == "modulosdc") {
          schedulerSelection = MODULOSDC;
        }
        else if(schedulerSelectionStr == "modulosdcfiege") {
          schedulerSelection = MODULOSDCFIEGE;
        }
        else if(schedulerSelectionStr == "rationalii") {
          schedulerSelection = RATIONALII;
        }
        else if(schedulerSelectionStr == "rationaliifimmel") {
            schedulerSelection = RATIONALIIFIMMEL;
        }
        else if(schedulerSelectionStr == "asapRationalII") {
          schedulerSelection = ASAPRATIONALII;
        }
        else {
          throw HatScheT::Exception("Scheduler " + valueStr + " unknown!");
        }
      }
      //HatScheT Auto Test Function
      else if(getCmdParameter(args[i],"--test=",value)) {
        #ifdef USE_SCALP
        string str = std::string(value);
        if(str=="READ" && HatScheT::Tests::readTest()==false) exit(-1);
        if(str=="MOOVAC" && HatScheT::Tests::moovacTest()==false) exit(-1);
        if(str=="RWRS" && HatScheT::Tests::readWriteReadScheduleTest()==false) exit(-1);
        if(str=="MODULOSDC" && HatScheT::Tests::moduloSDCTest()==false) exit(-1);
        if(str=="MODULOSDCFIEGE" && HatScheT::Tests::moduloSDCTestFiege()==false) exit(-1);
        if(str=="API" && HatScheT::Tests::apiTest()==false) exit(-1);
        if(str=="ASAPHC" && HatScheT::Tests::asapHCTest()==false) exit(-1);
        if(str=="ALAPHC" && HatScheT::Tests::alapHCTest()==false) exit(-1);
        if(str=="ULScheduler" && HatScheT::Tests::ulSchedulerTest()==false) exit(-1);
        if(str=="RATMINII" && HatScheT::Tests::rationalMinIITest()==false) exit(-1);
        if(str=="CRITPATH" && HatScheT::Tests::cpTest()==false) exit(-1);
        #else
        throw HatScheT::Exception("ScaLP not active! Test function disabled!");
        #endif

        exit(0);
      }
      else if((args[i][0] != '-') && getCmdParameter(args[i],"",value)) {
        string str = std::string(value);
        graphMLFile = std::string(value);

      } else {
        std::cout << "Error: Illegal Option: " << args[i] << std::endl;
        print_short_help();
        exit(-1);
      }
    }
    //output major settings:
    std::cout << "settings:" << std::endl;
    std::cout << "graph model=" << graphMLFile << endl;
    std::cout << "resource model=" << resourceModelFile << endl;
    std::cout << "target=" << targetFile << endl;
    std::cout << "timeout=" << timeout << std::endl;
    std::cout << "threads=" << threads << std::endl;
    std::cout << "scheduler=";
    switch(schedulerSelection) {
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
      case RAMS:
        cout << "RAMS";
        break;
      case ED97:
        cout << "ED97";
        break;
      case SH11:
        cout << "SH11";
        break;
      case SH11RA:
        cout << "SH11RA";
        break;
      case MODULOSDC:
        cout << "MODULOSDC";
        break;
      case MODULOSDCFIEGE:
        cout << "MODULOSDCFIEGE";
            break;
      case RATIONALII:
        cout << "RATIONALII";
        break;
        case RATIONALIIFIMMEL:
            cout << "RATIONALIIFIMMEL";
            break;
      case ASAPRATIONALII:
        cout << "ASAPRATIONALII";
        break;
      case NONE:
        cout << "NONE";
        break;
    }
    std::cout << std::endl;

    //read target if provided
    if(targetFile != "") {
      readerTarget.readHardwareTarget(targetFile.c_str());
      cout << target << endl;
    }

    //read resource model:
    readerRes.readResourceModel(resourceModelFile.c_str());

    //check target and resource model for consistency
    if(target.isEmpty() == false && rm.isEmpty() == false){
      if(HatScheT::Utility::resourceModelAndTargetValid(rm,target) == false){
        throw HatScheT::Exception("Hardware Target and Resource Model are inconsisent! Provide a valid resource model using --resource= and hardware target using --target==");
      }
    }

    //read graph:
    if(rm.isEmpty() == false) {
      HatScheT::GraphMLGraphReader graphReader(&rm, &g);
      graphReader.readGraph(graphMLFile.c_str());
      std::string graphName = graphMLFile.substr(0,graphMLFile.length()-8);
      g.setName(graphName);
      cout << "graphName=" << graphName << endl;
      cout << g << endl;
      cout << rm << endl;
    } else {
      throw HatScheT::Exception("Empty Resource Model Provided for graph parsing! Provide a valid resource model using --resource=");
    }

    if(dotFile != "") {
      cout << "Writing to dotfile " << dotFile << endl;
      HatScheT::DotWriter dw(dotFile, &g, &rm);
      dw.setDisplayNames(true);
      dw.write();
    }

    if(writeResourceModelFile != "") {
      if(rm.isEmpty() == true) throw HatScheT::Exception("Empty Resource Model Provided for resource writing! Provide a valid resource model using --resource=");
      cout << "Writing resource to file " << writeResourceModelFile << endl;
      HatScheT::XMLResourceWriter rw(writeResourceModelFile, &rm);
      rw.write();
    }

    if(writeGraphMLFile != "") {
      if(g.isEmpty()==true) throw HatScheT::Exception("No graph provided for graphML writing! Provide a graph using --graph=");
      if(rm.isEmpty() == true) throw HatScheT::Exception("Empty Resource Model Provided for graph writing! Provide a valid resource model using --resource=");
      cout << "Writing graph to file " << writeGraphMLFile << endl;
      HatScheT::GraphMLGraphWriter gw(writeGraphMLFile, &g, &rm);
      gw.write();
    }

    HatScheT::SchedulerBase *scheduler;

    bool isModuloScheduler=false;
    std::list<std::string> solverWishList;
    if(ilpSolver.empty()) {
      solverWishList = {"Gurobi","CPLEX","SCIP","LPSolve"};
    } else {
      solverWishList = {ilpSolver};
    }

    if(rm.isEmpty() == false && g.isEmpty() == false) {
      switch(schedulerSelection) {
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
          if(maxLatency > 0) ((HatScheT::MoovacScheduler*) scheduler)->setMaxLatencyConstraint(maxLatency);
          ((HatScheT::MoovacScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::MoovacScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case MOOVACMINREG:
          isModuloScheduler=true;
          scheduler = new HatScheT::MoovacMinRegScheduler(g,rm, solverWishList);
          if(timeout > 0) ((HatScheT::MoovacMinRegScheduler*) scheduler)->setSolverTimeout(timeout);
          if(maxLatency > 0) ((HatScheT::MoovacMinRegScheduler*) scheduler)->setMaxLatencyConstraint(maxLatency);
          ((HatScheT::MoovacMinRegScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::MoovacMinRegScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case RAMS:
          isModuloScheduler=true;
          scheduler = new HatScheT::MoovacResAwScheduler(g,rm, solverWishList, target);
          if(timeout > 0) ((HatScheT::MoovacResAwScheduler*) scheduler)->setSolverTimeout(timeout);
          if(maxLatency > 0) ((HatScheT::MoovacResAwScheduler*) scheduler)->setMaxLatencyConstraint(maxLatency);
          ((HatScheT::MoovacResAwScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::MoovacResAwScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case ED97: {
          isModuloScheduler = true;
          auto *ed97 = new HatScheT::EichenbergerDavidson97Scheduler(g, rm, solverWishList);
          if (timeout > 0)    ed97->setSolverTimeout(timeout);
          if (maxLatency > 0) ed97->setMaxLatencyConstraint(maxLatency);
          ed97->setThreads(threads);
          ed97->setSolverQuiet(solverQuiet);
          scheduler = ed97;
          break;
        }
        case SH11: {
          isModuloScheduler = true;
          auto *sh11 = new HatScheT::SuchaHanzalek11Scheduler(g, rm, solverWishList);
          if (timeout > 0)    sh11->setSolverTimeout(timeout);
          if (maxLatency > 0) sh11->setMaxLatencyConstraint(maxLatency);
          sh11->setThreads(threads);
          sh11->setSolverQuiet(solverQuiet);
          scheduler = sh11;
          break;
        }
        case SH11RA: {
          isModuloScheduler = true;
          auto *sh11ra = new HatScheT::SuchaHanzalek11ResAwScheduler(g, rm, target, solverWishList);
          if (timeout > 0)    sh11ra->setSolverTimeout(timeout);
          if (maxLatency > 0) sh11ra->setMaxLatencyConstraint(maxLatency);
          sh11ra->setThreads(threads);
          sh11ra->setSolverQuiet(solverQuiet);
          scheduler = sh11ra;
          break;
        }
        case MODULOSDC:
          isModuloScheduler=true;
          scheduler = new HatScheT::ModuloSDCScheduler(g,rm,solverWishList);
          if(timeout>0) ((HatScheT::ModuloSDCScheduler*) scheduler)->setSolverTimeout(timeout);
          if(maxLatency > 0) ((HatScheT::ModuloSDCScheduler*) scheduler)->setMaxLatencyConstraint(maxLatency);
          ((HatScheT::ModuloSDCScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::ModuloSDCScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
        case MODULOSDCFIEGE:
          isModuloScheduler=true;
              scheduler = new HatScheT::ModSDC(g,rm,solverWishList);
              if(timeout>0) ((HatScheT::ModSDC*) scheduler)->setSolverTimeout(timeout);
              if(maxLatency > 0) ((HatScheT::ModSDC*) scheduler)->setMaxLatencyConstraint(maxLatency);
              ((HatScheT::ModSDC*) scheduler)->setThreads(threads);
              ((HatScheT::ModSDC*) scheduler)->setSolverQuiet(solverQuiet);
              break;
        case RATIONALII:
          scheduler = new HatScheT::RationalIIScheduler(g,rm,solverWishList);
          if(timeout>0) ((HatScheT::RationalIIScheduler*) scheduler)->setSolverTimeout(timeout);
          if(maxLatency > 0) ((HatScheT::RationalIIScheduler*) scheduler)->setMaxLatencyConstraint(maxLatency);
          ((HatScheT::RationalIIScheduler*) scheduler)->setSamples(samples);
          ((HatScheT::RationalIIScheduler*) scheduler)->setModulo(modulo);
          ((HatScheT::RationalIIScheduler*) scheduler)->setThreads(threads);
          ((HatScheT::RationalIIScheduler*) scheduler)->setSolverQuiet(solverQuiet);
          break;
          case RATIONALIIFIMMEL:
              scheduler = new HatScheT::RationalIISchedulerFimmel(g,rm,solverWishList);
              if(timeout>0) ((HatScheT::RationalIISchedulerFimmel*) scheduler)->setSolverTimeout(timeout);
              if(maxLatency > 0) ((HatScheT::RationalIIScheduler*) scheduler)->setMaxLatencyConstraint(maxLatency);
              ((HatScheT::RationalIISchedulerFimmel*) scheduler)->setThreads(threads);
              ((HatScheT::RationalIISchedulerFimmel*) scheduler)->setSolverQuiet(solverQuiet);
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
        case ED97:
        case MODULOSDC:
        case MODULOSDCFIEGE:
        case RATIONALII:
        case RATIONALIIFIMMEL:
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

      if(isModuloScheduler) {
        if (HatScheT::verifyModuloSchedule(g, rm, scheduler->getSchedule(), scheduler->getII())){
          cout << "Modulo schedule verified successfully" << endl;
          cout << "Found II " << scheduler->getII() << " with sampleLatency " << scheduler->getScheduleLength() << endl;
        } else {
          cout << ">>> Modulo schedule verifivation failed! <<<" << endl;
          exit(-1);
        }
      }

      std::cout << "------------------------------------------------------------------------------------" << endl;
      std::cout << "---------------------------------- Schedule: ---------------------------------------" << endl;
      std::cout << "------------------------------------------------------------------------------------" << endl;
      std::cout << "latency = " << scheduler->getScheduleLength() << endl;
      HatScheT::Utility::printSchedule(scheduler->getSchedule());

      if(htmlFile != "") {
        scheduler->writeScheduleChart(htmlFile);
      }

      delete scheduler;
    }
  }
  catch(HatScheT::Exception &e) {
    std::cerr << "Error: " << e.msg << std::endl;
    exit(-1);
  }
#ifdef USE_SCALP
  catch(ScaLP::Exception &e) {
    std::cerr << "Scalp Error: " << e.msg << std::endl;
    exit(-1);
  }
#endif //USE_SCALP
  return 0;
#endif
}
