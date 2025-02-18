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
#include <chrono>
#include <memory>
#include <algorithm>
#include <HatScheT/utility/Exception.h>
#include <HatScheT/Graph.h>
#include <HatScheT/utility/writer/DotWriter.h>
#include "HatScheT/utility/Tests.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/scheduler/dev/DeSouzaRosa23NIS.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include <HatScheT/utility/writer/ScheduleAndBindingWriter.h>
#include <HatScheT/utility/reader/ScheduleAndBindingReader.h>

#ifdef USE_XERCESC

#include <HatScheT/utility/reader/GraphMLGraphReader.h>
#include <HatScheT/utility/reader/XMLResourceReader.h>
#include <HatScheT/utility/reader/XMLTargetReader.h>
#include <HatScheT/utility/writer/GraphMLGraphWriter.h>
#include <HatScheT/utility/writer/XMLResourceWriter.h>

#endif

#ifdef USE_SCALP

#include <HatScheT/scheduler/dev/ILPScheduleLengthSweeper.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include <HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h>
#include <HatScheT/scheduler/ilpbased/MoovacResAwScheduler.h>
#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h"
#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11ResAwScheduler.h"
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h>
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h>
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>
#include <HatScheT/scheduler/dev/SCCQScheduler.h>
#include <HatScheT/scheduler/dev/UniformRationalIIScheduler.h>
#include <HatScheT/scheduler/dev/UniformRationalIISchedulerNew.h>
#include <HatScheT/scheduler/dev/RationalIIModuloSDCScheduler.h>
#include <HatScheT/scheduler/dev/NonUniformRationalIIScheduler.h>
#include <HatScheT/scheduler/dev/UnrollRationalIIScheduler.h>
#include <HatScheT/scheduler/dev/DaiZhang19Scheduler.h>
#include <HatScheT/utility/reader/XMLTargetReader.h>
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include "HatScheT/scheduler/dev/NonModIlpTestScheduler.h"

#endif

#ifdef USE_CADICAL

#include <HatScheT/scheduler/dev/SDSScheduler.h>
#include <HatScheT/scheduler/satbased/SATScheduler.h>
#include <HatScheT/scheduler/satbased/SATCDLScheduler.h>
#include <HatScheT/scheduler/satbased/SATSchedulerLatOpt.h>
#include <HatScheT/scheduler/satbased/SATSchedulerBinEnc.h>
#include <HatScheT/scheduler/satbased/SATRatIIScheduler.h>
#include <HatScheT/scheduler/satbased/SATMinRegScheduler.h>
#include <HatScheT/scheduler/satbased/SATSCCScheduler.h>
#include <HatScheT/scheduler/satbased/SATCombinedScheduler.h>
#include <HatScheT/scheduler/satbased/SATCombinedRatIIScheduler.h>

#endif

#ifdef USE_Z3

#include <z3++.h>
#include <HatScheT/scheduler/smtbased/SMTUnaryScheduler.h>
#include <HatScheT/scheduler/smtbased/SMTCDLScheduler.h>
#include <HatScheT/scheduler/smtbased/SMASHScheduler.h>
#include <HatScheT/scheduler/smtbased/SMASHMinLatNonMod.h>

#endif

#if defined(USE_Z3) || defined(USE_SCALP)

#include <HatScheT/scheduler/SCCPreprocessingSchedulers/SCCSchedulerTemplate.h>

#endif

/**
 * Returns the value as string of a command line argument in syntax --key=value
 * @param argv the command line string
 * @param parameter the name of the parameter
 * @param value as string if parameter string was found
 * @return True if parameter string was found
 */
bool getCmdParameter(char *argv, const char *parameter, char *&value) {
    if (strstr(argv, parameter)) {
        value = argv + strlen(parameter);
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
    std::cout
        << "--------------------------------------------------------------------------------------------------------------------------------"
        << std::endl;
    std::cout << "--scheduler=<algorithm>  Scheduling algorithm, select one of the following:" << std::endl;
    std::cout << "                            ASAP: As-soon-as-possible scheduling" << std::endl;
    std::cout << "                            ALAP: As-last-as-possible scheduling" << std::endl;
    std::cout << "                            UL: Universal list scheduling" << std::endl;
    std::cout << "                            MOOVAC: Moovac ILP-based exact modulo scheduling" << std::endl;
    std::cout << "                            ED97: ILP formulation by Eichenberger & Davidson" << std::endl;
    std::cout << "                            SH11: ILP formulation by Sucha & Hanzalek" << std::endl;
    std::cout << "                            MODULOSDC: Modulo SDC modulo scheduling " << std::endl;
    std::cout << "                            RATIONALII: Experimental rational II scheduler" << std::endl;
    std::cout
        << "                            UNROLLRATIONALII: Experimental rational II scheduler which finds a rational II modulo schedule based on graph unrolling and integer II scheduling"
        << std::endl;
    std::cout
        << "                            UNIFORMRATIONALII: Experimental rational II scheduler which always creates uniform schedules"
        << std::endl;
    std::cout
        << "                            NONUNIFORMRATIONALII: Experimental rational II scheduler which does not guarantee uniformity"
        << std::endl;
    std::cout << "                            RATIONALIIFIMMEL: Third experimental rational II scheduler" << std::endl;
    std::cout << "                            RATIONALIIMODULOQ: Fourth experimental rational II scheduler (heuristic)"
              << std::endl;
    std::cout
        << "                            RATIONALIISCCQ: Fifth experimental rational II scheduler (SCC based heuristic)"
        << std::endl;
    std::cout
        << "                            RATIONALIIMODULOSDC: Sixth experimental rational II scheduler (SDC based heuristic)"
        << std::endl;
    std::cout
        << "                            SAT: Experimental integer II scheduler based on Boolean Satisfiability (uses CaDiCaL as backend)"
        << std::endl;
    std::cout
        << "                            SATRATII: Experimental rational II scheduler based on Boolean Satisfiability (uses CaDiCaL as backend)"
        << std::endl;
    std::cout
        << "                            SATSCC: Experimental integer II scheduler based on Boolean Satisfiability and partitioning the graph into Strongly Connected Components (uses CaDiCaL as backend)"
        << std::endl;
    std::cout
        << "                            SATCOMBINED: Experimental integer II scheduler based on Boolean Satisfiability and partitioning the graph into Strongly Connected Components with latency \n"
        << "                              refinement using the optimal SAT scheduler (uses CaDiCaL as backend)"
        << std::endl;
    std::cout
        << "                            SATCOMBINEDRATII: Experimental rational II scheduler based on Boolean Satisfiability and partitioning the graph into Strongly Connected Components with latency \n"
        << "                              refinement using the optimal SAT scheduler (uses CaDiCaL as backend)"
        << std::endl;
    std::cout
        << "                            SATMINREG: Experimental integer II scheduler based on Boolean Satisfiability including register minimization (uses CaDiCaL as backend)"
        << std::endl;
    std::cout
        << "                            NIS: Non-iterative SDC Modulo Scheduler"
        << std::endl;
    std::cout
        << "                            SCC: Experimental. Has two stages. First stage trys to find a schedule for the optimal II,\n"
        << "                              but is not optimized for latency. If a schedule is found the second stage try to optimize latency. User can\n"
        << "                              set which schedulers are used as backend." << std::endl;
    std::cout << "                            SMT: Experimental uses smtbased-formulation and Z3 Backend" << std::endl;
    std::cout
        << "--sccScheduler=[string]     Backendscheduler for 1st Stage of SCC-Scheduler. So far: [MOOVAC, ED97, SH11, SMT, SMTCDL, MODSDC]. Only effective if '--scheduler=SCC'."
        << std::endl;
    std::cout
        << "--finalScheduler=[string]   Backendscheduler for 2nd Stage of SCC-Scheduler. So far: [MOOVAC, ED97, SH11, SMT, SMTCDL, MODSDC]. Only effective if '--scheduler=SCC'."
        << std::endl;
    std::cout << "--resource=[string]         Path to XML resource constraint file" << std::endl;
    std::cout << "--target=[string]           Path to XML target constraint file" << std::endl;
    std::cout << "--graph=[string]            graphML graph file you want to read. (Make sure XercesC is enabled)"
              << std::endl;
    std::cout
        << "--dot=[string]              Optional path to dot file generated from graph+resource model (default: none)"
        << std::endl;
    std::cout << "--writegraph=[string]       Optional path to graphML file to write the graph model (default: none)"
              << std::endl;
    std::cout << "--writeresource=[string]    Optional path to xml file to write the resource model (default: none)"
              << std::endl;
    std::cout
        << "--writeschedule=[string]    Optional path to csv file to write the found schedule and bindings (default: none)"
        << std::endl;
    std::cout << "--html=[string]             Optional path to html file for a schedule chart" << std::endl;
    std::cout << "--quiet=[0/1]               Set scheduling algorithm quiet for no couts (default: 1)" << std::endl;
    std::cout << "--writeLPFile=[0/1]         Write LP file (only available for some schedulers; default: 0)"
              << std::endl;
    std::cout
        << "--set_bounds=[string]       Set bounds (i.e., II, max latency, and optional max #registers) from scheduling file"
        << std::endl;
    std::cout
        << "--boundSL=[0/1]             Automatically bound the schedule length to accelerate the solvers in proving infeasible IIs and optimal schedule lengths"
        << std::endl;
    std::cout << std::endl;

    std::cout << "Options for ILP-based schedulers:" << std::endl;
    std::cout << std::endl;
    std::cout
        << "--solver=<solver>           ILP solver (as available in ScaLP library), typicall one of the following: Gurobi, CPLEX, SCIP, LPSolve"
        << std::endl;
    std::cout << "--timeout=[int]             ILP solver timeout in seconds (default: -1, no timeout)" << std::endl;
    std::cout << "--threads=[int]             Number of threads for the ILP solver (default: 1)" << std::endl;
    std::cout << "--show_ilp_solver_output    Shows the ILP solver output" << std::endl;
    std::cout << std::endl;

    std::cout << "Options for iterative schedulers:" << std::endl;
    std::cout << "--max_runs=[int]            maximum number of allowed iterations (default: -1, no limit)"
              << std::endl;
    std::cout << std::endl;

    std::cout << "Options for register minimization schedulers:" << std::endl;
    std::cout << "--max_regs=[int]            maximum number of registers to allocate (default: -1, no limit)"
              << std::endl;
    std::cout << std::endl;

    std::cout << "Options for ILP-based rational II scheduling:" << std::endl;
    std::cout << "--samples=[int]           Demanded samples per modulo" << std::endl;
    std::cout << "--modulo=[int]            Demanded clock cycles for the repeating schedule" << std::endl;
    std::cout
        << "--uniform=[0/1]           Uniform(1) / Non-Uniform (0) schedule for samples in rational II modulo scheduling"
        << std::endl;
    std::cout << "--II=[int] specify candidate II (if supported by the scheduler)" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char *args[]) {

#ifndef USE_XERCESC
    throw HatScheT::Exception("XercesC not active! Without XML-parsing, this interface is disabled! Install XeresC for using the HatScheT binary");
#else
    std::string ilpSolver = "";
    int threads = 1;
    int timeout = -1; //default -1 means no timeout
    int maxLatency = -1;
    bool quiet = true;
    int maxRuns = -1;
    bool boundSL = false;

    //variables for Rational II scheduling
    int samples = -1; //default
    int modulo = -1; //default
    bool uniform = true; //default

    // variables for min reg schedulers
    int targetII = -1;
    int maxRegs = -1;

    //flag to enable mux optimal binding
    //experimental
    bool optBinding = false;
    bool printSCC = false;
    bool solverQuiet = true;
    bool writeLPFile = false;

    enum SchedulersSelection {
      NONMODILP,
      ILPSLSWEEPER,
      SCC,
      SMTCDL,
      SMT,
      SMASH,
      SMASHMINLATNONMOD,
      SDS,
      SAT,
      SATLAT,
      SATBIN,
      SATCDL,
      SATSCC,
      SATCOMBINED,
      SATCOMBINEDLAT,
      SATCOMBINEDBIN,
      SATCOMBINEDBINOVERLAP,
      SATCOMBINEDRATII,
      SATMINREG,
      SATRATII,
      ASAP,
      ALAP,
      UL,
      MOOVAC,
      MOOVACMINREG,
      RAMS,
      ED97,
      NIS,
      SH11,
      SH11RA,
      MODULOSDC,
      RATIONALIIMODULOSDC,
      RATIONALII,
      UNROLLRATIONALII,
      UNIFORMRATIONALII,
      NONUNIFORMRATIONALII,
      RATIONALIIMODULOQ,
      RATIONALIISCCQ,
      RATIONALIIFIMMEL,
      SUGRREDUCTION,
      LATENCYTEST,
      NONE
    };
    SchedulersSelection schedulerSelection = NONE;
    SchedulersSelection sccSchedulerSelection = SAT;
    SchedulersSelection finalSchedulerSelection = NONE;
    string schedulerSelectionStr;

    std::string graphMLFile = "";
    std::string writeGraphMLFile = "";
    std::string resourceModelFile = "";
    std::string writeResourceModelFile = "";
    std::string targetFile = "";
    std::string dotFile = "";
    std::string htmlFile = "";
    std::string scheduleFile = "";
    std::string inputScheduleFile = "";

    if (argc <= 1) {
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
            char *value;
            if (getCmdParameter(args[i], "--timeout=", value)) {
                timeout = atol(value);
            } else if (getCmdParameter(args[i], "--max_runs=", value)) {
                maxRuns = atol(value);
            } else if (getCmdParameter(args[i], "--boundSL=", value)) {
                boundSL = (bool) atol(value);
            } else if (getCmdParameter(args[i], "--max_regs=", value)) {
                maxRegs = atol(value);
            } else if (getCmdParameter(args[i], "--set_bounds=", value)) {
                inputScheduleFile = std::string(value);
            } else if (getCmdParameter(args[i], "--threads=", value)) {
                threads = atol(value);
            } else if (getCmdParameter(args[i], "--show_ilp_solver_output", value)) {
                solverQuiet = false;
            } else if (getCmdParameter(args[i], "--solver=", value)) {
#ifndef USE_SCALP
                throw HatScheT::Exception("ScaLP not active! Solver cant be chosen: " + std::string(value));
#endif
                ilpSolver = std::string(value);
            } else if (getCmdParameter(args[i], "--resource=", value)) {
                resourceModelFile = std::string(value);
            } else if (getCmdParameter(args[i], "--graph=", value)) {
                graphMLFile = std::string(value);
            } else if (getCmdParameter(args[i], "--scc=", value)) {
                if (atol(value) == 1) printSCC = true;
            } else if (getCmdParameter(args[i], "--writeschedule=", value)) {
                scheduleFile = std::string(value);
            } else if (getCmdParameter(args[i], "--target=", value)) {
                targetFile = std::string(value);
            } else if (getCmdParameter(args[i], "--dot=", value)) {
                dotFile = value;
            } else if (getCmdParameter(args[i], "--writegraph=", value)) {
                writeGraphMLFile = value;
            } else if (getCmdParameter(args[i], "--writeresource=", value)) {
                writeResourceModelFile = value;
            } else if (getCmdParameter(args[i], "--samples=", value)) {
                samples = atol(value);
            } else if (getCmdParameter(args[i], "--modulo=", value)) {
                modulo = atol(value);
            } else if (getCmdParameter(args[i], "--uniform=", value)) {
                if (atol(value) == 1) uniform = true;
                else uniform = false;
            } else if (getCmdParameter(args[i], "--binding=", value)) {
                if (atol(value) == 1) optBinding = true;
            } else if (getCmdParameter(args[i], "--maxlatency=", value)) {
                maxLatency = atol(value);
            } else if (getCmdParameter(args[i], "--II=", value)) {
                targetII = std::stoi(std::string(value));
            } else if (getCmdParameter(args[i], "--html=", value)) {
                htmlFile = value;
            } else if (getCmdParameter(args[i], "--quiet=", value)) {
                string v = string(value);
                if (v == "0" or v == "false" or v == "False" or v == "FALSE" or v == "zero" or v == "Zero" or
                    v == "ZERO") {
                    quiet = false;
                }
            } else if (getCmdParameter(args[i], "--writeLPFile=", value)) {
                string v = string(value);
                if (v == "1" or v == "true" or v == "True" or v == "TRUE" or v == "one" or v == "One" or v == "ONE") {
                    writeLPFile = true;
                }
            } else if (getCmdParameter(args[i], "--scheduler=", value)) {
                std::string valueStr = std::string(value);
                schedulerSelectionStr.resize(valueStr.size());
                std::transform(valueStr.begin(), valueStr.end(), schedulerSelectionStr.begin(), ::tolower);

                if (schedulerSelectionStr == "asap") {
                    schedulerSelection = ASAP;
                } else if (schedulerSelectionStr == "alap") {
                    schedulerSelection = ALAP;
                } else if (schedulerSelectionStr == "nonmodilp") {
                    schedulerSelection = NONMODILP;
                } else if (schedulerSelectionStr == "ilpslsweeper") {
                    schedulerSelection = ILPSLSWEEPER;
                } else if (schedulerSelectionStr == "smt") {
                    schedulerSelection = SMT;
                } else if (schedulerSelectionStr == "smtcdl") {
                    schedulerSelection = SMTCDL;
                } else if (schedulerSelectionStr == "smash") {
                    schedulerSelection = SMASH;
                } else if (schedulerSelectionStr == "smashminlatnonmod") {
                    schedulerSelection = SMASHMINLATNONMOD;
                } else if (schedulerSelectionStr == "sat") {
                    schedulerSelection = SAT;
                } else if (schedulerSelectionStr == "satlat") {
                    schedulerSelection = SATLAT;
                } else if (schedulerSelectionStr == "satbin") {
                    schedulerSelection = SATBIN;
                } else if (schedulerSelectionStr == "satcdl") {
                    schedulerSelection = SATCDL;
                } else if (schedulerSelectionStr == "sds") {
                    schedulerSelection = SDS;
                } else if (schedulerSelectionStr == "satminreg") {
                    schedulerSelection = SATMINREG;
                } else if (schedulerSelectionStr == "satscc") {
                    schedulerSelection = SATSCC;
                } else if (schedulerSelectionStr == "satcombined") {
                    schedulerSelection = SATCOMBINED;
                } else if (schedulerSelectionStr == "satcombinedlat") {
                    schedulerSelection = SATCOMBINEDLAT;
                } else if (schedulerSelectionStr == "satcombinedbin") {
                    schedulerSelection = SATCOMBINEDBIN;
                } else if (schedulerSelectionStr == "satcombinedbinoverlap") {
                    schedulerSelection = SATCOMBINEDBINOVERLAP;
                } else if (schedulerSelectionStr == "satcombinedratii") {
                    schedulerSelection = SATCOMBINEDRATII;
                } else if (schedulerSelectionStr == "satratii") {
                    schedulerSelection = SATRATII;
                } else if (schedulerSelectionStr == "ul") {
                    schedulerSelection = UL;
                } else if (schedulerSelectionStr == "moovac") {
                    schedulerSelection = MOOVAC;
                } else if (schedulerSelectionStr == "moovacminreg") {
                    schedulerSelection = MOOVACMINREG;
                } else if (schedulerSelectionStr == "rams") {
                    schedulerSelection = RAMS;
                } else if (schedulerSelectionStr == "ed97") {
                    schedulerSelection = ED97;
                } else if (schedulerSelectionStr == "nis") {
                    schedulerSelection = NIS;
                } else if (schedulerSelectionStr == "sh11") {
                    schedulerSelection = SH11;
                } else if (schedulerSelectionStr == "sh11ra") {
                    schedulerSelection = SH11RA;
                } else if (schedulerSelectionStr == "modulosdc") {
                    schedulerSelection = MODULOSDC;
                } else if (schedulerSelectionStr == "rationaliimodulosdc") {
                    schedulerSelection = RATIONALIIMODULOSDC;
                } else if (schedulerSelectionStr == "rationalii") {
                    schedulerSelection = RATIONALII;
                } else if (schedulerSelectionStr == "unrollrationalii") {
                    schedulerSelection = UNROLLRATIONALII;
                } else if (schedulerSelectionStr == "uniformrationalii") {
                    schedulerSelection = UNIFORMRATIONALII;
                } else if (schedulerSelectionStr == "nonuniformrationalii") {
                    schedulerSelection = NONUNIFORMRATIONALII;
                } else if (schedulerSelectionStr == "rationaliifimmel") {
                    schedulerSelection = RATIONALIIFIMMEL;
                } else if (schedulerSelectionStr == "rationaliimoduloq") {
                    schedulerSelection = RATIONALIIMODULOQ;
                } else if (schedulerSelectionStr == "rationaliisccq") {
                    schedulerSelection = RATIONALIISCCQ;
                } else if (schedulerSelectionStr == "subgrreduction") {
                    schedulerSelection = SUGRREDUCTION;
                } else if (schedulerSelectionStr == "scc") {
                    schedulerSelection = SCC;
                } else {
                    throw HatScheT::Exception("Scheduler " + valueStr + " unknown!");
                }
            } else if (getCmdParameter(args[i], "--sccScheduler=", value)) {
                std::string valueStr = std::string(value);
                schedulerSelectionStr.resize(valueStr.size());
                std::transform(valueStr.begin(), valueStr.end(), schedulerSelectionStr.begin(), ::tolower);
                if (schedulerSelectionStr == "moovac") {
                    sccSchedulerSelection = MOOVAC;
                } else if (schedulerSelectionStr == "ed97") {
                    sccSchedulerSelection = ED97;
                } else if (schedulerSelectionStr == "sh11") {
                    sccSchedulerSelection = SH11;
                } else if (schedulerSelectionStr == "smtcdl") {
                    sccSchedulerSelection = SMTCDL;
                } else if (schedulerSelectionStr == "modulosdc") {
                    sccSchedulerSelection = MODULOSDC;
                } else if (schedulerSelectionStr == "sds") {
                    sccSchedulerSelection = SDS;
                } else if (schedulerSelectionStr == "smash"){
                    sccSchedulerSelection = SMASH;
                } else {
                    sccSchedulerSelection = SMT;
                }
            } else if (getCmdParameter(args[i], "--finalScheduler=", value)) {
                std::string valueStr = std::string(value);
                schedulerSelectionStr.resize(valueStr.size());
                std::transform(valueStr.begin(), valueStr.end(), schedulerSelectionStr.begin(), ::tolower);
                if (schedulerSelectionStr == "moovac") {
                    finalSchedulerSelection = MOOVAC;
                } else if (schedulerSelectionStr == "ed97") {
                    finalSchedulerSelection = ED97;
                } else if (schedulerSelectionStr == "sh11") {
                    finalSchedulerSelection = SH11;
                } else if (schedulerSelectionStr == "smtcdl") {
                    finalSchedulerSelection = SMTCDL;
                } else if (schedulerSelectionStr == "modulosdc") {
                    finalSchedulerSelection = MODULOSDC;
                } else if (schedulerSelectionStr == "sds") {
                    finalSchedulerSelection = SDS;
                } else if (schedulerSelectionStr == "smash"){
                    finalSchedulerSelection = SMASH;
                } else if (schedulerSelectionStr == "none") {
                    finalSchedulerSelection = NONE;
                } else {
                    finalSchedulerSelection = SMT;
                }
            }
                //HatScheT Auto Test Function
            else if (getCmdParameter(args[i], "--test=", value)) {
                string str = std::string(value);

#ifdef USE_SCALP
                if (str == "READ"                   && !HatScheT::Tests::readTest()) exit(-1);
                if (str == "MOOVAC"                 && !HatScheT::Tests::moovacTest()) exit(-1);
                if (str == "RWRS"                   && !HatScheT::Tests::readWriteReadScheduleTest()) exit(-1);
                if (str == "INTEGERIINONRECT"       && !HatScheT::Tests::integerIINonRectTest()) exit(-1);
                if (str == "INTEGERIIPB"            && !HatScheT::Tests::integerIIPBTest()) exit(-1);
                if (str == "MODULOSDCFIEGE"         && !HatScheT::Tests::moduloSDCTestFiege()) exit(-1);
                if (str == "RATIONALIIMODULOSDC"    && !HatScheT::Tests::rationalIIModuloSDCTest()) exit(-1);
                if (str == "COMBINEDRATIONALII"     && !HatScheT::Tests::rationalIICombinedSchedulerTest()) exit(-1);
                if (str == "ILPBASEDINTIIBINDINGCONG"
                                                    && !HatScheT::Tests::ilpBasedIntIIBindingTestCong()) exit(-1);
                if (str == "ILPBASEDINTIIBINDING"   && !HatScheT::Tests::ilpBasedIntIIBindingTest()) exit(-1);
                if (str == "OPTIMALINTIIGENERALIZEDBINDING"
                                                    && !HatScheT::Tests::optimalIntegerIIGeneralizedBindingTest()) exit(-1);
                if (str == "API"                    && !HatScheT::Tests::apiTest()) exit(-1);
                if (str == "ASAPHC"                 && !HatScheT::Tests::asapHCTest()) exit(-1);
                if (str == "ALAPHC"                 && !HatScheT::Tests::alapHCTest()) exit(-1);
                if (str == "ULScheduler"            && !HatScheT::Tests::ulSchedulerTest()) exit(-1);
                if (str == "RATMINII"               && !HatScheT::Tests::rationalMinIITest()) exit(-1);
                if (str == "CRITPATH"               && !HatScheT::Tests::cpTest()) exit(-1);
                if (str == "KOSARAJU"               && !HatScheT::Tests::KosarajuTest()) exit(-1);
                if (str == "DAIZHANG"               && !HatScheT::Tests::DaiZhangTest()) exit(-1);
                if (str == "DAIZHANGTWO"            && !HatScheT::Tests::DaiZhangTestTwo()) exit(-1);
                if (str == "COMPAREMSALGORITHMS"    && !HatScheT::Tests::compareModuloSchedulerTest()) exit(-1);
                if (str == "RATIONALIISCHEDULER"    && !HatScheT::Tests::rationalIISchedulerTest()) exit(-1);
                if (str == "UNIFORMRATIONALIISCHEDULER"
                                                    && !HatScheT::Tests::uniformRationalIISchedulerTest())exit(-1);
                if (str == "UNIFORMRATIONALIISCHEDULERNEW"
                                                    && !HatScheT::Tests::uniformRationalIISchedulerNewTest()) exit(-1);
                if (str == "NONUNIFORMRATIONALIISCHEDULER"
                                                    && !HatScheT::Tests::nonUniformRationalIISchedulerTest()) exit(-1);
                if (str == "UNROLLSCHEDULER"        && !HatScheT::Tests::ratIIUnrollSchedulerTest()) exit(-1);
                if (str == "RATIONALIISCHEDULERFIMMEL"
                                                    && !HatScheT::Tests::rationalIISchedulerFimmelTest()) exit(-1);
                if (str == "RATIONALIIMODULOQ"      && !HatScheT::Tests::rationalIIModuloQTest()) exit(-1);
                if (str == "RATIONALIISCCQ"         && !HatScheT::Tests::rationalIISCCQTest()) exit(-1);
                if (str == "CADICAL"                && !HatScheT::Tests::cadicalTest()) exit(-1);
                if (str == "SDSSCHEDULER"           && !HatScheT::Tests::sdsSchedulerTest()) exit(-1);
                if (str == "ratIIVerifierWrongMRTDetected"
                                                    && !HatScheT::Tests::ratIIVerifierWrongMRTDetected()) exit(-1);
                if (str == "ratIIVerifierWrongCausalityDetected"
                                                    && !HatScheT::Tests::ratIIVerifierWrongCausalityDetected()) exit(-1);
                if (str == "RATIIOPTIMALITERATION"  && !HatScheT::Tests::ratIIOptimalIterationTest()) exit(-1);
                if (str == "FIRSAMRATIIIMPL"        && !HatScheT::Tests::firSAMRatIIImplementationsTest()) exit(-1);
                if (str == "FIRSHIRATIIIMPL"        && !HatScheT::Tests::firSHIRatIIImplementationsTest()) exit(-1);
                if (str == "TCADEXAMPLE"            && !HatScheT::Tests::tcadExampleTest()) exit(-1);
                if (str == "MAFIEGE"                && !HatScheT::Tests::maFiegeTest()) exit(-1);
                if (str == "SCCQFAIL"               && !HatScheT::Tests::sccqFailTest()) exit(-1);
                if (str == "IISMALLERONE"           && !HatScheT::Tests::iiSmallerOneTest()) exit(-1);
                if (str == "MININTIIFAIL"           && !HatScheT::Tests::minIntIIFailTest()) exit(-1);
                if (str == "FIBONACCI"              && !HatScheT::Tests::fibonacciTest()) exit(-1);
                if (str == "SDCSOLVE"               && !HatScheT::Tests::sdcSolverTest()) exit(-1);
                if (str == "TREEBIND"               && !HatScheT::Tests::treeBindTest()) exit(-1);
                if (str == "TREEBINDCOMMUTATIVE"    && !HatScheT::Tests::treeBindCommutativeTest()) exit(-1);
                if (str == "FCCMPAPER"              && !HatScheT::Tests::fccmPaperTest()) exit(-1);
                if (str == "MULTIMINREGSCHEDULER"   && !HatScheT::Tests::multiMinRegSchedulerTest()) exit(-1);
                if (str == "SATSCHEDULER"           && !HatScheT::Tests::satSchedulerTest()) exit(-1);
                if (str == "SATBINDING"             && !HatScheT::Tests::satBinding()) exit(-1);
                if (str == "ITLAYER"                && !HatScheT::Tests::iterativeLayerTest()) exit(-1);
                if (str == "ILPLATENCY"             && !HatScheT::Tests::ilpMinMaxLatencyEstimationTest()) exit(-1);
                if (str == "LATENCY"                && !HatScheT::Tests::utilityLatencyEstimation()) exit(-1);
                if (str == "SDCSOLVERINCREMENTAL"   && !HatScheT::Tests::sdcSolverIncrementalTest()) exit(-1);
                if (str == "CLOCKGATING"            && !HatScheT::Tests::clockGatingSchedulerTest()) exit(-1);
                if (str == "SCALP"                  && !HatScheT::Tests::ScaLPTest()) exit(-1);
#else
                throw HatScheT::Exception("ScaLP not active! Test function disabled!");
#endif

                if (str == "SCCTEMPLATE" && !HatScheT::Tests::SCCTemplateTest()) exit(-1);

#ifdef USE_Z3
                if (str == "Z3" && !HatScheT::Tests::z3Test()) exit(-1);
                if (str == "SMT" && !HatScheT::Tests::smtSchedulerTest()) exit(-1);
                if (str == "SMASH" && !HatScheT::Tests::smashTest()) exit(-1);
#endif
                exit(0);
            } else if ((args[i][0] != '-') && getCmdParameter(args[i], "", value)) {
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
        std::cout << "maxRuns=" << maxRuns << std::endl;
        std::cout << "threads=" << threads << std::endl;
        std::cout << "quiet=" << quiet << std::endl;
        std::cout << "writeLPFile=" << writeLPFile << std::endl;
        if (targetII >= 0) std::cout << "II=" << targetII << std::endl;
        std::cout << "scheduler=";
        switch (schedulerSelection) {
            case ASAP:
                cout << "ASAP";
                break;
            case NONMODILP:
                cout << "NONMODILP";
                break;
            case ILPSLSWEEPER:
                cout << "ILPSLSWEEPER";
                break;
            case ALAP:
                cout << "ALAP";
                break;
            case UL:
                cout << "UL";
                break;
            case SAT:
                cout << "SAT";
                break;
            case SATLAT:
                cout << "SATLAT";
                break;
            case SATBIN:
                cout << "SATBIN";
                break;
            case SATCDL:
                cout << "SATCDL";
                break;
            case SDS:
                cout << "SDS";
                break;
            case SATMINREG:
                cout << "SATMINREG";
                break;
            case SATSCC:
                cout << "SATSCC";
                break;
            case SATCOMBINED:
                cout << "SATCOMBINED";
                break;
            case SATCOMBINEDLAT:
                cout << "SATCOMBINEDLAT";
                break;
            case SATCOMBINEDBIN:
                cout << "SATCOMBINEDBIN";
                break;
            case SATCOMBINEDBINOVERLAP:
                cout << "SATCOMBINEDBINOVERLAP";
                break;
            case SATCOMBINEDRATII:
                cout << "SATCOMBINEDRATII";
                break;
            case SATRATII:
                cout << "SATRATII";
                break;
            case SMT:
                cout << "SMT";
                break;
            case SMTCDL:
                cout << "SMTCDL";
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
            case NIS:
                cout << "NIS";
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
            case RATIONALII:
                cout << "RATIONALII";
                break;
            case UNROLLRATIONALII:
                cout << "UNROLLRATIONALII";
                break;
            case UNIFORMRATIONALII:
                cout << "UNIFORMRATIONALII";
                break;
            case NONUNIFORMRATIONALII:
                cout << "NONUNIFORMRATIONALII";
                break;
            case RATIONALIIFIMMEL:
                cout << "RATIONALIIFIMMEL";
                break;
            case RATIONALIIMODULOQ:
                cout << "RATIONALIIMODULOQ";
                break;
            case RATIONALIISCCQ:
                cout << "RATIONALIISCCQ";
                break;
            case SUGRREDUCTION:
                cout << "SUGRREDUCTION";
                break;
            case NONE:
                cout << "NONE";
                break;
            case RATIONALIIMODULOSDC:
                cout << "RATIONALIIMODULOSDC";
                break;
            case LATENCYTEST:
                cout << "LATENCYTEST";
                break;
            case SCC:
                cout << "SCC" << endl;
                break;
            case SMASH:
                cout << "SMASH" << endl;
                break;
            case SMASHMINLATNONMOD:
                cout << "SMASHMINLATNONMOD" << endl;
                break;
        }
        std::cout << std::endl;

        //read target if provided
        if (targetFile != "") {
            readerTarget.readHardwareTarget(targetFile.c_str());
            cout << target << endl;
        }

        //read resource model:
        readerRes.readResourceModel(resourceModelFile.c_str());

        //check target and resource model for consistency
        if (target.isEmpty() == false && rm.isEmpty() == false) {
            if (HatScheT::Utility::resourceModelAndTargetValid(rm, target) == false) {
                throw HatScheT::Exception(
                    "Hardware Target and Resource Model are inconsisent! Provide a valid resource model using --resource= and hardware target using --target==");
            }
        }

        //read graph:
        if (rm.isEmpty() == false) {
            HatScheT::GraphMLGraphReader graphReader(&rm, &g);
            graphReader.readGraph(graphMLFile.c_str());
            std::string graphName = graphMLFile.substr(0, graphMLFile.length() - 8);
            g.setName(graphName);
            cout << "graphName=" << graphName << endl;
            cout << g << endl;
            cout << rm << endl;
        } else {
            throw HatScheT::Exception(
                "Empty Resource Model Provided for graph parsing! Provide a valid resource model using --resource=");
        }

        // read schedule file with upper bounds for latency/registers/II if necessary
        if (!inputScheduleFile.empty()) {
            HatScheT::ScheduleAndBindingReader sbr(&g, &rm);
            sbr.read(inputScheduleFile);
            if (!sbr.isRatII()) {
                targetII = (int) sbr.getII();
            }
            maxLatency = sbr.getScheduleLength();
            maxRegs = sbr.getMinNumRegs();
        }

        if (dotFile != "") {
            cout << "Writing to dotfile " << dotFile << endl;
            HatScheT::DotWriter dw(dotFile, &g, &rm);
            dw.setDisplayNames(true);
            dw.write();
        }

        if (printSCC) {
            cout << "Printing strongly connected components of the graph: " << endl;
            HatScheT::KosarajuSCC scc(g);
            scc.getSCCs();
        }

        if (writeResourceModelFile != "") {
            if (rm.isEmpty() == true)
                throw HatScheT::Exception(
                    "Empty Resource Model Provided for resource writing! Provide a valid resource model using --resource=");
            cout << "Writing resource to file " << writeResourceModelFile << endl;
            HatScheT::XMLResourceWriter rw(writeResourceModelFile, &rm);
            rw.write();
        }

        if (writeGraphMLFile != "") {
            if (g.isEmpty() == true)
                throw HatScheT::Exception("No graph provided for graphML writing! Provide a graph using --graph=");
            if (rm.isEmpty() == true)
                throw HatScheT::Exception(
                    "Empty Resource Model Provided for graph writing! Provide a valid resource model using --resource=");
            cout << "Writing graph to file " << writeGraphMLFile << endl;
            HatScheT::GraphMLGraphWriter gw(writeGraphMLFile, &g, &rm);
            gw.write();
        }

        HatScheT::SchedulerBase *scheduler;

        bool isModuloScheduler = false;
        bool isRationalIIScheduler = false; // this replaces isModuloScheduler for ratII schedulers
        std::list<std::string> solverWishList;
        if (ilpSolver.empty()) {
            solverWishList = {"Gurobi", "CPLEX", "SCIP", "LPSolve"};
        } else {
            solverWishList = {ilpSolver};
        }

        if (rm.isEmpty() == false && g.isEmpty() == false) {
            switch (schedulerSelection) {
                case ASAP:
                    scheduler = new HatScheT::ASAPScheduler(g, rm);
                    break;
                case ALAP:
                    scheduler = new HatScheT::ALAPScheduler(g, rm);
                    break;
                case UL:
                    scheduler = new HatScheT::ULScheduler(g, rm);
                    break;
                case NIS: {
                    isModuloScheduler = true;
                    auto *nis = new HatScheT::NonIterativeModuloScheduler(g, rm);
                    //if (timeout > 0) ed97->setSolverTimeout(timeout);
                    //if (maxLatency > 0) ed97->setMaxLatencyConstraint(maxLatency);
                    //ed97->setThreads(threads);
                    //ed97->setSolverQuiet(solverQuiet);
                    scheduler = nis;
                    break;
                }
#ifdef USE_CADICAL
                case SAT:
                    scheduler = new HatScheT::SATScheduler(g, rm);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SATScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0) ((HatScheT::SATScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATLAT:
                    scheduler = new HatScheT::SATSchedulerLatOpt(g, rm);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SATSchedulerLatOpt *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATSchedulerLatOpt *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATBIN:
                    scheduler = new HatScheT::SATSchedulerBinEnc(g, rm);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SATSchedulerBinEnc *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATSchedulerBinEnc *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATCDL:
                    scheduler = new HatScheT::SATCDLScheduler(g, rm);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SATCDLScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0) ((HatScheT::SATCDLScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SDS:
                    scheduler = new HatScheT::SDSScheduler(g, rm);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SDSScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0) ((HatScheT::SDSScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATRATII:
                    scheduler = new HatScheT::SATRatIIScheduler(g, rm);
                    isRationalIIScheduler = true;
                    if (timeout > 0) ((HatScheT::SATRatIIScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATRatIIScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATCOMBINED:
                    scheduler = new HatScheT::SATCombinedScheduler(g, rm);
                    isModuloScheduler = true;
                    ((HatScheT::SATCombinedScheduler *) scheduler)->setBackendSchedulerType(HatScheT::BACKEND_SAT);
                    if (timeout > 0) ((HatScheT::SATCombinedScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATCombinedScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATCOMBINEDLAT:
                    scheduler = new HatScheT::SATCombinedScheduler(g, rm);
                    isModuloScheduler = true;
                    ((HatScheT::SATCombinedScheduler *) scheduler)->setBackendSchedulerType(HatScheT::BACKEND_SATLAT);
                    if (timeout > 0) ((HatScheT::SATCombinedScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATCombinedScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATCOMBINEDBIN:
                    scheduler = new HatScheT::SATCombinedScheduler(g, rm);
                    isModuloScheduler = true;
                    ((HatScheT::SATCombinedScheduler *) scheduler)->setBackendSchedulerType(HatScheT::BACKEND_SATBIN);
                    if (timeout > 0) ((HatScheT::SATCombinedScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATCombinedScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATCOMBINEDBINOVERLAP:
                    scheduler = new HatScheT::SATCombinedScheduler(g, rm);
                    isModuloScheduler = true;
                    ((HatScheT::SATCombinedScheduler *) scheduler)->setBackendSchedulerType(
                        HatScheT::BACKEND_SATBINOVERLAP);
                    if (timeout > 0) ((HatScheT::SATCombinedScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATCombinedScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATCOMBINEDRATII:
                    scheduler = new HatScheT::SATCombinedRatIIScheduler(g, rm);
                    isRationalIIScheduler = true;
                    if (timeout > 0) ((HatScheT::SATCombinedRatIIScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATCombinedRatIIScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATSCC:
                    scheduler = new HatScheT::SATSCCScheduler(g, rm, targetII);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SATSCCScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0) ((HatScheT::SATSCCScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    break;
                case SATMINREG:
                    scheduler = new HatScheT::SATMinRegScheduler(g, rm);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SATMinRegScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::SATMinRegScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    if (maxRegs > 0) ((HatScheT::SATMinRegScheduler *) scheduler)->setRegMax(maxRegs);
                    if (targetII > 0) ((HatScheT::SATMinRegScheduler *) scheduler)->overrideII(targetII);
                    break;
#endif
#ifdef USE_SCALP
                case NONMODILP:
                    scheduler = new HatScheT::NonModIlpTestScheduler(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::NonModIlpTestScheduler *) scheduler)->setSolverTimeout(timeout);
                    isModuloScheduler = false;
                    ((HatScheT::NonModIlpTestScheduler *) scheduler)->setQuiet(false);
                    break;
                case ILPSLSWEEPER:
                    if (scheduleFile.empty()) {
                        std::cerr << "specify result file for ILP schedule length sweeper" << std::endl;
                        exit(0);
                    }
                    scheduler = new HatScheT::ILPScheduleLengthSweeper(g, rm, solverWishList, scheduleFile);
                    if (timeout > 0) ((HatScheT::ILPScheduleLengthSweeper *) scheduler)->setSolverTimeout(timeout);
                    isModuloScheduler = true;
                    scheduleFile.clear(); // don't override schedule file again later
                    break;
                case MOOVAC:
                    isModuloScheduler = true;
                    scheduler = new HatScheT::MoovacScheduler(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::MoovacScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0) ((HatScheT::MoovacScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    ((HatScheT::MoovacScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::MoovacScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case MOOVACMINREG:
                    isModuloScheduler = true;
                    scheduler = new HatScheT::MoovacMinRegScheduler(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::MoovacMinRegScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::MoovacMinRegScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    if (targetII > 0) {
                        ((HatScheT::MoovacMinRegScheduler *) scheduler)->overrideII(targetII);
                        ((HatScheT::MoovacMinRegScheduler *) scheduler)->setMaxRuns(1);
                    }
                    ((HatScheT::MoovacMinRegScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::MoovacMinRegScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case RAMS:
                    isModuloScheduler = true;
                    scheduler = new HatScheT::MoovacResAwScheduler(g, rm, solverWishList, target);
                    if (timeout > 0) ((HatScheT::MoovacResAwScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::MoovacResAwScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    ((HatScheT::MoovacResAwScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::MoovacResAwScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case ED97: {
                    isModuloScheduler = true;
                    auto *ed97 = new HatScheT::EichenbergerDavidson97Scheduler(g, rm, solverWishList);
                    if (timeout > 0) ed97->setSolverTimeout(timeout);
                    if (maxLatency > 0) ed97->setMaxLatencyConstraint(maxLatency);
                    ed97->setThreads(threads);
                    ed97->setSolverQuiet(solverQuiet);
                    scheduler = ed97;
                    break;
                }
                case SH11: {
                    isModuloScheduler = true;
                    auto *sh11 = new HatScheT::SuchaHanzalek11Scheduler(g, rm, solverWishList);
                    if (timeout > 0) sh11->setSolverTimeout(timeout);
                    if (maxLatency > 0) sh11->setMaxLatencyConstraint(maxLatency);
                    sh11->setThreads(threads);
                    sh11->setSolverQuiet(solverQuiet);
                    scheduler = sh11;
                    break;
                }
                case SH11RA: {
                    isModuloScheduler = true;
                    auto *sh11ra = new HatScheT::SuchaHanzalek11ResAwScheduler(g, rm, target, solverWishList);
                    if (timeout > 0) sh11ra->setSolverTimeout(timeout);
                    if (maxLatency > 0) sh11ra->setMaxLatencyConstraint(maxLatency);
                    sh11ra->setThreads(threads);
                    sh11ra->setSolverQuiet(solverQuiet);
                    scheduler = sh11ra;
                    break;
                }
                case MODULOSDC:
                    isModuloScheduler = true;
                    scheduler = new HatScheT::ModuloSDCScheduler(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::ModuloSDCScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::ModuloSDCScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    ((HatScheT::ModuloSDCScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::ModuloSDCScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case RATIONALIIMODULOSDC:
                    isRationalIIScheduler = true;
                    scheduler = new HatScheT::RationalIIModuloSDCScheduler(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::RationalIIModuloSDCScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::RationalIIModuloSDCScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    ((HatScheT::RationalIIModuloSDCScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::RationalIIModuloSDCScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case RATIONALII:
                    isRationalIIScheduler = true;
                    scheduler = new HatScheT::RationalIIScheduler(g, rm, solverWishList);
                    ((HatScheT::RationalIIScheduler *) scheduler)->setUniformScheduleFlag(uniform);
                    if (timeout > 0) ((HatScheT::RationalIIScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::RationalIIScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    if (samples > 0) ((HatScheT::RationalIIScheduler *) scheduler)->setSamples(samples);
                    if (modulo > 0) ((HatScheT::RationalIIScheduler *) scheduler)->setModulo(modulo);
                    ((HatScheT::RationalIIScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::RationalIIScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case UNROLLRATIONALII:
                    isRationalIIScheduler = true;
                    scheduler = new HatScheT::UnrollRationalIIScheduler(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::UnrollRationalIIScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::UnrollRationalIIScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    if (samples > 0) ((HatScheT::UnrollRationalIIScheduler *) scheduler)->setSamples(samples);
                    if (modulo > 0) ((HatScheT::UnrollRationalIIScheduler *) scheduler)->setModulo(modulo);
                    ((HatScheT::UnrollRationalIIScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::UnrollRationalIIScheduler *) scheduler)->setSolverQuiet(solverQuiet);
#ifdef USE_CADICAL
                    ((HatScheT::UnrollRationalIIScheduler *) scheduler)->setIntIIScheduler(
                        HatScheT::SchedulerType::SATCOMBINED);
#endif
                    break;
                case UNIFORMRATIONALII:
                    isRationalIIScheduler = true;
                    scheduler = new HatScheT::UniformRationalIISchedulerNew(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::UniformRationalIISchedulerNew *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::UniformRationalIISchedulerNew *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    if (samples > 0) ((HatScheT::UniformRationalIISchedulerNew *) scheduler)->setSamples(samples);
                    if (modulo > 0) ((HatScheT::UniformRationalIISchedulerNew *) scheduler)->setModulo(modulo);
                    ((HatScheT::UniformRationalIISchedulerNew *) scheduler)->setThreads(threads);
                    ((HatScheT::UniformRationalIISchedulerNew *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case NONUNIFORMRATIONALII:
                    isRationalIIScheduler = true;
                    scheduler = new HatScheT::NonUniformRationalIIScheduler(g, rm, solverWishList);
                    scheduler->setQuiet(false);
                    if (timeout > 0) ((HatScheT::NonUniformRationalIIScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::NonUniformRationalIIScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    if (samples > 0) ((HatScheT::NonUniformRationalIIScheduler *) scheduler)->setSamples(samples);
                    if (modulo > 0) ((HatScheT::NonUniformRationalIIScheduler *) scheduler)->setModulo(modulo);
                    ((HatScheT::NonUniformRationalIIScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::NonUniformRationalIIScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case RATIONALIIFIMMEL:
                    scheduler = new HatScheT::RationalIISchedulerFimmel(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::RationalIISchedulerFimmel *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0)
                        ((HatScheT::RationalIIScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    ((HatScheT::RationalIISchedulerFimmel *) scheduler)->setThreads(threads);
                    ((HatScheT::RationalIISchedulerFimmel *) scheduler)->setSolverQuiet(solverQuiet);
                    break;
                case RATIONALIIMODULOQ:
                    isRationalIIScheduler = true;
                    scheduler = new HatScheT::ModuloQScheduler(g, rm, solverWishList);
                    if (timeout > 0) ((HatScheT::ModuloQScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0) ((HatScheT::ModuloQScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    ((HatScheT::ModuloQScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::ModuloQScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    if (samples > 0) ((HatScheT::ModuloQScheduler *) scheduler)->setSamples(samples);
                    if (modulo > 0) ((HatScheT::ModuloQScheduler *) scheduler)->setSamples(modulo);
                    break;
                case RATIONALIISCCQ:
                    isRationalIIScheduler = true;
                    scheduler = new HatScheT::SCCQScheduler(g, rm, solverWishList);
                    ((HatScheT::SCCQScheduler *) scheduler)->setQuiet(false);
                    if (timeout > 0) ((HatScheT::SCCQScheduler *) scheduler)->setSolverTimeout(timeout);
                    if (maxLatency > 0) ((HatScheT::SCCQScheduler *) scheduler)->setMaxLatencyConstraint(maxLatency);
                    ((HatScheT::SCCQScheduler *) scheduler)->setThreads(threads);
                    ((HatScheT::SCCQScheduler *) scheduler)->setSolverQuiet(solverQuiet);
                    if (samples > 0) ((HatScheT::SCCQScheduler *) scheduler)->setSamples(samples);
                    if (modulo > 0) ((HatScheT::SCCQScheduler *) scheduler)->setSamples(modulo);
                    break;
                case SUGRREDUCTION: {
                    isModuloScheduler = true;
                    auto *subgrred = new HatScheT::DaiZhang19Scheduler(g, rm, solverWishList);
                    if (timeout > 0) subgrred->setSolverTimeout(timeout);
                    if (maxLatency > 0) subgrred->setMaxLatencyConstraint(maxLatency);
                    subgrred->setThreads(threads);
                    subgrred->setSolverQuiet(solverQuiet);
                    scheduler = subgrred;
                    break;
                }
#ifdef USE_Z3
                case SCC: {
                    HatScheT::SCCSchedulerTemplate::scheduler sccSched;
                    switch (sccSchedulerSelection) {
                        case MOOVAC :
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::MOOVAC;
                            break;
                        case SMTCDL :
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::SMTCDL;
                            break;
                        case ED97 :
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::ED97;
                            break;
                        case SH11 :
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::SH11;
                            break;
                        case MODULOSDC :
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::MODSDC;
                            break;
                        case SAT :
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::SAT;
                            break;
                        case SDS :
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::SDS;
                            break;
                        case SMASH:
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::SMASH;
                            break;
                        default:
                            sccSched = HatScheT::SCCSchedulerTemplate::scheduler::SMT;
                            break;
                    }
                    HatScheT::SCCSchedulerTemplate::scheduler finSched;
                    switch (finalSchedulerSelection) {
                        case MOOVAC :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::MOOVAC;
                            break;
                        case SMTCDL :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::SMTCDL;
                            break;
                        case ED97 :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::ED97;
                            break;
                        case SH11 :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::SH11;
                            break;
                        case MODULOSDC :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::MODSDC;
                            break;
                        case SAT :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::SAT;
                            break;
                        case SMT :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::SMT;
                            break;
                        case SDS :
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::SDS;
                            break;
                        case SMASH:
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::SMASH;
                            break;
                        default:
                            finSched = HatScheT::SCCSchedulerTemplate::scheduler::NONE;
                            break;
                    }
                    scheduler = new HatScheT::SCCSchedulerTemplate(g, rm, sccSched, finSched);
                    ((HatScheT::SCCSchedulerTemplate *) scheduler)->setExpandMode(HatScheT::SCCSchedulerTemplate::sccExpandMode::fast);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SMTUnaryScheduler *) scheduler)->setSolverTimeout(timeout);
                    break;
                }
                case SMASH:
                    scheduler = new HatScheT::SMASHScheduler(g, rm);
                    if (timeout > 0) ((HatScheT::SMASHScheduler *) scheduler)->setSolverTimeout(timeout);
                    isModuloScheduler = true;
                    cout << "SMASH is currently WIP" << endl; // ToDo: Finish it!
                    break;
                case SMT:
                    scheduler = new HatScheT::SMTUnaryScheduler(g, rm);
                    isModuloScheduler = true;
                    if (timeout > 0) ((HatScheT::SMTUnaryScheduler *) scheduler)->setSolverTimeout(timeout);
                    break;
                case SMTCDL:
                    scheduler = new HatScheT::SMTCDLScheduler(g, rm);
                    isModuloScheduler = true;
                    ((HatScheT::SMTCDLScheduler *) scheduler)->setLayerQuiet(false);
                    if (timeout > 0) ((HatScheT::SMTCDLScheduler *) scheduler)->setSolverTimeout(timeout);
                    break;
                case SMASHMINLATNONMOD:
                    scheduler = new HatScheT::SMASHMinLatNonMod(g, rm);
                    isModuloScheduler = false;
                    cout << "SMASHMINLATNONMOD is currently WIP" << endl; // ToDo: Finish it!
                    if (timeout > 0) ((HatScheT::SMTCDLScheduler *) scheduler)->setSolverTimeout(timeout);
                    break;

#else
                    case SMT:
                        throw HatScheT::Exception("scheduler " + schedulerSelectionStr +
                                                                            " not available without SCALP and z3 library. Please build with SCALP.");
                        break;
#endif
#else
                    case MOOVAC:
                    case MOOVACMINREG:
                    case ED97:
                    case MODULOSDC:
                    case RATIONALII:
                    case UNROLLRATIONALII:
                    case UNIFORMRATIONALII:
                    case NONUNIFORMRATIONALII:
                    case RATIONALIIFIMMEL:
                    case RATIONALIIMODULOQ:
                    case RATIONALIISCCQ:
                        throw HatScheT::Exception("scheduler " + schedulerSelectionStr + " not available without SCALP library. Please build with SCALP.");
                        break;
#endif //USE_SCALP
                case NONE:
                    throw HatScheT::Exception("No scheduler given, please select a scheduler using --scheduler=...");
                default:
                    throw HatScheT::Exception("Scheduler " + schedulerSelectionStr + " not available!");
            }

#ifdef USE_SCALP
            // set max runs
            if (maxRuns > 0) {
                auto iterativeSchedulerBase = dynamic_cast<HatScheT::IterativeSchedulerBase *>(scheduler);
                if (iterativeSchedulerBase != nullptr) {
                    iterativeSchedulerBase->setMaxRuns(maxRuns);
                }
            }

            // set target II
            if (targetII >= 0) {
                auto modSchedBase = dynamic_cast<HatScheT::ModuloSchedulerBase *> (scheduler);
                if (modSchedBase != nullptr) {
                    modSchedBase->overrideII(targetII);
                }
            }

            // set writeILPFile
            auto ilpSchedulerBase = dynamic_cast<HatScheT::ILPSchedulerBase *>(scheduler);
            if (ilpSchedulerBase != nullptr) {
                ilpSchedulerBase->setWriteLPFile(writeLPFile);
            }

            // bound schedule length if wanted
            if (boundSL) {
                auto modSchedLayer = dynamic_cast<HatScheT::IterativeModuloSchedulerLayer *>(scheduler);
                if (modSchedLayer != nullptr) {
                    modSchedLayer->setBoundSL(boundSL);
                }
            }
#endif

            // set quiet or loud
            scheduler->setQuiet(quiet);

            //chossing binding
            //experimental
            scheduler->setUseOptBinding(optBinding);

            cout << "HatScheT: Performing schedule" << endl;
            auto startTime = std::chrono::steady_clock::now();
            scheduler->schedule();
            auto endTime = std::chrono::steady_clock::now();
            auto schedulingTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count() / 1000.0;
            cout << "HatScheT: Finished scheduling in " << schedulingTime << " seconds" << endl;

            if (isModuloScheduler and scheduler->getScheduleFound()) {
                if (HatScheT::verifyModuloSchedule(g, rm, scheduler->getSchedule(), (int) scheduler->getII())) {
                    cout << "Modulo schedule verified successfully" << endl;
                    cout << "Found II " << scheduler->getII() << " with sampleLatency "
                         << scheduler->getScheduleLength() << endl;
                } else {
                    cout << ">>> Modulo schedule verification failed! <<<" << endl;
                    exit(-1);
                }
            }else if (!isModuloScheduler and scheduler->getScheduleFound()) {
                if (HatScheT::verifyResourceConstrainedSchedule(g, rm, scheduler->getSchedule(), (int) scheduler->getScheduleLength())) {
                    cout << "Modulo schedule verified successfully" << endl;
                    cout << "Found Latency: " << scheduler->getScheduleLength() << endl;
                } else {
                    cout << ">>> Schedule verification failed! <<<" << endl;
                    exit(-1);
                }
            }

#ifdef USE_SCALP
            auto *ratIILayer = dynamic_cast<HatScheT::RationalIISchedulerLayer *>(scheduler);
            if (ratIILayer != nullptr and isRationalIIScheduler) {
                if (ratIILayer->getScheduleValid()) {
                    cout << "Rational II Modulo schedule verified successfully" << endl;
                    cout << "Found II " << scheduler->getII() << " with sampleLatency "
                         << scheduler->getScheduleLength() << endl;
                } else {
                    cout << ">>> Rational II Modulo schedule verification failed! <<<" << endl;
                    exit(-1);
                }
            }
#endif //USE_SCALP

            if (scheduler->getScheduleFound()) {
                std::cout << "------------------------------------------------------------------------------------"
                          << endl;
                std::cout << "---------------------------------- Schedule: ---------------------------------------"
                          << endl;
                std::cout << "------------------------------------------------------------------------------------"
                          << endl;
                std::cout << "latency = " << scheduler->getScheduleLength() << endl;
                HatScheT::Utility::printSchedule(scheduler->getSchedule());
                if (!isRationalIIScheduler) {
                    std::map<const HatScheT::Vertex *, int> b = scheduler->getBindings();
                    HatScheT::Utility::printBinding(*&b, rm);

                    if (!htmlFile.empty()) {
                        scheduler->writeScheduleChart(htmlFile);
                    }
                }
            } else {
                cout << "No schedule found!" << endl;
            }

            // write schedule to csv file if requested
            //if(!scheduleFile.empty() and scheduler->getScheduleFound()) {
            if (!scheduleFile.empty()) {
                std::cout << "Start writing results to schedule file '" << scheduleFile << "'" << std::endl;
                std::map<HatScheT::Vertex *, int> intIISchedule;
                std::map<const HatScheT::Vertex *, int> intIIBindings;
                std::vector<std::map<HatScheT::Vertex *, int>> ratIISchedule;
                std::vector<std::map<const HatScheT::Vertex *, int>> ratIIBindings;
                int minNumRegs = -1;
                int minNumRegsChain = -1;
                if (scheduler->getScheduleFound()) {
#ifdef USE_SCALP
                    if (ratIILayer == nullptr) {
                        // integer II modulo scheduler
                        intIISchedule = scheduler->getSchedule();
                        intIIBindings = scheduler->getBindings();
                        minNumRegs = HatScheT::Utility::calcMinNumRegs(&g, &rm, intIISchedule,
                                                                       (int) scheduler->getII());
                        minNumRegsChain = HatScheT::Utility::calcMinNumChainRegs(&g, &rm, intIISchedule, intIIBindings,
                                                                                 (int) scheduler->getII());
                    } else {
                        // rational II modulo scheduler
                        ratIISchedule = ratIILayer->getStartTimeVector();
                        ratIIBindings = ratIILayer->getRationalIIBindings();
                        minNumRegs = HatScheT::Utility::calcMinNumRegs(&g, &rm, ratIISchedule,
                                                                       ratIILayer->getM_Found());
                        minNumRegsChain = HatScheT::Utility::calcMinNumChainRegs(&g, &rm, ratIISchedule, ratIIBindings,
                                                                                 ratIILayer->getM_Found());
                    }
#else
                    intIISchedule = scheduler->getSchedule();
                    intIIBindings = scheduler->getBindings();
                    minNumRegs = HatScheT::Utility::calcMinNumRegs(&g, &rm, intIISchedule, (int)scheduler->getII());
                    minNumRegsChain = HatScheT::Utility::calcMinNumChainRegs(&g, &rm, intIISchedule, intIIBindings, (int)scheduler->getII());
#endif //USE_SCALP
                }
                std::unique_ptr<HatScheT::ScheduleAndBindingWriter> sBWriter;
                std::pair<bool, bool> objectivesOptimal(false, false);

#ifdef USE_Z3
                if (schedulerSelection == SMASHMINLATNONMOD) {
                    objectivesOptimal.second = dynamic_cast<HatScheT::SMASHMinLatNonMod *>(scheduler)->isLatencyOptimal();
                }
                if (schedulerSelection == NONMODILP) {
                    objectivesOptimal.second = dynamic_cast<HatScheT::NonModIlpTestScheduler *>(scheduler)->isLatencyOptimal();
                }
#endif

#ifdef USE_SCALP
                auto modSchedBase = dynamic_cast<HatScheT::ModuloSchedulerBase *>(scheduler);
                if (modSchedBase != nullptr) {
                    objectivesOptimal = modSchedBase->getObjectivesOptimal();
                }
                if (ratIILayer == nullptr) {
                    sBWriter = std::unique_ptr<HatScheT::ScheduleAndBindingWriter>(
                        new HatScheT::ScheduleAndBindingWriter(scheduleFile, intIISchedule, intIIBindings,
                                                               (int) scheduler->getII()));
                } else {
                    sBWriter = std::unique_ptr<HatScheT::ScheduleAndBindingWriter>(
                        new HatScheT::ScheduleAndBindingWriter(scheduleFile, ratIISchedule, ratIIBindings,
                                                               ratIILayer->getS_Found(), ratIILayer->getM_Found()));
                }
#else
                sBWriter = std::unique_ptr<HatScheT::ScheduleAndBindingWriter>(new HatScheT::ScheduleAndBindingWriter(scheduleFile, intIISchedule, intIIBindings, (int)scheduler->getII()));
#endif //USE_SCALP
#ifdef USE_SCALP
                sBWriter->setMinII(HatScheT::Utility::calcMinII(HatScheT::Utility::calcResMII(&rm),
                                                                HatScheT::Utility::calcRecMII(&g, &rm)));
#endif
                sBWriter->setRMPath(resourceModelFile);
                sBWriter->setGraphPath(graphMLFile);
                sBWriter->setScheduleLength(scheduler->getScheduleLength());
                sBWriter->setMinNumRegs(minNumRegs);
                sBWriter->setMinNumRegsChain(minNumRegsChain);
                sBWriter->setSolvingTime(schedulingTime);
                sBWriter->setObjectivesOptimal(objectivesOptimal);
                sBWriter->write();
            }

            delete scheduler;
        }
    }
    catch (HatScheT::Exception &e) {
        std::cerr << "Error: " << e.msg << std::endl;
        exit(-1);
    }
#ifdef USE_SCALP
    catch (ScaLP::Exception &e) {
        std::cerr << "Scalp Error: " << e.msg << std::endl;
        exit(-1);
    }
#endif //USE_SCALP
    return 0;
#endif
}
