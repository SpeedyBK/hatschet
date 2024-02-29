# This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
# Author: Martin Kumm, Patrick Sittel, Julian Oppermann ({kumm, sittel}@uni-kassel.de, oppermann@esa.tu-darmstadt.de)
#
# Copyright (C) 2018
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 2.8)
project(hatschet)

####################
#Set options for used libs
####################
option(USE_XERCESC "Use XercesC for parsing" ON)
#option(USE_CADICAL "Use CaDiCaL SAT solving" ON)
option(OLD_CXX_ABI "Build with an old C++-abi (activate if you get undefined references with '__cxx11')" OFF)


# Deactivate C++11-abi?
if (OLD_CXX_ABI)
    add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
endif ()

####################
#Find ScaLP
####################
FIND_LIBRARY(SCALP_LIB
        NAMES libScaLP libScaLP.so libScaLP.dylib
        HINTS "${SCALP_PREFIX_PATH}/lib"
        DOC "Directory of the ScaLP library"
        )

FIND_PATH(SCALP_H
        ScaLP/Solver.h
        HINTS "${SCALP_PREFIX_PATH}/include"
        DOC "Path to main ScaLP header file Solver.h"
        )

IF (SCALP_H AND SCALP_LIB)
    add_definitions(-DUSE_SCALP)
    MESSAGE(STATUS "Found ScaLP: ${SCALP_H},${SCALP_LIB}")
ELSE (SCALP_H AND SCALP_LIB)
    MESSAGE(STATUS "Could not find ScaLP (${SCALP_H},${SCALP_LIB}). Please specify path by setting SCALP_PREFIX_PATH (-DSCALP_PREFIX_PATH=<path to scalp install dir>)")
ENDIF (SCALP_H AND SCALP_LIB)

####################
#Find XerxesC lib
####################
FIND_LIBRARY(XERCESC_LIB
        NAMES xerces-c
        HINTS "${XERCESC_LIBRARY_PATH}"
        DOC "Directory of the XercesC library")

IF (XERCESC_LIB)
    MESSAGE(STATUS "Found XercesC Lib: ${XERCESC_LIB}")
ELSE (XERCESC_LIB)
    MESSAGE(STATUS "Could not find XercesC Lib (${XERCESC_LIB}). Please specify path by setting XERCESC_LIBRARY_PATH (-DXERCESC_LIBRARY_PATH=<path to xerces lib dir>)")
ENDIF (XERCESC_LIB)

####################
#Find XerxesC headers
####################
FIND_PATH(XERCESC_H
        xercesc/parsers/SAXParser.hpp
        HINTS "${XERCESC_INCLUDE_PATH}"
        DOC "Path to main XercesC header files"
        )

IF (XERCESC_H)
    MESSAGE(STATUS "Found XercesC Headers: ${XERCESC_H}")
ELSE (XERCESC_H)
    MESSAGE(STATUS "Could not find XercesC Headers (${XERCESC_H}). Please specify path by setting XERCESC_INCLUDE_PATH (-DXERCESC_INCLUDE_PATH=<path to xerces include dir>)")
ENDIF (XERCESC_H)

####################
#set define for XercesC
####################
IF (USE_XERCESC)
    IF (XERCESC_H AND XERCESC_LIB)
        add_definitions(-DUSE_XERCESC)
        MESSAGE(STATUS "Found XercesC: GraphML parser will be available")
    ELSE (XERCESC_H AND XERCESC_LIB)
        MESSAGE(STATUS "Did not found XercesC: GraphML parser will not be available")
    ENDIF (XERCESC_H AND XERCESC_LIB)
ELSE (USE_XERCESC)
    MESSAGE(STATUS "XercesC was turned off manually (-DUSE_XERCESC=OFF): GraphML parser will not be available")
ENDIF (USE_XERCESC)

####################
#Find CaDiCaL
####################
FIND_LIBRARY(CADICAL_LIB
        NAMES libcadical.a
        HINTS "${CADICAL_LIBRARY_PATH}"
        DOC "Directory of the CaDiCaL library")

FIND_PATH(CADICAL_H
        cadical.hpp
        HINTS "${CADICAL_INCLUDE_PATH}"
        DOC "Path to main CaDiCal header file"
        )

IF (CADICAL_H AND CADICAL_LIB)
    add_definitions(-DUSE_CADICAL)
    MESSAGE(STATUS "Found CaDiCaL Lib: ${CADICAL_LIB} and Headers: ${CADICAL_H}")
ELSE (CADICAL_H AND CADICAL_LIB)
    MESSAGE(STATUS "Could not find CaDiCaL Lib (${CADICAL_LIB}) or CaDiCaL Header (${CADICAL_H}). Please specify paths by appending CaDiCaL install dir to CMAKE_PREFIX_PATH or by setting CADICAL_LIBRARY_PATH (-DCADICAL_LIBRARY_PATH=<path to CaDiCaL lib dir>) and CADICAL_INCLUDE_PATH (-DCADICAL_INCLUDE_PATH=<path to CaDiCaL include dir>)")
ENDIF (CADICAL_H AND CADICAL_LIB)

####################
#Find Kissat (C++ API version)
####################
FIND_LIBRARY(KISSAT_LIB
        NAMES libkissatpp.a
        HINTS "${KISSAT_LIBRARY_PATH}"
        DOC "Directory of the kissat library")

FIND_PATH(KISSAT_H
        kissatpp.hpp
        HINTS "${KISSAT_INCLUDE_PATH}"
        DOC "Path to main kissat header file"
        )

IF (KISSAT_H AND KISSAT_LIB)
    add_definitions(-DUSE_KISSAT)
    MESSAGE(STATUS "Found Kissat Lib: ${KISSAT_LIB} and Headers: ${KISSAT_H}")
ELSE (KISSAT_H AND KISSAT_LIB)
    MESSAGE(STATUS "Could not find Kissat Lib (${KISSAT_LIB}) or Kissat Header (${KISSAT_H}). Please specify paths by appending Kissat install dir to CMAKE_PREFIX_PATH or by setting KISSAT_LIBRARY_PATH (-DKISSAT_LIBRARY_PATH=<path to KISSAT lib dir>) and KISSAT_INCLUDE_PATH (-DKISSAT_INCLUDE_PATH=<path to Kissat include dir>)")
ENDIF (KISSAT_H AND KISSAT_LIB)

####################
#Find SAT SCM
####################
FIND_LIBRARY(SATSCM_LIB
        NAMES libsatscm.a
        HINTS "${SATSCM_LIBRARY_PATH}"
        DOC "Directory of the SAT SCM library")

FIND_PATH(SATSCM_H
        scm_cadical.h
        HINTS "${SATSCM_INCLUDE_PATH}"
        DOC "Path to CaDiCaL SAT SCM header file"
        )

IF (SATSCM_H AND SATSCM_LIB)
    add_definitions(-DUSE_SATSCM)
    MESSAGE(STATUS "Found SAT SCM Lib: ${SATSCM_LIB} and Headers: ${SATSCM_H}")
ELSE (SATSCM_H AND SATSCM_LIB)
    MESSAGE(STATUS "Could not find SAT SCM Lib (${SATSCM_LIB}) or SAT SCM Header (${SATSCM_H}). Please specify paths by either appending CMAKE_PREFIX_PATH or by setting SATSCM_LIBRARY_PATH (-DSATSCM_LIBRARY_PATH=<path to SAT SCM lib dir>) and SATSCM_INCLUDE_PATH (-DSATSCM_INCLUDE_PATH=<path to SAT SCM include dir>)")
ENDIF (SATSCM_H AND SATSCM_LIB)

####################
#Find Z3-smtbased
####################
FIND_LIBRARY(Z3_LIB
        NAMES libz3.so
        DOC "Directory of the Z3 library")

FIND_PATH(Z3_H
        z3++.h
        DOC "Path to main z3 header file"
        )

IF (Z3_H AND Z3_LIB)
    add_definitions(-DUSE_Z3)
    MESSAGE(STATUS "Found Z3 Lib: ${Z3_LIB} and Headers: ${Z3_H}")
ELSE (Z3_H AND Z3_LIB)
    MESSAGE(STATUS "Could not find Z3 Lib (${Z3_LIB}) or Z3 Header (${Z3_H}). Please specify paths by setting CMAKE_PREFIX_PATH (-DCMAKE_PREFIX_PATH=<path to Z3 install dir>)")
ENDIF (Z3_H AND Z3_LIB)

####################
#SRC and Headerfiles
####################
set(HatScheT_SOURCES
        src/HatScheT/Vertex.cpp
        src/HatScheT/Edge.cpp
        src/HatScheT/Graph.cpp
        src/HatScheT/TargetModel.cpp
        src/HatScheT/utility/Exception.cpp
        src/HatScheT/base/SchedulerBase.cpp
        src/HatScheT/layers/RationalIISchedulerLayer.cpp
        src/HatScheT/ResourceModel.cpp
        src/HatScheT/utility/reader/Reader.cpp
        src/HatScheT/utility/reader/GraphMLGraphReader.cpp
        src/HatScheT/utility/reader/XMLResourceReader.cpp
        src/HatScheT/utility/reader/ScheduleAndBindingReader.cpp
        src/HatScheT/utility/subgraphs/KosarajuSCC.cpp
        src/HatScheT/utility/subgraphs/Occurrence.cpp
        src/HatScheT/utility/subgraphs/OccurrenceSet.cpp
        src/HatScheT/utility/subgraphs/OccurrenceSetCombination.cpp
        src/HatScheT/utility/writer/Writer.cpp
        src/HatScheT/utility/writer/GraphMLGraphWriter.cpp
        src/HatScheT/utility/writer/DotWriter.cpp
        src/HatScheT/utility/writer/ScheduleAndBindingWriter.cpp
        src/HatScheT/utility/writer/DGLWriter.cpp
        src/HatScheT/utility/Utility.cpp
        src/HatScheT/utility/Binding.cpp
        src/HatScheT/utility/BindingBase.cpp
        src/HatScheT/utility/TreeBind.cpp
        src/HatScheT/utility/SDCSolver.cpp
        src/HatScheT/utility/SDCSolverBellmanFord.cpp
        src/HatScheT/utility/SDCSolverIncremental.cpp
        src/HatScheT/scheduler/ASAPScheduler.cpp
        src/HatScheT/scheduler/ALAPScheduler.cpp
        src/HatScheT/scheduler/ULScheduler.cpp
        src/HatScheT/utility/Verifier.cpp
        src/HatScheT/utility/reader/XMLTargetReader.cpp
        src/HatScheT/utility/writer/XMLResourceWriter.cpp
        src/HatScheT/base/IterativeSchedulerBase.cpp
        src/HatScheT/utility/subgraphs/SCC.cpp
        src/HatScheT/utility/SDCSolver.cpp
        src/HatScheT/base/ModuloSchedulerBase.cpp
        src/HatScheT/layers/IterativeModuloSchedulerLayer.cpp
        src/HatScheT/scheduler/dev/DeSouzaRosa23NIS.cpp)


set(HatScheT_HEADERS
        src/HatScheT/base/SATSchedulerBase.h
        src/HatScheT/base/Z3SchedulerBase.h
        src/HatScheT/base/ILPSchedulerBase.h
        src/HatScheT/base/ModuloSchedulerBase.h
        src/HatScheT/base/SchedulerBase.h
        src/HatScheT/base/IterativeSchedulerBase.h
        src/HatScheT/layers/RationalIISchedulerLayer.h
        src/HatScheT/layers/IterativeModuloSchedulerLayer.h
        src/HatScheT/TargetModel.h
        src/HatScheT/Edge.h
        src/HatScheT/Graph.h
        src/HatScheT/ResourceModel.h
        src/HatScheT/scheduler/ALAPScheduler.h
        src/HatScheT/scheduler/ASAPScheduler.h
        src/HatScheT/scheduler/graphBased/SGMScheduler.h
        src/HatScheT/scheduler/ilpbased/ASAPILPScheduler.h
        src/HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h
        src/HatScheT/scheduler/ilpbased/MoovacResAwScheduler.h
        src/HatScheT/scheduler/ilpbased/MoovacScheduler.h
        src/HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h
        src/HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h
        src/HatScheT/scheduler/ilpbased/SuchaHanzalek11ResAwScheduler.h
        src/HatScheT/scheduler/ilpbased/RationalIIScheduler.h
        src/HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h
        src/HatScheT/scheduler/dev/IntegerIINonRectScheduler.h
        src/HatScheT/scheduler/dev/ClockGatingModuloScheduler.h
        src/HatScheT/scheduler/dev/MinRegMultiScheduler.h
        src/HatScheT/scheduler/graphBased/PBScheduler.h
        src/HatScheT/scheduler/dev/ModuloQScheduler.h
        src/HatScheT/scheduler/dev/UniformRationalIIScheduler.h
        src/HatScheT/scheduler/dev/UniformRationalIISchedulerNew.h
        src/HatScheT/scheduler/dev/RationalIIModuloSDCScheduler.h
        src/HatScheT/scheduler/dev/NonUniformRationalIIScheduler.h
        src/HatScheT/scheduler/dev/UnrollRationalIIScheduler.h
        src/HatScheT/scheduler/dev/SCCQScheduler.h
        src/HatScheT/scheduler/dev/CombinedRationalIIScheduler.h
        src/HatScheT/scheduler/dev/DeSouzaRosa23NIS.h
        src/HatScheT/scheduler/ULScheduler.h
        src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h
        src/HatScheT/utility/Exception.h
        src/HatScheT/utility/reader/GraphMLGraphReader.h
        src/HatScheT/utility/reader/XMLResourceReader.h
        src/HatScheT/utility/reader/XMLTargetReader.h
        src/HatScheT/utility/reader/ScheduleAndBindingReader.h
        src/HatScheT/utility/reader/Reader.h
        src/HatScheT/utility/subgraphs/KosarajuSCC.h
        src/HatScheT/utility/subgraphs/Occurrence.h
        src/HatScheT/utility/subgraphs/OccurrenceSet.h
        src/HatScheT/utility/subgraphs/OccurrenceSetCombination.h
        src/HatScheT/utility/Tests.h
        src/HatScheT/utility/ILPScheduleLengthEstimation.h
        src/HatScheT/scheduler/dev/ILPScheduleLengthSweeper.h
        src/HatScheT/utility/Utility.h
        src/HatScheT/utility/Binding.h
        src/HatScheT/utility/BindingBase.h
        src/HatScheT/utility/TreeBind.h
        src/HatScheT/utility/OptimalIntegerIIBinding.h
        src/HatScheT/utility/OptimalIntegerIISATBinding.h
        src/HatScheT/utility/OptimalIntegerIIBindingCong.h
        src/HatScheT/utility/OptimalIntegerIIGeneralizedBinding.h
        src/HatScheT/utility/Verifier.h
        src/HatScheT/utility/writer/DotWriter.h
        src/HatScheT/utility/writer/ScheduleAndBindingWriter.h
        src/HatScheT/utility/writer/DGLWriter.h
        src/HatScheT/utility/writer/Writer.h
        src/HatScheT/utility/writer/GraphMLGraphWriter.h
        src/HatScheT/utility/writer/XMLResourceWriter.h
        src/HatScheT/utility/subgraphs/SCC.h
        src/HatScheT/Vertex.h
        src/HatScheT/scheduler/dev/DaiZhang19Scheduler.h
        src/HatScheT/scheduler/dev/SDSScheduler.h
        src/HatScheT/scheduler/satbased/SATScheduler.h
        src/HatScheT/scheduler/satbased/SATCDLScheduler.h
        src/HatScheT/scheduler/satbased/SATSCCScheduler.h
        src/HatScheT/scheduler/satbased/SATRatIIScheduler.h
        src/HatScheT/scheduler/satbased/SATSCCRatIIScheduler.h
        src/HatScheT/scheduler/satbased/SATSchedulerLatOpt.h
        src/HatScheT/scheduler/satbased/SATSchedulerBinEnc.h
        src/HatScheT/scheduler/satbased/SATSchedulerBinEncOverlap.h
        src/HatScheT/scheduler/satbased/SATSchedulerRes.h
        src/HatScheT/scheduler/satbased/SATRatIIScheduler.h
        src/HatScheT/scheduler/satbased/SATSCCScheduler.h
        src/HatScheT/scheduler/satbased/SATSCCRatIIScheduler.h
        src/HatScheT/scheduler/satbased/SATCombinedScheduler.h
        src/HatScheT/scheduler/satbased/SATCombinedRatIIScheduler.h
        src/HatScheT/scheduler/satbased/SATMinRegScheduler.h
        src/HatScheT/scheduler/satbased/CaDiCaLTerminator.h
        src/HatScheT/scheduler/smtbased/SMTModScheduler.h
        src/HatScheT/utility/SDCSolver.h
        src/HatScheT/utility/SDCSolverBellmanFord.h
        src/HatScheT/utility/SDCSolverIncremental.h
        src/HatScheT/utility/FibonacciHeap.h
        src/HatScheT/scheduler/smtbased/SMTUnaryScheduler.h
        src/HatScheT/scheduler/smtbased/SMTCDLScheduler.h
        )

IF (SCALP_H AND SCALP_LIB)
    list(APPEND HatScheT_SOURCES
            src/HatScheT/base/ILPSchedulerBase.cpp
            src/HatScheT/scheduler/ilpbased/ASAPILPScheduler.cpp
            src/HatScheT/scheduler/ilpbased/MoovacScheduler.cpp
            src/HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.cpp
            src/HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.cpp
            src/HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.cpp
            src/HatScheT/scheduler/ilpbased/SuchaHanzalek11ResAwScheduler.cpp
            src/HatScheT/scheduler/ilpbased/RationalIIScheduler.cpp
            src/HatScheT/scheduler/ilpbased/RationalIIScheduler.cpp
            src/HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.cpp
            src/HatScheT/scheduler/ilpbased/MoovacResAwScheduler.cpp
            src/HatScheT/scheduler/dev/ClockGatingModuloScheduler.cpp
            src/HatScheT/scheduler/dev/IntegerIINonRectScheduler.cpp
            src/HatScheT/scheduler/dev/MinRegMultiScheduler.cpp
            src/HatScheT/scheduler/graphBased/PBScheduler.cpp
            src/HatScheT/scheduler/dev/UnrollRationalIIScheduler.cpp
            src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.cpp
            src/HatScheT/scheduler/dev/ModuloQScheduler.cpp
            src/HatScheT/scheduler/dev/SCCQScheduler.cpp
            src/HatScheT/scheduler/dev/UniformRationalIIScheduler.cpp
            src/HatScheT/scheduler/dev/UniformRationalIISchedulerNew.cpp
            src/HatScheT/scheduler/dev/RationalIIModuloSDCScheduler.cpp
            src/HatScheT/scheduler/dev/CombinedRationalIIScheduler.cpp
            src/HatScheT/scheduler/dev/NonUniformRationalIIScheduler.cpp
            src/HatScheT/scheduler/dev/DaiZhang19Scheduler.cpp
            src/HatScheT/scheduler/graphBased/SGMScheduler.cpp
            src/HatScheT/utility/ILPScheduleLengthEstimation.cpp
            src/HatScheT/scheduler/dev/ILPScheduleLengthSweeper.cpp
            src/HatScheT/utility/Tests.cpp
            src/HatScheT/utility/OptimalIntegerIIBinding.cpp
            src/HatScheT/utility/OptimalIntegerIIBindingCong.cpp
            src/HatScheT/utility/OptimalIntegerIIGeneralizedBinding.cpp)
ENDIF (SCALP_H AND SCALP_LIB)

IF (SCALP_H AND SCALP_LIB AND Z3_H AND Z3_LIB)
    list(APPEND HatScheT_SOURCES
            src/HatScheT/scheduler/SCCPreprocessingSchedulers/SCCSchedulerTemplate.cpp
            )
ENDIF (SCALP_H AND SCALP_LIB AND Z3_H AND Z3_LIB)

IF (SCALP_H AND SCALP_LIB AND CADICAL_H AND CADICAL_LIB)
    list(APPEND HatScheT_SOURCES
            src/HatScheT/utility/OptimalIntegerIISATBinding.cpp
            )
ENDIF (SCALP_H AND SCALP_LIB AND CADICAL_H AND CADICAL_LIB)

IF (CADICAL_H AND CADICAL_LIB)
    list(APPEND HatScheT_SOURCES
            src/HatScheT/base/SATSchedulerBase.cpp
            src/HatScheT/scheduler/dev/SDSScheduler.cpp
            src/HatScheT/scheduler/satbased/SATScheduler.cpp
            src/HatScheT/scheduler/satbased/SATCDLScheduler.cpp
            src/HatScheT/scheduler/satbased/SATSchedulerRes.cpp
            src/HatScheT/scheduler/satbased/SATSCCScheduler.cpp
            src/HatScheT/scheduler/satbased/SATRatIIScheduler.cpp
            src/HatScheT/scheduler/satbased/SATSCCRatIIScheduler.cpp
            src/HatScheT/scheduler/satbased/SATSchedulerLatOpt.cpp
            src/HatScheT/scheduler/satbased/SATSchedulerBinEnc.cpp
            src/HatScheT/scheduler/satbased/SATSchedulerBinEncOverlap.cpp
            src/HatScheT/scheduler/satbased/SATRatIIScheduler.cpp
            src/HatScheT/scheduler/satbased/SATSCCScheduler.cpp
            src/HatScheT/scheduler/satbased/SATSCCRatIIScheduler.cpp
            src/HatScheT/scheduler/satbased/SATCombinedScheduler.cpp
            src/HatScheT/scheduler/satbased/SATCombinedRatIIScheduler.cpp
            src/HatScheT/scheduler/satbased/SATMinRegScheduler.cpp
            src/HatScheT/scheduler/satbased/CaDiCaLTerminator.cpp
            )
ENDIF (CADICAL_H AND CADICAL_LIB)

IF (Z3_H AND Z3_LIB)
    list(APPEND HatScheT_SOURCES
            src/HatScheT/base/Z3SchedulerBase.cpp
            src/HatScheT/scheduler/smtbased/SMTModScheduler.cpp
            src/HatScheT/scheduler/smtbased/SMTUnaryScheduler.cpp
            src/HatScheT/scheduler/smtbased/SMTCDLScheduler.cpp)
ENDIF (Z3_H AND Z3_LIB)

IF ((Z3_H AND Z3_LIB) AND (SCALP_H AND SCALP_LIB))
    list(APPEND HatScheT_SOURCES
            src/HatScheT/scheduler/SCCPreprocessingSchedulers/SCCSchedulerTemplate.h
            )
ENDIF ()

add_library(hatschet ${HatScheT_SOURCES})
add_executable(hatschet-bin src/hatschet.cpp)
set_target_properties(hatschet-bin PROPERTIES OUTPUT_NAME hatschet)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
target_link_libraries(hatschet-bin hatschet)

IF (SCALP_H AND SCALP_LIB)
    target_link_libraries(hatschet-bin ${SCALP_LIB})
ENDIF (SCALP_H AND SCALP_LIB)
IF (XERCESC_H AND XERCESC_LIB)
    target_link_libraries(hatschet-bin ${XERCESC_LIB})
ENDIF (XERCESC_H AND XERCESC_LIB)
IF (CADICAL_H AND CADICAL_LIB)
    target_link_libraries(hatschet-bin ${CADICAL_LIB})
ENDIF (CADICAL_H AND CADICAL_LIB)
IF (KISSAT_H AND KISSAT_LIB)
    target_link_libraries(hatschet-bin ${KISSAT_LIB})
ENDIF (KISSAT_H AND KISSAT_LIB)
IF (SATSCM_H AND SATSCM_LIB)
    target_link_libraries(hatschet-bin ${SATSCM_LIB})
ENDIF (SATSCM_H AND SATSCM_LIB)

include_directories(src/)

IF (SCALP_H AND SCALP_LIB)
    include_directories(${SCALP_H})
ENDIF (SCALP_H AND SCALP_LIB)

IF (XERCESC_H AND XERCESC_LIB)
    include_directories(${XERCESC_H})
ENDIF (XERCESC_H AND XERCESC_LIB)

IF (CADICAL_H AND CADICAL_LIB)
    include_directories(${CADICAL_H})
ENDIF (CADICAL_H AND CADICAL_LIB)

IF (KISSAT_H AND KISSAT_LIB)
    include_directories(${KISSAT_H})
ENDIF (KISSAT_H AND KISSAT_LIB)

IF (SATSCM_H AND SATSCM_LIB)
    include_directories(${SATSCM_H})
ENDIF (SATSCM_H AND SATSCM_LIB)

####################
#Z3 related
####################
IF (Z3_H AND Z3_LIB)
    target_link_libraries(hatschet-bin ${Z3_LIB})
ENDIF (Z3_H AND Z3_LIB)

IF (Z3_H AND Z3_LIB)
    include_directories(${Z3_H})
ENDIF (Z3_H AND Z3_LIB)

####################
# HaTScheTConfig.cmake
####################

configure_file("${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake.in" "${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake" @ONLY)

####################
# testing
####################

enable_testing()

IF (SCALP_H AND SCALP_LIB AND XERCESC_H AND XERCESC_LIB)
    add_test("Read" "./hatschet" "--test=READ")
    add_test("Moovac" "./hatschet" "--test=MOOVAC")
    add_test("ReadWriteReadSchedule" "./hatschet" "--test=RWRS")
    add_test("ModuloSDC" "./hatschet" "--test=MODULOSDC")
    add_test("API" "./hatschet" "--test=API")
    add_test("ASAPHC" "./hatschet" "--test=ASAPHC")
    add_test("ALAPHC" "./hatschet" "--test=ALAPHC")
    add_test("ULScheduler" "./hatschet" "--test=ULScheduler")
    add_test("criticalpath" "./hatschet" "--test=CRITPATH")
    add_test("KosarajuSCC" "./hatschet" "--test=KOSARAJU")
    add_test("CompareMSAlgorithms" "./hatschet" "--test=COMPAREMSALGORITHMS")
    add_test("rationalIISchedulerTest" "./hatschet" "--test=RATIONALIISCHEDULER")
    add_test("rationalIISchedulerFimmelTest" "./hatschet" "--test=RATIONALIISCHEDULERFIMMEL")
    add_test("ratIIVerifierWrongMRTDetected" "./hatschet" "--test=ratIIVerifierWrongMRTDetected")
    add_test("ratIIVerifierWrongCausalityDetected" "./hatschet" "--test=ratIIVerifierWrongCausalityDetected")
    add_test("ratIIVerifierWrongCausalityDetected" "./hatschet" "--test=ratIIVerifierWrongCausalityDetected")
    add_test("uniformRatII" "./hatschet" "--test=UNIFORMRATIONALIISCHEDULER")
    add_test("uniformRatIINew" "./hatschet" "--test=UNIFORMRATIONALIISCHEDULERNEW")
    add_test("nonUniformRatII" "./hatschet" "--test=NONUNIFORMRATIONALIISCHEDULER")
    add_test("ratIIModuloQ" "./hatschet" "--test=RATIONALIIMODULOQ")
    add_test("ratIISCCQ" "./hatschet" "--test=RATIONALIISCCQ")
    add_test("ratIIUnroll" "./hatschet" "--test=UNROLLSCHEDULER")
    add_test("integerIINonRect" "./hatschet" "--test=INTEGERIINONRECT")
    add_test("optimalBinding" "./hatschet" "--test=ILPBASEDINTIIBINDING")
    add_test("treeBind" "./hatschet" "--test=TREEBIND")
    add_test("treeBindCommutative" "./hatschet" "--test=TREEBINDCOMMUTATIVE")
    add_test("congBind" "./hatschet" "--test=ILPBASEDINTIIBINDINGCONG")
    add_test("LatencyEstimation" "./hatschet" "--test=LATENCY")
    add_test("clockGatingScheduler" "./hatschet" "--test=CLOCKGATING")
ENDIF (SCALP_H AND SCALP_LIB AND XERCESC_H AND XERCESC_LIB)

IF (CADICAL_H AND CADICAL_LIB)
    add_test("CaDiCaL" "./hatschet" "--test=CADICAL")
    add_test("SATScheduler" "./hatschet" "--test=SATSCHEDULER")
ENDIF (CADICAL_H AND CADICAL_LIB)

IF (Z3_H AND Z3_LIB)
    add_test("Z3" "./hatschet" "--test=Z3")
ENDIF (Z3_H AND Z3_LIB)

####################
# example project
####################
IF (SCALP_H AND SCALP_LIB)
    add_executable(example-bin example/example.cpp)
    target_link_libraries(example-bin hatschet)
    target_link_libraries(example-bin ${SCALP_LIB})
ENDIF (SCALP_H AND SCALP_LIB)

####################
# Installation
####################

install(TARGETS hatschet-bin DESTINATION bin)
install(TARGETS hatschet DESTINATION lib)
install(DIRECTORY src/HatScheT/ DESTINATION include/HatScheT FILES_MATCHING PATTERN "*.h")
