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


cmake_minimum_required (VERSION 2.8)
project (hatschet)

####################
#Set options for used libs
####################
option(USE_XERCESC "Use XercesC for parsing" ON)

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
ELSE(USE_XERCESC)
    MESSAGE(STATUS "XercesC was turned off manually (-DUSE_XERCESC=OFF): GraphML parser will not be available")
ENDIF (USE_XERCESC)

set(HatScheT_SOURCES
src/HatScheT/Vertex.cpp
src/HatScheT/Edge.cpp
src/HatScheT/Graph.cpp
src/HatScheT/TargetModel.cpp
src/HatScheT/utility/Exception.cpp
src/HatScheT/base/SchedulerBase.cpp
src/HatScheT/layers/RationalIISchedulerLayer.cpp
src/HatScheT/ResourceModel.cpp
src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.cpp
src/HatScheT/scheduler/ilpbased/RationalIIScheduler.cpp
src/HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.cpp
src/HatScheT/scheduler/dev/ModSDC.cpp
src/HatScheT/utility/reader/Reader.cpp
src/HatScheT/utility/reader/GraphMLGraphReader.cpp
src/HatScheT/utility/reader/XMLResourceReader.cpp
src/HatScheT/utility/subgraphs/Occurrence.cpp
src/HatScheT/utility/subgraphs/OccurrenceSet.cpp
src/HatScheT/utility/subgraphs/OccurrenceSetCombination.cpp
src/HatScheT/utility/writer/Writer.cpp
src/HatScheT/utility/writer/GraphMLGraphWriter.cpp
src/HatScheT/utility/writer/DotWriter.cpp
src/HatScheT/utility/Utility.cpp
src/HatScheT/scheduler/ASAPScheduler.cpp
src/HatScheT/scheduler/ALAPScheduler.cpp
src/HatScheT/scheduler/ULScheduler.cpp
src/HatScheT/utility/Verifier.cpp
src/HatScheT/utility/reader/XMLTargetReader.cpp
src/HatScheT/scheduler/ilpbased/MoovacResAwScheduler.cpp
src/HatScheT/utility/writer/XMLResourceWriter.cpp
src/HatScheT/base/IterativeSchedulerBase.cpp
)

set(HatScheT_HEADERS
src/HatScheT/base/ILPSchedulerBase.h
src/HatScheT/base/ModuloSchedulerBase.h
src/HatScheT/base/SchedulerBase.h
src/HatScheT/base/IterativeSchedulerBase.h
src/HatScheT/layers/RationalIISchedulerLayer.h
src/HatScheT/TargetModel.h
src/HatScheT/Edge.h
src/HatScheT/Graph.h
src/HatScheT/ResourceModel.h
src/HatScheT/scheduler/ALAPScheduler.h
src/HatScheT/scheduler/ASAPScheduler.h
src/HatScheT/scheduler/graphBased/graphBasedMs.h
src/HatScheT/scheduler/graphBased/SGMScheduler.h
src/HatScheT/scheduler/ilpbased/ASAPILPScheduler.h
src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h
src/HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h
src/HatScheT/scheduler/ilpbased/MoovacResAwScheduler.h
src/HatScheT/scheduler/ilpbased/MoovacScheduler.h
src/HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h
src/HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h
src/HatScheT/scheduler/ilpbased/RationalIIScheduler.h
src/HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h
src/HatScheT/scheduler/ULScheduler.h
src/HatScheT/scheduler/dev/ModSDC.cpp
src/HatScheT/utility/Exception.h
src/HatScheT/utility/reader/GraphMLGraphReader.h
src/HatScheT/utility/reader/XMLResourceReader.h
src/HatScheT/utility/reader/XMLTargetReader.h
src/HatScheT/utility/reader/Reader.h
src/HatScheT/utility/subgraphs/Occurrence.h
src/HatScheT/utility/subgraphs/OccurrenceSet.h
src/HatScheT/utility/subgraphs/OccurrenceSetCombination.h
src/HatScheT/utility/Tests.h
src/HatScheT/utility/Utility.h
src/HatScheT/utility/Verifier.h
src/HatScheT/utility/writer/DotWriter.h
src/HatScheT/utility/writer/Writer.h
src/HatScheT/utility/writer/GraphMLGraphWriter.h
src/HatScheT/utility/writer/XMLResourceWriter.h
src/HatScheT/Vertex.h
)

IF (SCALP_H AND SCALP_LIB)
list(APPEND HatScheT_SOURCES
src/HatScheT/base/ILPSchedulerBase.cpp
src/HatScheT/base/ModuloSchedulerBase.cpp
src/HatScheT/scheduler/ilpbased/ASAPILPScheduler.cpp
src/HatScheT/scheduler/ilpbased/MoovacScheduler.cpp
src/HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.cpp
src/HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.cpp
src/HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.cpp
src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.cpp
src/HatScheT/scheduler/ilpbased/RationalIIScheduler.cpp
src/HatScheT/scheduler/graphBased/graphBasedMs.cpp
src/HatScheT/scheduler/graphBased/SGMScheduler.cpp
src/HatScheT/utility/Tests.cpp
)
ENDIF (SCALP_H AND SCALP_LIB)

add_library(hatschet ${HatScheT_SOURCES})

add_executable(hatschet-bin src/hatschet.cpp)
set_target_properties(hatschet-bin PROPERTIES OUTPUT_NAME hatschet)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

target_link_libraries(hatschet-bin hatschet)
IF (SCALP_H AND SCALP_LIB)
target_link_libraries(hatschet-bin ${SCALP_LIB})
ENDIF (SCALP_H AND SCALP_LIB)
IF (XERCESC_H AND XERCESC_LIB)
target_link_libraries(hatschet-bin ${XERCESC_LIB})
ENDIF (XERCESC_H AND XERCESC_LIB)

include_directories(src/)
IF (SCALP_H AND SCALP_LIB)
include_directories(${SCALP_H})
ENDIF (SCALP_H AND SCALP_LIB)
IF (XERCESC_H AND XERCESC_LIB)
include_directories(${XERCESC_H})
ENDIF (XERCESC_H AND XERCESC_LIB)


####################
# HaTScheTConfig.cmake
####################

configure_file("${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake.in" "${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake" @ONLY)

####################
# testing
####################

enable_testing()

add_test("Read" "./hatschet" "--test=READ")
add_test("Moovac" "./hatschet" "--test=MOOVAC")
add_test("ReadWriteReadSchedule" "./hatschet" "--test=RWRS")
add_test("ModuloSDC" "./hatschet" "--test=MODULOSDC")
add_test("API" "./hatschet" "--test=API")
add_test("ASAPHC" "./hatschet" "--test=ASAPHC")
add_test("ALAPHC" "./hatschet" "--test=ALAPHC")
add_test("ULScheduler" "./hatschet" "--test=ULScheduler")
add_test("Occurrence" "./hatschet" "--test=OCC")
add_test("OccurrenceSet" "./hatschet" "--test=OCCS")
add_test("OccurrenceSetCombination" "./hatschet" "--test=OCCSC")
add_test("SGMScheduler" "./hatschet" "--test=SGMS")
#currently disabled cause of ambiguous edges in input graph
#think about supporting this feature somehow
#add_test("RationalMinII" "./hatschet" "--test=RATMINII")
add_test("criticalpath" "./hatschet" "--test=CRITPATH")

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

install (TARGETS hatschet-bin DESTINATION bin)
install (TARGETS hatschet DESTINATION lib)
install (FILES ${HatScheT_HEADERS} DESTINATION include/HatScheT)
