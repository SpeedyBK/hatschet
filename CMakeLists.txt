cmake_minimum_required (VERSION 2.8)
project (hatschet)

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
    MESSAGE(STATUS "Found ScaLP: ${SCALP_H},${SCALP_LIB}")
ELSE (SCALP_H AND SCALP_LIB)
    MESSAGE(FATAL_ERROR "Could not find ScaLP (${SCALP_H},${SCALP_LIB}). Please specify path by setting SCALP_PREFIX_PATH (-DSCALP_PREFIX_PATH=<path to scalp install dir>)")
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
IF (XERCESC_H AND XERCESC_LIB)
    add_definitions(-DUSE_XERCESC)
    MESSAGE(STATUS "Found XercesC: GraphML parser will be available")
ELSE (XERCESC_H AND XERCESC_LIB)
    MESSAGE(STATUS "Did not found XercesC: GraphML parser will not be available")
ENDIF (XERCESC_H AND XERCESC_LIB)

add_library(hatschet
src/HatScheT/Vertex.cpp
src/HatScheT/Edge.cpp
src/HatScheT/Graph.cpp 
src/HatScheT/utility/Exception.cpp
src/HatScheT/base/SchedulerBase.cpp
src/HatScheT/scheduler/ilpbased/MoovacScheduler.cpp
src/HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.cpp
src/HatScheT/base/IterativeSchedulerBase.cpp
src/HatScheT/base/ILPSchedulerBase.cpp
src/HatScheT/base/ModuloSchedulerBase.cpp
src/HatScheT/base/RationalIISchedulerBase.cpp
src/HatScheT/ResourceModel.cpp
src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.cpp
        src/HatScheT/scheduler/ilpbased/rationalIIScheduler.cpp
src/HatScheT/utility/reader/Reader.cpp
src/HatScheT/utility/reader/GraphMLGraphReader.cpp
src/HatScheT/utility/reader/GraphMLResourceReader.cpp
src/HatScheT/utility/subgraphs/Occurrence.cpp
src/HatScheT/utility/subgraphs/OccurrenceSet.cpp
src/HatScheT/utility/subgraphs/OccurrenceSetCombination.cpp
src/HatScheT/utility/writer/Writer.cpp
src/HatScheT/utility/writer/DotWriter.cpp
src/HatScheT/utility/Tests.cpp
src/HatScheT/utility/Utility.cpp
src/HatScheT/scheduler/ASAPScheduler.cpp
src/HatScheT/scheduler/ALAPScheduler.cpp
src/HatScheT/scheduler/ULScheduler.cpp
src/HatScheT/scheduler/graphBased/graphBasedMs.cpp
src/HatScheT/scheduler/graphBased/SGMScheduler.cpp
src/HatScheT/utility/Verifier.cpp
)

add_executable(hatschet-bin src/hatschet.cpp)

set_target_properties(hatschet-bin PROPERTIES OUTPUT_NAME hatschet)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${SCALP_H} ${XERCESC_H} src/)

target_link_libraries(hatschet-bin hatschet)
target_link_libraries(hatschet-bin ${SCALP_LIB})
target_link_libraries(hatschet-bin ${XERCESC_LIB})


####################
# HaTScheTConfig.cmake
####################

configure_file("${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake.in" "${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake" @ONLY)

####################
# testing
####################

enable_testing()

add_test("XercesC_Read" "./hatschet" "--test=READ")
add_test("XercesC_ScaLP_Moovac" "./hatschet" "--test=MOOVAC")
add_test("ScaLP_ModuloSDC" "./hatschet" "--test=MODULOSDC")
add_test("API" "./hatschet" "--test=API")
add_test("ASAP" "./hatschet" "--test=ASAP")
add_test("ASAPHC" "./hatschet" "--test=ASAPHC")
add_test("ALAPHC" "./hatschet" "--test=ALAPHC")
add_test("ULScheduler" "./hatschet" "--test=ULScheduler")
add_test("Occurrence" "./hatschet" "--test=OCC")
add_test("OccurrenceSet" "./hatschet" "--test=OCCS")
add_test("OccurrenceSetCombination" "./hatschet" "--test=OCCSC")
add_test("SGMScheduler" "./hatschet" "--test=SGMS")
