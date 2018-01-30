cmake_minimum_required (VERSION 2.8)
project (hatschet)

#-------------------------
#Find ScaLP
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

#-------------------------
#Find XerxesC lib
FIND_LIBRARY(XERCESC_LIB
        NAMES xerces-c
        HINTS "${XERCESC_LIBRARY_PATH}"
        DOC "Directory of the XercesC library")

IF (XERCESC_LIB)
    MESSAGE(STATUS "Found XercesC Lib: ${XERCESC_LIB}")
ELSE (XERCESC_LIB)
    MESSAGE(STATUS "Could not find XercesC Lib (${XERCESC_LIB}). Please specify path by setting XERCESC_LIBRARY_PATH (-DXERCESC_LIBRARY_PATH=<path to xerces lib dir>)")
ENDIF (XERCESC_LIB)

#-------------------------
#Find XerxesC headers
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

#-------------------------
#set define for XercesC
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
src/HatScheT/Exception.cpp
src/HatScheT/ASAPScheduler.cpp 
src/HatScheT/SchedulerBase.cpp 
src/HatScheT/MoovacScheduler.cpp
src/HatScheT/IterativeSchedulerBase.cpp
src/HatScheT/ILPSchedulerBase.cpp
src/HatScheT/ResourceConstrainedSchedulerBase.cpp
src/HatScheT/ResourceModel.cpp
src/HatScheT/ModuloSchedulerBase.cpp
src/HatScheT/utility/reader/Reader.cpp
src/HatScheT/utility/reader/GraphMLGraphReader.cpp
src/HatScheT/utility/reader/GraphMLResourceReader.cpp
src/HatScheT/utility/writer/Writer.cpp
src/HatScheT/utility/writer/DotWriter.cpp
)

add_executable(hatschet-bin src/hatschet.cpp)

set_target_properties(hatschet-bin PROPERTIES OUTPUT_NAME hatschet)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${SCALP_H} ${XERCESC_H} src/)

target_link_libraries(hatschet-bin hatschet)
target_link_libraries(hatschet-bin ${SCALP_LIB})
target_link_libraries(hatschet-bin ${XERCESC_LIB})


